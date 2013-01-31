/*
 * CTW6120 based touchscreen and sensor driver
 *
 * Copyright (c) 2010 Variscite LTD.
 *
 * Using code from:
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ctw6120.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modifys
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c/ctw6120.h>
#include <linux/module.h>
#include <asm/irq.h>
#include <linux/workqueue.h>


#define TS_POLL_DELAY	(1 * 1000000)	/* ns delay before the first sample */
#define TS_POLL_PERIOD	(30 * 1000000)	// OrenR : change to 100 for better double click

#define	MAX_12BIT		((1 << 12) - 1)

struct ctw6120 {
	char			phys[32];
	char			name[32];

	struct input_dev	*input;
	struct i2c_client	*client;

	struct hrtimer		timer;
	struct work_struct	work;

	spinlock_t		lock;

	unsigned		pendown:1;	/* P: lock */
	unsigned		pending:1;	/* P: lock */
	unsigned		irq_disabled:1;	/* P: lock */
	unsigned		disabled:1;
};

static int is_pen_down(struct device *dev)
{
	struct ctw6120	*ts = dev_get_drvdata(dev);

	return ts->pendown;
}

static ssize_t ctw6120_pen_down_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", is_pen_down(dev));
}

static DEVICE_ATTR(pen_down, S_IRUGO, ctw6120_pen_down_show, NULL);

static ssize_t ctw6120_disable_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct ctw6120	*ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ts->disabled);
}

static ssize_t ctw6120_disable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct ctw6120 *ts = dev_get_drvdata(dev);
	unsigned long i;

	if (strict_strtoul(buf, 10, &i))
		return -EINVAL;

	spin_lock_irq(&ts->lock);

	if (i) {
		ts->disabled = 1;
		ts->irq_disabled = 1;
	}
	else {
		ts->disabled = 0;
		ts->irq_disabled = 0;
	}

	spin_unlock_irq(&ts->lock);

	return count;
}

static DEVICE_ATTR(disable, 0664, ctw6120_disable_show, ctw6120_disable_store);

static struct attribute *ctw6120_attributes[] = {
	&dev_attr_pen_down.attr,
	&dev_attr_disable.attr,
	NULL,
};

static struct attribute_group ctw6120_attr_group = {
	.attrs = ctw6120_attributes,
};

static int ctw6120_read(struct i2c_client *client, char reg, unsigned char *buf, int size)
{
	int ret;
	/* send request to controller */
	ret = i2c_master_send(client, &reg, 1);
	if (ret < 0)
		return ret;

	/* read data from controller */
	ret = i2c_master_recv(client, buf, size);
	if (ret < 0)
		return ret;
	if (ret != size)
		return -EIO;
	return 0;
}


static int ctw6120_collect_report(struct ctw6120 *ts)
{
	int ret;
	unsigned char touch_regs[26];


	ret = -1;

	/* read all the data from controller */
	ctw6120_read(ts->client, 0xF9, touch_regs, 26);

	if (unlikely(!touch_regs[3])) {

		/* check for transition state */
		if (ts->pendown) {
			//input_mt_sync(ts->input);
			input_report_key(ts->input, BTN_TOUCH, 0);
			input_report_abs(ts->input, ABS_PRESSURE, 0);
			input_sync(ts->input);
			ts->pendown = 0;
		}


	} else {
		unsigned short curr_x;
		unsigned short curr_y;
		unsigned short curr_x2;
		unsigned short curr_y2;

		curr_x = (touch_regs[5] << 8) + touch_regs[6];
		curr_x = curr_x & 0x7FF;

		curr_y = (touch_regs[7] << 8) + touch_regs[8];
		curr_y &= 0xFFF;

		curr_x2 = (touch_regs[9] << 8) + touch_regs[10];
		curr_x2 = curr_x2 & 0x7FF;

		curr_y2 = (touch_regs[11] << 8) + touch_regs[12];
		curr_y2 &= 0xFFF;

		/* report pendown event if need */
		if (!ts->pendown) {

			input_report_key(ts->input, BTN_TOUCH, 1);
			ts->pendown = 1;
		}

		curr_x = (curr_x * 800) / 1790;
		curr_y = (curr_y * 480) / 1024;

		curr_x2 = (curr_x2 * 800) / 1790;
		curr_y2 = (curr_y2 * 480) / 1024;



		/* set data to be reported */
		input_report_abs(ts->input, ABS_X, curr_x);
		input_report_abs(ts->input, ABS_Y, curr_y);
		input_report_abs(ts->input, ABS_PRESSURE, 100);	/* FIXME: calculate real pressure */
//printk("%s:%d: x = %u y = %u\n", __func__, __LINE__, curr_x, curr_y);
#if 0
		if (touch_regs[3] > 1){
		input_report_abs(ts->input, ABS_MT_POSITION_X, curr_x2);
		input_report_abs(ts->input, ABS_MT_POSITION_Y, curr_y2);
//printk("%s:%d: x2 = %u y2 = %u\n", __func__, __LINE__, curr_x, curr_y);
		input_mt_sync(ts->input);
		}
#endif
		/* report to input subsystem */
		input_sync(ts->input);
	//	printk(KERN_INFO "x %d, y %d, x2 %d, y2 %d num %d\n", curr_x, curr_y, curr_x2, curr_y2, touch_regs[3]);
		ret = 0;

	}

	return ret;
}

static void ctw6120_work(struct work_struct *work)
{
	int ret;
	struct ctw6120 *ts = container_of(work, struct ctw6120, work);

	/* collect data from controller and report it to input subsytem */
	ret = ctw6120_collect_report(ts);

	if (!ret)
		/* arm timer for collection of next sample */
		hrtimer_start(&ts->timer, ktime_set(0, TS_POLL_PERIOD), HRTIMER_MODE_REL);
}

static enum hrtimer_restart ctw6120_timer(struct hrtimer *handle)
{
	struct ctw6120	*ts = container_of(handle, struct ctw6120, timer);
	ts->irq_disabled = 1;
//	printk(KERN_ERR "t\n");

	spin_lock(&ts->lock);
	/* schedule the work */
	if (!work_pending(&ts->work))
		schedule_work(&ts->work);

	spin_unlock(&ts->lock);

	return HRTIMER_NORESTART;
}

static irqreturn_t ctw6120_irq(int irq, void *handle)
{
	struct ctw6120 *ts = handle;

	/* schedule the work */
	if (!work_pending(&ts->work))
		schedule_work(&ts->work);

	return IRQ_HANDLED;
}

static int __devinit ctw6120_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ctw6120 *ts;
	struct input_dev *input_dev;
	unsigned char revid;
	int status;
	char reg = 0;
	int ret;

	/* check i2c interface */
	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0) {
		dev_dbg(&client->dev, "can't talk I2C\n");
		return -EIO;
	}

	reg = 0;
	i2c_master_send(client, &reg, 1);

	/* read data from controller */
	ret = i2c_master_recv(client, &revid,  sizeof(revid));
	if (ret != sizeof(revid))
		return -EIO;

	dev_info(&client->dev, "Rev.%x touchscreen, irq %d\n", revid, client->irq);
	printk(KERN_ERR "CTW6120: revid %x\n",revid );

	// Set static pointer to reg zero
	reg=CTW6120_STATUS_REG;
	i2c_master_send(client, &reg, 1);

	/* allocate touchscreen data structure */
	ts = kzalloc(sizeof(struct ctw6120), GFP_KERNEL);
	/* store touchscreen data structure in the i2c driver context */
	i2c_set_clientdata(client, ts);

	/* store I2C interface for comunication with controller */
	ts->client = client;

	/* initializa the timer */
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = ctw6120_timer;

	/* initialize spin lock */
	spin_lock_init(&ts->lock);

	/* initialize work queue */
	INIT_WORK(&ts->work, ctw6120_work);

	/* install interrupt handler */
	if (request_irq(client->irq, ctw6120_irq, IRQF_TRIGGER_FALLING,
		client->dev.driver->name, ts)) {
		printk(KERN_ERR "CTW6120: unable install IRQ\n");
	}
	/* enable IRQ handling */
	ts->irq_disabled = 0;

	/* register input device */
	input_dev = input_allocate_device();
	/* set driver name */
	input_dev->name = "CTW6120 Touchscreen";
	/* store phys structure */
	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&client->dev));
	input_dev->phys = ts->phys;
	/* store I2C interface for in the input descriptor */
	input_dev->dev.parent = &client->dev;



	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);


	/* set range of the parameters */
	input_set_abs_params(input_dev, ABS_X, 0, 799, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 479, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);


	status = sysfs_create_group(&client->dev.kobj, &ctw6120_attr_group);
	if (status)
		printk(KERN_ERR "CTW6120: can't create fs group");

	/* register input device */
	status = input_register_device(input_dev);

	/* store input device descriptor for further use */
	ts->input = input_dev;
	/* set pen status as penup */
	ts->pendown = 0;

	return status;
}

static int __devexit ctw6120_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ctw6120_id[] = {
	{ "ctw6120", 0 },
	{ }
};

static struct i2c_driver ctw6120_driver = {
	.driver = {
		.name	= "ctw6120",
		.owner	= THIS_MODULE,
	},
	.probe		= ctw6120_probe,
	.remove		= __devexit_p(ctw6120_remove),
	.id_table	= ctw6120_id,
};

static int __init ctw6120_init(void)
{
	return i2c_add_driver(&ctw6120_driver);
}
module_init(ctw6120_init);

static void __exit ctw6120_exit(void)
{
	i2c_del_driver(&ctw6120_driver);
}
module_exit(ctw6120_exit);

MODULE_DESCRIPTION("CTW6120 TouchScreen Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:ctw6120");
MODULE_AUTHOR("Alex Bikhdriker");
