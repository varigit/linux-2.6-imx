/*
 * Driver for Pixcir I2C touchscreen controllers.
 *
 * Copyright (C) 2010-2011 Pixcir, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/i2c/ctw6120.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define	 MAX_12BIT		((1 << 12) - 1)

struct ctw6120_i2c_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	const struct ctw6120_ts_platform_data *pdata;
	bool exiting;
	unsigned pendown:1;	/* P: lock */
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

static int ctw6120_ts_report(struct ctw6120_i2c_ts_data *tsdata)
{
	unsigned char touch_regs[26];

	if(tsdata->exiting)
		return 0;

	/* read all the data from controller */
	ctw6120_read(tsdata->client, 0xF9, touch_regs, 26);

	if (unlikely(!touch_regs[3])) {

		/* check for transition state */
		if (tsdata->pendown) {
			input_report_key(tsdata->input, BTN_TOUCH, 0);
			input_report_abs(tsdata->input, ABS_PRESSURE, 0);
			input_sync(tsdata->input);
			tsdata->pendown = 0;
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
		if (!tsdata->pendown) {
			input_report_key(tsdata->input, BTN_TOUCH, 1);
			tsdata->pendown = 1;
		}

		curr_x = (curr_x * 800) / 1790;
		curr_y = (curr_y * 480) / 1024;
		curr_x2 = (curr_x2 * 800) / 1790;
		curr_y2 = (curr_y2 * 480) / 1024;

		/* set data to be reported */
		input_report_abs(tsdata->input, ABS_X, curr_x);
		input_report_abs(tsdata->input, ABS_Y, curr_y);
		input_report_abs(tsdata->input, ABS_PRESSURE, 100);	/* FIXME: calculate real pressure */

		/* report to input subsystem */
		input_sync(tsdata->input);
	}

	return 0;
}

static irqreturn_t ctw6120_ts_isr(int irq, void *dev_id)
{
	struct ctw6120_i2c_ts_data *tsdata = dev_id;

	ctw6120_ts_report(tsdata);

	return IRQ_HANDLED;
}

/*
 * Enable/disable interrupt generation
 */
static int ctw6120_int_enable(struct ctw6120_i2c_ts_data *ts, bool enable)
{
	//struct device *dev = &ts->client->dev;

	return 0;
}

static int ctw6120_start(struct ctw6120_i2c_ts_data *ts)
{
	struct device *dev = &ts->client->dev;
	int ret;

	enable_irq(ts->client->irq);

	/* enable interrupt generation */
	ret = ctw6120_int_enable(ts, 1);
	if (ret) {
		dev_err(dev, "Failed to enable interrupt generation\n");
		return ret;
	}

	return 0;
}

static int ctw6120_stop(struct ctw6120_i2c_ts_data *ts)
{
	struct device *dev = &ts->client->dev;
	int ret;

	/* disable interrupt generation */
	ret = ctw6120_int_enable(ts, 0);
	if (ret) {
		dev_err(dev, "Failed to disable interrupt generation\n");
		return ret;
	}

	disable_irq(ts->client->irq);

	return 0;
}

static int ctw6120_input_open(struct input_dev *dev)
{
	struct ctw6120_i2c_ts_data *ts = input_get_drvdata(dev);

	return ctw6120_start(ts);
}

static void ctw6120_input_close(struct input_dev *dev)
{
	struct ctw6120_i2c_ts_data *ts = input_get_drvdata(dev);

	ctw6120_stop(ts);

	return;
}

#if defined(CONFIG_OF)
static const struct of_device_id ctw6120_of_match[];

static struct ctw6120_ts_platform_data *ctw6120_parse_dt(struct device *dev)
{
	struct ctw6120_ts_platform_data *pdata;
	struct device_node *np = dev->of_node;
	const struct of_device_id *match;

	match = of_match_device(of_match_ptr(ctw6120_of_match), dev);
	if (!match)
		return ERR_PTR(-EINVAL);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	if (of_property_read_u32(np, "x-size", &pdata->x_size)) {
		dev_err(dev, "Failed to get x-size property\n");
		return ERR_PTR(-EINVAL);
	}

	if (of_property_read_u32(np, "y-size", &pdata->y_size)) {
		dev_err(dev, "Failed to get y-size property\n");
		return ERR_PTR(-EINVAL);
	}

	dev_dbg(dev, "%s: x %d, y %d\n", __func__,
				pdata->x_size, pdata->y_size);

	return pdata;
}
#else
static struct ctw6120_ts_platform_data *ctw6120_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static int ctw6120_i2c_ts_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	const struct ctw6120_ts_platform_data *pdata = client->dev.platform_data;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	struct ctw6120_i2c_ts_data *tsdata;
	struct input_dev *input;
	int error;
	unsigned char revid;
	char reg = 0;

	if (np) {
		pdata = ctw6120_parse_dt(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	} else if (!pdata) {
		dev_err(&client->dev, "platform data not defined\n");
		return -EINVAL;
	}

	i2c_master_send(client, &reg, 1);

	/* read data from controller */
	error = i2c_master_recv(client, &revid,  sizeof(revid));
	if (error != sizeof(revid))
		return -EIO;

	dev_info(&client->dev, "CTW6120: revid %x touchscreen, irq %d\n", revid, client->irq);

	// Set static pointer to reg zero
	reg=CTW6120_STATUS_REG;
	i2c_master_send(client, &reg, 1);

	tsdata = devm_kzalloc(dev, sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata)
		return -ENOMEM;

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(&client->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	tsdata->client = client;
	tsdata->input = input;
	tsdata->pdata = pdata;

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	input->open = ctw6120_input_open;
	input->close = ctw6120_input_close;

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);

	/* set range of the parameters */
	input_set_abs_params(input, ABS_X, 0, 799, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 479, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);

	input_set_drvdata(input, tsdata);

	error = devm_request_threaded_irq(dev, client->irq, NULL, ctw6120_ts_isr,
				     IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
				     | IRQF_ONESHOT,
				     client->name, tsdata);
	if (error) {
		dev_err(dev, "failed to request irq %d\n", client->irq);
		return error;
	}

	/* Stop device till opened */
	error = ctw6120_stop(tsdata);
	if (error)
		return error;

	error = input_register_device(input);
	if (error)
		return error;

	i2c_set_clientdata(client, tsdata);
	device_init_wakeup(&client->dev, 1);

	return 0;
}

static int ctw6120_i2c_ts_remove(struct i2c_client *client)
{
	struct ctw6120_i2c_ts_data *tsdata = i2c_get_clientdata(client);

	device_init_wakeup(&client->dev, 0);

	tsdata->exiting = true;
	mb();

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ctw6120_i2c_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ctw6120_i2c_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *input = ts->input;
	int ret = 0;

	mutex_lock(&input->mutex);

	if (device_may_wakeup(&client->dev)) {
		/* need to start device if not open, to be wakeup source */
		if (!input->users) {
			ret = ctw6120_start(ts);
			if (ret)
				goto unlock;
		}

		enable_irq_wake(client->irq);

	} else if (input->users) {
		ret = ctw6120_stop(ts);
	}

unlock:
	mutex_unlock(&input->mutex);

	return ret;
}

static int ctw6120_i2c_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ctw6120_i2c_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *input = ts->input;
	int ret = 0;

	mutex_lock(&input->mutex);

	if (device_may_wakeup(&client->dev)) {
		disable_irq_wake(client->irq);

		/* need to stop device if it was not open on suspend */
		if (!input->users) {
			ret = ctw6120_stop(ts);
			if (ret)
				goto unlock;
		}

	} else if (input->users) {
		ret = ctw6120_start(ts);
	}

unlock:
	mutex_unlock(&input->mutex);

	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(ctw6120_dev_pm_ops,
				ctw6120_i2c_ts_suspend, ctw6120_i2c_ts_resume);

static const struct i2c_device_id ctw6120_i2c_ts_id[] = {
	{ "ctw6120_tsc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ctw6120_i2c_ts_id);

#if defined(CONFIG_OF)
static const struct of_device_id ctw6120_of_match[] = {
	{ .compatible = "fsl,ctw6120-tsc", },
	{ }
};
MODULE_DEVICE_TABLE(of, ctw6120_of_match);
#endif

static struct i2c_driver ctw6120_i2c_ts_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ctw6120_tsc",
		.pm	= &ctw6120_dev_pm_ops,
		.of_match_table = of_match_ptr(ctw6120_of_match),
	},
	.probe		= ctw6120_i2c_ts_probe,
	.remove		= ctw6120_i2c_ts_remove,
	.id_table	= ctw6120_i2c_ts_id,
};

module_i2c_driver(ctw6120_i2c_ts_driver);

MODULE_AUTHOR("Uri Yosef <uri.yf@variscite.com>");
MODULE_DESCRIPTION("CTW6120 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
