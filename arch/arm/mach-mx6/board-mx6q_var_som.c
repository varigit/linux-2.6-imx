/*
 * Copyright (C) 2013 Variscite Ltd. All Rights Reserved.
 * Maintainer: Uri Yosef <uri.y@variscite.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/ion.h>
#include <linux/etherdevice.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <sound/wm8962.h>
#include <linux/mfd/mxc-hdmi-core.h>
#include <linux/ti_wilink_st.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>
#include <linux/wl12xx.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_var_som.h"
#include "board-mx6solo_var_som.h"
#include <mach/imx_rfkill.h>

#define VAR_SOM_ECSPI3_CS0      IMX_GPIO_NR(4, 24)
#define VAR_SOM_ADS7846_INT     IMX_GPIO_NR(4, 25)
#define VAR_SOM_ADS7846_PD      IMX_GPIO_NR(4, 25)
#define VAR_SOM_BACKLIGHT_EN    IMX_GPIO_NR(4, 30)

#define VAR_SOM_MIPICSI_PWN	    IMX_GPIO_NR(3, 13)
#define VAR_SOM_MIPICSI_RST     IMX_GPIO_NR(4, 10)

#define VAR_SOM_SD2_CD          IMX_GPIO_NR(4, 14)
#define VAR_SOM_SD2_WP          IMX_GPIO_NR(4, 15)

#define VAR_SOM_PCIE_DIS_B      IMX_GPIO_NR(3, 21)
#define VAR_SOM_PCIE_WAKE_B     IMX_GPIO_NR(3, 22)
#define VAR_SOM_PCIE_PWR_EN     IMX_GPIO_NR(2, 12)
#define VAR_SOM_PCIE_RST_B_REVB IMX_GPIO_NR(5, 13)

#define VAR_SOM_KEY_BACK_GPIO   IMX_GPIO_NR(5, 20)

#define VAR_SOM_WL1271_WL_IRQ   IMX_GPIO_NR(6, 17)
#define VAR_SOM_WL1271_WL_EN    IMX_GPIO_NR(7, 8)
#define VAR_SOM_WL1271_BT_EN    IMX_GPIO_NR(6, 18)

#define VAR_SOM_PFUZE_INT       IMX_GPIO_NR(7, 12)

#define VAR_SOM_TSC_CTW6120_IRQ_GPIO IMX_GPIO_NR(3, 7)

static struct clk *sata_clk;
static struct clk *clko;
static int enable_lcd_ldb;
static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern int epdc_enabled;
extern volatile int num_cpu_idle_lock;

#define VAR_LCD_UTM     0
#define VAR_LCD_CTW6120 1
static int var_lcd_index = VAR_LCD_UTM;
 
/*
 * parse touch screen from bootargs; var_ts=ctw6120|reststive.
 * (default is reststive).
 */
static int __init var_ts_type_setup(char *str)
{
	if (str && strcmp("ctw6120", str) == 0) {
		var_lcd_index = VAR_LCD_CTW6120;
	}

	return 1;
}
__setup("var_ts_type=", var_ts_type_setup);

/* 
 * GPIO Buttons
 */
static struct gpio_keys_button gpio_keys_button_pins[] = {
	{
		.code		= KEY_BACK,
		.gpio		= VAR_SOM_KEY_BACK_GPIO,
		.desc		= "Back",
		.active_low	= 1,
		.wakeup		= 1,
	},
};

static struct gpio_keys_platform_data mx6q_var_som_button_data = {
	.buttons	= gpio_keys_button_pins,
	.nbuttons	= ARRAY_SIZE(gpio_keys_button_pins),
};

static struct platform_device mx6q_var_som_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &mx6q_var_som_button_data,
	},
};

static void __init imx6q_add_device_buttons(void)
{
	platform_device_register(&mx6q_var_som_button_device);
}

enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

static int plt_sd_pad_change(unsigned int index, int clock)
{
	/* LOW speed is the default state of SD pads */
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	iomux_v3_cfg_t *sd_pads_200mhz = NULL;
	iomux_v3_cfg_t *sd_pads_100mhz = NULL;
	iomux_v3_cfg_t *sd_pads_50mhz = NULL;

	u32 sd_pads_200mhz_cnt;
	u32 sd_pads_100mhz_cnt;
	u32 sd_pads_50mhz_cnt;

	switch (index) {
	case 2:
		if (cpu_is_mx6q()) {
			sd_pads_200mhz = mx6q_sd3_200mhz;
			sd_pads_100mhz = mx6q_sd3_100mhz;
			sd_pads_50mhz = mx6q_sd3_50mhz;

			sd_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd3_200mhz);
			sd_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd3_100mhz);
			sd_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd3_50mhz);
		}
		else if (cpu_is_mx6dl()) {
			sd_pads_200mhz = mx6dl_sd3_200mhz;
			sd_pads_100mhz = mx6dl_sd3_100mhz;
			sd_pads_50mhz = mx6dl_sd3_50mhz;

			sd_pads_200mhz_cnt = ARRAY_SIZE(mx6dl_sd3_200mhz);
			sd_pads_100mhz_cnt = ARRAY_SIZE(mx6dl_sd3_100mhz);
			sd_pads_50mhz_cnt = ARRAY_SIZE(mx6dl_sd3_50mhz);
		}
		break;
	default:
		printk(KERN_ERR "no such SD host controller index %d\n", index);
		return -EINVAL;
	}

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;
		BUG_ON(!sd_pads_200mhz);
		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_200mhz,
							sd_pads_200mhz_cnt);
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;
		BUG_ON(!sd_pads_100mhz);
		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_100mhz,
							sd_pads_100mhz_cnt);
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;
		BUG_ON(!sd_pads_50mhz);
		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_50mhz,
							sd_pads_50mhz_cnt);
	}
}

/* SD/MMC */
static const struct esdhc_platform_data mx6q_var_som_sd2_data __initconst = {
	.cd_gpio = VAR_SOM_SD2_CD,
	.wp_gpio = VAR_SOM_SD2_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.runtime_pm = 1,
};

#ifdef CONFIG_WL12XX_PLATFORM_DATA

static void wlan_sdio_set_power(int on)
{
	pr_debug("%s:%s: set power(%d)\n",
		 __FILE__, __func__, on);
	gpio_set_value(VAR_SOM_WL1271_WL_EN,on);
}

/* WLAN */

static void sdio_set_power(int on)
{
	pr_debug("%s:%s: set power(%d)\n",
		 __FILE__, __func__, on);
	gpio_set_value(VAR_SOM_WL1271_WL_EN,on);
}

static struct esdhc_platform_data mx6q_var_som_sd3_data = {
	.always_present = 1,
	.cd_gpio = -1,
	.wp_gpio = -1,
	.keep_power_at_suspend = 0,
	.caps = MMC_CAP_POWER_OFF_CARD,
	.platform_pad_change = plt_sd_pad_change,
	.set_power = wlan_sdio_set_power,
};
#endif

static int __init gpmi_nand_platform_init(void)
{
	iomux_v3_cfg_t *nand_pads = NULL;
	u32 nand_pads_cnt;

	if (cpu_is_mx6q()) {
		nand_pads = mx6q_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(mx6q_gpmi_nand);
	}
	else if (cpu_is_mx6dl()) {
		nand_pads = mx6dl_gpmi_nand;
		nand_pads_cnt = ARRAY_SIZE(mx6dl_gpmi_nand);
	}

	BUG_ON(!nand_pads);

	return mxc_iomux_v3_setup_multiple_pads(nand_pads, nand_pads_cnt);
}

static const struct gpmi_nand_platform_data mx6q_gpmi_nand_platform_data __initconst = {
	.platform_init           = gpmi_nand_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 1,
	.enable_bbt              = 1,
	.enable_ddr              = 0,
};

static const struct anatop_thermal_platform_data
	mx6q_var_som_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static const struct imxuart_platform_data mx6_var_som_uart1_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS,	// | IMXUART_SDMA,
	.dma_req_rx = MX6Q_DMA_REQ_UART2_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART2_TX,
};

static const struct imxuart_platform_data mx6_var_som_uart2_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS, 
	.dma_req_rx = MX6Q_DMA_REQ_UART3_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART3_TX,
};

static inline void mx6q_var_som_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, &mx6_var_som_uart1_data);
	imx6q_add_imx_uart(2, &mx6_var_som_uart2_data);
}

static int mx6q_var_som_fec_phy_init(struct phy_device *phydev)
{
	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_var_som_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
};

/* SPI
 */
static int mx6q_var_som_spi_cs[] = {
	VAR_SOM_ECSPI3_CS0,
};

static const struct spi_imx_master mx6q_var_som_spi_data __initconst = {
	.chipselect     = mx6q_var_som_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_var_som_spi_cs),
};

static struct ads7846_platform_data ads7846_config = {
	.x_max               = 8080,
	.y_max               = 7980,
	.x_min               = 4200,
	.y_min               = 4330,
	.x_plate_ohms        = 180,
	.pressure_max        = 255,
	.debounce_max        = 10,
	.debounce_tol        = 3,
	.debounce_rep        = 1,
	.gpio_pendown        = VAR_SOM_ADS7846_PD,
	.keep_vref_on        = 1,
	.settle_delay_usecs  = 150,
	.wakeup              = true,
};

static struct spi_board_info mx6_var_som_spi_ts_device[] __initdata = {
	{
		.modalias = "ads7846",
		.max_speed_hz = 1500000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 2,
		.chip_select = 0,
		.irq = gpio_to_irq(VAR_SOM_ADS7846_INT),
		.platform_data = &ads7846_config,
	},
};

static void spi_device_init(void)
{
	if (var_lcd_index != VAR_LCD_CTW6120) {
		spi_register_board_info(mx6_var_som_spi_ts_device,
				ARRAY_SIZE(mx6_var_som_spi_ts_device));
	}
}

/* Audio
 */
static struct imx_ssi_platform_data mx6_var_som_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_var_som_audio_wm8962_device = {
	.name = "imx-wm8962",
};

static struct mxc_audio_platform_data wm8962_data;

static int wm8962_clk_enable(int enable)
{
	if (enable)
		clk_enable(clko);
	else
		clk_disable(clko);

	return 0;
}

static int mxc_wm8962_init(void)
{
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	/*  audio codec CLKO clk*/
	rate = clk_round_rate(clko, 24000000);
	clk_set_rate(clko, rate);

	wm8962_data.sysclk = rate;

	return 0;
}

static struct wm8962_pdata wm8962_config_data = {
	.gpio_init = {
		[2] = WM8962_GPIO_FN_DMICCLK,
		[4] = 0x8000 | WM8962_GPIO_FN_DMICDAT,
	},
};

static struct mxc_audio_platform_data wm8962_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_gpio = -1,
	.hp_active_low = 1,
	.mic_gpio = -1,
	.mic_active_low = 1,
	.init = mxc_wm8962_init,
	.clock_enable = wm8962_clk_enable,
};

/*
 * MIPI CSI2 CAM
 */
static void mx6q_mipi_powerdown(int powerdown)
{
	pr_info("%s: powerdown=%d, power_gp=0x%x\n",
			__func__, powerdown, VAR_SOM_MIPICSI_PWN);
	
	if (powerdown)
		gpio_set_value(VAR_SOM_MIPICSI_PWN, 0);
	else
		gpio_set_value(VAR_SOM_MIPICSI_PWN, 1);

	msleep(2);
}

static void mx6q_mipi_sensor_io_init(void)
{
	/* Camera reset */
	gpio_request(VAR_SOM_MIPICSI_RST, "cam-reset");
	gpio_direction_output(VAR_SOM_MIPICSI_RST, 1);

	/* Camera power down */
	gpio_request(VAR_SOM_MIPICSI_PWN, "cam-pwdn");
	gpio_direction_output(VAR_SOM_MIPICSI_PWN, 1);
	msleep(5);
	gpio_set_value(VAR_SOM_MIPICSI_PWN, 0);
	msleep(5);
	gpio_set_value(VAR_SOM_MIPICSI_RST, 0);
	msleep(1);
	gpio_set_value(VAR_SOM_MIPICSI_RST, 1);
	msleep(5);
	gpio_set_value(VAR_SOM_MIPICSI_PWN, 1);

	if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 0);
}

static struct fsl_mxc_camera_platform_data mipi_csi2_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx6q_mipi_sensor_io_init,
	.pwdn = mx6q_mipi_powerdown,
};

static struct imxi2c_platform_data mx6q_var_som_i2c_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data mx6q_var_som_i2c_400k_data = {
	.bitrate = 400000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("ov5640_mipi", 0x3c),
		.platform_data = (void *)&mipi_csi2_data,
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x1b),
	},
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("ctw6120-mt", 0x38),
		.flags = I2C_CLIENT_WAKE,
		.irq = gpio_to_irq(VAR_SOM_TSC_CTW6120_IRQ_GPIO),
	},
	{
		I2C_BOARD_INFO("ds1307", 0x68),
		.type		= "ds1307",
	},
};

static void imx6q_var_som_usbotg_vbus(bool on)
{
}

static void __init imx6q_var_som_init_usb(void)
{
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	mx6_set_otghost_vbus_func(imx6q_var_som_usbotg_vbus);
}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_var_som_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;

release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

static void mx6q_var_som_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_var_som_sata_data = {
	.init = mx6q_var_som_sata_init,
	.exit = mx6q_var_som_sata_exit,
};
static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M + SZ_64M, 
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data var_som_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "VAR-WVGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "hdmi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1920x1080M@60",
	.default_bpp = 32,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "VAR-WVGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x var_som board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_var_som_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_var_som_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_var_som_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_var_som_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_var_som_i2c2_pads,
			ARRAY_SIZE(mx6dl_var_som_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_var_som_i2c2_pads,
			ARRAY_SIZE(mx6q_var_som_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
	.phy_reg_vlev = 0x0294,
	.phy_reg_cksymtx = 0x800d,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SEP0,
	.sec_ipu_id = 1,
	.sec_disp_id = 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false, 
	},
};


static struct ion_platform_data imx_ion_data = {
	.nr = 1,
	.heaps = {
		{
			.id = 0,
			.type = ION_HEAP_TYPE_CARVEOUT,
			.name = "vpu_ion",
			.size = SZ_16M,
			.cacheable = 1,
		},
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};

struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = SZ_128M,
};

static void var_som_suspend_enter(void)
{

}

static void var_som_suspend_exit(void)
{

}
static const struct pm_platform_data mx6q_var_som_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = var_som_suspend_enter,
	.suspend_exit = var_som_suspend_exit,
};

#ifdef CONFIG_WL12XX_PLATFORM_DATA
static struct regulator_consumer_supply vwl1271_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
};

static struct regulator_init_data vwl1271_reg_init = {
	.constraints            = {
		.name           = "VDD_1.8V",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(vwl1271_consumers),
	.consumer_supplies = vwl1271_consumers,
};

static struct fixed_voltage_config vwl1271_reg_config = {
	.supply_name		= "vwl1271",
	.microvolts		= 1800000, /* 1.80V */
	.gpio			= VAR_SOM_WL1271_WL_EN,
	.startup_delay		= 70000, /* 70ms */
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &vwl1271_reg_init,
};

static struct platform_device vwl1271_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 4,
	.dev	= {
		.platform_data = &vwl1271_reg_config,
	},
};

#endif


#ifdef CONFIG_TI_ST
/* TI-ST for WL12xx BT */

static int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

static int plat_kim_chip_enable(struct kim_data_s *kim_data)
{
	gpio_direction_output(kim_data->nshutdown, 0);
	msleep(1);
	gpio_direction_output(kim_data->nshutdown, 1);

	return 0;
}

static int plat_kim_chip_disable(struct kim_data_s *kim_data)
{
	gpio_direction_output(kim_data->nshutdown, 0);

	return 0;
}

static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = VAR_SOM_WL1271_BT_EN,
	.dev_name = "/dev/ttymxc1",
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
	.chip_enable = plat_kim_chip_enable,
	.chip_disable = plat_kim_chip_disable,
};

static struct platform_device wl12xx_device = {
	.name       = "kim",
	.id     = -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static void var_som_init_btwilink(void)
{
	pr_info("var_som_mx6: bt init\n");

	platform_device_register(&wl12xx_device);
	platform_device_register(&btwilink_device);
}
#endif



static struct regulator_consumer_supply var_som_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vcc", "spi2.0"),
};

static struct regulator_init_data var_som_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(var_som_vmmc_consumers),
	.consumer_supplies = var_som_vmmc_consumers,
};

static struct fixed_voltage_config var_som_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &var_som_vmmc_init,
};

static struct platform_device var_som_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &var_som_vmmc_reg_config,
	},
};

static int __init imx6q_init_audio(void)
{
	mxc_register_device(&mx6_var_som_audio_wm8962_device,
			&wm8962_data);
	imx6q_add_imx_ssi(1, &mx6_var_som_ssi_pdata);

	mxc_wm8962_init();

	return 0;
}

static struct platform_pwm_backlight_data mx6_var_som_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data var_som_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
   char *str;
   struct tag *t;
   int i = 0;
   struct ipuv3_fb_platform_data *pdata_fb = var_som_fb_data;

   for_each_tag(t, tags) {
       if (t->hdr.tag == ATAG_CMDLINE) {
           str = t->u.cmdline.cmdline;
           str = strstr(str, "fbmem=");
           if (str != NULL) {
               str += 6;
               pdata_fb[i++].res_size[0] = memparse(str, &str);
               while (*str == ',' &&
                   i < ARRAY_SIZE(var_som_fb_data)) {
                   str++;
                   pdata_fb[i++].res_size[0] = memparse(str, &str);
               }
           }
           /* ION reserved memory */
           str = t->u.cmdline.cmdline;
           str = strstr(str, "ionmem=");
           if (str != NULL) {
               str += 7;
               imx_ion_data.heaps[0].size = memparse(str, &str);
           }
           /* Primary framebuffer base address */
           str = t->u.cmdline.cmdline;
           str = strstr(str, "fb0base=");
           if (str != NULL) {
               str += 8;
               pdata_fb[0].res_base[0] =
                       simple_strtol(str, &str, 16);
           }
           /* GPU reserved memory */
           str = t->u.cmdline.cmdline;
           str = strstr(str, "gpumem=");
           if (str != NULL) {
               str += 7;
               imx6q_gpu_pdata.reserved_mem_size = memparse(str, &str);
               printk(KERN_ERR "Setting GPU Mem to %ld\n", (long) imx6q_gpu_pdata.reserved_mem_size);
           }
           break;
       }
   }
}

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 0,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

static const struct imx_pcie_platform_data mx6_var_som_pcie_data __initconst = {
	.pcie_pwr_en	= VAR_SOM_PCIE_PWR_EN,
	.pcie_rst	= VAR_SOM_PCIE_RST_B_REVB,
	.pcie_wake_up	= VAR_SOM_PCIE_WAKE_B,
	.pcie_dis	= VAR_SOM_PCIE_DIS_B,
	.pcie_power_always_on = 1,
};

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource = {
   .name = "android ram console",
   .flags = IORESOURCE_MEM,
};

static struct platform_device android_ram_console = {
   .name = "ram_console",
   .num_resources = 1,
   .resource = &ram_console_resource,
};

static int __init imx6x_add_ram_console(void)
{
   return platform_device_register(&android_ram_console);
}
#else
#define imx6x_add_ram_console() do {} while (0)
#endif

static int __init early_enable_lcd_ldb(char *p)
{
	enable_lcd_ldb = 1;
	return 0;
}
early_param("enable_lcd_ldb", early_enable_lcd_ldb);

#ifdef CONFIG_WL12XX_PLATFORM_DATA
struct wl12xx_platform_data wlan_data __initdata = {
	.irq = gpio_to_irq(VAR_SOM_WL1271_WL_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_38, /* 38.4 MHz */
};

static struct gpio wl1271_gpios[] __initdata = {
	{.label = "wl1271_int",		.gpio = VAR_SOM_WL1271_WL_IRQ,	.flags = GPIOF_DIR_IN},
	{.label = "wl1271_bt_en",	.gpio = VAR_SOM_WL1271_BT_EN,	.flags = 0},
	{.label = "wl1271_wl_en",	.gpio = VAR_SOM_WL1271_WL_EN,	.flags = 0},
};

static int init_wlan(void)
{
	int status;

	status = gpio_request_array(wl1271_gpios, ARRAY_SIZE(wl1271_gpios));
	if (status) {
		pr_err("gpio_request_array failed(%d) for wl1271_gpios\n", status);
		return -1;
	}

	imx6q_add_sdhci_usdhc_imx(2, &mx6q_var_som_sd3_data);

	/* WL12XX WLAN init */
	if (wl12xx_set_platform_data(&wlan_data))
		pr_err("error setting wl12xx data\n");

	platform_device_register(&vwl1271_reg_devices);
#ifdef CONFIG_TI_ST
	var_som_init_btwilink();
#endif
	gpio_set_value(VAR_SOM_WL1271_WL_EN, 1); /* momentarily enable */
	gpio_set_value(VAR_SOM_WL1271_BT_EN, 1);
	mdelay(2);
	/*gpio_set_value(VAR_SOM_WL1271_WL_EN, 0);  leave enabled for enumeration */
	gpio_set_value(VAR_SOM_WL1271_BT_EN, 0);

	gpio_free(VAR_SOM_WL1271_WL_EN);
	gpio_free(VAR_SOM_WL1271_BT_EN);

	mdelay(1);

	gpio_export(VAR_SOM_WL1271_WL_EN,1);
	gpio_export(VAR_SOM_WL1271_BT_EN,1);

	return 0;
}
#endif

#define VAR_SOM_KSZ9031_PHY_ID      0x00221621
#define VAR_SOM_KSZ9031_PHY_MASK    0xffffffff

static int var_som_ksz9031_phy_fixup(struct phy_device *phydev)
{

	/* min rx data delay */
	phy_write(phydev, 0x0d, 0x2);
	phy_write(phydev, 0x0e, 0x4);
	phy_write(phydev, 0x0d, 0x4002);
	phy_write(phydev, 0x0e, 0);

    phy_write(phydev, 0x0d, 0x2);
    phy_write(phydev, 0x0e, 0x6);
    phy_write(phydev, 0x0d, 0x4002);
    phy_write(phydev, 0x0e, 0);

    phy_write(phydev, 0x0d, 0x2);
    phy_write(phydev, 0x0e, 0x8);
    phy_write(phydev, 0x0d, 0x4002);
    phy_write(phydev, 0x0e, 0x3F4);

	return 0;
}

static char var_lcd_ctw_str[] = "VAR-WVGA-CTW";

/*
 * Board specific initialization.
 */
static void __init mx6_var_som_board_init(void)
{
	int i;
	int ret;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;
	struct platform_device *voutdev;

	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_var_som_pads,
				ARRAY_SIZE(mx6q_var_som_pads));
	else if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_var_som_pads,
				ARRAY_SIZE(mx6dl_var_som_pads));

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = var_som_dvfscore_data.reg_id;
	soc_reg_id = var_som_dvfscore_data.soc_id;

	mx6q_var_som_init_uart();
	imx6x_add_ram_console();

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.disp_id = 0;
		ldb_data.sec_ipu_id = 0;
		ldb_data.sec_disp_id = 1;
		hdmi_core_data.disp_id = 1;
		if (enable_lcd_ldb) {
			ldb_data.disp_id = 1;
			ldb_data.mode = LDB_SIN1;
		}
	}

	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	if (var_lcd_index == VAR_LCD_CTW6120)
		var_som_fb_data[0].mode_str = var_lcd_ctw_str;

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(var_som_fb_data); i++)
			imx6q_add_ipuv3fb(i, &var_som_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(var_som_fb_data); i++)
			imx6q_add_ipuv3fb(i, &var_som_fb_data[i]);


	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	voutdev = imx6q_add_v4l2_output(0);
	if (vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    vout_mem.res_mbase,
					    vout_mem.res_mbase,
					    vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
					     DMA_MEMORY_EXCLUSIVE));
	}

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_imx_i2c(0, &mx6q_var_som_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_var_som_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_var_som_i2c_400k_data);

	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	mxc_i2c1_board_info[0].platform_data = &wm8962_config_data;
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	ret = gpio_request(VAR_SOM_PFUZE_INT, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return;
	} else {
		gpio_direction_input(VAR_SOM_PFUZE_INT);
		mx6q_var_som_init_pfuze100(VAR_SOM_PFUZE_INT);
	}

	gpio_request(VAR_SOM_BACKLIGHT_EN, "ldb-backlight");
	gpio_direction_output(VAR_SOM_BACKLIGHT_EN, 1);

	/* SPI */
	imx6q_add_ecspi(2, &mx6q_var_som_spi_data);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_var_som_anatop_thermal_data);
	imx6_init_fec(fec_data);
	imx6q_add_pm_imx(0, &mx6q_var_som_pm_data);

	imx6q_add_sdhci_usdhc_imx(1, &mx6q_var_som_sd2_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_var_som_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q())
		imx6q_add_ahci(0, &mx6q_var_som_sata_data);
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&var_som_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm_backlight(1, &mx6_var_som_pwm_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();
	imx6q_add_dvfs_core(&var_som_dvfscore_data);

	if (imx_ion_data.heaps[0].size)
       imx6q_add_ion(0, &imx_ion_data,
           sizeof(imx_ion_data) + sizeof(struct ion_platform_heap));

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}

	imx6q_add_gpmi(&mx6q_gpmi_nand_platform_data);

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

#ifdef CONFIG_WL12XX_PLATFORM_DATA
	init_wlan();
#endif

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);

	imx6q_add_busfreq();

#ifdef CONFIG_IMX_PCIE
	imx6q_add_pcie(&mx6_var_som_pcie_data);
#endif	

	imx6q_add_device_buttons();

	phy_register_fixup_for_uid(VAR_SOM_KSZ9031_PHY_ID, VAR_SOM_KSZ9031_PHY_MASK,
				   var_som_ksz9031_phy_fixup);

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init mx6_var_som_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_var_som_timer = {
	.init   = mx6_var_som_timer_init,
};

static void __init mx6q_var_som_reserve(void)
{
	phys_addr_t phys;
	int i, fb0_reserved = 0, fb_array_size;

	/*
	 * Reserve primary framebuffer memory if its base address
	 * is set by kernel command line.
	 */
	fb_array_size = ARRAY_SIZE(var_som_fb_data);
	if (fb_array_size > 0 && var_som_fb_data[0].res_base[0] &&
			var_som_fb_data[0].res_size[0]) {
		memblock_reserve(var_som_fb_data[0].res_base[0],
				var_som_fb_data[0].res_size[0]);
		memblock_remove(var_som_fb_data[0].res_base[0],
				var_som_fb_data[0].res_size[0]);
		var_som_fb_data[0].late_init = true;
		ipu_data[ldb_data.ipu_id].bypass_reset = true;
		fb0_reserved = 1;
	}
	for (i = fb0_reserved; i < fb_array_size; i++)
		if (var_som_fb_data[i].res_size[0]) {
			/* Reserve for other background buffer. */
			phys = memblock_alloc(var_som_fb_data[i].res_size[0],
					SZ_4K);
			memblock_remove(phys, var_som_fb_data[i].res_size[0]);
			var_som_fb_data[i].res_base[0] = phys;
		}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	phys = memblock_alloc_base(SZ_128K, SZ_4K, SZ_1G);
	memblock_remove(phys, SZ_128K);
	memblock_free(phys, SZ_128K);
	ram_console_resource.start = phys;
	ram_console_resource.end   = phys + SZ_128K - 1;
#endif

#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
				SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

#if defined(CONFIG_ION)
	if (imx_ion_data.heaps[0].size) {
		phys = memblock_alloc(imx_ion_data.heaps[0].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[0].size);
		imx_ion_data.heaps[0].base = phys;
	}
#endif
}

/*
 * initialize __mach_desc_VAR_SOM_MX6 data structure.
 */
MACHINE_START(VAR_SOM_MX6, "Freescale i.MX 6Quad/DualLite/Solo Variscite SOM")
	/* Maintainer: Uri Yosef <uri.y@variscite.com>  */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_var_som_board_init,
	.timer = &mx6_var_som_timer,
	.reserve = mx6q_var_som_reserve,
MACHINE_END
