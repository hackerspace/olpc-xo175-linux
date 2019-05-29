/*
 *  linux/arch/arm/mach-mmp/mk2.c
 *
 *  Support for the Marvell MMP3 Abilene Development Platform.
 *
 *  Copyright (C) 2009-2010 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/smc91x.h>
#include <linux/mfd/max77601.h>
#include <linux/pwm_backlight.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/ds4432.h>
#include <linux/i2c/tpk_r800.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/regulator/fixed.h>
#include <linux/switch_headset.h>
#include <linux/switch.h>
#include <linux/workqueue.h> //william
#if defined(CONFIG_SENSORS_LSM303DLHC_ACC) || \
	defined(CONFIG_SENSORS_LSM303DLHC_MAG)
#include <linux/i2c/lsm303dlhc.h>
#endif
#include <linux/sd8x_rfkill.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <mach/addr-map.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp3.h>
#include <mach/irqs.h>
#include <mach/regs-mpmu.h>
#include <mach/soc_vmeta.h>
#include <mach/tc35876x.h>
#include <mach/camera.h>
#include <mach/isp_dev.h>
#include <mach/hsi_dev.h>
#include <plat/pmem.h>
#include <plat/usb.h>
#include <mach/sram.h>
#include <mach/axis_sensor.h>
#include <mach/uio_hdmi.h>
#include <media/soc_camera.h>
#include <mach/mmp3_pm.h>
#include <mach/regs-apmu.h>
#include <linux/proc_fs.h>
#include <mach/wistron.h>
#include <linux/module.h>
#include <mach/cm3217a.h> //T-Lite2, for CM3217A ambient light sensor
#include "common.h"
#include "onboard.h"

#ifdef CONFIG_TOUCHSCREEN_WACOM_W8001
#include <linux/dualmode.h>
#endif


#define MK2_NR_IRQS		(IRQ_BOARD_START + 64)

static unsigned long mk2_pin_config[] __initdata = {
	/* UART3 */
	GPIO51_UART3_RXD,
	GPIO52_UART3_TXD,

	/* UART2 GPS*/
	GPIO47_UART2_RXD,
	GPIO48_UART2_TXD,
//	GPIO49_UART2_CTS, /*NC in MK2*/
//	GPIO50_UART2_RTS, /*NC in MK2*/

	/* TWSI5 */
	GPIO99_TWSI5_SCL,
	GPIO100_TWSI5_SDA,

	/* TWSI6 */
	GPIO97_TWSI6_SCL,
	GPIO98_TWSI6_SDA,

	/* TWSI2 */
	//GPIO43_TWSI2_SCL,//NC pin
	//GPIO44_TWSI2_SDA,//NC pin

	/* TWSI3 */
	GPIO71_TWSI3_SCL,
	GPIO72_TWSI3_SDA,

	/* TWSI4 */
	TWSI4_SCL,
	TWSI4_SDA,

	/*PWM3*/
	GPIO53_PWM3 | MFP_LPM_DRIVE_LOW,

	/* SSPA1 (I2S) */
	GPIO24_I2S_SYSCLK,
	GPIO25_I2S_BITCLK,
	GPIO26_I2S_SYNC,
	GPIO27_I2S_DATA_OUT,
	GPIO28_I2S_SDATA_IN,

	/* Camera */
	//GPIO67_GPIO, /*NC in MK2*/
	//GPIO73_CAM_MCLK, /*NC in MK2*/

	GPIO0_GPIO,	/* OV5642_RST_N */
	GPIO64_GPIO,	/* OV5642_POWER_DOWN */
	GPIO69_GPIO_LPM_LOW,	/* OV5642 crystal enable */
	GPIO2_GPIO,	/* OV5642 1V5, HIGH ENABLE */
	GPIO82_GPIO,	/* OV5642 vcm enable */

	GPIO1_GPIO,	/* OV2656_RST_N */
	GPIO3_GPIO,	/* OV2656 1V5, HIGH ENABLE*/
	GPIO68_GPIO,	/* OV2656_POWER_DOWN */
	GPIO70_GPIO_LPM_LOW,	/* OV2656 crystal enable */

	/* Camera Flasher */
	GPIO84_GPIO,
	GPIO85_GPIO,

	/* Gyroscope L3G4200D */
	//GPIO65_GPIO, /* GYRO_INT_1 */
	//GPIO66_GPIO, /* GYRO_INT_2 */

	/* mk2 hp detect */
	GPIO23_GPIO,
	GPIO130_GPIO,
	//GPIO139_GPIO, /*NC in MK2*/

	/* mk2 gpio keypad */
	GPIO147_GPIO,
	GPIO148_GPIO,
	GPIO150_GPIO,
	GPIO154_GPIO,

	PMIC_PMIC_INT | MFP_LPM_EDGE_FALL,
	GPIO06_WM8994_LDOEN,
	/* LCD */
	GPIO152_VLCD_3V3 | MFP_LPM_DRIVE_LOW,
	GPIO128_LCD_RST | MFP_LPM_DRIVE_LOW,
	/* backlight */
	GPIO17_BL_EN | MFP_LPM_DRIVE_LOW,

	/* OTG vbus enable signal */
	/* VBUS_EN for MK2 */
	GPIO77_GPIO,

	/* HSIC1 reset pin*/
	GPIO96_HSIC_RESET,
	/* HSI */
	HSI_ACWAKE,
	HSI_ACREADY,
	HSI_ACFLAG,
	HSI_ACDATA,
	HSI_CAWAKE,
	HSI_CAREADY,
	HSI_CAFLAG,
	HSI_CADATA,

	/* BB Power Enable */
	//GPIO63_BB_POWER_EN, /*NC in MK2*/

	/* GPS power enable */
	GPIO12_GPIO | MFP_LPM_DRIVE_LOW,
	/* SSP4 */
	//GPIO78_SSP_CLK, /*NC in MK2*/
	GPIO79_SSP_FRM,
	//GPIO80_SSP_TXD, /*NC in MK2*/
	//GPIO81_SSP_RXD, /*NC in MK2*/
	GPIO101_TSI_INT, /* TS INT*/
	GPIO85_GPIO, /* TS_IO_EN */

	/*Digitizer*/
	GPIO89_GPIO,//RST
	GPIO90_GPIO,//FW
	GPIO91_GPIO,//SLP
	GPIO92_GPIO,//PDE
	GPIO65_UART4_RXD,       /* digitizer UART4 */
	GPIO66_UART4_TXD,       /* digitizer UART4 */

	/*3G VDD_3G_3v3_en */
	GPIO129_GPIO,
	/*BB_RST_N, should always keep high, but low in LPM mode!*/
	GPIO95_GPIO,
	/*W_DISABLE_N (BB_ENABLE) */
	GPIO94_GPIO,
	/*BB_WAKE*/
	GPIO93_GPIO,
  /*USB HUB EN*/
  GPIO78_GPIO, 

	/* touch for mk2*/
	GPIO153_GPIO,
	GPIO101_GPIO,

	/* HDMI */
	GPIO54_HDMI_CEC,
	GPIO59_HDMI_DET,

	/* V_GSEN_EN  G-SENSOR POWER*/
//	GPIO87_GPIO,

	/* Light Sensor CM3217a */
//	GPIO16_GPIO,                    /* power */

	/* P Sensor */
	GPIO106_GPIO, /* RF_PSEN_TOUT */
	GPIO107_GPIO, /* RF_PSEN_TOUT1 */

	//DSI/LVDS configure, 
	//high for transmitter, low for direct LVDS
	GPIO19_GPIO,

	GPIO10_GPIO | MFP_PULL_NONE,

};

static unsigned long mk2_nc_pin_config[] __initdata = {
	GPIO4_GPIO_NC,
	GPIO5_GPIO_NC,
	GPIO7_GPIO_NC,
	GPIO8_GPIO_NC,
	GPIO9_GPIO_NC,
	GPIO13_GPIO_NC,
	GPIO14_GPIO_NC,
	GPIO15_GPIO_NC,
	GPIO16_GPIO_NC,
	GPIO18_GPIO_NC,
	GPIO20_GPIO_NC,
	GPIO21_GPIO_NC,
	GPIO22_GPIO_NC,
	GPIO29_GPIO_NC,
	GPIO30_GPIO_NC,
	GPIO31_GPIO_NC,
	GPIO32_GPIO_NC,
	GPIO43_GPIO_NC,
	GPIO44_GPIO_NC,
	GPIO45_GPIO_NC,
	GPIO46_GPIO_NC,
	GPIO49_GPIO_NC,
	GPIO50_GPIO_NC,
	GPIO61_GPIO_NC,
	GPIO62_GPIO_NC,
	GPIO63_GPIO_NC,
	GPIO67_GPIO_NC,
	GPIO73_GPIO_NC,
	GPIO80_GPIO_NC,
	GPIO81_GPIO_NC,
	GPIO87_GPIO_NC,
	GPIO114_GPIO_NC,
	GPIO115_GPIO_NC,
	GPIO116_GPIO_NC,
	GPIO117_GPIO_NC,
	GPIO118_GPIO_NC,
	GPIO119_GPIO_NC,
	GPIO120_GPIO_NC,
	GPIO121_GPIO_NC,
	GPIO122_GPIO_NC,
	GPIO123_GPIO_NC,
	GPIO124_GPIO_NC,
	GPIO125_GPIO_NC,
	GPIO137_GPIO_NC,
	GPIO139_GPIO_NC,
	GPIO143_GPIO_NC,
	GPIO144_GPIO_NC,
	GPIO151_GPIO_NC,
	GPIO155_GPIO_NC,
	GPIO165_GPIO_NC,
	GPIO166_GPIO_NC,
	GPIO167_GPIO_NC,
	GPIO168_GPIO_NC,
	GPIO171_GPIO_NC,
 };

static unsigned long mmc1_pin_config[] __initdata = {
	GPIO131_MMC1_DAT3,
	GPIO132_MMC1_DAT2,
	GPIO133_MMC1_DAT1,
	GPIO134_MMC1_DAT0,
	GPIO136_MMC1_CMD,
	GPIO135_MMC1_CLK,
	GPIO140_MMC1_CD | MFP_PULL_HIGH,
	GPIO141_MMC1_WP | MFP_PULL_HIGH,
	/* V_SD_EN */
	GPIO138_GPIO | MFP_LPM_DRIVE_LOW ,
};

/* MMC2 is used for WIB card */
static unsigned long mmc2_pin_config[] __initdata = {
	GPIO37_MMC2_DAT3,
	GPIO38_MMC2_DAT2,
	GPIO39_MMC2_DAT1,
	GPIO40_MMC2_DAT0,
	GPIO41_MMC2_CMD,
	GPIO42_MMC2_CLK,

	/* GPIO used for power */
        GPIO58_GPIO | MFP_LPM_DRIVE_HIGH, /* WIFI_RST_N */
        GPIO57_GPIO | MFP_LPM_DRIVE_LOW, /* WIFI_PD_N */

	/* GPIO for wake */
	GPIO55_GPIO, /* WL_BT_WAKE */
	GPIO56_GPIO | MFP_LPM_EDGE_FALL | MFP_PULL_HIGH, /* WLAN_WAKE (8787->soc) */
};
static unsigned long mmc3_pin_config[] __initdata = {
	GPIO108_MMC3_DAT7,
	GPIO109_MMC3_DAT6,
	GPIO161_MMC3_DAT5,
	GPIO163_MMC3_DAT4,
	GPIO111_MMC3_DAT3,
	GPIO110_MMC3_DAT2,
	GPIO162_MMC3_DAT1,
	GPIO164_MMC3_DAT0,
	GPIO145_MMC3_CMD,
	GPIO146_MMC3_CLK,
	/* MMC3_nRST */
	GPIO149_GPIO | MFP_LPM_DRIVE_HIGH,
};

static struct sram_bank mmp3_audiosram_info = {
	.pool_name = "audio sram",
	.step = AUDIO_SRAM_GRANULARITY,
};

static struct sram_bank mmp3_videosram_info = {
	.pool_name = "mmp-videosram",
	.step = VIDEO_SRAM_GRANULARITY,
};

#ifdef CONFIG_UIO_VMETA
static struct vmeta_plat_data mmp_vmeta_plat_data = {
	.bus_irq_handler = NULL,
	.axi_clk_available = 0,
	.power_down_ms = 100,
};

static void __init mmp_init_vmeta(void)
{
	mmp_set_vmeta_info(&mmp_vmeta_plat_data);
}
#endif

#if defined(CONFIG_VIDEO_MV)
/* soc  camera */
static int camera_ov5642_power(struct device *dev, int on)
{
	static struct regulator *avdd_2v8;
	static struct regulator *dovdd_1v8;
	int cam_enable = mfp_to_gpio(GPIO64_GPIO);
	int cam_reset = mfp_to_gpio(GPIO0_GPIO);
	int crystal_enable = mfp_to_gpio(GPIO69_GPIO_LPM_LOW);
	int cam_1v5 = mfp_to_gpio(GPIO2_GPIO);
	int cam_vcm = mfp_to_gpio(GPIO82_GPIO);

	if (!avdd_2v8) {
		avdd_2v8 = regulator_get(NULL, "PMIC_V3_2V8");
		if (IS_ERR(avdd_2v8)) {
			pr_err("%s regulator get avdd_2v8 error!\n", __func__);
			avdd_2v8 = NULL;
			goto regu_avdd_2v8;
		}
	}

	if (!dovdd_1v8) {
		dovdd_1v8 = regulator_get(NULL, "PMIC_V2_1V8");
		if (IS_ERR(dovdd_1v8)) {
			pr_err("%s regulator get dovdd_1v8 error!\n", __func__);
			dovdd_1v8 = NULL;
			goto regu_dovdd_1v8;
		}
	}

	if (gpio_request(cam_enable, "CAM_ENABLE_HI_SENSOR")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", cam_enable);
		goto cam_enable_failed;
	}
	if (gpio_request(cam_reset, "CAM_OV5642_RESET")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", cam_reset);
		goto cam_reset_failed;
	}
	if (gpio_request(crystal_enable, "CAM_OV5642_CRYSTAL")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n"
				, crystal_enable);
		goto crystal_enable_failed;
	}
	if (gpio_request(cam_1v5, "CAM_ENABLE_1V5")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", cam_1v5);
		goto cam_enable_1v5_failed;
	}
	if (gpio_request(cam_vcm, "CAM_ENABLE_VCM")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", cam_vcm);
		goto cam_enable_vcm_failed;
	}

	if (on) {
		regulator_set_voltage(dovdd_1v8, 1800000, 1800000);
		regulator_enable(dovdd_1v8);

		regulator_set_voltage(avdd_2v8, 2800000, 2800000);
		regulator_enable(avdd_2v8);

		gpio_direction_output(crystal_enable, 1);
		gpio_direction_output(cam_reset, 0);
		gpio_direction_output(cam_enable, 1);
		gpio_direction_output(cam_1v5, 1);
		gpio_direction_output(cam_vcm, 1);
		msleep(10);

		gpio_direction_output(cam_enable, 0);
		msleep(10);
		gpio_direction_output(cam_reset, 1);
		msleep(10);
	} else {
		gpio_direction_output(cam_1v5, 0);
		gpio_direction_output(cam_vcm, 0);
		gpio_direction_output(crystal_enable, 0);
		gpio_direction_output(cam_enable, 1);
		gpio_direction_output(cam_reset, 0);

		regulator_disable(avdd_2v8);
		regulator_disable(dovdd_1v8);
	}

	gpio_free(crystal_enable);
	gpio_free(cam_enable);
	gpio_free(cam_reset);
	gpio_free(cam_1v5);
	gpio_free(cam_vcm);

	return 0;

cam_enable_vcm_failed:
	gpio_free(cam_1v5);
cam_enable_1v5_failed:
	gpio_free(crystal_enable);
crystal_enable_failed:
	gpio_free(cam_reset);
cam_reset_failed:
	gpio_free(cam_enable);
cam_enable_failed:
	regulator_put(dovdd_1v8);
regu_dovdd_1v8:
	regulator_put(avdd_2v8);
regu_avdd_2v8:
	return -EIO;
}

static int camera_ov2656_power(struct device *dev, int on)
{
	int cam_enable = mfp_to_gpio(GPIO68_GPIO);
	int cam_reset = mfp_to_gpio(GPIO1_GPIO);
	int crystal_enable = mfp_to_gpio(GPIO70_GPIO);
	int cam_1v5 = mfp_to_gpio(GPIO3_GPIO);

	if (gpio_request(cam_enable, "CAM_ENABLE_HI_SENSOR")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", cam_enable);
		goto cam_enable_failed;
	}
	if (gpio_request(cam_reset, "CAM_OV2656_RESET")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", cam_reset);
		goto cam_reset_failed;
	}
	if (gpio_request(crystal_enable, "CAM_OV2656_CRYSTAL")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n"
				, crystal_enable);
		goto crystal_enable_failed;
	}
	if (gpio_request(cam_1v5, "CAM_ENABLE_1V5")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", cam_1v5);
		goto cam_1v5_failed;
	}

	if (on) {
		gpio_direction_output(crystal_enable, 1);
		gpio_direction_output(cam_reset, 0);
		gpio_direction_output(cam_enable, 1);
		gpio_direction_output(cam_1v5, 1);
		msleep(10);

		gpio_direction_output(cam_enable, 0);
		msleep(10);
		gpio_direction_output(cam_reset, 1);
		msleep(10);
	} else {
		gpio_direction_output(crystal_enable, 0);
		gpio_direction_output(cam_enable, 1);
		gpio_direction_output(cam_reset, 0);
		gpio_direction_output(cam_1v5, 0);
	}

	gpio_free(crystal_enable);
	gpio_free(cam_enable);
	gpio_free(cam_reset);
	gpio_free(cam_1v5);

	return 0;
cam_1v5_failed:
	gpio_free(crystal_enable);
crystal_enable_failed:
	gpio_free(cam_reset);
cam_reset_failed:
	gpio_free(cam_enable);
cam_enable_failed:
	return -EIO;
}

static struct i2c_pxa_platform_data twsi3_pxa_data = {

	.fast_mode = 1,
};

static struct i2c_board_info mk2_i2c_camera[] = {
	{
		I2C_BOARD_INFO("ov5642", 0x3c),
	},
	{
		I2C_BOARD_INFO("ov2656", 0x30),
	},
};

static struct soc_camera_link iclink_ov5642 = {
	.bus_id         = 0,            /* Must match with the camera ID */
	.power          = camera_ov5642_power,
	.board_info     = &mk2_i2c_camera[0],
	.i2c_adapter_id = 2,
	.flags = SOCAM_MIPI,
	.module_name    = "ov5642",
	.priv = "mk2-mipi",
};

static struct soc_camera_link iclink_ov2656 = {
	.bus_id         = 3,            /* Must match with the camera ID */
	.power          = camera_ov2656_power,
	.board_info     = &mk2_i2c_camera[1],
	.i2c_adapter_id = 2,
	.flags = SOCAM_MIPI,
	.module_name    = "ov2656",
	.priv = "mk2-mipi",
};

static struct platform_device mk2_ov5642 = {
	.name   = "soc-camera-pdrv",
	.id     = 0,
	.dev    = {
		.platform_data = &iclink_ov5642,
	},
};

static struct platform_device mk2_ov2656 = {
	.name   = "soc-camera-pdrv",
	.id     = 3,
	.dev    = {
		.platform_data = &iclink_ov2656,
	},
};

static void pxa2128_cam_ctrl_power(int on)
{
	return;
}

static void pxa2128_ov2656_ctrl_power(int on)
{
	return;
}

static int pxa2128_cam_clk_init(struct device *dev, int init)
{
	struct mv_cam_pdata *data = dev->platform_data;
	unsigned long tx_clk_esc;
	struct clk *pll1;

	pll1 = clk_get(dev, "pll1");
	if (IS_ERR(pll1)) {
		dev_err(dev, "Could not get pll1 clock\n");
		return PTR_ERR(pll1);
	}

	tx_clk_esc = clk_get_rate(pll1) / 1000000 / 12;
	/* Update dphy6 according to current tx_clk_esc */
	data->dphy[2] = ((534 * tx_clk_esc / 2000 - 1) & 0xff) << 8
			| ((38 * tx_clk_esc / 1000 - 1) & 0xff);

	clk_put(pll1);

	if (init) {
		if (!data->clk_enabled) {
			data->clk = clk_get(dev, "CCICRSTCLK");
			if (IS_ERR(data->clk)) {
				dev_err(dev, "Could not get rstclk\n");
				return PTR_ERR(data->clk);
			}
			data->clk_enabled = 1;
		}
	} else {
		if (data->clk_enabled) {
			clk_put(data->clk);
			return 0;
		}
	}
	return 0;
}

static void pxa2128_cam_set_clk(struct device *dev, int on)
{
	struct mv_cam_pdata *data = dev->platform_data;

	isppwr_power_control(on);

	if (on)
		clk_enable(data->clk);
	else
		clk_disable(data->clk);
}

static int get_mclk_src(int src)
{
	switch (src) {
	case 3:
		return 400;
	case 2:
		return 266;
	default:
		BUG();
	}

	return 0;
}

static struct mv_cam_pdata mv_cam_data = {
	.name = "MK2_OV5642",
	.clk_enabled = 0,
	//.dphy = {0x1b0b, 0x33, 0x1a03},
	.dphy = {0x0a06, 0x33, 0x0900},
	.qos_req_min = 800,
	.qos_req_cpu_no_dfc = 1,
	.qos_req_ddr_min = 800000,
	.qos_req_ddr_no_dfc = 1,
	.dma_burst = 256,
	.bus_type = SOCAM_MIPI,
	.ccic_num_flag = 0,
	.lane_num = 2, /* 2lane */
	.mclk_min = 24,
	.mclk_src = 2,
	.controller_power = pxa2128_cam_ctrl_power,
	.mipi_enabled = 0,
	.init_clk = pxa2128_cam_clk_init,
	.enable_clk = pxa2128_cam_set_clk,
	.get_mclk_src = get_mclk_src,
};

static struct mv_cam_pdata mv_cam_data_ov2656 = {
	.name = "MK2_OV2656",
	.clk_enabled = 0,
	.dphy = {0x1b0b, 0x33, 0x1a03},
	.qos_req_min = 0,
	.qos_req_cpu_no_dfc = 0,
	.qos_req_ddr_min = 800000,
	.qos_req_ddr_no_dfc = 1,
	.dma_burst = 128,
	.bus_type = SOCAM_MIPI,
	.ccic_num_flag = 1,
	.lane_num = 1, /* 1lane */
	.mclk_min = 26,
	.mclk_src = 3,
	.controller_power = pxa2128_ov2656_ctrl_power,
	.init_clk = pxa2128_cam_clk_init,
	.enable_clk = pxa2128_cam_set_clk,
	.get_mclk_src = get_mclk_src,
};
/* sensor init over */
#endif

/* mk2 GPIO Keyboard */
#define INIT_KEY(_code, _gpio, _active_low, _desc)	\
	{						\
		.code       = KEY_##_code,		\
		.gpio       = _gpio,			\
		.active_low = _active_low,		\
		.desc       = _desc,			\
		.type       = EV_KEY,			\
		.wakeup     = 0,			\
		.debounce_interval     = 0,			\
	}

static struct gpio_keys_button gpio_keys_buttons[] = {
	INIT_KEY(BACK,	mfp_to_gpio(GPIO148_GPIO),	1,	"Back button"),
	INIT_KEY(MENU,	mfp_to_gpio(GPIO147_GPIO),	1,	"Menu button"),
	INIT_KEY(HOME,	mfp_to_gpio(GPIO150_GPIO),	1,	"Home button"),
	INIT_KEY(F5,	mfp_to_gpio(GPIO154_GPIO),	1,	"User button"),
};

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons = gpio_keys_buttons,
	.nbuttons = ARRAY_SIZE(gpio_keys_buttons),
	.rep = 1,
};

static struct platform_device gpio_keys = {
	.name = "gpio-keys",
	.dev  = {
		.platform_data = &gpio_keys_data,
	},
	.id   = -1,
};

static int motion_sensor_set_power(int on, const char *device_name)
{
	static struct regulator *pmic_2p8v_sens[3];
	static int is_enabled[3] = {0, 0, 0};
	int device_index = -1;
//	int gsen_pwr_en = mfp_to_gpio(GPIO87_GPIO);

#if defined(CONFIG_SENSORS_LSM303DLHC_ACC)
	if (!strcmp(device_name, LSM303DLHC_ACC_DEV_NAME))
		device_index = 0;
#endif
#if defined(CONFIG_SENSORS_LSM303DLHC_MAG)
	if (!strcmp(device_name, LSM303DLHC_MAG_DEV_NAME))
		device_index = 1;
#endif

	if ((device_index >= 0) && (device_index <= 1)) {
		/* GPIO power enable */
//		if (gpio_request(gsen_pwr_en, "GSENSOR Enable")) {
//			printk(KERN_INFO "gpio %d request failed\n", gsen_pwr_en);
//			return -1;
//		}

		if (on && (!is_enabled[device_index])) {
			pmic_2p8v_sens[device_index] = regulator_get(NULL, "PMIC_V3_2V8");
			if (IS_ERR(pmic_2p8v_sens[device_index])) {
				pmic_2p8v_sens[device_index] = NULL;
				return -ENODEV;
			} else {
				regulator_set_voltage(pmic_2p8v_sens[device_index], 2800000, 2800000);
				regulator_enable(pmic_2p8v_sens[device_index]);
				is_enabled[device_index] = 1;

//				gpio_direction_output(gsen_pwr_en, 1);
			}
		}
		if ((!on) && is_enabled[device_index]) {
			regulator_disable(pmic_2p8v_sens[device_index]);
			regulator_put(pmic_2p8v_sens[device_index]);
			pmic_2p8v_sens[device_index] = NULL;
			is_enabled[device_index] = 0;

//			gpio_direction_output(gsen_pwr_en, 0);
		}

//		gpio_free(gsen_pwr_en);
	} else
		return -EPERM;
	return 0;
}

#if defined(CONFIG_SENSORS_LSM303DLHC_ACC)
static struct lsm303dlhc_acc_platform_data lsm303dlhc_acc_data = {
	.poll_interval = 50,
	.min_interval = 10,
	.g_range = LSM303DLHC_ACC_G_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 1,
	.negate_y = 1,
	.negate_z = 1,
	.gpio_int1 = -EINVAL,
	.gpio_int2 = -EINVAL,
	.set_power = motion_sensor_set_power,
};
#endif

#if defined(CONFIG_SENSORS_LSM303DLHC_MAG)
static struct lsm303dlhc_mag_platform_data lsm303dlhc_mag_data = {
	.poll_interval = 100,
	.min_interval = 10,
	.h_range = LSM303DLHC_H_8_1G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 1,
	.negate_y = 1,
	.negate_z = 1,
	.set_power = motion_sensor_set_power,
};
#endif


static struct i2c_board_info mk2_twsi4_info[] = {
#if defined(CONFIG_SENSORS_LSM303DLHC_ACC)
	{
		.type           = LSM303DLHC_ACC_DEV_NAME,
		.addr           = (0x32>>1),
		.platform_data  = &lsm303dlhc_acc_data,
	},
#endif
#if defined(CONFIG_SENSORS_LSM303DLHC_MAG)
	{
		.type           = LSM303DLHC_MAG_DEV_NAME,
		.addr           = (0x3C>>1),
		.platform_data  = &lsm303dlhc_mag_data,
	},
#endif
};

static int mk2_max77601_setup(struct max77601_chip *chip)
{
	u8 data = 0;
	if (!chip)
		return -EINVAL;
	/* Set LDO0 and LDO1 work at NOT FPS mode
	 * which can be en/disabled dynamically */
	max77601_set_bits(chip, MAX77601_FPS_L0,
		MAX77601_FPSSRC_NOTFPS, MAX77601_FPSSRC_NOTFPS);
	max77601_set_bits(chip, MAX77601_FPS_L1,
		MAX77601_FPSSRC_NOTFPS, MAX77601_FPSSRC_NOTFPS);

	/*Set LDO5 init voltage to 1.8V to save ~2mA/8V */
	max77601_set_bits(chip, MAX77601_VREG_LDO5,
		MAX77601_LDO_VOLT_M, 0x14);

	/* Set GPIO5 as SD0 DVS(DVSSD0) input: PMIC_GPIO5(VCXO_EN) */
	max77601_set_bits(chip, MAX77601_AME_GPIO,
			MAX77601_AME5_MASK, MAX77601_AME5_MASK);
	/* Set GPIO5 active low, VCXO_EN is low when suspend */
	max77601_set_bits(chip, MAX77601_CNFG_GPIO5, MAX77601_GPIO_DIR, 0);

	/* Set GPIO4 as 32kHZ output(32K_OUT1): EXT_32K_IN */
	max77601_set_bits(chip, MAX77601_AME_GPIO,
			MAX77601_AME4_MASK, MAX77601_AME4_MASK);

	/* Set suspend voltage(DVSSD0) */
	data = 0x10; /* 0.8V */
	max77601_write(chip, MAX77601_VREG_DVSSD0, &data, 1);

	return 0;
};

static struct regulator_consumer_supply regulator_supplies[] = {
	/* Step-down regulators: SD[0..3] */
	[MAX77601_ID_SD0]	 = REGULATOR_SUPPLY("VCC_CORE", NULL),
	[MAX77601_ID_DVSSD0] = REGULATOR_SUPPLY("VCC_CORE_DVS", NULL),
	[MAX77601_ID_SD1]	 = REGULATOR_SUPPLY("PMIC_V1", NULL),
	[MAX77601_ID_DVSSD1] = REGULATOR_SUPPLY("PMIC_V1_DVS", NULL),
	[MAX77601_ID_SD2]	 = REGULATOR_SUPPLY("PMIC_V2_1V8", NULL),
	[MAX77601_ID_SD3]	 = REGULATOR_SUPPLY("PMIC_V3_2V8", NULL),
	/* Linear regulators: L[0..8] */
	[MAX77601_ID_L0] = REGULATOR_SUPPLY("PMIC_LDO0", NULL),
	[MAX77601_ID_L1] = REGULATOR_SUPPLY("PMIC_LDO1", NULL),
	[MAX77601_ID_L2] = REGULATOR_SUPPLY("PMIC_LDO2", NULL),
	[MAX77601_ID_L3] = REGULATOR_SUPPLY("PMIC_LDO3", NULL),
	[MAX77601_ID_L4] = REGULATOR_SUPPLY("PMIC_LDO4", NULL),
	[MAX77601_ID_L5] = REGULATOR_SUPPLY("PMIC_LDO5", NULL),
	[MAX77601_ID_L6] = REGULATOR_SUPPLY("PMIC_LDO6", NULL),
	[MAX77601_ID_L7] = REGULATOR_SUPPLY("PMIC_LDO7", NULL),
	[MAX77601_ID_L8] = REGULATOR_SUPPLY("PMIC_LDO8", NULL),
};

#define REG_INIT(_name, _min, _max, _always, _boot) \
{								\
	.constraints = {					\
		.name		= __stringify(_name),		\
		.min_uV		= _min,				\
		.max_uV		= _max,				\
		.always_on	= _always,			\
		.boot_on	= _boot,			\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE	\
				| REGULATOR_CHANGE_STATUS,	\
	},							\
	.num_consumer_supplies	= 1,				\
	.consumer_supplies	= &regulator_supplies[MAX77601_ID_##_name], \
}

static struct regulator_init_data max77601_regulator_data[] = {
	/* Step-down regulators: SD[0..3] */
	[MAX77601_ID_SD0]	 = REG_INIT(SD0,	600000, 3387500, 1, 1),
	[MAX77601_ID_DVSSD0] = REG_INIT(DVSSD0,	600000, 3387500, 1, 1),
	[MAX77601_ID_SD1]	 = REG_INIT(SD1,	800000, 1587500, 1, 1),
	[MAX77601_ID_DVSSD1] = REG_INIT(DVSSD1, 800000, 1587500, 1, 1),
	[MAX77601_ID_SD2]	 = REG_INIT(SD2,	600000, 3387500, 1, 1),
	[MAX77601_ID_SD3]	 = REG_INIT(SD3,	600000, 3387500, 1, 1),
	/* Linear regulators: L[0..8] */
	[MAX77601_ID_L0] = REG_INIT(L0, 800000, 2350000, 0, 1),
	[MAX77601_ID_L1] = REG_INIT(L1, 800000, 2350000, 0, 1),
	[MAX77601_ID_L2] = REG_INIT(L2, 800000, 3950000, 1, 1),
	[MAX77601_ID_L3] = REG_INIT(L3, 800000, 3950000, 0, 0),
	[MAX77601_ID_L4] = REG_INIT(L4, 800000, 1587500, 0, 0),
	[MAX77601_ID_L5] = REG_INIT(L5, 800000, 3950000, 1, 1),
	[MAX77601_ID_L6] = REG_INIT(L6, 800000, 3950000, 1, 1),
	[MAX77601_ID_L7] = REG_INIT(L7, 800000, 3950000, 0, 0),
	[MAX77601_ID_L8] = REG_INIT(L8, 800000, 3950000, 0, 0),
};

static struct max77601_platform_data mk2_max77601_pdata = {
	.irq_base  = IRQ_BOARD_START,
	.regulator = max77601_regulator_data,
	.setup     = mk2_max77601_setup,
};

struct platform_device wis_kbc_power_device = {

	.name = "wis-kbc-power",
	.id = -1,
};

static struct i2c_board_info mk2_twsi1_info[] = {
#if defined(CONFIG_WIS_KBC)
	{
		.type		= "wis-kbc",
		.addr		= 0x15,
	},
#endif
	{
		.type		= "max77601",
		.addr		= 0x1c,
		.irq		= IRQ_MMP3_PMIC,
		.platform_data	= &mk2_max77601_pdata,
	},
};

static struct platform_pwm_backlight_data mk2_lcd_backlight_data = {
	/* primary backlight */
	.pwm_id = 2,
	.max_brightness = 100,
	.dft_brightness = 20,
	.pwm_period_ns = 2500000,
};

static struct platform_device mk2_lcd_backlight_devices = {
	.name = "pwm-backlight",
	.id = 2,
	.dev = {
		.platform_data = &mk2_lcd_backlight_data,
	},
};

#ifdef CONFIG_MMC_SDHCI_PXAV3
#ifdef CONFIG_SD8XXX_RFKILL
/*
 * during wifi enabled, power should always on and 8787 should always
 * released from reset. 8787 handle power management by itself.
*/
static unsigned long wifi_pin_config_on[] = {
	GPIO57_GPIO | MFP_LPM_DRIVE_HIGH,
};
static unsigned long wifi_pin_config_off[] = {
	GPIO57_GPIO | MFP_LPM_DRIVE_LOW,
};
static void mmp3_8787_set_power(unsigned int on)
{
	/*
	 * 1. WIFI_1V8 <-- PMIC_V2_1V8
	 * this regulator is defined as alwasy on.
	 * but the control code can still be put here.
	 * 2. WIFI_3D3V <-- 3D3VS0
	 * this power domain is supplied by kbc which can't be accessed here.
	 */
	static struct regulator *wifi_1v8;
	static int f_enabled = 0;

	if (!wifi_1v8) {
		wifi_1v8 = regulator_get(NULL, "PMIC_V2_1V8");
		if (IS_ERR(wifi_1v8)) {
			wifi_1v8 = NULL;
			printk(KERN_ERR"get wifi_1v8 failed %s %d \n", __func__, __LINE__);
			return;
		}
	}
	if (on && (!f_enabled)) {
		regulator_set_voltage(wifi_1v8, 1800000, 1800000);
		regulator_enable(wifi_1v8);
		f_enabled = 1;
		mfp_config(ARRAY_AND_SIZE(wifi_pin_config_on));
	}
	if (f_enabled && (!on)) {
		mfp_config(ARRAY_AND_SIZE(wifi_pin_config_off));
		regulator_disable(wifi_1v8);
		f_enabled = 0;
	}
}
#endif

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc0 = {
	.clk_delay_cycles	= 0x1F,
	.flags			= PXA_FLAG_ENABLE_CLOCK_GATING,
	.quirks		= SDHCI_QUIRK_MISSING_CAPS,
	/*
	 * All of sd power is supplied by PMIC_V3_2V8,
	 * which is shared by many devices like emmc.
	 * So it must be fixed at 2.8v and can not support
	 * 1.8V signal function and uhs mode.
	 */
	.host_caps_disable	=
			MMC_CAP_UHS_SDR12 |
			MMC_CAP_UHS_SDR25 |
			MMC_CAP_UHS_SDR104 |
			MMC_CAP_UHS_SDR50 |
			MMC_CAP_UHS_DDR50,
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc1 = {
	.flags          = PXA_FLAG_CARD_PERMANENT |
				PXA_FLAG_WAKEUP_HOST,
	.pm_caps	= MMC_PM_KEEP_POWER,
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc2 = {
	.flags		= PXA_FLAG_SD_8_BIT_CAPABLE_SLOT |
				PXA_FLAG_CARD_PERMANENT |
				PXA_FLAG_ENABLE_CLOCK_GATING,
	.clk_delay_cycles	= 0xF,
};

#include <linux/wakelock.h>
static struct wake_lock wlan_gpio_wakeup;

static irqreturn_t wlan_wake_gpio_irq(int irq, void *data)
{
	/* reset mfp edge status to clear pending wake up source */
	int gpio = irq_to_gpio(irq);
	u32 mfp = mfp_read(gpio);
	mfp_write(gpio, (1 << 6) | mfp);
	mfp_write(gpio, mfp);

	/* hold a 3s wakelock to handle the wakeup event */
	wake_lock_timeout(&wlan_gpio_wakeup, HZ * 3);
	printk(KERN_INFO "%s: set wakelock, timout after 3 seconds\n",
			__func__);

	return IRQ_HANDLED;
}

static void __init mk2_init_mmc(void)
{
	int v_sd_en = mfp_to_gpio(GPIO138_GPIO);
	int emmc_reset_n = mfp_to_gpio(GPIO149_GPIO);
	int wlan_wake = mfp_to_gpio(GPIO56_GPIO);
#ifdef CONFIG_SD8XXX_RFKILL
	int WIB_PDn = mfp_to_gpio(GPIO57_GPIO_LPM_LOW);
	int WIB_RESETn = mfp_to_gpio(GPIO58_GPIO);
	add_sd8x_rfkill_device(WIB_PDn, WIB_RESETn,\
			&mmp3_sdh_platdata_mmc1.pmmc, mmp3_8787_set_power);
#endif
        if (cpu_is_mmp3_b0()) {
                mmp3_sdh_platdata_mmc0.regs_extended = 1;
                mmp3_sdh_platdata_mmc1.regs_extended = 1;
                mmp3_sdh_platdata_mmc2.regs_extended = 1;
        }

	mfp_config(ARRAY_AND_SIZE(mmc3_pin_config));
	/*
	 * H/W reset function is temporarily disabled by default,
	 * which can be written once by extended CSD register. (JESD84-A441)
	 * Release emmc from reset anyway here.
	 */
	if (gpio_request(emmc_reset_n, "EMMC_RESET_N")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", emmc_reset_n);
	} else {
		gpio_direction_output(emmc_reset_n, 1);
		gpio_free(emmc_reset_n);
	}
	mmp3_add_sdh(2, &mmp3_sdh_platdata_mmc2); /* eMMC */

	mfp_config(ARRAY_AND_SIZE(mmc1_pin_config));
	/*
	 * The gpio v_sd_en control the vdd of card slot.
	 * Enable it here.
	 */
	if (gpio_request(v_sd_en, "V_SD_EN")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", v_sd_en);
	} else {
		gpio_direction_output(v_sd_en, 1);
		gpio_free(v_sd_en);
	}
	mmp3_add_sdh(0, &mmp3_sdh_platdata_mmc0); /* SD/MMC */

	/* SDIO for WIFI card */
	mfp_config(ARRAY_AND_SIZE(mmc2_pin_config));
	/*
	 * The gpio wlan_wake will wake up soc from suspend.
	 */
	if (gpio_request(wlan_wake, "WLAN_WAKE"))
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", wlan_wake);
	else
		gpio_direction_input(wlan_wake);
	if (request_irq(gpio_to_irq(wlan_wake), wlan_wake_gpio_irq,
				IRQF_NO_SUSPEND | IRQF_TRIGGER_FALLING,
				"WLAN WAKEUP irq", NULL))
		printk(KERN_ERR "Request wlan wake irq failed\n");
	else
		wake_lock_init(&wlan_gpio_wakeup, WAKE_LOCK_SUSPEND, "wifi_hs_wakeups");
	mmp3_add_sdh(1, &mmp3_sdh_platdata_mmc1);
}
#endif /* CONFIG_MMC_SDHCI_PXAV3 */

#ifdef CONFIG_USB_SUPPORT
#if defined(CONFIG_USB_PXA_U2O) || defined(CONFIG_USB_EHCI_PXA_U2O)

static char *mmp3_usb_clock_name[] = {
	[0] = "U2OCLK",
};

static int pxa_usb_set_vbus(unsigned int vbus)
{
	int gpio = mfp_to_gpio(GPIO77_GPIO); /*mk2*/

	printk(KERN_INFO "%s: set %d\n", __func__, vbus);

	/* 5V power supply to external port */
	if (gpio_request(gpio, "OTG VBUS Enable")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio);
		return -1;
	}

	if (vbus)
		gpio_direction_output(gpio, 1);
	else
		gpio_direction_output(gpio, 0);

	gpio_free(gpio);

	return 0;
}

static struct mv_usb_platform_data mmp3_usb_pdata = {
	.clknum		= 1,
	.clkname	= mmp3_usb_clock_name,
	.vbus		= NULL,
	.mode		= MV_USB_MODE_OTG,
	.phy_init	= pxa_usb_phy_init,
	.phy_deinit	= pxa_usb_phy_deinit,
	.set_vbus	= pxa_usb_set_vbus,
	.otg_force_a_bus_req = 1,
};
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H_HSIC
/* USB3503 hub reset:HSIC_RST_N */
static int hsic_hub_reset(void)
{
	int rst_n = mfp_to_gpio(GPIO96_HSIC_RESET);
	if (gpio_request(rst_n, "hsic hub reset")) {
		pr_err("Failed to request hsic hub reset gpio:%d\n", rst_n);
		return -EIO;
	}
	gpio_direction_output(rst_n, 0);
	mdelay(100);
	gpio_direction_output(rst_n, 1);
	gpio_free(rst_n);
	mdelay(10);
	return 0;
}
/* USB3503 hub power:USB_VBAT */
static int hsic_hub_power(int on)
{
	static struct regulator *vbat;
	static bool enabled;
	if (!vbat) {
		vbat = regulator_get(NULL, "PMIC_LDO2");
		if (IS_ERR(vbat)) {
			pr_err("%s:Failed to get PMIC_LDO2!\n", __func__);
			vbat = NULL;
			return -EIO;
		}
	}
	if (on) {
		if (enabled)
			return 0;
		regulator_set_voltage(vbat, 3300000, 3300000);
		regulator_enable(vbat);
		enabled = 1;
	} else {
		if (!enabled)
			return 0;
		regulator_disable(vbat);
		enabled = 0;
	}
	mdelay(5);
	return 0;
}
/* HSIC1_PAD_VDDQ */
static int hsic2_pad_vdd_power(int on)
{
	static struct regulator *vdd;
	static bool enabled;
	if (!vdd) {
		vdd = regulator_get(NULL, "PMIC_LDO0");
		if (IS_ERR(vdd)) {
			pr_err("%s:Failed to get PMIC_LDO0!\n", __func__);
			vdd = NULL;
			return -EIO;
		}
	}
	if (on) {
		if (enabled)
			return 0;
		regulator_set_voltage(vdd, 1200000, 1200000);
		regulator_enable(vdd);
		enabled = 1;
	} else {
		if (!enabled)
			return 0;
		regulator_disable(vdd);
		enabled = 0;
	}
	mdelay(5);
	return 0;
}

static int mmp3_hsic_set_vbus(unsigned int on)
{
	int ret = 0;
	/* NOTE: keep pad vdd always on */
	ret = hsic2_pad_vdd_power(1);
	if (ret)
		goto out;
	ret = hsic_hub_power(on);
	if (ret)
		goto out;
	if (on)
		ret = hsic_hub_reset();
out:
	if (ret)
		pr_err("%s: failed to set vbus\n", __func__);
	return ret;
}

static char *mmp3_hsic2_clock_name[] = {
	[0] = "U2OCLK",
	[1] = "HSIC2CLK",
};

static struct mv_usb_platform_data mmp3_hsic2_pdata = {
	.clknum		= 2,
	.clkname	= mmp3_hsic2_clock_name,
	.vbus		= NULL,
	.mode		= MV_USB_MODE_HOST,
	.phy_init	= mmp3_hsic_phy_init,
	.phy_deinit	= mmp3_hsic_phy_deinit,
	.set_vbus	= mmp3_hsic_set_vbus,
	.private_init = mmp3_hsic_private_init,
};
#endif

#endif

#if defined(CONFIG_TC35876X)
static int tc358765_init(void)
{
	return 0;
}

static struct tc35876x_platform_data tc358765_data = {
	.platform_init = tc358765_init,
	.id = TC358765_CHIPID,
	.id_reg = TC358765_CHIPID_REG,
};
#endif

/* enable EETI EXC7200 touch controller */
#ifdef CONFIG_TOUCHSCREEN_EGALAX_I2C
static void exc7200_config(void)
{
	int gpio = 0,
	gpio_value = 0,
	ret = 0;
	pr_info("%s()\n", __func__);

	/* DIGITIZER_3V3_EN - GPIO153_GPIO153 */
	gpio = mfp_to_gpio(GPIO153_GPIO);
	ret = gpio_request(gpio, "digitizer_3v3");
	if(ret < 0){
		printk(KERN_ERR "%s: Fail to digitizer_3v3 (gpio %d) for \
			touch! (errno = %d)\n", __func__, gpio, ret);
		return;
	}

	gpio_value = gpio_get_value(gpio);
	if(!gpio_value){ /* digitizer_3v3_en is not powered */
		pr_info("%s: Power up touch (pull up gpio %d).\n",
			__func__, gpio);
		gpio_direction_output(gpio, 1);
	}
	else{
		pr_info("%s: gpio %d state is high (%d).\n",
			__func__, gpio, gpio_value);
	}

	mdelay(1);
	gpio_free(gpio);

	/* LCD_TOUCH_INT - GPIO101_GPIO101 */
	gpio = mfp_to_gpio(GPIO101_GPIO);
	ret = gpio_request(gpio, "lcd_touch_int");

	if(ret < 0){
		printk(KERN_ERR "%s: Fail to rquest lcd_touch_int (gpio %d)\
			for touch irq! (errno = %d)\n", __func__, gpio, ret);
		return;
	}

	pr_info("%s: config lcd_touch_int (gpio %d) input!\n", __func__, gpio);
	gpio_direction_input(gpio);
	mdelay(1);

	gpio_free(gpio);
}
#endif /* CONFIG_TOUCHSCREEN_EGALAX_I2C */

static void InitDigitizerPins(void)
{
 /* Put below pins to low                        */
 /* VDD    GPIO153, RES    GPIO89, SLP    GPIO91 , PDCT GPIO92*/
	int gpio = 0, gpio1 = 0, gpio2 = 0, gpio3 = 0,
	ret = 0;		
        gpio = mfp_to_gpio(GPIO153_GPIO);
        ret = gpio_request(gpio, "digitizer_3v3");
        if(ret < 0)
        {
                printk(KERN_ERR "%s: Fail to digitizer_3v3 (gpio %d) for touch! (errno = %d)\n", __func__, gpio, ret);
                return;
        }
        gpio_direction_output(gpio, 0);

	gpio1 = mfp_to_gpio(GPIO89_GPIO);
	if(gpio_request(gpio1, "digitizer_rst"))
		printk(KERN_DEBUG "gpio1 %d request failed\n", gpio1);
	gpio_direction_output(gpio1, 0);

	gpio2 = mfp_to_gpio(GPIO91_GPIO);
	if(gpio_request(gpio2, "digitizer_slp"))		
		printk(KERN_DEBUG "gpio2 %d request failed\n", gpio2);			
	gpio_direction_output(gpio2, 0);		

	gpio3 = mfp_to_gpio(GPIO92_GPIO);
	if(gpio_request(gpio3, "digitizer_pdet"))
		printk(KERN_DEBUG "gpio3 %d request failed\n", gpio3);
	gpio_direction_output(gpio3, 0);

	/*Power on sequence           */
	/*VDD->105m min->RES->100ms/ max 220ms->PDCT->205ms->SLP */
        gpio_direction_output(gpio, 1);
	mdelay(105);
	gpio_direction_output(gpio1, 1);
	mdelay(100);
	gpio_direction_output(gpio3, 1);
	mdelay(205);
	gpio_direction_output(gpio2, 1);		

	gpio_free(gpio);
	gpio_free(gpio1);
	gpio_free(gpio2);
	gpio_free(gpio3);
}
#ifdef CONFIG_TOUCHSCREEN_WACOM_W8001 
static struct dualmode_platform_data dualmode_data = {
	.gpio = mfp_to_gpio(GPIO92_GPIO),
};

// Enable power and configure for WACOM W8001 digitizer
static void w8001_config(void)
{	
	int gpio = 0,
	gpio_value = 0,
	ret = 0;		

	/* DIGITIZER_3V3_EN */	
	gpio = mfp_to_gpio(GPIO153_GPIO);	
	ret = gpio_request(gpio, "digitizer_3v3");
	if(ret < 0)
	{
		printk(KERN_ERR "%s: Fail to digitizer_3v3 (gpio %d) for touch! (errno = %d)\n", __func__, gpio, ret);
		return;
	}

   	gpio_value = gpio_get_value(gpio);
   	if(!gpio_value) /* digitizer_3v3_en is not powered */
   	{
   		pr_info("%s: Power up touch (pull up gpio %d).\n", __func__, gpio);	
   		gpio_direction_output(gpio, 1);
	}
	else
	{
		pr_info("%s: gpio %d state is high (%d).\n", __func__, gpio, gpio_value);
	}
	
	mdelay(1);	
	gpio_free(gpio);

	/* DIGITIZER_FWE */	
	gpio = mfp_to_gpio(GPIO90_GPIO);//T-Lite2, SB, MMP3 B0	
	if(gpio_request(gpio, "digitizer_fwe"))		
		printk(KERN_DEBUG "gpio %d request failed\n", gpio);			

	pr_info("%s: pull down gpio %d.\n", __func__, gpio);	
	gpio_direction_output(gpio, 0);		

	mdelay(1);	
	gpio_free(gpio);		

	/* DIGITIZER_SLP */	
	//T-Lite2, SB, MMP3 B0
	gpio = mfp_to_gpio(GPIO91_GPIO);

	if(gpio_request(gpio, "digitizer_slp"))		
		printk(KERN_DEBUG "gpio %d request failed\n", gpio);			

	//pr_info("%s: pull down gpio %d.\n", __func__, gpio);
	pr_info("%s: pull down gpio %d (SLP).\r\n", __func__, gpio); //T-Lite, digitizer_debug
	gpio_direction_output(gpio, 0);		

	mdelay(1);	
	gpio_free(gpio);


	gpio = mfp_to_gpio(GPIO92_GPIO);

	if(gpio_request(gpio, "digitizer_pdet"))
		printk(KERN_DEBUG "gpio %d request failed\n", gpio);

	pr_info("%s: pull down gpio %d (PDET).\r\n", __func__, gpio); //T-Lite, digitizer_debug
	gpio_direction_input(gpio);

	mdelay(1);
	gpio_free(gpio);

	
	//T-Lite2, reset the device once, SB, MMP3 B0
	/* DIGITIZER_RST */
	gpio = mfp_to_gpio(GPIO89_GPIO);
	if(gpio_request(gpio, "digitizer_rst"))
		printk(KERN_DEBUG "gpio %d request failed\n", gpio);

	pr_info("%s: pull down gpio %d and wait for 50 ms.\n", __func__, gpio);                              
	gpio_direction_output(gpio, 0);
	mdelay(50);
                                        
	pr_info("%s: pull up gpio %d.\n", __func__, gpio);
	gpio_direction_output(gpio, 1); 
                                                        
	mdelay(1);
	gpio_free(gpio);
}
#endif //CONFIG_TOUCHSCREEN_WACOM_W8001

#if defined(CONFIG_SENSORS_CM3217A) //T-Lite2

int cm3217a_set_power(int on){
#if 0
        int gpio = mfp_to_gpio(GPIO16_GPIO);

        if (gpio_request(gpio, "cm3217a power")) {
                printk(KERN_INFO "gpio %d request failed\n", gpio);
                return -1;
        }

        if(on){
                //v_ldo8_set_power(1);
                gpio_direction_output(gpio, 1);
        }
        else
                gpio_direction_output(gpio, 0);

        gpio_free(gpio);
#endif
        return 0;
}


static struct cm3217a_platform_data cm3217a_pdata = {
        .set_power      = cm3217a_set_power,
};
#endif //CONFIG_SENSORS_CM3217A //T-Lite2

/* WM8994 external power supply */
static struct regulator_consumer_supply wm8994_power1_supplies[] = {
	REGULATOR_SUPPLY("DBVDD", NULL),
	REGULATOR_SUPPLY("AVDD2", NULL),
	REGULATOR_SUPPLY("CPVDD", NULL),
};

static struct regulator_consumer_supply wm8994_power2_supplies[] = {
	REGULATOR_SUPPLY("SPKVDD1", NULL),
	REGULATOR_SUPPLY("SPKVDD2", NULL),
};

static struct regulator_init_data wm8994_fixed_power1_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(wm8994_power1_supplies),
	.consumer_supplies	= wm8994_power1_supplies,
};

static struct regulator_init_data wm8994_fixed_power2_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(wm8994_power2_supplies),
	.consumer_supplies	= wm8994_power2_supplies,
};

static struct fixed_voltage_config wm8994_fixed_power1_config = {
	.supply_name = "V1P8_8994",
	.microvolts = 1800000,
	.gpio = -EINVAL,
	.init_data = &wm8994_fixed_power1_init_data,
};

static struct fixed_voltage_config wm8994_fixed_power2_config = {
	.supply_name = "5V_S0_8994",
	.microvolts = 5000000,
	.gpio = -EINVAL,
	.init_data = &wm8994_fixed_power2_init_data,
};

static struct platform_device wm8994_fixed_power1 = {
	.name = "reg-fixed-voltage",
	.id = 0,
	.dev = {
		.platform_data = &wm8994_fixed_power1_config,
	},
};

static struct platform_device wm8994_fixed_power2 = {
	.name = "reg-fixed-voltage",
	.id = 1,
	.dev = {
		.platform_data = &wm8994_fixed_power2_config,
	},
};

static struct platform_device *mk2_fixed_rdev[] __initdata = {
	&wm8994_fixed_power1,
	&wm8994_fixed_power2,
};

static void mk2_fixed_regulator_init(void)
{
	platform_add_devices(mk2_fixed_rdev, ARRAY_SIZE(mk2_fixed_rdev));
}

/* WM8994: LDO1 => AVDD1(3.0V); LDO2 => DCVDD(1.0V) */
static struct regulator_consumer_supply wm8994_avdd1_supply =
	REGULATOR_SUPPLY("AVDD1", NULL);

static struct regulator_consumer_supply wm8994_dcvdd_supply =
	REGULATOR_SUPPLY("DCVDD", NULL);

static struct regulator_init_data wm8994_ldo1_data = {
	.constraints = {
		.name = "AVDD1_3.0V",
		.min_uV = 2400000,
		.max_uV = 3100000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &wm8994_avdd1_supply,
};

static struct regulator_init_data wm8994_ldo2_data = {
	.constraints = {
		.name = "DCVDD_1.0V",
		.min_uV = 900000,
		.max_uV = 1200000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = 1,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &wm8994_dcvdd_supply,
};

static struct wm8994_pdata mk2_wm8994_pdata = {
	/* GPIO1: Codec IRQ */
	.gpio_defaults[0] = 0x0003,
	/* AIF2 */
	.gpio_defaults[2] = 0x8100,
	.gpio_defaults[3] = 0x8100,
	.gpio_defaults[4] = 0x8100,
	.gpio_defaults[5] = 0x8100,
	.gpio_defaults[6] = 0x0100,
	/* AIF3 */
	.gpio_defaults[7] = 0x8100,
	.gpio_defaults[8] = 0x0100,
	.gpio_defaults[9] = 0x8100,
	.gpio_defaults[10] = 0x8100,
	.ldo[0]	= {
		/* FIXME: keep LDO output always on */
		/* .enable = mfp_to_gpio(GPIO06_WM8994_LDOEN), */
		.init_data = &wm8994_ldo1_data
	},
	.ldo[1]	= {
		.init_data = &wm8994_ldo2_data
	},
};

static struct switch_headset_pdata headset_pdata = {
	.name = "h2w",
	.gpio = mfp_to_gpio(GPIO130_GPIO),
};

static struct platform_device headset_switch_device = {
	.name = "headset",
	.id = 0,
	.dev = {
		.platform_data = &headset_pdata,
	},
};

static int hdmi_power(int on)
{
	int hdmi_pwr_en = mfp_to_gpio(GPIO160_GPIO);

	if (gpio_request(hdmi_pwr_en, "hdmi_pwr_en")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d.\n", hdmi_pwr_en);
		return -1;
	}
	if (on){
		gpio_direction_output(hdmi_pwr_en, 1);

	}
	else{
		gpio_direction_output(hdmi_pwr_en, 0);

	}
	gpio_free(hdmi_pwr_en);
	mdelay(10);
	return 0;
}

#if defined(CONFIG_SENSORS_STM8T143) //T-Lite2
static struct gpio_switch_platform_data stm8t143_device_data = {
	.name = "stm8t143",
	.gpio = NULL,
	.name_on = NULL,
	.name_off = NULL,
	.state_on = NULL,
	.state_off = NULL,
};

static struct platform_device stm8t143_device = {
	.name            = "stm8t143",
	.id              = 0,
	.dev             = {
		.platform_data = &stm8t143_device_data,
	},
};

static void __init brownstone_init_stm8t143(void)
{
	platform_device_register(&stm8t143_device);
}
#endif //defined(CONFIG_SENSORS_STM8T143) //T-Lite2



#ifdef CONFIG_UIO_HDMI
static struct uio_hdmi_platform_data mmp3_hdmi_info __initdata = {
	.sspa_reg_base = 0xD42A0C00,
	/* Fix me: gpio 59 lpm pull ? */
	.gpio = mfp_to_gpio(GPIO59_HDMI_DET),
	.edid_bus_num = 6,
	.hdmi_v5p_power = &hdmi_power,
	.hpd_val = 0,
};
#endif

static struct i2c_board_info mk2_twsi3_info[] = {
#if defined(CONFIG_TC35876X)
	{
		.type		= "tc35876x",
		.addr		= 0x0f,
		.platform_data	= &tc358765_data,
	},
#endif
	{
		.type = "wm8994",
		.addr = 0x1a,
		.platform_data = &mk2_wm8994_pdata,
	},
};

static struct i2c_board_info mk2_twsi5_info[] = {
/* enable EETI EXC7200 touch controller, 20110812 */
#ifdef CONFIG_TOUCHSCREEN_EGALAX_I2C
	{
		.type           = "egalax_i2c",
		.addr           = 0x04,
		.irq            = gpio_to_irq(101),
#ifdef CONFIG_TOUCHSCREEN_WACOM_W8001
		.platform_data = &dualmode_data,
#endif

	},
#endif /* CONFIG_TOUCHSCREEN_EGALAX_I2C */

#if defined(CONFIG_SENSORS_CM3217A) //T-Lite
        {
                .type           = "cm3217a_als_msb",
                .addr           = 0x10,
                .platform_data  = &cm3217a_pdata,
        },
        {
                .type           = "cm3217a_als_lsb",
                .addr           = 0x11,
                .platform_data  = &cm3217a_pdata,
        },
#endif //CONFIG_SENSORS_CM3217A //T-Lite
};

static struct i2c_board_info mk2_twsi6_info[] = {
};
#if 0
static int gsensor_power_en(int onoff)
{
	int gsen_pwr_en = mfp_to_gpio(GPIO87_GPIO);
	/* GPIO power enable */
	if (gpio_request(gsen_pwr_en, "GSENSOR Enable")) {
		printk(KERN_INFO "gpio %d request failed\n", gsen_pwr_en);
		return -1;
	}
	gpio_direction_output(gsen_pwr_en, onoff);

	printk( "G-sensor %s\r\n", onoff?"turn ON":"turn OFF");
	gpio_free(gsen_pwr_en);
	return 0;
}
#endif

static int wm8994_ldo_enable(void)
{
	int ldo_en = mfp_to_gpio(GPIO06_WM8994_LDOEN);
	pr_info("%s: Enable WM8994 LDO output\n", __func__);
	if (gpio_request(ldo_en, "WM8994_LDOEN")) {
		pr_err("Failed to request gpio%d!\n", ldo_en);
		return -EIO;
	}
	gpio_direction_output(ldo_en, 1);
	gpio_free(ldo_en);
	mdelay(50);
	return 0;
}

extern int __raw_i2c_bus_reset(u8 bus_num);
extern int __raw_i2c_write_reg(u8 bus_num, u8 addr, u8 reg, u8 val);
extern int __raw_i2c_read_reg(u8 bus_num, u8 addr, u8 reg, u8 *buf, int len);
#define MAX77601_RTC_RETRY_LIMIT 10
static int max77601_rtc_raw_write(int reg, u8 data)
{
	int retry = 0;
	u8 tmp;

	__raw_i2c_write_reg(1, 0x48, reg, data);

	__raw_i2c_write_reg(1, 0x48, MAX77601_RTCUPDATE0, MAX77601_UDF);
	while (retry++ < MAX77601_RTC_RETRY_LIMIT) {
		mdelay(50);
		__raw_i2c_read_reg(1, 0x48, MAX77601_RTCUPDATE1, &tmp, 1);
		if (tmp & MAX77601_UDF)
			break;
	}
	if (retry >= MAX77601_RTC_RETRY_LIMIT)
		return -1;
	else
		return 0;
}

static int max77601_rtc_raw_read(int reg, u8 *buf)
{
	int retry = 0;
	u8 tmp;

	__raw_i2c_write_reg(1, 0x48, MAX77601_RTCUPDATE0, MAX77601_RBUDR);
	while (retry++ < MAX77601_RTC_RETRY_LIMIT) {
		mdelay(50);
		__raw_i2c_read_reg(1, 0x48, MAX77601_RTCUPDATE1, &tmp, 1);
		if (tmp & MAX77601_RBUDF)
			break;
	}
	if (retry >= MAX77601_RTC_RETRY_LIMIT)
		return -1;
	__raw_i2c_read_reg(1, 0x48, reg, buf, 1);
	return 0;
}

static int mk2_board_reset(char mode, const char *cmd)
{
	u8 data, dataKBC;
	int i, rc;

	pr_err("enter mk2_board_reset\n");

	mdelay(500);
	/* Reset TWSI1 unit firstly */
	__raw_i2c_bus_reset(1);

	/* set recovery bit */
	if (cmd && !strcmp(cmd, "recovery")) {
		/* try 40 times to set recovery flag in case of failure */
		for (i = 0; i < 40; i++) {
			if (max77601_rtc_raw_write(MAX77601_RTCSECA2, 0x01)) {
				pr_err("Recovery flag write failed!\n");
				continue;
			}
			if (max77601_rtc_raw_read(MAX77601_RTCSECA2, &data)) {
				pr_err("Recovery flag read failed!\n");
				continue;
			}
			if (data == 0x01)
				break;
		}
		if (i == 40)
			pr_err("Recovery flag set failed!\n");
	}
	mdelay(500);
	/* 0. Tell KBC reboot sequence is starting */
	for (i = 0; i < 10; i++) {
		dataKBC = 0x1;
		rc = __raw_i2c_write_reg(1, 0x15, 0x44, dataKBC);
		if(rc < 0)
		{
		pr_err("44 command set failed!\n");
		mdelay(10);
		}
		else
		break;
	}
	/* 1. Enable SW reset wake up */
	__raw_i2c_read_reg(1, 0x1c, MAX77601_ONOFFCNFG2, &data, 1);
	data |= MAX77601_SFT_RST_WK;
	__raw_i2c_write_reg(1, 0x1c, MAX77601_ONOFFCNFG2, data);
	/* 2. Issue SW reset */
	__raw_i2c_read_reg(1, 0x1c, MAX77601_ONOFFCNFG1, &data, 1);
	data |= MAX77601_SFT_RST;
	__raw_i2c_write_reg(1, 0x1c, MAX77601_ONOFFCNFG1, data);

	mdelay(200);
	return 1;
}

static void mk2_power_off(void)
{
	u8 data;

	mdelay(200);
	printk(KERN_ERR "poweroff enter\n");

#if 1
	/*disable ldo2*/
	__raw_i2c_write_reg(1, 0x1c, MAX77601_VREG_LDO2, 0);

printk(KERN_ERR " =====> %d %s %s\n", __LINE__, __func__, __FILE__);

	__raw_i2c_read_reg(1, 0x15, 0x40, &data, 1);
 	data &= ~(1<<4);
 	__raw_i2c_write_reg(1, 0x15, 0x40, data);

#endif
	max77601_system_poweroff();
	mdelay(200);
}
static unsigned long mk2_pin_config_off[] = {
 	GPIO10_GPIO | MFP_LPM_DRIVE_LOW,
 	GPIO11_GPIO | MFP_LPM_DRIVE_LOW,
};
static unsigned long mk2_pin_config_on[] = {
 	GPIO10_GPIO | MFP_PULL_NONE,
 	GPIO11_GPIO | MFP_PULL_NONE,
};

void 	InitChargeLedPins(void)
{
	int gpio;
	gpio = mfp_to_gpio(GPIO10_GPIO);

	if(gpio_request(gpio, "GPIO10"))
		printk(KERN_DEBUG "gpio %d request failed\n", gpio);

	gpio_direction_input(gpio);

	mdelay(1);
	gpio_free(gpio);
	
	gpio = mfp_to_gpio(GPIO11_GPIO);

	if(gpio_request(gpio, "GPIO11"))
		printk(KERN_DEBUG "gpio %d request failed\n", gpio);

	gpio_direction_input(gpio);

	mdelay(1);
	gpio_free(gpio);
	
}

static int LDO1_CFG,LDO3_CFG,LDO5_CFG;
void gpio_suspend_off(unsigned long *mfp_cfgs, int num)
{
	int i,gpio;
	for (i = 0; i < num; i++, mfp_cfgs++) {
	gpio = mfp_to_gpio(*mfp_cfgs);
	if(gpio_request(gpio, "gpio_off"))		
		printk(KERN_DEBUG "gpio %d request failed\n", gpio);			
	gpio_direction_output(gpio, 0);
	gpio_free(gpio);	
	}
}
void gpio_suspend_enter(void)
{
	int gpio_hub_en,gpio_sd_en,gpio_lcd_rst;
	gpio_hub_en = mfp_to_gpio(GPIO78_GPIO);
	if(gpio_request(gpio_hub_en, "gpio_hub_en"))		
		printk(KERN_DEBUG "gpio_hub_en %d request failed\n", gpio_hub_en);			
	gpio_direction_output(gpio_hub_en, 0);
	gpio_free(gpio_hub_en);
	gpio_sd_en = mfp_to_gpio(GPIO138_GPIO);
	if(gpio_request(gpio_sd_en, "gpio_sd_en"))		
		printk(KERN_DEBUG "gpio_sd_en %d request failed\n", gpio_sd_en);			
	gpio_direction_output(gpio_sd_en, 0);
	gpio_free(gpio_sd_en);	
#if 1	
	gpio_lcd_rst = mfp_to_gpio(GPIO128_LCD_RST);
	if(gpio_request(gpio_lcd_rst, "gpio_lcd_rst"))		
		printk(KERN_DEBUG "gpio_lcd_rst %d request failed\n", gpio_lcd_rst);			
	gpio_direction_output(gpio_lcd_rst, 0);
	gpio_free(gpio_lcd_rst);	
#endif
  mfp_config(ARRAY_AND_SIZE(mk2_pin_config_off));
	gpio_suspend_off(ARRAY_AND_SIZE(mk2_pin_config_off));	
}

void gpio_suspend_finish(void)
{
	int gpio_hub_en,gpio_sd_en,gpio_lcd_rst;
	gpio_hub_en = mfp_to_gpio(GPIO78_GPIO);
	if(gpio_request(gpio_hub_en, "gpio_hub_en"))		
		printk(KERN_DEBUG "gpio_hub_en %d request failed\n", gpio_hub_en);			
	gpio_direction_output(gpio_hub_en, 1);
	gpio_free(gpio_hub_en);
	gpio_sd_en = mfp_to_gpio(GPIO138_GPIO);
	if(gpio_request(gpio_sd_en, "gpio_sd_en"))		
		printk(KERN_DEBUG "gpio_sd_en %d request failed\n", gpio_sd_en);			
	gpio_direction_output(gpio_sd_en, 1);
	gpio_free(gpio_sd_en);	
#if 1		
	gpio_lcd_rst = mfp_to_gpio(GPIO128_LCD_RST);
	if(gpio_request(gpio_lcd_rst, "gpio_lcd_rst"))		
		printk(KERN_DEBUG "gpio_lcd_rst %d request failed\n", gpio_lcd_rst);			
	gpio_direction_output(gpio_lcd_rst, 1);
	gpio_free(gpio_lcd_rst);
#endif
	mfp_config(ARRAY_AND_SIZE(mk2_pin_config_on));
	InitChargeLedPins();
}

void mk2_suspend_enter(void)
{
		u8 data;
	gpio_suspend_enter();
	/* Reset TWSI1 unit firstly */
#if 1
  //pr_err("LDO1 off!\n");
	/*disable ldo1*/
//	__raw_i2c_read_reg(1, 0x3c, MAX77601_VREG_LDO1, &LDO1_CFG, 1);
//	__raw_i2c_read_reg(1, 0x1c, MAX77601_VREG_LDO1, &LDO1_CFG, 1);
LDO1_CFG=0xD0;
//	pr_err("LDO1_CFG=0x%x!\n",LDO1_CFG);
	__raw_i2c_write_reg(1, 0x3c, MAX77601_VREG_LDO1, 0);
	/*disable ldo1*/
	__raw_i2c_write_reg(1, 0x1c, MAX77601_VREG_LDO1, 0);

  //pr_err("LDO3 off!\n");
	__raw_i2c_read_reg(1, 0x3c, MAX77601_VREG_LDO3, &LDO3_CFG, 1);
	__raw_i2c_read_reg(1, 0x1c, MAX77601_VREG_LDO3, &LDO3_CFG, 1);
	//pr_err("LDO3_CFG=0x%x!\n",LDO3_CFG);
	
	/*disable ldo1*/
	__raw_i2c_write_reg(1, 0x3c, MAX77601_VREG_LDO3, 0);
	/*disable ldo1*/
	__raw_i2c_write_reg(1, 0x1c, MAX77601_VREG_LDO3, 0);

  //pr_err("LDO5 off!\n");
	__raw_i2c_read_reg(1, 0x3c, MAX77601_VREG_LDO5, &LDO5_CFG, 1);
	__raw_i2c_read_reg(1, 0x1c, MAX77601_VREG_LDO5, &LDO5_CFG, 1);
	//pr_err("LDO5_CFG=0x%x!\n",LDO5_CFG);
	
	/*disable ldo1*/
	__raw_i2c_write_reg(1, 0x3c, MAX77601_VREG_LDO5, 0);
	/*disable ldo1*/
	__raw_i2c_write_reg(1, 0x1c, MAX77601_VREG_LDO5, 0);

#endif


	mdelay(50);
}

void mk2_suspend_finish(void)
{
			u8 data;
	
	/* Reset TWSI1 unit firstly */
#if 1
  //pr_err("LDO1 on!\n");
	/*disable ldo1*/
	__raw_i2c_write_reg(1, 0x3c, MAX77601_VREG_LDO1, LDO1_CFG);
	/*disable ldo1*/
	__raw_i2c_write_reg(1, 0x1c, MAX77601_VREG_LDO1, LDO1_CFG);

  //pr_err("LDO3 on!\n");
		
	/*disable ldo1*/
	__raw_i2c_write_reg(1, 0x3c, MAX77601_VREG_LDO3, LDO3_CFG);
	/*disable ldo1*/
	__raw_i2c_write_reg(1, 0x1c, MAX77601_VREG_LDO3, LDO3_CFG);

	//pr_err("LDO5 on!\n");
		
	/*disable ldo1*/
	__raw_i2c_write_reg(1, 0x3c, MAX77601_VREG_LDO5, LDO5_CFG);
	/*disable ldo1*/
	__raw_i2c_write_reg(1, 0x1c, MAX77601_VREG_LDO5, LDO5_CFG);

#endif

  
	mdelay(10);
	gpio_suspend_finish();
}


static void gps_1v8_power(unsigned int on)
{
	static struct regulator *pmic_1p8v_gps;//LDO8 for 1.8V
	static int ldoenable;	
	if (!pmic_1p8v_gps) 
	{
        
		pmic_1p8v_gps = regulator_get(NULL, "PMIC_LDO8"); /*Fix Me: conflict with schematics*/
		if (IS_ERR(pmic_1p8v_gps)) {
			pmic_1p8v_gps = NULL;
			return;
		}
	}
	//pmic_1p8v_gps is not NULL;
	if (on) 
	{
		printk("GPS 1V8 on\r\n");
		regulator_set_voltage(pmic_1p8v_gps, 1800000, 1800000);
		if(!ldoenable)
		{
			regulator_enable(pmic_1p8v_gps);
			ldoenable = 1;
		}
	}


	if(!on) {
		printk("GPS 1V8 off\r\n");
		if(ldoenable)
		{		
			regulator_disable(pmic_1p8v_gps);
			regulator_put(pmic_1p8v_gps);
			pmic_1p8v_gps = NULL;
			ldoenable = 0;
		} 
       }

}
static void gps_power_on(void)
{
	int rf_gps_pwr_en;
	
	pr_info("%s()\n", __func__);

	gps_1v8_power(1);	
        
	// GPS RF power on
	rf_gps_pwr_en = mfp_to_gpio(GPIO12_GPIO); /* RF_GPS_PWR_EN */
	if(gpio_request(rf_gps_pwr_en, "rf_gps_pwr_en")) 
	{
	   pr_err("Failed to request rf_gps_pwr_en!\n");
	   return;
	}
	mdelay(50);
	gpio_direction_output(rf_gps_pwr_en, 1);
	gpio_free(rf_gps_pwr_en);
        
	pr_info("%s: power up u-blox G6010 GPS & RF.\n", __func__);
}

static void gps_power_off(void)
{
	int rf_gps_pwr_en;
	pr_info("%s()\n", __func__);

	//gps_1v8_power(0);	
	// Pull Up RF_GPS_PWR_EN
	rf_gps_pwr_en = mfp_to_gpio(GPIO12_GPIO);
	if(gpio_request(rf_gps_pwr_en, "rf_gps_pwr_en"))
	{ 
	   pr_err("Failed to request rf_gps_pwr_en!\n");
	   return;
	
	}	
	mdelay(50);
	gps_1v8_power(0);
	gpio_direction_output(rf_gps_pwr_en, 0);
	gpio_free(rf_gps_pwr_en);
	
	pr_info("%s: power off u-blox G6010 GPS & RF.\n", __func__);
}

static void gps_reset(int flag)
{
	pr_info("%s()\n", __func__);
	gps_power_on();
	mdelay(1);
	gps_power_off();
}

static void gps_on_off(int flag)
{
	pr_info("%s(%d)\n", __func__, flag);
}

#define UART_DEV "/dev/ttyS1" /* UART2 */
static void gps_uart_write(char* buf)
{
	mm_segment_t old_fs;
	struct file *fp = NULL;
	char data[12] = {0};
	//int i = 0; // Debug
	
	// Check input arguments
	//pr_info("%s: buf addr = 0x%p.\r\n", __func__, buf);
	if(!buf)
	{
		pr_info("%s: Invalid input buffer!\r\n", __func__);
		return;
	}
	memcpy(data, buf, sizeof(data));
	
	// Open UART port
	fp = filp_open(UART_DEV, O_RDWR, 0);
	if(!fp)
	{
		pr_info("%s: Fail to open %s!\r\n", __func__, UART_DEV);
		return;
	}
	
	// Access the file
	old_fs = get_fs();
	set_fs(get_ds());
	// Debug
	/*
	pr_info("%s: data[%d]: ", __func__, sizeof(data));
	for(i = 0; i < sizeof(data); i++)
		pr_info("[%02d] 0x%x\r\n", i, data[i]);
	*/
	fp->f_op->write(fp, data, sizeof(data), &fp->f_pos);
	set_fs(old_fs);
	
	// Close UART port
	filp_close(fp, NULL);
	
	pr_info("%s: Success to write command to %s.\r\n", __func__, UART_DEV);
	return;
}

static void gps_cold_start(void)
{
	//char cmd[] = {0xb5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xff, 0x07, 0x02, 0x00, 0x16, 0x79}; /* U-Blox Cold Start Command, it arises a problem in runing again! */
	char cmd[] = {0xb5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xff, 0xff, 0x02, 0x00, 0x0e, 0x61}; /* U-Blox Cold Start Command 2 */
	pr_info("%s()\n", __func__);
	//pr_info("%s: cmd addr = 0x%p.\n", __func__, cmd);
	
	// Write command to uart4
	gps_uart_write(cmd);
	return;
}

static void gps_warm_start(void)
{
	char cmd[] = {0xb5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x01, 0x00, 0x02, 0x00, 0x11, 0x6c}; /* U-Blox Warm Start Command */
	pr_info("%s()\n", __func__);
	//pr_info("%s: cmd addr = 0x%p.\n", __func__, cmd);
	
	// Write command to uart2
	gps_uart_write(cmd);
	return;
}

static void gps_hot_start(void)
{
	char cmd[] = {0xb5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x68}; /* U-Blox Warm Start Command */
	pr_info("%s()\n", __func__);
	//pr_info("%s: cmd addr = 0x%p.\n", __func__, cmd);
	
	// Write command to uart2
	gps_uart_write(cmd);
	return;
}


static char sirf_status[9] = "off";
static ssize_t sirf_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = strlen(sirf_status);

	sprintf(page, "%s\n", sirf_status);
	return len + 1;
}

static ssize_t sirf_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	int flag, ret;
	char buffer[7];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "off", 3) == 0) {
		strcpy(sirf_status, "off");
		gps_power_off();
	} else if (strncmp(messages, "on", 2) == 0) {
		strcpy(sirf_status, "on");
		gps_power_on();
	} else if (strncmp(messages, "reset", 5) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_reset(flag);
	} else if (strncmp(messages, "sirfon", 5) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_on_off(flag);
	} else if (strncmp(messages, "coldstart", 9) == 0) { /* Cold Start */
		strcpy(sirf_status, "coldstart");
		gps_cold_start();
	} else if (strncmp(messages, "warmstart", 9) == 0) { /* Warm Start */
		strcpy(sirf_status, "warmstart");
		gps_warm_start();
	} else if (strncmp(messages, "hotstart", 8) == 0) { /* Hot Start */
		strcpy(sirf_status, "hotstart");
		gps_hot_start();
	} else {
		printk("usage: echo {on/off} > /proc/driver/sirf\n");
	}

	return len;
}

static void create_sirf_proc_file(void)
{
	struct proc_dir_entry *sirf_proc_file =
		create_proc_entry("driver/sirf", 0644, NULL);

	if (sirf_proc_file) {
		sirf_proc_file->read_proc = sirf_read_proc;
		sirf_proc_file->write_proc = (write_proc_t  *)sirf_write_proc;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

extern const char* getBasebandType();
int wwan_enable(int on)
{
	int gpio129=mfp_to_gpio(GPIO129_GPIO);  //VDD_3G_3v3_EN
	int BB_ENABLE = mfp_to_gpio(GPIO94_GPIO); // BB_ENABLE
    int BB_RST_N = mfp_to_gpio(GPIO95_GPIO); // BB_RST_N
	int BB_WAKE = mfp_to_gpio(GPIO93_GPIO);
	
	if (gpio_request(BB_WAKE, "bb wake")) {
		printk("gpio 93 (BB_WAKE) request fail");
		return -1;
	}    
	if (gpio_request(gpio129,"3g enable")){
		printk("gpio %d reuqest failed\n",gpio129);
		return -1;
	}
	if (gpio_request(BB_ENABLE, "bb enable")) {
		printk("gpio 94 (BB_ENABLE) request fail");
		return -1;
	}
	if (gpio_request(BB_RST_N, "bb reset")) {
		printk("gpio 95 (BB_RST_N) request fail");
		return -1;
	}

	if (on) {
		printk("WWAN power on!!!\n");
		/* I remove this part because I move these code to uboot!!
		gpio_direction_output(BB_ENABLE, 0);
		gpio_direction_output(gpio,0);
		mdelay(300); // make sure they are originally low
		*/
		gpio_direction_input(BB_WAKE);
		gpio_direction_output(gpio129,1);
		//mdelay(10); // MOS switch act very fast!
		gpio_direction_output(BB_ENABLE, 1);
		gpio_direction_output(BB_RST_N, 1);
	}else{
		printk("WWAN power off, delay 5 second for radio de-registration!!!\n");
		gpio_direction_output(BB_ENABLE, 0);
		if (strstr(getBasebandType(), "MC7750"))
		    mdelay(5000);
		gpio_direction_output(BB_RST_N, 0);
		gpio_direction_output(gpio129, 0);
		gpio_direction_output(BB_WAKE, 0);
	}

	gpio_free(BB_WAKE);
	gpio_free(BB_RST_N);		
	gpio_free(BB_ENABLE);
	gpio_free(gpio129);
	return 0;
}
/*
 * For not make boot time longer, I use delayed work 
 */
static struct workqueue_struct *my_wq;
static struct delayed_work my_ws;

static void my_wq_function( struct work_struct *work)
{
  wwan_enable(1);
  return;
}

static int wwan_init()
{
	/* place holder */
	int ret = -1;

	my_wq = create_workqueue("my_queue");
	if (my_wq) {
		/* Queue some work (item 1) */
		INIT_DELAYED_WORK(&my_ws, my_wq_function );
		printk("delay work time is %lu", cpu_clock(smp_processor_id()));
		ret = queue_delayed_work( my_wq, &my_ws , msecs_to_jiffies(3000) /* Jiffies */);

	}
	return ret;//wwan_enable(1);
}

/*added by stwang for DM*/
static int wwan_status = 1;
static ssize_t wwan_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	return sprintf(page, "%d\n", wwan_status);
}

static ssize_t wwan_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[256];
    unsigned int wwan_cmd;
	//unsigned char tmp = INITIAL_LSB_DATA;	
	printk(KERN_ERR "%s: n\n", __func__);
	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
	{
		printk(KERN_ERR "%s error: %s \n", __func__,messages);
		return -EFAULT;
	}
	msleep(5);
	wwan_cmd = (unsigned int) simple_strtoul(messages, NULL, 10);	
	
	switch(wwan_cmd){
	case 0:
		wwan_status=0;
		wwan_enable(0);
		break;
	case 1:
		wwan_status=1;
		wwan_enable(1);
		break;
	
	 default:
		pr_info("%s  default  \n", __func__);
		break;
	}
	return len;
}

static void create_wwan_proc_file(void)
{
	struct proc_dir_entry *threeg_proc_file = create_proc_entry("driver/threeg", 0666, NULL);
	if (threeg_proc_file) {
		printk(KERN_INFO "%s proc file create successful\n", __func__);
	} else {
		printk(KERN_ERR "proc file create failed!\n");
	}
	threeg_proc_file->read_proc = (read_proc_t *)wwan_read_proc;
	threeg_proc_file->write_proc = (write_proc_t  *)wwan_write_proc;

}
/*end by stwang for DM*/

static void mk2_backup_battery_init(void)
{
	u8 data = 0x75;
	/* Reset TWSI1 unit firstly */
	__raw_i2c_bus_reset(1);
	__raw_i2c_write_reg(1, 0x1c, MAX77601_CNFGBBC, data);
	mdelay(100);
	__raw_i2c_read_reg(1, 0x1c, MAX77601_CNFGBBC, &data, 1);
	printk(KERN_ERR " ###########CNFGBBC = %x\n", data);	
}

#define DMCU_SDRAM_TIMING1 0x80
#define DMCU_SDRAM_TIMING2 0x84
#define DMCU_SDRAM_TIMING3 0x88
#define DMCU_SDRAM_TIMING4 0x8c
#define DMCU_SDRAM_TIMING5 0x90
#define DMCU_SDRAM_TIMING6 0x94
#define DMCU_SDRAM_TIMING7 0x98
#define DMCU_SDRAM_TIMING8 0x9c
#define DMCU_PHY_CTRL3 0x220
#define DMCU_PHY_DQ_BYTE_SEL 0x300
#define DMCU_PHY_DLL_CTRL_BYTE1 0x304
#define DMCU_PHY_DLL_WL_SEL 0x380
#define DMCU_PHY_DLL_WL_CTRL0 0x384
#define ALLBITS (0xFFFFFFFF)

static struct dmc_regtable_entry edb8132b3ma_2x200mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x488D0065},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x524301A5},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0x201C1C12},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x3012804F},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x0A0900A1},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00005201},
	{DMCU_SDRAM_TIMING8, ALLBITS, 0x00000033},
};

static struct dmc_regtable_entry edb8132b3ma_2x266mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x48910065},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x63540235},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0x20262612},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x3012A868},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x0A0C00E1},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00005201},
	{DMCU_SDRAM_TIMING8, ALLBITS, 0x00000044},
};



static struct dmc_regtable_entry edb8132b3ma_2x400mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x4CDA0065},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x94860345},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0x2038381B},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x3012FC9D},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x0A110141},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00005201},
	{DMCU_SDRAM_TIMING8, ALLBITS, 0x00000066},
};

/*
 * drc is a mux of ddr source clk.
 * 0x0 = PLL1 div by 2
 * 0x1 = PLL1
 * 0x2 = PLL2
  *0x3 = PLL1 CLKOUTP
 */

static struct dmc_timing_entry edb8132b3ma_table[] = {

	{
		.dsrc = 0,
		.mode4x = 0,
		.pre_d = 0,
		.cas = 0x0008800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, edb8132b3ma_2x200mhz),
		},
	},

	{
		.dsrc = 3,
		.mode4x = 0,
		.pre_d = 1,
		.cas = 0x0008800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, edb8132b3ma_2x266mhz),
		},
	},

	{
		.dsrc = 1,
		.mode4x = 0,
		.pre_d = 0,
		.cas = 0x0008800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, edb8132b3ma_2x400mhz),
		},
	},

/* FIXME remove 533Mhz since it hangs. */
/*
	{
		.dsrc = 3,
		.mode4x = 0,
		.pre_d = 0,
		.cas = 0x0008800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, edb8132b3ma_2x533mhz),
		},
	},
*/
};

static void mk2_update_ddr_info(void)
{
	mmp3_pm_update_dram_timing_table(ARRAY_SIZE(edb8132b3ma_table),
						edb8132b3ma_table);
}

static void fsic3_clk_disable(void)
{
	int tmp;
	tmp = readl(APMU_FSIC3_CLK_RES_CTRL);
	tmp &= ~0x1F;
	writel(tmp, APMU_FSIC3_CLK_RES_CTRL);
}

static void __init mk2_init(void)
{
	extern int (*board_reset)(char mode, const char *cmd);
	board_reset = mk2_board_reset;
	pm_power_off = mk2_power_off;
	mfp_config(ARRAY_AND_SIZE(mk2_pin_config));

	/*configure NC pins*/
	mfp_config(ARRAY_AND_SIZE(mk2_nc_pin_config));

	/* Reset TWSI1 unit firstly */
	__raw_i2c_bus_reset(1);
	/* clear recovery bit */
	if (max77601_rtc_raw_write(MAX77601_RTCSECA2, 0x00))
		pr_err("Recovery flag clear failed!\n");
	
	mk2_backup_battery_init();

	mk2_update_ddr_info();

	/* on-chip devices */
	mmp3_add_uart(2);//GPS
	mmp3_add_uart(3);
	mmp3_add_uart(4);//digitizer
	mmp3_add_twsi(1, NULL, ARRAY_AND_SIZE(mk2_twsi1_info));
	mmp3_add_twsi(4, NULL, ARRAY_AND_SIZE(mk2_twsi4_info));
	mmp3_add_twsi(3, &twsi3_pxa_data, ARRAY_AND_SIZE(mk2_twsi3_info));
	mmp3_add_twsi(5, NULL, ARRAY_AND_SIZE(mk2_twsi5_info));
	/* for hdmi edid */
	mmp3_add_twsi(6, NULL, ARRAY_AND_SIZE(mk2_twsi6_info));

	platform_device_register(&gpio_keys);

	mmp3_add_videosram(&mmp3_videosram_info);
#ifdef CONFIG_FB_PXA168
	mk2_add_lcd_mipi();
	mmp3_add_tv_out();
#endif

#ifdef CONFIG_UIO_HDMI
	mmp3_add_hdmi(&mmp3_hdmi_info);
#endif
	mmp3_add_ddr_devfreq();

#if defined(CONFIG_SENSORS_STM8T143)
	brownstone_init_stm8t143();
#endif
	/* backlight */
	mmp3_add_pwm(3);
	platform_device_register(&mk2_lcd_backlight_devices);

	mmp3_add_thermal();
#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem();
#endif

#ifdef CONFIG_UIO_VMETA
	mmp_init_vmeta();
#endif

#ifdef CONFIG_MMC_SDHCI_PXAV3
	mk2_init_mmc();
#endif /* CONFIG_MMC_SDHCI_PXAV3 */

#if defined(CONFIG_VIDEO_MV)
	platform_device_register(&mk2_ov5642);
	mmp3_add_cam(0, &mv_cam_data);

	platform_device_register(&mk2_ov2656);
	mmp3_add_cam(1, &mv_cam_data_ov2656);
#endif

	platform_device_register(&mmp3_device_rtc);

	wm8994_ldo_enable();
	mk2_fixed_regulator_init();

	/* audio sspa support */
	mmp3_add_sspa(1);
	mmp3_add_sspa(2);
	mmp3_add_audiosram(&mmp3_audiosram_info);

	platform_device_register(&headset_switch_device);

#ifdef CONFIG_USB_PXA_U2O
	pxa_usb_set_vbus(0);
	mmp3_device_u2o.dev.platform_data = (void *)&mmp3_usb_pdata;
	platform_device_register(&mmp3_device_u2o);
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2O
	mmp3_device_u2oehci.dev.platform_data = (void *)&mmp3_usb_pdata;
	platform_device_register(&mmp3_device_u2oehci);

#ifdef CONFIG_USB_PXA_U2O_OTG
	mmp3_device_u2ootg.dev.platform_data = (void *)&mmp3_usb_pdata;
	platform_device_register(&mmp3_device_u2ootg);
#endif
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H_HSIC
	mmp3_hsic2_device.dev.platform_data = (void *)&mmp3_hsic2_pdata;
	platform_device_register(&mmp3_hsic2_device);
#endif
	fsic3_clk_disable();

        create_sirf_proc_file();
        create_wwan_proc_file();         
#if 0
	gsensor_power_en(1);
#endif
	/* Put digitizer related pins to low first*/
	/* in order to meet power sequence spec   */
	InitDigitizerPins();
/* enable EETI EXC7200 touch controller */
#ifdef CONFIG_TOUCHSCREEN_EGALAX_I2C
	/* Configure irq for exc7200 touch panel */
	exc7200_config();
#endif /*CONFIG_TOUCHSCREEN_EGALAX_I2C */

#ifdef CONFIG_TOUCHSCREEN_WACOM_W8001 //T-Lite, enable WACOM w8001 digitizer, 20110817
	// Enable power and configure for wacom w8001 digitizer
	w8001_config();
#endif //CONFIG_TOUCHSCREEN_WACOM_W8001 //T-Lite

	InitChargeLedPins();
	/* If we have a full configuration then disable any regulators
	 * which are not in use or always_on. */
	regulator_has_full_constraints();
	mmp3_set_vcc_main_reg_id("VCC_CORE");

	create_wis_proj_procfs();
#ifdef CONFIG_WIS_KBC_POWER
	platform_device_register(&wis_kbc_power_device);
#endif
	wwan_init(); //Not only power on the modem, but also configure gpios.
        gps_power_off();
}

MACHINE_START(MK2, "toughpad")
	.map_io		= mmp_map_io,
	.nr_irqs	= MK2_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.reserve	= mmp3_reserve,
	.init_machine	= mk2_init,
MACHINE_END
