/*
 *  linux/arch/arm/mach-mmp/orchid.c
 *
 *  Support for the Marvell MMP3 Orchid Development Platform.
 *
 *  Copyright (C) 2009-2011 Marvell International Ltd.
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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/smc91x.h>
#include <linux/mfd/88pm80x.h>
#include <linux/pwm_backlight.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/fan53555.h>
#include <linux/switch.h>
#include <linux/sd8x_rfkill.h>
#include <linux/mmc/sdhci.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <mach/addr-map.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp3.h>
#include <mach/irqs.h>
#include <mach/regs-mpmu.h>
#include <mach/soc_vmeta.h>
#include <mach/uio_hdmi.h>
#if defined(CONFIG_SPI_PXA2XX)
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/ntrig_spi.h>
#endif
#include <plat/usb.h>
#include <media/soc_camera.h>
#include <mach/hsi_dev.h>
#include <mach/sram.h>
#include <plat/pmem.h>

#include "common.h"
#include "onboard.h"

#define ORCHID_NR_IRQS		(IRQ_BOARD_START + 64)

static unsigned long orchid_pin_config[] __initdata = {
	/* UART3 */
	GPIO51_UART3_RXD,
	GPIO52_UART3_TXD,

	/* UART1 */
	GPIO29_UART1_RXD,
	GPIO30_UART1_TXD,
	GPIO31_UART1_CTS,
	GPIO32_UART1_RTS,

	/* UART2 */
	GPIO74_UART2_RXD,
	GPIO75_UART2_TXD,
	GPIO76_UART2_CTS,
	GPIO77_UART2_RTS,

	/* TWSI5 */
	GPIO99_TWSI5_SCL,
	GPIO100_TWSI5_SDA,

	/* TWSI2 */
	GPIO53_TWSI2_SCL,
	GPIO54_TWSI2_SDA,

	/* TWSI3 */
	GPIO71_TWSI3_SCL,
	GPIO72_TWSI3_SDA,

	/* TWSI6 */
	GPIO47_TWSI6_SCL,
	GPIO48_TWSI6_SDA,

	/* SSPA1 (I2S) */
	GPIO24_I2S_SYSCLK,
	GPIO25_I2S_BITCLK,
	GPIO26_I2S_SYNC,
	GPIO27_I2S_DATA_OUT,
	GPIO28_I2S_SDATA_IN,

	/* SSPA2 */
	GPIO33_SSPA2_CLK,	/* ap audio ref clk */

	/* camera */
	GPIO20_GPIO,	/* CAM2 PWR DWN */
	GPIO21_GPIO,	/* CAM1 reset input */
	GPIO22_GPIO,	/* CAM2 reset input */

	/* Keypad */
	GPIO08_KP_MKIN4,
	GPIO09_KP_MKOUT4,
	GPIO10_KP_MKIN5,
	GPIO11_KP_MKOUT5,

	/* PMIC */
	PMIC_PMIC_INT | MFP_LPM_EDGE_FALL,
	GPIO127_GPIO,		/* PM800 DVC2 */
	GPIO128_GPIO,		/* PM800 DVC1 */

	/* LCD */
	GPIO49_LCD_RST_N,
	GPIO50_LCD_PWM,

	GPIO59_LED_R_CTRL,
	GPIO60_LED_G_CTRL,
	GPIO61_LED_B_CTRL,

	/* HDMI */
	GPIO18_GPIO | MFP_PULL_HIGH,	/* HDMI_EN */
	GPIO43_GPIO,			/* HDMI_HPD */
	GPIO113_HDMI_CEC,

	/* HSI */
	HSI_ACWAKE,
	HSI_ACREADY,
	HSI_ACFLAG,
	HSI_ACDATA,
	HSI_CAWAKE,
	HSI_CAREADY,
	HSI_CAFLAG,
	HSI_CADATA,

	/* CMMB SPI I/F */
	GPIO00_SPI_DCLK,
	GPIO01_SPI_CS0,
	GPIO02_SPI_DIN,
	GPIO03_SPI_DOUT,
	GPIO4_GPIO,
	GPIO5_GPIO,
	GPIO6_GPIO,
	GPIO7_GPIO,

	/* misc */
	SMART_BAT,
	GPIO114_MN_CLK_OUT,
	GPIO62_GPIO,		/* GPS ative flag */

	/* charger */
	GPIO44_GPIO,		/* charger disable, high active */
	GPIO46_GPIO,		/* charger status signal */
};

static unsigned long ap_cp_pin_config[] __initdata = {
	GPIO151_GPIO,		/* AP_CP_GPIO0 */
	GPIO144_GPIO,		/* AP_CP_GPIO1 */
	GPIO147_GPIO,		/* AP_CP_GPIO2 */
	GPIO112_GPIO,		/* AP_CP_GPIO3 */
	GPIO150_GPIO,		/* AP_CP_GPIO4 */
	GPIO160_GPIO,		/* AP_CP_GPIO5 */
	GPIO152_GPIO,		/* AP_CP_GPIO6 */
	GPIO153_GPIO,		/* AP_CP_GPIO7 */

	AP_RESET_OUT_N,		/* CP reset input, low active */
};

static unsigned long mmc1_pin_config[] __initdata = {
	GPIO131_MMC1_DAT3,
	GPIO132_MMC1_DAT2,
	GPIO133_MMC1_DAT1,
	GPIO134_MMC1_DAT0,
	GPIO136_MMC1_CMD,
	GPIO135_MMC1_CLK,
	GPIO140_MMC1_CD | MFP_PULL_HIGH,
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
	GPIO17_GPIO, /* WIFI_RST_N */
	GPIO16_GPIO, /* WIFI_PD_N */
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
};

#if defined(CONFIG_VIDEO_MV)
/* soc  camera */
static int camera_sensor_power(struct device *dev, int on)
{
	int cam_pwdn = mfp_to_gpio(MFP_PIN_GPIO68);

	if (gpio_request(cam_pwdn, "CAM_PWDN")) {
		printk(KERN_ERR"Request GPIO failed, gpio: %d\n", cam_pwdn);
		return -EIO;
	}

	/* pull up camera pwdn pin to disable camera sensor */
	/* pull down camera pwdn pin to enable camera sensor */
	if (on)
		gpio_direction_output(cam_pwdn, 0);
	else
		gpio_direction_output(cam_pwdn, 1);

	msleep(100);

	gpio_free(cam_pwdn);
	return 0;
}

static struct i2c_board_info orchid_i2c_camera[] = {
	{
		I2C_BOARD_INFO("ov5642", 0x3c),
	},
};

static struct soc_camera_link iclink_ov5642 = {
	.bus_id         = 1,            /* Must match with the camera ID */
	.power          = camera_sensor_power,
	.board_info     = &orchid_i2c_camera[0],
	.i2c_adapter_id = 2,
	.flags = SOCAM_MIPI,
	.module_name    = "ov5642",
	.priv = "pxa2128-mipi",
};

static struct platform_device orchid_ov5642 = {
	.name   = "soc-camera-pdrv",
	.id     = 0,
	.dev    = {
		.platform_data = &iclink_ov5642,
	},
};
static void pxa2128_cam_ctrl_power(int on)
{
	return;
}

static int pxa2128_cam_clk_init(struct device *dev, int init)
{
	static struct regulator *af_vcc;
	static struct regulator *avdd;
	struct mv_cam_pdata *data = dev->platform_data;
	int cam_enable = mfp_to_gpio(MFP_PIN_GPIO1);
	unsigned long tx_clk_esc;
	struct clk *pll1;

	pll1 = clk_get(dev, "pll1");
	if (IS_ERR(pll1)) {
		dev_err(dev, "Could not get pll1 clock\n");
		return PTR_ERR(pll1);
	}

	tx_clk_esc = clk_get_rate(pll1) / 1000000 / 12;
	clk_put(pll1);

	/* Update dphy6 according to current tx_clk_esc */
	data->dphy[2] = ((534 * tx_clk_esc / 2000 - 1) & 0xff) << 8
			| ((38 * tx_clk_esc / 1000 - 1) & 0xff);

	if (gpio_request(cam_enable, "CAM_ENABLE_HI_SENSOR")) {
		printk(KERN_ERR"Request GPIO failed, gpio: %d\n", cam_enable);
		return -EIO;
	}

	af_vcc = regulator_get(NULL, "V_2P8");
	if (IS_ERR(af_vcc)) {
		af_vcc = NULL;
		return -EIO;
	}

	avdd = regulator_get(NULL, "AVDD_CAM_2P8V");
	if (IS_ERR(avdd)) {
		avdd = NULL;
		return -EIO;
	}
	if ((!data->clk_enabled) && init) {
		data->clk = clk_get(dev, "CCICRSTCLK");
		if (IS_ERR(data->clk)) {
			dev_err(dev, "Could not get rstclk\n");
			return PTR_ERR(data->clk);
		}
		data->clk_enabled = 1;
		regulator_set_voltage(af_vcc, 3000000, 3000000);
		regulator_enable(af_vcc);
		regulator_set_voltage(avdd, 2800000, 2800000);
		regulator_enable(avdd);

		gpio_direction_output(cam_enable, 1);
		gpio_free(cam_enable);
		return 0;
	}

	if (!init && data->clk_enabled) {
		clk_put(data->clk);
		regulator_disable(af_vcc);
		regulator_put(af_vcc);
		regulator_disable(avdd);
		regulator_put(avdd);
		gpio_direction_output(cam_enable, 0);
		gpio_free(cam_enable);
		return 0;
	}
	return -EFAULT;
}

static void pxa2128_cam_set_clk(struct device *dev, int on)
{
	struct mv_cam_pdata *data = dev->platform_data;
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
		return 400;
	default:
		BUG();
	}

	return 0;
}

static struct mv_cam_pdata mv_cam_data = {
	.name = "ABILENE",
	.clk_enabled = 0,
	.dphy = {0x1b0b, 0x33, 0x1a03},
	.qos_req_min = 0,
	.dma_burst = 128,
	.bus_type = SOCAM_MIPI,
	.mclk_min = 26,
	.mclk_src = 3,
	.controller_power = pxa2128_cam_ctrl_power,
	.init_clk = pxa2128_cam_clk_init,
	.enable_clk = pxa2128_cam_set_clk,
	.get_mclk_src = get_mclk_src,
};
/* sensor init over */
#endif

static unsigned int orchid_matrix_key_map[] = {
	KEY(4, 4, KEY_VOLUMEUP),
	KEY(5, 4, KEY_VOLUMEDOWN),
	KEY(4, 5, KEY_CAMERA),
	KEY(5, 5, KEY_CAMERA),
};

static struct pxa27x_keypad_platform_data mmp3_keypad_info = {
	.matrix_key_rows	= 6,
	.matrix_key_cols	= 6,
	.matrix_key_map		= orchid_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(orchid_matrix_key_map),
	.debounce_interval	= 30,
};

/* PMIC Regulator 88PM800 */
static struct regulator_consumer_supply regulator_supplies[] = {
	/* BUCK power supplies: BUCK[1..5] */
	[PM800_ID_BUCK1] = REGULATOR_SUPPLY("V_BUCK1_AP_CORE", NULL),
	[PM800_ID_BUCK2] = REGULATOR_SUPPLY("V_BUCK2_1V2", NULL),
	[PM800_ID_BUCK3] = REGULATOR_SUPPLY("V_BUCK3_1V8", NULL),
	[PM800_ID_BUCK4] = REGULATOR_SUPPLY("V_BUCK4_1V8", NULL),
	[PM800_ID_BUCK5] = REGULATOR_SUPPLY("V_BUCK5_CP_CORE", NULL),
	/* LDO power supplies: LDO[1..19] */
	[PM800_ID_LDO1]  = REGULATOR_SUPPLY("V_LDO1_1V1", NULL),
	[PM800_ID_LDO2]  = REGULATOR_SUPPLY("V_LDO2_2V8", NULL),
	[PM800_ID_LDO3]  = REGULATOR_SUPPLY("V_LDO3_1V2", NULL),
	[PM800_ID_LDO4]  = REGULATOR_SUPPLY("V_LDO4_2V9", NULL),
	[PM800_ID_LDO5]  = REGULATOR_SUPPLY("V_LDO5_3V3", NULL),
	[PM800_ID_LDO6]  = REGULATOR_SUPPLY("V_LDO6_1V8", NULL),
	[PM800_ID_LDO7]  = REGULATOR_SUPPLY("V_LDO7_2V8", NULL),
	[PM800_ID_LDO8]  = REGULATOR_SUPPLY("V_LDO8_1V5", NULL),
	[PM800_ID_LDO9]  = REGULATOR_SUPPLY("V_LDO9_1V8", NULL),
	[PM800_ID_LDO10] = REGULATOR_SUPPLY("V_LDO10_2V8", NULL),
	[PM800_ID_LDO11] = REGULATOR_SUPPLY("V_LDO11_2V8", NULL),
	[PM800_ID_LDO12] = REGULATOR_SUPPLY("V_LDO12_2V8", NULL),
	[PM800_ID_LDO13] = REGULATOR_SUPPLY("V_LDO13_2V8", NULL),
	[PM800_ID_LDO14] = REGULATOR_SUPPLY("V_LDO14_2V8", NULL),
	[PM800_ID_LDO15] = REGULATOR_SUPPLY("V_LDO15_1V8", NULL),
	[PM800_ID_LDO16] = REGULATOR_SUPPLY("V_LDO16_3V3", NULL),
	[PM800_ID_LDO17] = REGULATOR_SUPPLY("V_LDO17_3V1", NULL),
	[PM800_ID_LDO18] = REGULATOR_SUPPLY("V_LDO18_2V8", NULL),
	[PM800_ID_LDO19] = REGULATOR_SUPPLY("V_LDO19_1V8", NULL),
};

static int regulator_index[] = {
	PM800_ID_BUCK1,
	PM800_ID_BUCK2,
	PM800_ID_BUCK3,
	PM800_ID_BUCK4,
	PM800_ID_BUCK5,
	PM800_ID_LDO1,
	PM800_ID_LDO2,
	PM800_ID_LDO3,
	PM800_ID_LDO4,
	PM800_ID_LDO5,
	PM800_ID_LDO6,
	PM800_ID_LDO7,
	PM800_ID_LDO8,
	PM800_ID_LDO9,
	PM800_ID_LDO10,
	PM800_ID_LDO11,
	PM800_ID_LDO12,
	PM800_ID_LDO13,
	PM800_ID_LDO14,
	PM800_ID_LDO15,
	PM800_ID_LDO16,
	PM800_ID_LDO17,
	PM800_ID_LDO18,
	PM800_ID_LDO19,
};

#define REG_INIT(_name, _min, _max, _always, _boot)	\
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
	.consumer_supplies	= &regulator_supplies[PM800_ID_##_name], \
	.driver_data = &regulator_index[PM800_ID_##_name],	\
}
static struct regulator_init_data pm800_regulator_data[] = {
	/* BUCK power supplies: BUCK[1..5] */
	[PM800_ID_BUCK1] = REG_INIT(BUCK1,  600000, 3950000, 1, 1),
	[PM800_ID_BUCK2] = REG_INIT(BUCK2,  600000, 3950000, 1, 1),
	[PM800_ID_BUCK3] = REG_INIT(BUCK3,  600000, 3950000, 1, 1),
	[PM800_ID_BUCK4] = REG_INIT(BUCK4,  600000, 3950000, 1, 1),
	[PM800_ID_BUCK5] = REG_INIT(BUCK5,  600000, 3950000, 1, 1),
	/* LDO power supplies: LDO[1..19] */
	[PM800_ID_LDO1]  = REG_INIT(LDO1,   600000, 1500000, 0, 0),
	[PM800_ID_LDO2]  = REG_INIT(LDO2,   600000, 1500000, 0, 0),
	[PM800_ID_LDO3]  = REG_INIT(LDO3,  1200000, 3300000, 0, 0),
	[PM800_ID_LDO4]  = REG_INIT(LDO4,  1200000, 3300000, 1, 1),
	[PM800_ID_LDO5]  = REG_INIT(LDO5,  1200000, 3300000, 1, 1),
	[PM800_ID_LDO6]  = REG_INIT(LDO6,  1200000, 3300000, 1, 1),
	[PM800_ID_LDO7]  = REG_INIT(LDO7,  1200000, 3300000, 1, 1),
	[PM800_ID_LDO8]  = REG_INIT(LDO8,  1200000, 3300000, 0, 0),
	[PM800_ID_LDO9]  = REG_INIT(LDO9,  1200000, 3300000, 1, 1),
	[PM800_ID_LDO10] = REG_INIT(LDO10, 1200000, 3300000, 0, 0),
	[PM800_ID_LDO11] = REG_INIT(LDO11, 1200000, 3300000, 0, 0),
	[PM800_ID_LDO12] = REG_INIT(LDO12, 1200000, 3300000, 1, 1),
	[PM800_ID_LDO13] = REG_INIT(LDO13, 1200000, 3300000, 1, 1),
	[PM800_ID_LDO14] = REG_INIT(LDO14, 1200000, 3300000, 1, 1),
	[PM800_ID_LDO15] = REG_INIT(LDO15, 1200000, 3300000, 0, 1),
	[PM800_ID_LDO16] = REG_INIT(LDO16, 1200000, 3300000, 1, 1),
	[PM800_ID_LDO17] = REG_INIT(LDO17, 1200000, 3300000, 1, 1),
	[PM800_ID_LDO18] = REG_INIT(LDO18, 1700000, 3300000, 1, 1),
	[PM800_ID_LDO19] = REG_INIT(LDO19, 1700000, 3300000, 1, 1),
};

static struct pm80x_rtc_pdata pm80x_rtc = {
	.rtc_wakeup	= 0,
};

static int pm800_plat_config(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata)
{
	if (!chip || !pdata ||
		chip->id != CHIP_PM800 ||
		!chip->base_page) {
		pr_err("%s:chip or pdata is not availiable!\n", __func__);
		return -EINVAL;
	}
	/* Disable watch dog */
	pm80x_set_bits(chip->base_page, PM800_WAKEUP2, (0xF << 4), 0);
	/* Select XO 32KHZ(USE_XO)
	 * Force all CLK32K_1/2/3 buffers to use the XO 32KHZ */
	pm80x_set_bits(chip->base_page, PM800_RTC_CONTROL, (1 << 7), (1 << 7));
	/* Enable 32K out1 from XO: AP_32K_CLK */
	pm80x_set_bits(chip->base_page, PM800_RTC_MISC2, 0x3, 0x2);
	/* Enable 32K out2 from XO: WLAN_32K_CLK */
	pm80x_set_bits(chip->base_page, PM800_RTC_MISC2,
					(0x3 << 2), (0x2 << 2));
	/* Enable 32K out3 from XO, low-jitter: REF_32K_CLK */
	pm80x_set_bits(chip->base_page, PM800_RTC_MISC2,
					(0x3 << 4), (0x2 << 4));

	return 0;
}

static struct pm80x_platform_data pm80x_info = {
	.base_page_addr = 0x30,		/* BASE page */
	.power_page_addr = 0x31,	/* POWER */
	.gpadc_page_addr = 0x32,	/* GPADC */
	.test_page_addr = 0x37,		/* TEST */
	.irq_mode = 0,	/* 0: clear IRQ by read */
	.irq_base = IRQ_BOARD_START,
	/* Codec PM805 */
	.companion_addr = 0x39,
	.irq_companion = gpio_to_irq(mfp_to_gpio(GPIO97_GPIO)),

	.num_regulators = ARRAY_SIZE(pm800_regulator_data),
	.regulator	    = pm800_regulator_data,
	.rtc = &pm80x_rtc,
	.pm800_plat_config = pm800_plat_config,
};
/* End Of PM800 */

#if 0
/* Core Voltage Regulator: V_MAIN */
static struct regulator_consumer_supply fan53555_regulator_supply[] = {
	REGULATOR_SUPPLY("V_MAIN", NULL),
};

static struct regulator_init_data fan53555_regulator_data = {
	.constraints = {
		.name = "V_MAIN",
		.min_uV = 600000,
		.max_uV = 1230000,
		.always_on = 1,
		.boot_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
		.valid_modes_mask = REGULATOR_MODE_FAST | REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(fan53555_regulator_supply),
	.consumer_supplies = fan53555_regulator_supply,
};

static struct fan53555_platform_data fan53555_pdata = {
	.regulator = &fan53555_regulator_data,
	.slew_rate = FAN53555_SLEW_RATE_64MV, /* mV/uS */
	.sleep_vsel_id = FAN53555_VSEL_ID_0, /* VSEL0 */
	.sleep_vol = 1000000, /* uV */
};
#endif

static struct i2c_board_info orchid_twsi1_info[] = {
	{
		.type = "88PM80x",
		.addr = 0x34,
		.irq  = IRQ_MMP3_PMIC,
		.platform_data = &pm80x_info,
	},
#if 0
	{
		.type = "fan53555",
		.addr = 0x60,
		.platform_data = &fan53555_pdata,
	},
#endif
};

/* TODO: for charger FAN5405 */
static struct i2c_board_info orchid_twsi2_info[] = {
};

/* TODO: for 3 AXIS GRYOSCOPE, E-COMPASS,
 *       PROXIMITY and AMBIENT sensor and
 *       GPS*/
static struct i2c_board_info orchid_twsi4_info[] = {
};

static int orchid_pwm_init(struct device *dev)
{
	int gpio = mfp_to_gpio(GPIO84_GPIO);

	if (!cpu_is_mmp3_b0())
		return 0;

	if (gpio_request(gpio, "LCD_BKL_EN")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio);
		return -1;
	}

	gpio_direction_output(gpio, 1);
	gpio_free(gpio);

	return 0;
}

static struct platform_pwm_backlight_data orchid_lcd_backlight_data = {
	/* primary backlight */
	.pwm_id = 2,
	.max_brightness = 100,
	.dft_brightness = 50,
	.pwm_period_ns = 2000000,
	.init = orchid_pwm_init,
};

static struct platform_device orchid_lcd_backlight_devices = {
	.name = "pwm-backlight",
	.id = 2,
	.dev = {
		.platform_data = &orchid_lcd_backlight_data,
	},
};

/* TODO: For touch panel */
static struct i2c_board_info orchid_twsi5_info[] = {
};

/* TODO: For NFC module and HDMI */
static struct i2c_board_info orchid_twsi6_info[] = {
};

/* TODO: for QV8820, OV9740 */
static struct i2c_board_info orchid_twsi3_info[] = {
};

#ifdef CONFIG_SD8XXX_RFKILL
static void mmp3_8787_set_power(unsigned int on)
{
	static struct regulator *v_ldo16;
	static int f_enabled = 0;

	/* VBAT_FEM 3.3v */
	if (on && (!f_enabled)) {
		v_ldo16 = regulator_get(NULL, "V_LDO16_3V3");
		if (IS_ERR(v_ldo16)) {
			v_ldo16 = NULL;
			pr_err("get V_LDO16_3V3 failed %s.\n", __func__);
		} else {
			regulator_set_voltage(v_ldo16, 3300000, 3300000);
			regulator_enable(v_ldo16);
			f_enabled = 1;
		}
	}

	if (f_enabled && (!on)) {
		if (v_ldo16) {
			regulator_disable(v_ldo16);
			regulator_put(v_ldo16);
			v_ldo16 = NULL;
		}
		f_enabled = 0;
	}
}
#endif

#ifdef CONFIG_MMC_SDHCI_PXAV3
#include <linux/mmc/host.h>
static void orchid_sd_signal_1v8(int set)
{
	static struct regulator *v_sdmmc;
	int vol;

	v_sdmmc = regulator_get(NULL, "V_LDO13_2V8");
	if (IS_ERR(v_sdmmc)) {
		printk(KERN_ERR "Failed to get V_LDO13_2V8\n");
		return;
	}

	vol = set ? 1800000 : 3000000;
	regulator_set_voltage(v_sdmmc, vol, vol);
	regulator_enable(v_sdmmc);

	mmp3_io_domain_1v8(AIB_SDMMC_IO_REG, set);

	msleep(10);
	regulator_put(v_sdmmc);
}

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc0 = {
	.clk_delay_cycles	= 0x1F,
	.signal_1v8		= orchid_sd_signal_1v8,
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc1 = {
	.flags          = PXA_FLAG_CARD_PERMANENT,
	.pm_caps        = MMC_PM_KEEP_POWER | MMC_PM_IRQ_ALWAYS_ON,
#ifndef CONFIG_SD8XXX_RFKILL
	.host_caps      = MMC_CAP_POWER_OFF_CARD,
#endif
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc2 = {
	.flags		= PXA_FLAG_SD_8_BIT_CAPABLE_SLOT,
	.host_caps	= MMC_CAP_1_8V_DDR,
};

static struct regulator_consumer_supply sdio_power_supplies[] = {
	REGULATOR_SUPPLY("vsdio", "sdhci-pxa.1"),
};

static struct regulator_init_data sdio_power_data = {
	.constraints    = {
		.valid_ops_mask         = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(sdio_power_supplies),
	.consumer_supplies      = sdio_power_supplies,
};

static struct fixed_voltage_config sdio_power = {
	.supply_name            = "vsdio",
	.microvolts             = 1800000,
	.gpio                   = mfp_to_gpio(MFP_PIN_GPIO16),
	.enable_high            = 1,
	.enabled_at_boot        = 0,
	.init_data              = &sdio_power_data,
};

static struct platform_device sdio_power_device = {
	.name           = "reg-fixed-voltage",
	.id             = 1,
	.dev = {
		.platform_data = &sdio_power,
	},
};

static void __init orchid_init_mmc(void)
{
#ifdef CONFIG_SD8XXX_RFKILL
	int WIB_PDn = mfp_to_gpio(GPIO16_GPIO);
	int WIB_RESETn = mfp_to_gpio(GPIO17_GPIO);
	add_sd8x_rfkill_device(WIB_PDn, WIB_RESETn,\
			&mmp3_sdh_platdata_mmc1.pmmc, mmp3_8787_set_power);
#else
#ifdef CONFIG_PM_RUNTIME
	int RESETn = mfp_to_gpio(MFP_PIN_GPIO17);
	if (gpio_request(RESETn, "sdio RESETn")) {
		pr_err("Failed to request sdio RESETn gpio\n");
		return;
	}
	gpio_direction_output(RESETn, 0);
	msleep(1);
	gpio_direction_output(RESETn, 1);
	gpio_free(RESETn);
        platform_device_register(&sdio_power_device);
#endif
#endif
	mfp_config(ARRAY_AND_SIZE(mmc3_pin_config));
	mmp3_add_sdh(2, &mmp3_sdh_platdata_mmc2); /* eMMC */

	mfp_config(ARRAY_AND_SIZE(mmc1_pin_config));
	if (cpu_is_mmp3_b0())
		mmp3_sdh_platdata_mmc0.quirks = SDHCI_QUIRK_INVERTED_WRITE_PROTECT;
	mmp3_add_sdh(0, &mmp3_sdh_platdata_mmc0); /* SD/MMC */

	/* SDIO for WIFI card */
	mfp_config(ARRAY_AND_SIZE(mmc2_pin_config));
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
	int gpio = mfp_to_gpio(GPIO82_VBUS_EN);

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
	.phy_deinit     = pxa_usb_phy_deinit,
	.set_vbus	= pxa_usb_set_vbus,
};
#endif

#endif


#ifdef CONFIG_UIO_HDMI
static struct uio_hdmi_platform_data mmp3_hdmi_info __initdata = {
	.sspa_reg_base = 0xD42A0C00,
	/* Fix me: gpio 59 lpm pull ? */
	.gpio = mfp_to_gpio(GPIO59_HDMI_DET),
};
#endif

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

#ifdef CONFIG_MMP3_HSI
static struct hsi_platform_data mmp_hsi_plat_data = {
	.hsi_config_int = NULL,
};
static void __init mmp_init_hsi(void)
{
	mmp_register_hsi(&mmp_hsi_plat_data);
}
#endif

static void __init mmp_init_ap_cp(void)
{
	mfp_config(ARRAY_AND_SIZE(ap_cp_pin_config));
#ifdef CONFIG_MMP3_HSI
	mmp_init_hsi();
#endif
}

/* Only for reboot routine */
static const u32 reg_idbr = APB_VIRT_BASE + 0x11000 + 0x08;
static const u32 reg_icr = APB_VIRT_BASE + 0x11000 + 0x10;
static const u32 reg_isr = APB_VIRT_BASE + 0x11000 + 0x18;

/* Control register bits */
#define ICR_START	(1 << 0)
#define ICR_STOP	(1 << 1)
#define ICR_ACKNAK	(1 << 2)
#define ICR_TB		(1 << 3)
#define ICR_ALDIE	(1 << 12)
/* Status register bits */
#define ISR_ACKNAK	(1 << 1)
#define ISR_IBB		(1 << 3)
#define ISR_ITE		(1 << 6)
#define ISR_IRF		(1 << 7)

static int i2c_isr_set_cleared(u32 set, u32 cleared)
{
	int isr, timeout = 1000;
	do {
		isr = readl(reg_isr);
		udelay(10);
		if (timeout-- < 0)
			return 0;
	} while (((isr & set) != set) ||
			((isr & cleared) != 0));
	return 1;
}

static int pm800_i2c_write_reg(u8 addr, u8 reg, u8 val)
{
	/* Is bus busy? */
	if (!i2c_isr_set_cleared(0, ISR_IBB))
		return -1;
	/* -----1. Send chip addr -------  */
	/* Clear START and STOP bits */
	writel(readl(reg_icr) & ~(ICR_START), reg_icr);
	writel(readl(reg_icr) & ~(ICR_STOP), reg_icr);
	/* Write chip addr */
	writel((addr << 1), reg_idbr);
	/* Send start bit */
	writel(readl(reg_icr) | ICR_START, reg_icr);
	/* Clear ALDIE */
	writel(readl(reg_icr) & ~(ICR_ALDIE), reg_icr);
	/* Transfer, send a byte */
	writel(readl(reg_icr) | ICR_TB, reg_icr);
	/* Transmit empty? */
	if (!i2c_isr_set_cleared(ISR_ITE, 0))
		return -1;
	/* Clear ITE: W1C */
	writel(readl(reg_isr) | ISR_ITE, reg_isr);
	/* Wait for ACK */
	if (!i2c_isr_set_cleared(0, ISR_ACKNAK))
		return -1;
	/* -----2. Send reg addr ------ */
	/* Clear START and STOP bits */
	writel(readl(reg_icr) & ~(ICR_START), reg_icr);
	writel(readl(reg_icr) & ~(ICR_STOP), reg_icr);
	/* Write reg addr */
	writel(reg, reg_idbr);
	/* Clear ALDIE */
	writel(readl(reg_icr) & ~(ICR_ALDIE), reg_icr);
	/* Transfer, send a byte */
	writel(readl(reg_icr) | ICR_TB, reg_icr);
	/* Transmit empty? */
	if (!i2c_isr_set_cleared(ISR_ITE, 0))
		return -1;
	/* Clear ITE: W1C */
	writel(readl(reg_isr) | ISR_ITE, reg_isr);
	/* Wait for ACK */
	if (!i2c_isr_set_cleared(0, ISR_ACKNAK))
		return -1;
	/* -----3. Send val ------ */
	/* Clear START and STOP bits */
	writel(readl(reg_icr) & ~(ICR_START), reg_icr);
	writel(readl(reg_icr) & ~(ICR_STOP), reg_icr);
	/* Write val */
	writel(val, reg_idbr);
	/* Send Stop bit */
	writel(readl(reg_icr) | ICR_STOP, reg_icr);
	/* Clear ALDIE */
	writel(readl(reg_icr) & ~(ICR_ALDIE), reg_icr);
	/* Transfer, send a byte */
	writel(readl(reg_icr) | ICR_TB, reg_icr);
	/* Transmit empty? */
	if (!i2c_isr_set_cleared(ISR_ITE, 0))
		return -1;
	/* Clear ITE: W1C */
	writel(readl(reg_isr) | ISR_ITE, reg_isr);
	/* Wait for ACK */
	if (!i2c_isr_set_cleared(0, ISR_ACKNAK))
		return -1;

	return 0;
}

static int pm800_i2c_read_reg(u8 addr, u8 reg, u8 *buf, int len)
{
	/* Is bus busy? */
	if (!i2c_isr_set_cleared(0, ISR_IBB))
		return -1;
	/* -----1. Send chip addr -------  */
	/* Clear START and STOP bits */
	writel(readl(reg_icr) & ~(ICR_START), reg_icr);
	writel(readl(reg_icr) & ~(ICR_STOP), reg_icr);
	/* Write chip addr */
	writel((addr << 1), reg_idbr);
	/* Send start bit */
	writel(readl(reg_icr) | ICR_START, reg_icr);
	/* Clear ALDIE */
	writel(readl(reg_icr) & ~(ICR_ALDIE), reg_icr);
	/* Transfer, send a byte */
	writel(readl(reg_icr) | ICR_TB, reg_icr);
	/* Transmit empty? */
	if (!i2c_isr_set_cleared(ISR_ITE, 0))
		return -1;
	/* Clear ITE: W1C */
	writel(readl(reg_isr) | ISR_ITE, reg_isr);
	/* Wait for ACK */
	if (!i2c_isr_set_cleared(0, ISR_ACKNAK))
		return -1;
	/* -----2. Send reg addr ------ */
	/* Clear START and STOP bits */
	writel(readl(reg_icr) & ~(ICR_START), reg_icr);
	writel(readl(reg_icr) & ~(ICR_STOP), reg_icr);
	/* Write reg addr */
	writel(reg, reg_idbr);
	/* Clear ALDIE */
	writel(readl(reg_icr) & ~(ICR_ALDIE), reg_icr);
	/* Transfer, send a byte */
	writel(readl(reg_icr) | ICR_TB, reg_icr);
	/* Transmit empty? */
	if (!i2c_isr_set_cleared(ISR_ITE, 0))
		return -1;
	/* Clear ITE: W1C */
	writel(readl(reg_isr) | ISR_ITE, reg_isr);
	/* Wait for ACK */
	if (!i2c_isr_set_cleared(0, ISR_ACKNAK))
		return -1;
	/* -----3. Start read sequence -------  */
	/* Clear START and STOP bits */
	writel(readl(reg_icr) & ~(ICR_START), reg_icr);
	writel(readl(reg_icr) & ~(ICR_STOP), reg_icr);
	/* Write chip addr: R/nW=1 */
	writel((addr << 1) | 0x1, reg_idbr);
	/* Send start bit */
	writel(readl(reg_icr) | ICR_START, reg_icr);
	/* Clear ALDIE */
	writel(readl(reg_icr) & ~(ICR_ALDIE), reg_icr);
	/* Transfer, send a byte */
	writel(readl(reg_icr) | ICR_TB, reg_icr);
	/* Transmit empty? */
	if (!i2c_isr_set_cleared(ISR_ITE, 0))
		return -1;
	/* Clear ITE: W1C */
	writel(readl(reg_isr) | ISR_ITE, reg_isr);
	/* Wait for ACK */
	if (!i2c_isr_set_cleared(0, ISR_ACKNAK))
		return -1;
	while (len--) {
		/* Clear START and STOP bits */
		writel(readl(reg_icr) & ~(ICR_START), reg_icr);
		writel(readl(reg_icr) & ~(ICR_STOP), reg_icr);
		if (len == 0)
			/* Send NACK */
			writel(readl(reg_icr) | ICR_ACKNAK, reg_icr);
		else
			/* Send ACK */
			writel(readl(reg_icr) & ~ICR_ACKNAK, reg_icr);
		/* Clear ALDIE */
		writel(readl(reg_icr) & ~(ICR_ALDIE), reg_icr);
		/* Transfer, receive a byte */
		writel(readl(reg_icr) | ICR_TB, reg_icr);
		/* Receive register full? */
		if (!i2c_isr_set_cleared(ISR_IRF, 0))
			return -1;
		*buf = readl(reg_idbr);
		buf++;
		/* Clear ITE: W1C */
		writel(readl(reg_isr) | ISR_IRF, reg_isr);
	}
	return 0;
}

static int orchid_board_reset(char mode, const char *cmd)
{
	u8 buf;
	/* Reset TWSI1 unit firstly */
	writel(0x4060, reg_icr);
	udelay(500);
	writel(0x60, reg_icr);
	udelay(500);
	/* 1.Enable FAULT_WU and FAULT_WU_EN */
	pm800_i2c_read_reg(0x30, 0xE7, &buf, 1);
	buf |= ((1 << 3) | (1 << 2));
	pm800_i2c_write_reg(0x30, 0xE7, buf);
	/* 2.Issue SW power down */
	pm800_i2c_write_reg(0x30, 0x0D, 0x20);
	/* Rebooting... */
	return 1;
}

static void orchid_poweroff(void)
{
	pm8xxx_system_poweroff();
}

static void __init orchid_init(void)
{
	extern int (*board_reset)(char mode, const char *cmd);
	board_reset = orchid_board_reset;
	pm_power_off = orchid_poweroff;
	mfp_config(ARRAY_AND_SIZE(orchid_pin_config));

	/* on-chip devices */
	mmp3_add_uart(3);
	mmp3_add_twsi(1, NULL, ARRAY_AND_SIZE(orchid_twsi1_info));
	mmp3_add_twsi(2, NULL, ARRAY_AND_SIZE(orchid_twsi2_info));
	mmp3_add_twsi(4, NULL, ARRAY_AND_SIZE(orchid_twsi4_info));
	mmp3_add_twsi(5, NULL, ARRAY_AND_SIZE(orchid_twsi5_info));
	mmp3_add_twsi(6, NULL, ARRAY_AND_SIZE(orchid_twsi6_info));

	mmp3_add_keypad(&mmp3_keypad_info);
	mmp3_add_videosram(&mmp3_videosram_info);
#ifdef CONFIG_FB_PXA168
	/* orchid_add_lcd_mipi(); */
	/* mmp3_add_tv_out(); */
#endif

#ifdef CONFIG_UIO_HDMI
	mmp3_add_hdmi(&mmp3_hdmi_info);
#endif

	/* backlight */
	mmp3_add_pwm(3);
	platform_device_register(&orchid_lcd_backlight_devices);

	mmp3_add_thermal();
#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem();
#endif
	mmp_init_ap_cp();
#ifdef CONFIG_UIO_VMETA
	mmp_init_vmeta();
#endif
#ifdef CONFIG_MMC_SDHCI_PXAV3
	orchid_init_mmc();
#endif /* CONFIG_MMC_SDHCI_PXAV3 */

	platform_device_register(&mmp3_device_rtc);

#if defined(CONFIG_TOUCHSCREEN_VNC)
	platform_device_register(&mmp3_device_vnc_touch);
#endif

	mmp3_add_twsi(3, NULL, ARRAY_AND_SIZE(orchid_twsi3_info));

	/* audio sspa support */
	mmp3_add_sspa(1);
	mmp3_add_sspa(2);
	mmp3_add_audiosram(&mmp3_audiosram_info);

#if defined(CONFIG_VIDEO_MV)
	platform_device_register(&orchid_ov5642);
	mmp3_add_cam(1, &mv_cam_data);
#endif

#ifdef CONFIG_USB_PXA_U2O
	/* Place VBUS_EN low by default */
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

	/* If we have a full configuration then disable any regulators
	 * which are not in use or always_on. */
	regulator_has_full_constraints();
}

MACHINE_START(ORCHID, "Orchid")
	.map_io		= mmp_map_io,
	.nr_irqs	= ORCHID_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.reserve	= mmp3_reserve,
	.init_machine	= orchid_init,
MACHINE_END
