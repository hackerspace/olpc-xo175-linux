/*
 *  linux/arch/arm/mach-mmp/yellowstone.c
 *
 *  Support for the Marvell MMP3 YellowStone Development Platform.
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
#if defined(CONFIG_SENSORS_LSM303DLHC_ACC) || \
	defined(CONFIG_SENSORS_LSM303DLHC_MAG)
#include <linux/i2c/lsm303dlhc.h>
#endif
#if defined(CONFIG_SENSORS_L3G4200D_GYR)
#include <linux/i2c/l3g4200d.h>
#endif
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
#include <mach/sram.h>
#include <plat/pmem.h>

#include "common.h"
#include "onboard.h"

#define YELLOWSTONE_NR_IRQS		(IRQ_BOARD_START + 64)

static unsigned long yellowstone_pin_config[] __initdata = {
	/* UART3 */
	GPIO51_UART3_RXD,
	GPIO52_UART3_TXD,

	/* TWSI5 */
	GPIO99_TWSI5_SCL,
	GPIO100_TWSI5_SDA,

	/* TWSI6 */
	GPIO97_TWSI6_SCL,
	GPIO98_TWSI6_SDA,

	/* TWSI2 */
	GPIO43_TWSI2_SCL,
	GPIO44_TWSI2_SDA,

	/* TWSI3 */
	GPIO71_TWSI3_SCL,
	GPIO72_TWSI3_SDA,

	/* TWSI4 */
	TWSI4_SCL,
	TWSI4_SDA,

	/*PWM3*/
	GPIO53_PWM3,
	GPIO84_GPIO,

	/* SSPA1 (I2S) */
	GPIO23_GPIO,
	GPIO24_I2S_SYSCLK,
	GPIO25_I2S_BITCLK,
	GPIO26_I2S_SYNC,
	GPIO27_I2S_DATA_OUT,
	GPIO28_I2S_SDATA_IN,

	/* camera */
	GPIO68_GPIO,
	GPIO1_GPIO,
	GPIO73_CAM_MCLK,

	/* DFI */
	GPIO168_DFI_D0,
	GPIO167_DFI_D1,
	GPIO166_DFI_D2,
	GPIO165_DFI_D3,
	GPIO107_DFI_D4,
	GPIO106_DFI_D5,
	GPIO105_DFI_D6,
	GPIO104_DFI_D7,
	GPIO111_DFI_D8,
	GPIO164_DFI_D9,
	GPIO163_DFI_D10,
	GPIO162_DFI_D11,
	GPIO161_DFI_D12,
	GPIO110_DFI_D13,
	GPIO109_DFI_D14,
	GPIO108_DFI_D15,
	GPIO143_ND_nCS0,
	GPIO144_ND_nCS1,
	GPIO147_ND_nWE,
	GPIO148_ND_nRE,
	GPIO150_ND_ALE,
	GPIO149_ND_CLE,
	GPIO112_ND_RDY0,
	GPIO160_ND_RDY1,

	/* Keypad */
	GPIO16_KP_DKIN0 | MFP_PULL_HIGH,
	GPIO17_KP_DKIN1 | MFP_PULL_HIGH,
	GPIO18_KP_DKIN2 | MFP_PULL_HIGH,
	GPIO19_KP_DKIN3 | MFP_PULL_HIGH,
	GPIO20_KP_DKIN4 | MFP_PULL_HIGH,
	GPIO22_KP_DKIN6 | MFP_PULL_HIGH,

	PMIC_PMIC_INT | MFP_LPM_EDGE_FALL,

	GPIO128_LCD_RST,

	/* OTG vbus enable signal */
	GPIO82_VBUS_EN,

	/* HSIC1 reset pin*/
	GPIO96_HSIC_RESET,

	/* HDMI */
	GPIO54_HDMI_CEC,

	/* SSP4 */
	GPIO78_SSP_CLK,
	GPIO79_SSP_FRM,
	GPIO80_SSP_TXD,
	GPIO81_SSP_RXD,
	GPIO101_GPIO, /* TS INT*/
	GPIO85_GPIO, /* TS_IO_EN */
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
	GPIO58_GPIO, /* WIFI_RST_N */
	GPIO57_GPIO, /* WIFI_PD_N */
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

static struct i2c_board_info yellowstone_i2c_camera[] = {
	{
		I2C_BOARD_INFO("ov5642", 0x3c),
	},
};

static struct soc_camera_link iclink_ov5642 = {
	.bus_id         = 1,            /* Must match with the camera ID */
	.power          = camera_sensor_power,
	.board_info     = &yellowstone_i2c_camera[0],
	.i2c_adapter_id = 2,
	.flags = SOCAM_MIPI,
	.module_name    = "ov5642",
	.priv = "pxa2128-mipi",
};

static struct platform_device yellowstone_ov5642 = {
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

static struct pxa27x_keypad_platform_data mmp3_keypad_info = {
	.direct_key_map = {
		KEY_BACK,
		KEY_MENU,
		KEY_HOME,
		KEY_SEARCH,
		KEY_VOLUMEUP,
		KEY_RESERVED,
		KEY_VOLUMEDOWN,
	},
	.direct_key_num = 7,
	.debounce_interval = 30,
	.active_low = 1,
};

/* PMIC Regulator 88PM800 */
/* Power Supply ECOs:
 * ECO#6: V_2P8(LDO14) is wired to LDO7, so LDO14 should keep off */
static struct regulator_consumer_supply regulator_supplies[] = {
	/* BUCK power supplies: BUCK[1..5] */
	[PM800_ID_BUCK1] = REGULATOR_SUPPLY("V_PMIC_SD0", NULL),
	[PM800_ID_BUCK2] = REGULATOR_SUPPLY("V_DDR3", NULL),
	[PM800_ID_BUCK3] = REGULATOR_SUPPLY("V_SD3", NULL),
	[PM800_ID_BUCK4] = REGULATOR_SUPPLY("V_1P8", NULL),
	[PM800_ID_BUCK5] = REGULATOR_SUPPLY("V_SD5", NULL),
	/* LDO power supplies: LDO[1..19] */
	[PM800_ID_LDO1]  = REGULATOR_SUPPLY("V_LDO1", NULL),
	[PM800_ID_LDO2]  = REGULATOR_SUPPLY("V_MIC_BIAS", NULL),
	[PM800_ID_LDO3]  = REGULATOR_SUPPLY("V_1P2_MIPI", NULL),
	[PM800_ID_LDO4]  = REGULATOR_SUPPLY("V_LDO4", NULL),
	[PM800_ID_LDO5]  = REGULATOR_SUPPLY("V_3P3", NULL),
	[PM800_ID_LDO6]  = REGULATOR_SUPPLY("V_PMIC", NULL),
	[PM800_ID_LDO7]  = REGULATOR_SUPPLY("V_2P8"/*V_LDO7*/, NULL),
	[PM800_ID_LDO8]  = REGULATOR_SUPPLY("V_1P2_HSIC", NULL),
	[PM800_ID_LDO9]  = REGULATOR_SUPPLY("V_1P8_USBFE", NULL),
	[PM800_ID_LDO10] = REGULATOR_SUPPLY("V_LCD", NULL),
	[PM800_ID_LDO11] = REGULATOR_SUPPLY("V_1P2_CODEC", NULL),
	[PM800_ID_LDO12] = REGULATOR_SUPPLY("V_LDO12", NULL),
	[PM800_ID_LDO13] = REGULATOR_SUPPLY("V_SDMMC", NULL),
	[PM800_ID_LDO14] = REGULATOR_SUPPLY("V_LDO14"/*V_2P8*/, NULL),
	[PM800_ID_LDO15] = REGULATOR_SUPPLY("V_LDO15", NULL),
	[PM800_ID_LDO16] = REGULATOR_SUPPLY("VBAT_FEM", NULL),
	[PM800_ID_LDO17] = REGULATOR_SUPPLY("V_BB", NULL),
	[PM800_ID_LDO18] = REGULATOR_SUPPLY("AVDD_CAM_2P8V", NULL),
	[PM800_ID_LDO19] = REGULATOR_SUPPLY("V_1P8_ANA", NULL),
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
	[PM800_ID_BUCK5] = REG_INIT(BUCK5,  600000, 3950000, 0, 0),
	/* LDO power supplies: LDO[1..19] */
	[PM800_ID_LDO1]  = REG_INIT(LDO1,   600000, 1500000, 0, 0),
	[PM800_ID_LDO2]  = REG_INIT(LDO2,   600000, 1500000, 0, 0),
	[PM800_ID_LDO3]  = REG_INIT(LDO3,  1200000, 3300000, 1, 1),
	[PM800_ID_LDO4]  = REG_INIT(LDO4,  1200000, 3300000, 0, 0),
	[PM800_ID_LDO5]  = REG_INIT(LDO5,  1200000, 3300000, 1, 1),
	[PM800_ID_LDO6]  = REG_INIT(LDO6,  1200000, 3300000, 1, 1),
	[PM800_ID_LDO7]  = REG_INIT(LDO7,  1200000, 3300000, 1, 1),
	[PM800_ID_LDO8]  = REG_INIT(LDO8,  1200000, 3300000, 1, 1),
	[PM800_ID_LDO9]  = REG_INIT(LDO9,  1200000, 3300000, 1, 1),
	[PM800_ID_LDO10] = REG_INIT(LDO10, 1200000, 3300000, 1, 1),
	[PM800_ID_LDO11] = REG_INIT(LDO11, 1200000, 3300000, 1, 1),
	[PM800_ID_LDO12] = REG_INIT(LDO12, 1200000, 3300000, 0, 0),
	[PM800_ID_LDO13] = REG_INIT(LDO13, 1200000, 3300000, 1, 1),
	[PM800_ID_LDO14] = REG_INIT(LDO14, 1200000, 3300000, 0, 0),
	[PM800_ID_LDO15] = REG_INIT(LDO15, 1200000, 3300000, 0, 0),
	[PM800_ID_LDO16] = REG_INIT(LDO16, 1200000, 3300000, 0, 0),
	[PM800_ID_LDO17] = REG_INIT(LDO17, 1200000, 3300000, 0, 0),
	[PM800_ID_LDO18] = REG_INIT(LDO18, 1700000, 3300000, 0, 0),
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
	/* Enable 32K out1 from XO: EXT_32K_IN */
	pm80x_set_bits(chip->base_page, PM800_RTC_MISC2, 0x3, 0x2);
	/* Enable 32K out2 from XO: 32K_CLK for WIFI,PM805 */
	pm80x_set_bits(chip->base_page, PM800_RTC_MISC2,
					(0x3 << 2), (0x2 << 2));

	return 0;
}

static struct pm80x_platform_data pm800_info = {
	.base_page_addr = 0x30,		/* BASE page */
	.power_page_addr = 0x31,	/* POWER */
	.gpadc_page_addr = 0x32,	/* GPADC */
	.test_page_addr = 0x37,		/* TEST */
	.irq_mode = 0,	/* 0: clear IRQ by read */
	.irq_base = IRQ_BOARD_START,

	.num_regulators = ARRAY_SIZE(pm800_regulator_data),
	.regulator	    = pm800_regulator_data,
	.rtc = &pm80x_rtc,
	.pm800_plat_config = pm800_plat_config,
};
/* End Of PM800 */

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

static struct i2c_board_info yellowstone_twsi1_info[] = {
	{
		.type = "88PM80x",
		.addr = 0x34,
		.irq  = IRQ_MMP3_PMIC,
		.platform_data = &pm800_info,
	},
	{
		.type = "fan53555",
		.addr = 0x60,
		.platform_data = &fan53555_pdata,
	},
};

static int motion_sensor_set_power(int on, const char *device_name)
{
	static struct regulator *vdd[3];
	static int is_enabled[3] = {0, 0, 0};
	int device_index = -1;

#if defined(CONFIG_SENSORS_LSM303DLHC_ACC)
	if (!strcmp(device_name, LSM303DLHC_ACC_DEV_NAME))
		device_index = 0;
#endif
#if defined(CONFIG_SENSORS_LSM303DLHC_MAG)
	if (!strcmp(device_name, LSM303DLHC_MAG_DEV_NAME))
		device_index = 1;
#endif
#if defined(CONFIG_SENSORS_L3G4200D_GYR)
	if (!strcmp(device_name, L3G4200D_GYR_DEV_NAME))
		device_index = 2;
#endif

	if ((device_index >= 0) && (device_index <= 2)) {
		if (on && (!is_enabled[device_index])) {
			vdd[device_index] = regulator_get(NULL, "V_2P8");
			if (IS_ERR(vdd[device_index])) {
				vdd[device_index] = NULL;
				return -ENODEV;
			} else {
				regulator_set_voltage(vdd[device_index],
						2800000, 2800000);
				regulator_enable(vdd[device_index]);
				is_enabled[device_index] = 1;
			}
		}
		if ((!on) && is_enabled[device_index]) {
			regulator_disable(vdd[device_index]);
			regulator_put(vdd[device_index]);
			vdd[device_index] = NULL;
			is_enabled[device_index] = 0;
		}
	} else
		return -EPERM;
	return 0;
}

#if defined(CONFIG_SENSORS_LSM303DLHC_ACC)
static struct lsm303dlhc_acc_platform_data lsm303dlhc_acc_data = {
	.poll_interval = 1000,
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
	.poll_interval = 1000,
	.min_interval = 10,
	.h_range = LSM303DLHC_H_2_5G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.set_power = motion_sensor_set_power,
};
#endif

#if defined(CONFIG_SENSORS_L3G4200D_GYR)
static struct l3g4200d_gyr_platform_data l3g4200d_gyr_data = {
	.poll_interval = 1000,
	.min_interval = 10,
	.fs_range = L3G4200D_GYR_FS_2000DPS,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.set_power = motion_sensor_set_power,
};
#endif

static struct i2c_board_info yellowstone_twsi4_info[] = {
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
#if defined(CONFIG_SENSORS_L3G4200D_GYR)
	{
		.type           = L3G4200D_GYR_DEV_NAME,
		.addr           = (0xD2>>1),
		.platform_data  = &l3g4200d_gyr_data,
	},
#endif
};

static int yellowstone_pwm_init(struct device *dev)
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

static struct platform_pwm_backlight_data yellowstone_lcd_backlight_data = {
	/* primary backlight */
	.pwm_id = 2,
	.max_brightness = 100,
	.dft_brightness = 50,
	.pwm_period_ns = 2000000,
	.init = yellowstone_pwm_init,
};

static struct platform_device yellowstone_lcd_backlight_devices = {
	.name = "pwm-backlight",
	.id = 2,
	.dev = {
		.platform_data = &yellowstone_lcd_backlight_data,
	},
};

static struct i2c_board_info yellowstone_twsi5_info[] = {
};

static struct i2c_board_info yellowstone_twsi6_info[] = {
};

static struct pm80x_platform_data pm805_info = {
	.irq_mode = 0,
	.irq_base = IRQ_BOARD_START + PM800_MAX_IRQ,
};

static struct i2c_board_info yellowstone_twsi3_info[] = {
	{
		.type = "88PM80x",
		.addr = 0x38,
		.irq  = gpio_to_irq(mfp_to_gpio(GPIO23_GPIO)),
		.platform_data = &pm805_info,
	},
};

#ifdef CONFIG_SD8XXX_RFKILL
static void mmp3_8787_set_power(unsigned int on)
{
	static struct regulator *vbat_fem;
	static int f_enabled = 0;
	/* VBAT_FEM 3.3v */
	if (on && (!f_enabled)) {
		vbat_fem = regulator_get(NULL, "VBAT_FEM");
		if (IS_ERR(vbat_fem)) {
			vbat_fem = NULL;
			pr_err("get VBAT_FEM failed %s.\n", __func__);
		} else {
			regulator_set_voltage(vbat_fem, 3300000, 3300000);
			regulator_enable(vbat_fem);
			f_enabled = 1;
		}
	}

	if (f_enabled && (!on)) {
		if (vbat_fem) {
			regulator_disable(vbat_fem);
			regulator_put(vbat_fem);
			vbat_fem = NULL;
		}
		f_enabled = 0;
	}
}
#endif

#ifdef CONFIG_MMC_SDHCI_PXAV3
#include <linux/mmc/host.h>
static void yellowstone_sd_signal_1v8(int set)
{
	static struct regulator *v_sdmmc;
	int vol;

	v_sdmmc = regulator_get(NULL, "V_SDMMC");
	if (IS_ERR(v_sdmmc)) {
		printk(KERN_ERR "Failed to get V_SDMMC\n");
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
	.signal_1v8		= yellowstone_sd_signal_1v8,
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc1 = {
	.flags          = PXA_FLAG_CARD_PERMANENT,
	.pm_caps        = MMC_PM_KEEP_POWER | MMC_PM_IRQ_ALWAYS_ON,
	.host_caps      = MMC_CAP_POWER_OFF_CARD,
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
	.gpio                   = mfp_to_gpio(MFP_PIN_GPIO57),
	.enable_high            = 1,
	.enabled_at_boot        = 0,
	.init_data              = &sdio_power_data,
};

static struct platform_device sdio_power_device = {
	.name           = "reg-fixed-voltage",
	.id             = 2,
	.dev = {
		.platform_data = &sdio_power,
	},
};

static void __init yellowstone_init_mmc(void)
{
#ifdef CONFIG_PM_RUNTIME
	int RESETn = mfp_to_gpio(MFP_PIN_GPIO58);
	if (gpio_request(RESETn, "sdio RESETn")) {
		pr_err("Failed to request sdio RESETn gpio\n");
		return;
	}
	gpio_direction_output(RESETn, 1);
	gpio_free(RESETn);
	platform_device_register(&sdio_power_device);
#else
#ifdef CONFIG_SD8XXX_RFKILL
	int WIB_PDn = mfp_to_gpio(GPIO57_GPIO);
	int WIB_RESETn = mfp_to_gpio(GPIO58_GPIO);
	add_sd8x_rfkill_device(WIB_PDn, WIB_RESETn,\
			&mmp3_sdh_platdata_mmc1.pmmc, mmp3_8787_set_power);
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

#ifdef CONFIG_USB_EHCI_PXA_U2H_HSIC
static int mmp3_hsic1_reset(void)
{
	int reset;
	reset = mfp_to_gpio(GPIO96_HSIC_RESET);

	if (gpio_request(reset, "hsic reset")) {
		pr_err("Failed to request hsic reset gpio\n");
		return -EIO;
	}

	gpio_direction_output(reset, 0);
	mdelay(100);
	gpio_direction_output(reset, 1);
	mdelay(50);

	gpio_free(reset);
	return 0;
}

static int mmp3_hsic1_set_vbus(unsigned int vbus)
{
	static struct regulator *v_1p2_hsic;

	printk(KERN_INFO "%s: set %d\n", __func__, vbus);
	if (vbus) {
		if (!v_1p2_hsic) {
			v_1p2_hsic = regulator_get(NULL, "V_1P2_HSIC");
			if (IS_ERR(v_1p2_hsic)) {
				printk(KERN_INFO "V_1P2_HSIC not found\n");
				return -EIO;
			}
			regulator_set_voltage(v_1p2_hsic, 1200000, 1200000);
			regulator_enable(v_1p2_hsic);
			printk(KERN_INFO "%s: enable regulator\n", __func__);
			udelay(2);
		}

		mmp3_hsic1_reset();
	} else {
		if (v_1p2_hsic) {
			regulator_disable(v_1p2_hsic);
			regulator_put(v_1p2_hsic);
			v_1p2_hsic = NULL;
		}
	}

	return 0;
}

static char *mmp3_hsic1_clock_name[] = {
	[0] = "U2OCLK",
	[1] = "HSIC1CLK",
};

static struct mv_usb_platform_data mmp3_hsic1_pdata = {
	.clknum		= 2,
	.clkname	= mmp3_hsic1_clock_name,
	.vbus		= NULL,
	.mode		= MV_USB_MODE_HOST,
	.phy_init	= mmp3_hsic_phy_init,
	.phy_deinit     = mmp3_hsic_phy_deinit,
	.set_vbus	= mmp3_hsic1_set_vbus,
	.private_init	= mmp3_hsic_private_init,
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

#if (defined(CONFIG_SPI_PXA2XX) || defined(CONFIG_SPI_PXA2XX_MODULE)) \
       && defined(CONFIG_NTRIG_SPI)
static int ntrig_set_power(int on)
{
	struct regulator *v_3p3;
	v_3p3 = regulator_get(NULL, "V_3P3");
	if (IS_ERR(v_3p3)) {
		v_3p3 = NULL;
		pr_err("%s: enable V_3P3 for touch fail!\n", __func__);
		return -EIO;
	}
	else {
		if (on) {
			regulator_set_voltage(v_3p3, 3300000, 3300000);
			regulator_enable(v_3p3);
		}
		else {
			regulator_disable(v_3p3);
			v_3p3 = NULL;
		}
		msleep(100);
		regulator_put(v_3p3);
	}
	return 1;
}

static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect = 1,
	.enable_dma = 1,
};

static struct pxa2xx_spi_chip touch_spi_device = {
	.tx_threshold = 1,
	.rx_threshold = 1,
};
static struct ntrig_spi_platform_data ntrig_data = {
	.oe_gpio = mfp_to_gpio(GPIO85_GPIO), /*magic number print from vendor's code*/
	.oe_inverted = 1,/*magic number print from vendor's code*/
	.pwr_gpio = -1,
	.irq_flags = IRQF_DISABLED | IRQF_TRIGGER_RISING,
	.set_power = ntrig_set_power,
};

static struct spi_board_info __initdata ntrig_spi_board_info[] = {
	{
		.modalias = "ntrig_spi",
		.bus_num = 5,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.max_speed_hz = 13000000,
		.platform_data = &ntrig_data,
		.irq = IRQ_GPIO(mfp_to_gpio(GPIO101_GPIO)),
		.controller_data = &touch_spi_device,
	},
};

static int ntrig_gpio_set(void)
{
	int gpio = mfp_to_gpio(GPIO101_GPIO);

	if (gpio_request(gpio, "N-trig irq")) {
			pr_err("gpio %d request failed\n", gpio);
			return -1;
	}
	gpio_direction_input(gpio);
	mdelay(1);
	gpio_free(gpio);
	return 0;
}

static void __init mmp3_init_spi(void)
{

	ntrig_gpio_set();
	mmp3_add_ssp(4);
	mmp3_add_spi(5, &pxa_ssp_master_info);

	if ((spi_register_board_info(ntrig_spi_board_info, ARRAY_SIZE(ntrig_spi_board_info))) != 0)
			pr_err("%s: spi_register_board_info returned error\n", __func__);
}
#else
static inline void mmp3_init_spi(void) {
}
#endif

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

static int yellowstone_board_reset(char mode, const char *cmd)
{
	u8 buf[4];
	u32 ticks;
	/* Reset TWSI1 unit firstly */
	writel(0x4060, reg_icr);
	udelay(500);
	writel(0x60, reg_icr);
	udelay(500);
	/* 1. Disable alarm1 */
	pm800_i2c_write_reg(0x30, 0xD0, 0x80);
	/* 2. Read RTC counter */
	pm800_i2c_read_reg(0x30, 0xD1, buf, 4);
	ticks = (buf[3] << 24) | (buf[2] << 16) |
			(buf[1] << 8) | buf[0];
	/* 3.Calculate alarm expire counter: after 1s here */
	ticks += 1;
	buf[0] = ticks & 0xFF;
	buf[1] = (ticks >> 8) & 0xFF;
	buf[2] = (ticks >> 16) & 0xFF;
	buf[3] = (ticks >> 24) & 0xFF;
	/* 4.Set alarm1 */
	pm800_i2c_write_reg(0x30, 0xD5, buf[0]);
	pm800_i2c_write_reg(0x30, 0xD6, buf[1]);
	pm800_i2c_write_reg(0x30, 0xD7, buf[2]);
	pm800_i2c_write_reg(0x30, 0xD8, buf[3]);
	/* 5.Use XO and enable alarm1 */
	pm800_i2c_write_reg(0x30, 0xD0, 0x81);
	/* 6.Issue SW power down */
	pm800_i2c_write_reg(0x30, 0x0D, 0x20);
	/* Rebooting... */
	return 1;
}

static void yellowstone_poweroff(void)
{
	pm8xxx_system_poweroff();
}

static void __init yellowstone_init(void)
{
	extern int (*board_reset)(char mode, const char *cmd);
	board_reset = yellowstone_board_reset;
	pm_power_off = yellowstone_poweroff;
	mfp_config(ARRAY_AND_SIZE(yellowstone_pin_config));

	/* on-chip devices */
	mmp3_add_uart(3);
	mmp3_add_twsi(1, NULL, ARRAY_AND_SIZE(yellowstone_twsi1_info));
	mmp3_add_twsi(4, NULL, ARRAY_AND_SIZE(yellowstone_twsi4_info));
	mmp3_add_twsi(5, NULL, ARRAY_AND_SIZE(yellowstone_twsi5_info));
	mmp3_add_twsi(6, NULL, ARRAY_AND_SIZE(yellowstone_twsi6_info));

	mmp3_add_keypad(&mmp3_keypad_info);
	mmp3_add_videosram(&mmp3_videosram_info);
#ifdef CONFIG_FB_PXA168
	yellowstone_add_lcd_mipi();
	mmp3_add_tv_out();
#endif

#ifdef CONFIG_UIO_HDMI
	mmp3_add_hdmi(&mmp3_hdmi_info);
#endif

	/* backlight */
	mmp3_add_pwm(3);
	platform_device_register(&yellowstone_lcd_backlight_devices);

	mmp3_add_thermal();
#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem();
#endif

#ifdef CONFIG_UIO_VMETA
	mmp_init_vmeta();
#endif
#ifdef CONFIG_MMC_SDHCI_PXAV3
	yellowstone_init_mmc();
#endif /* CONFIG_MMC_SDHCI_PXAV3 */
	mmp3_init_spi();

	platform_device_register(&mmp3_device_rtc);

#if defined(CONFIG_TOUCHSCREEN_VNC)
	platform_device_register(&mmp3_device_vnc_touch);
#endif

	mmp3_add_twsi(3, NULL, ARRAY_AND_SIZE(yellowstone_twsi3_info));

	/* audio sspa support */
	mmp3_add_sspa(1);
	mmp3_add_sspa(2);
	mmp3_add_audiosram(&mmp3_audiosram_info);

#if defined(CONFIG_VIDEO_MV)
	platform_device_register(&yellowstone_ov5642);
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

#ifdef CONFIG_USB_EHCI_PXA_U2H_HSIC
	mmp3_hsic1_device.dev.platform_data = (void *)&mmp3_hsic1_pdata;
	platform_device_register(&mmp3_hsic1_device);
#endif
	/* If we have a full configuration then disable any regulators
	 * which are not in use or always_on. */
	regulator_has_full_constraints();
}

MACHINE_START(YELLOWSTONE, "YellowStone")
	.map_io		= mmp_map_io,
	.nr_irqs	= YELLOWSTONE_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.reserve	= mmp3_reserve,
	.init_machine	= yellowstone_init,
MACHINE_END
