/*
 *  linux/arch/arm/mach-mmp/thunderstonem.c
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
#include <linux/gpio_keys.h>
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
#ifdef CONFIG_SENSORS_CM3218
#include <linux/cm3218.h>
#endif
#if defined (CONFIG_INPUT_KXT_9)
#include <linux/kxt_9.h>
#endif
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
#if defined(CONFIG_TOUCHSCREEN_SIS9200)
#include <linux/i2c/sis_i2c.h>
#endif
#if defined(CONFIG_BATTERY_BQ27x00)
#include <linux/power/bq27x00_battery.h>
#endif

#include <plat/usb.h>
#include <media/soc_camera.h>
#include <mach/sram.h>
#include <plat/pmem.h>
#include <mach/camera.h>
#include <mach/isp_dev.h>
#include "common.h"
#include "onboard.h"

#define THUNDERSTONEM_NR_IRQS		(IRQ_BOARD_START + 64)

static int thunderstonem_board_version;

static unsigned long thunderstonem_pin_config[] __initdata = {
	/*board version*/
	GPIO16_BOARD_VERS0,
	GPIO17_BOARD_VERS1,
	GPIO18_BOARD_VERS2,

	/* UART3 */
	GPIO51_UART3_RXD,
	GPIO52_UART3_TXD,

	/* GPS - UART2  on GPIO 47,48 */
	GPIO47_UART2_RXD,
	GPIO48_UART2_TXD,

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

	/*Thunderstone gpio keypad*/
	GPIO2_GPIO,
	GPIO20_GPIO,
	GPIO22_GPIO,

#if defined(CONFIG_TOUCHSCREEN_SIS9200)
	GPIO86_GPIO,
	GPIO83_GPIO,
	GPIO101_GPIO,
#endif

	/* SSPA1 (I2S) */
	GPIO23_GPIO,
	GPIO24_I2S_SYSCLK,
	GPIO25_I2S_BITCLK,
	GPIO26_I2S_SYNC,
	GPIO27_I2S_DATA_OUT,
	GPIO28_I2S_SDATA_IN,

	/* camera OV9726 */
	GPIO1_GPIO,
	GPIO68_GPIO,
	GPIO70_GPIO,

	/* DFI */
	GPIO168_DFI_D0,
	GPIO167_DFI_D1,
	GPIO166_DFI_D2,
	GPIO165_DFI_D3,
	GPIO107_DFI_D4,
	GPIO106_DFI_D5,
	GPIO105_DFI_D6,
	GPIO104_DFI_D7,
	GPIO143_ND_nCS0,
	GPIO144_ND_nCS1,
	GPIO147_ND_nWE,
	GPIO148_ND_nRE,
	GPIO150_ND_ALE,
	GPIO112_ND_RDY0,
	GPIO160_ND_RDY1,

	PMIC_PMIC_INT | MFP_LPM_EDGE_FALL,

	/* LCD_EN */
	GPIO84_GPIO,
	/* LCD_STBY_N */
	GPIO19_GPIO,
	/* LCD_RST_N */
	GPIO128_LCD_RST,

	/* OTG vbus enable signal */
	GPIO82_VBUS_EN,

	/* HSIC1 reset pin*/
	GPIO96_HSIC_RESET,

	/* HDMI */
	GPIO54_HDMI_CEC,
	GPIO59_HDMI_DET,
	GPIO94_HDMI_LS_OE,
	GPIO95_HDMI_CT_HPD,

	/* SSP4 */
	GPIO78_SSP_CLK,
	GPIO79_SSP_FRM,
	GPIO80_SSP_TXD,
	GPIO81_SSP_RXD,
	GPIO101_GPIO, /* TS INT*/
	GPIO85_GPIO, /* TS_IO_EN */

	/* Battery */
#ifdef CONFIG_BATTERY_BQ27X00_PLATFORM
	GPIO171_GPIO,
#endif
	/* G Sensor KXT_9 */
#ifdef CONFIG_INPUT_KXT_9
	GPIO126_GPIO,
#endif

#ifdef CONFIG_GPS_88L1000
	/* GPS */
	GPIO14_GPS_ON,
	GPIO15_GPS_RST,
#endif

#ifdef CONFIG_SENSORS_CM3218
	GPIO93_GPIO,
#endif
	/* 3V3_EN */
	GPIO87_GPIO,
	/* 5V_ON */
	GPIO88_GPIO,

};

static unsigned long mmc1_pin_config[] __initdata = {
	GPIO131_MMC1_DAT3 | MFP_PULL_HIGH,
	GPIO132_MMC1_DAT2 | MFP_PULL_HIGH,
	GPIO133_MMC1_DAT1 | MFP_PULL_HIGH,
	GPIO134_MMC1_DAT0 | MFP_PULL_HIGH,
	GPIO136_MMC1_CMD | MFP_PULL_HIGH,
	GPIO135_MMC1_CLK,
	GPIO140_MMC1_CD | MFP_PULL_HIGH,
	GPIO141_MMC1_WP | MFP_PULL_HIGH,
};

/* MMC2 is used for WIB card */
static unsigned long mmc2_pin_config[] __initdata = {
	GPIO37_MMC2_DAT3 | MFP_PULL_HIGH,
	GPIO38_MMC2_DAT2 | MFP_PULL_HIGH,
	GPIO39_MMC2_DAT1 | MFP_PULL_HIGH,
	GPIO40_MMC2_DAT0 | MFP_PULL_HIGH,
	GPIO41_MMC2_CMD | MFP_PULL_HIGH,
	GPIO42_MMC2_CLK,

	/* GPIO used for power */
	GPIO58_GPIO | MFP_LPM_DRIVE_HIGH, /* WIFI_RST_N */
	GPIO57_GPIO | MFP_LPM_DRIVE_LOW, /* WIFI_PD_N */

	/* GPIO for wake */
	GPIO55_GPIO, /* WL_BT_WAKE */
	GPIO56_GPIO | MFP_LPM_EDGE_FALL | MFP_PULL_HIGH, /* WLAN_WAKE (8787->soc) */
};

static unsigned long mmc3_pin_config[] __initdata = {
	GPIO108_MMC3_DAT7 | MFP_PULL_HIGH,
	GPIO109_MMC3_DAT6 | MFP_PULL_HIGH,
	GPIO161_MMC3_DAT5 | MFP_PULL_HIGH,
	GPIO163_MMC3_DAT4 | MFP_PULL_HIGH,
	GPIO111_MMC3_DAT3 | MFP_PULL_HIGH,
	GPIO110_MMC3_DAT2 | MFP_PULL_HIGH,
	GPIO162_MMC3_DAT1 | MFP_PULL_HIGH,
	GPIO164_MMC3_DAT0 | MFP_PULL_HIGH,
	GPIO145_MMC3_CMD | MFP_PULL_HIGH,
	GPIO146_MMC3_CLK,
	/* MMC3_nRST */
	GPIO149_GPIO | MFP_LPM_DRIVE_HIGH,
};

/* PMIC Regulator 88PM800 */
/* BUCK power supplies: BUCK[1..5] */
static struct regulator_consumer_supply regulator_supplies_BUCK1[] = {
	REGULATOR_SUPPLY("V_PMIC_BUCK1", NULL),
};
static struct regulator_consumer_supply regulator_supplies_BUCK2[] = {
	REGULATOR_SUPPLY("V_DDR3", NULL),
};
static struct regulator_consumer_supply regulator_supplies_BUCK3[] = {
	REGULATOR_SUPPLY("V_SD3", NULL),
};
static struct regulator_consumer_supply regulator_supplies_BUCK4[] = {
	REGULATOR_SUPPLY("V_1P8", NULL),
};
static struct regulator_consumer_supply regulator_supplies_BUCK5[] = {
	REGULATOR_SUPPLY("V_SD5", NULL),
};
/* LDO power supplies: LDO[1..19] */
static struct regulator_consumer_supply regulator_supplies_LDO1[] = {
	REGULATOR_SUPPLY("V_LDO1", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO2[] = {
	REGULATOR_SUPPLY("V_LDO2", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO3[] = {
	REGULATOR_SUPPLY("V_1P2_MIPI", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO4[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-pxa.2"),
	REGULATOR_SUPPLY("V_2P8", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO5[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-pxa.0"),
	REGULATOR_SUPPLY("V_3P3", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO6[] = {
	REGULATOR_SUPPLY("V_PMIC", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO7[] = {
	REGULATOR_SUPPLY("V_WIFI_3V3", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO8[] = {
	REGULATOR_SUPPLY("V_1P2_HSIC", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO9[] = {
	REGULATOR_SUPPLY("V_1P8_USBFE", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO10[] = {
	REGULATOR_SUPPLY("V_LCD", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO11[] = {
	REGULATOR_SUPPLY("V_1P2_CODEC", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO12[] = {
	REGULATOR_SUPPLY("V_LDO12", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO13[] = {
	REGULATOR_SUPPLY("V_LDO13", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO14[] = {
	REGULATOR_SUPPLY("V_MIC_BIAS", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO15[] = {
	REGULATOR_SUPPLY("V_MOTOR", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO16[] = {
	REGULATOR_SUPPLY("VBAT_FEM", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO17[] = {
	REGULATOR_SUPPLY("V_BB", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO18[] = {
	REGULATOR_SUPPLY("AVDD_CAM_2P8V", NULL),
};
static struct regulator_consumer_supply regulator_supplies_LDO19[] = {
	REGULATOR_SUPPLY("AVDD_LVDS", NULL),
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

#define REG_INIT(_name, _min, _max, _always, _boot, _suspend) \
{								\
	.constraints = {					\
		.name		= __stringify(_name),		\
		.min_uV		= _min,				\
		.max_uV		= _max,				\
		.always_on	= _always,			\
		.boot_on	= _boot,			\
		.state_mem	= {				\
			.disabled = _suspend,			\
		},						\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE	\
				| REGULATOR_CHANGE_STATUS,	\
	},							\
	.num_consumer_supplies = ARRAY_SIZE(regulator_supplies_##_name), \
	.consumer_supplies = regulator_supplies_##_name,	\
	.driver_data = &regulator_index[PM800_ID_##_name],	\
}

/* Domain		Range(V)	Imax(mA)
 * BUCK1:		0.6 - 1.8	3000
 * BUCK[2..5]:	0.6 - 3.3	1200
 * LDO1:		0.6 - 1.5	200
 * LDO2:		1.7 - 2.8	10
 * LDO[3..17]:	1.2 - 3.3	300
 * LDO[18..19]:	1.7 - 3.3	200
 * */
static struct regulator_init_data pm800_regulator_data[] = {
	/* BUCK power supplies: BUCK[1..5] */
	[PM800_ID_BUCK1] = REG_INIT(BUCK1, 600000, 1800000, 1, 1, 0),
	[PM800_ID_BUCK2] = REG_INIT(BUCK2, 600000, 3300000, 1, 1, 0),
	[PM800_ID_BUCK3] = REG_INIT(BUCK3, 600000, 3300000, 1, 1, 0),
	[PM800_ID_BUCK4] = REG_INIT(BUCK4, 600000, 3300000, 1, 1, 0),
	[PM800_ID_BUCK5] = REG_INIT(BUCK5, 600000, 3300000, 0, 0, 0),
	/* LDO power supplies: LDO[1..19] */
	[PM800_ID_LDO1]  = REG_INIT(LDO1,   600000, 1500000, 0, 0, 0),
	[PM800_ID_LDO2]  = REG_INIT(LDO2,  1700000, 2800000, 0, 0, 0),
	[PM800_ID_LDO3]  = REG_INIT(LDO3,  1200000, 3300000, 1, 1, 0),
	[PM800_ID_LDO4]  = REG_INIT(LDO4,  1200000, 3300000, 1, 1, 0),
	[PM800_ID_LDO5]  = REG_INIT(LDO5,  1200000, 3300000, 1, 1, 0),
	[PM800_ID_LDO6]  = REG_INIT(LDO6,  1200000, 3300000, 1, 1, 0),
	[PM800_ID_LDO7]  = REG_INIT(LDO7,  1200000, 3300000, 0, 0, 0),
	[PM800_ID_LDO8]  = REG_INIT(LDO8,  1200000, 3300000, 0, 0, 0),
	[PM800_ID_LDO9]  = REG_INIT(LDO9,  1200000, 3300000, 0, 0, 0),
	[PM800_ID_LDO10] = REG_INIT(LDO10, 1200000, 3300000, 1, 1, 0),
	[PM800_ID_LDO11] = REG_INIT(LDO11, 1200000, 3300000, 1, 1, 0),
	[PM800_ID_LDO12] = REG_INIT(LDO12, 1200000, 3300000, 0, 0, 0),
	[PM800_ID_LDO13] = REG_INIT(LDO13, 1200000, 3300000, 1, 1, 0),
	[PM800_ID_LDO14] = REG_INIT(LDO14, 1200000, 3300000, 1, 1, 0),
	[PM800_ID_LDO15] = REG_INIT(LDO15, 1200000, 3300000, 0, 0, 0),
	[PM800_ID_LDO16] = REG_INIT(LDO16, 1200000, 3300000, 0, 0, 0),
	[PM800_ID_LDO17] = REG_INIT(LDO17, 1200000, 3300000, 0, 0, 0),
	[PM800_ID_LDO18] = REG_INIT(LDO18, 1700000, 3300000, 0, 0, 0),
	[PM800_ID_LDO19] = REG_INIT(LDO19, 1700000, 3300000, 1, 1, 0),
};

static struct pm80x_rtc_pdata pm80x_rtc = {
	.rtc_wakeup	= 0,
};

static int pm800_plat_config(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata)
{
	if (!chip || !pdata ||
		chip->id != CHIP_PM800 || !chip->base_page) {
		pr_err("%s:chip or pdata is not availiable!\n", __func__);
		return -EINVAL;
	}

	/* Disable watch dog */
	if (chip->chip800_version <= PM800_CHIP_B0)
		pm80x_set_bits(chip->base_page, PM800_WAKEUP2, (0xF << 4), 0);

	/* Select XO 32KHZ(USE_XO)
	 * Force all CLK32K_1/2/3 buffers to use the XO 32KHZ */
	pm80x_set_bits(chip->base_page, PM800_RTC_CONTROL, (1 << 7), (1 << 7));
	/* Enable 32K out1 from XO: EXT_32K_IN */
	pm80x_set_bits(chip->base_page, PM800_RTC_MISC2, 0x3, 0x2);
	/* Enable 32K out2 from XO: 32K_CLK for WIFI,PM805 */
	pm80x_set_bits(chip->base_page, PM800_RTC_MISC2,
					(0x3 << 2), (0x2 << 2));
	/* Enable Low jitter for Audio 32K clk */
	pm80x_reg_write(chip->base_page, 0x21, 0x20);

	/* Modified for Audio codec 32K clk enable */
	pm80x_set_bits(chip->base_page, PM800_RTC_MISC2,
			0x3f, ((0x2 << 4) | (0x2 << 2) | 0x2));

	return 0;
}

static struct pm80x_headset_pdata pm80x_headset = {
	.gpio	= 3,			/* GPIO 3 */
	.gpio_ctl = 0x31,		/* PM800 GPIO 3 */
	.gpio_enable_irq = (1 << 3),
	.gpio_set_mask = 0x0f,
	.gpio_set_val = 0x0,
	.gpio_val_bit = (1 << 4),
};

static struct pm80x_platform_data pm800_info = {
	.headset_flag = 1,		/* For Headset detect signal invert */
	.headset = &pm80x_headset,
	.base_page_addr = 0x30,		/* BASE page */
	.power_page_addr = 0x31,	/* POWER */
	.gpadc_page_addr = 0x32,	/* GPADC */
	.test_page_addr = 0x37,		/* TEST */
	.irq_mode = 0,	/* 0: clear IRQ by read */
	.irq_base = IRQ_BOARD_START,

	.num_regulators = ARRAY_SIZE(pm800_regulator_data),
	.regulator = pm800_regulator_data,
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
		.max_uV = 1410000,
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

/* V_5V */
static struct regulator_consumer_supply v_5v_power_supplies[] = {
	REGULATOR_SUPPLY("V_5V", NULL),
};

static struct regulator_init_data v_5v_fixed_power_init_data = {
	.constraints = {
		.boot_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(v_5v_power_supplies),
	.consumer_supplies = v_5v_power_supplies,
};

static struct fixed_voltage_config v_5v_fixed_power_config = {
	.supply_name = "5V_PWR",
	.microvolts = 5000000,
	.gpio = mfp_to_gpio(GPIO88_GPIO),
	.enable_high = 1,
	.startup_delay = 50,	/* in microseconds */
	.init_data = &v_5v_fixed_power_init_data,
};

static struct platform_device v_5v_fixed_power = {
	.name = "reg-fixed-voltage",
	.id = 0,
	.dev = {
		.platform_data = &v_5v_fixed_power_config,
	},
};

/* V_3V3 */
static struct regulator_consumer_supply v_3v3_power_supplies[] = {
	REGULATOR_SUPPLY("V_3V3", NULL),
};

static struct regulator_init_data v_3v3_fixed_power_init_data = {
	.constraints = {
		.boot_on = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(v_3v3_power_supplies),
	.consumer_supplies = v_3v3_power_supplies,
};

static struct fixed_voltage_config v_3v3_fixed_power_config = {
	.supply_name = "3V3_PWR",
	.microvolts = 3300000,
	.gpio = mfp_to_gpio(GPIO87_GPIO),
	.enable_high = 1,
	.startup_delay = 50,	/* in microseconds */
	.init_data = &v_3v3_fixed_power_init_data,
};

static struct platform_device v_3v3_fixed_power = {
	.name = "reg-fixed-voltage",
	.id = 1,
	.dev = {
		.platform_data = &v_3v3_fixed_power_config,
	},
};

static struct platform_device *fixed_rdevs[] __initdata = {
	&v_5v_fixed_power,
	&v_3v3_fixed_power,
};

static void thunderstonem_fixed_power_init(void)
{
	platform_add_devices(fixed_rdevs, ARRAY_SIZE(fixed_rdevs));
}

static struct i2c_board_info thunderstonem_twsi1_info[] = {
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
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x = 1,
	//this value specific to thunderstonem
	.negate_y = 0,
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
	.negate_z = 1,
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

#if defined(CONFIG_BATTERY_BQ27X00_PLATFORM)
struct bq27000_platform_data bq27000_hdq_data = {
	.name = "bq27000-battery",
	/* TODO
	 * Need support low level access for HDQ
	 */
	.read = NULL,
	.irq  = gpio_to_irq(mfp_to_gpio(GPIO171_GPIO)),
	};

struct platform_device thunderstonem_hdq_bq27000 = {
	.name = "bq27000-battery",
	.id     = 0,
	.dev = {
		.platform_data = &bq27000_hdq_data,
	},
};
#endif

#if defined(CONFIG_INPUT_KXT_9)
struct KXT_9_platform_data zoom2_KXT_9_data = {
	.min_interval   = 1,
	.poll_interval  = 1000,

	.g_range        = KXT_9_G_8G,
	.shift_adj      = SHIFT_ADJ_2G,

	.axis_map_x     = 0,
	.axis_map_y     = 1,
	.axis_map_z     = 2,

	.negate_x       = 0,
	.negate_y       = 0,
	.negate_z       = 0,

	.data_odr_init          = ODR12_5F,
	.ctrl_reg1_init         = KXT_9_G_8G | RES_12BIT | TDTE | WUFE | TPE,
	.int_ctrl_init          = KXT_9_IEN | KXT_9_IEA | KXT_9_IEL,
	.tilt_timer_init        = 0x03,
	.engine_odr_init        = OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init         = 0x16,
	.wuf_thresh_init        = 0x28,
	.tdt_timer_init         = 0x78,
	.tdt_h_thresh_init      = 0xFF,
	.tdt_l_thresh_init      = 0x14,
	.tdt_tap_timer_init     = 0x53,
	.tdt_total_timer_init   = 0x24,
	.tdt_latency_timer_init = 0x10,
	.tdt_window_timer_init  = 0xA0,

	.gpio = mfp_to_gpio(GPIO126_GPIO),
};
#endif

static struct i2c_board_info thunderstonem_twsi4_info[] = {
#if defined(CONFIG_INPUT_KXT_9)
	{
		.type		= "KXT_9",
		.addr		= 0x0F,
		.platform_data	= &zoom2_KXT_9_data,
	},
#endif
#if defined(CONFIG_SENSORS_LSM303DLHC_ACC)
	{
		.type		= LSM303DLHC_ACC_DEV_NAME,
		.addr		= (0x32>>1),
		.platform_data	= &lsm303dlhc_acc_data,
	},
#endif
#if defined(CONFIG_SENSORS_LSM303DLHC_MAG)
	{
		.type		= LSM303DLHC_MAG_DEV_NAME,
		.addr		= (0x3C>>1),
		.platform_data	= &lsm303dlhc_mag_data,
	},
#endif
#if defined(CONFIG_SENSORS_L3G4200D_GYR)
	{
		.type		= L3G4200D_GYR_DEV_NAME,
		.addr		= (0xD0>>1),
		.platform_data	= &l3g4200d_gyr_data,
	},
#endif
};

static int thunderstonem_pwm_init(struct device *dev)
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

static struct platform_pwm_backlight_data thunderstonem_lcd_backlight_data = {
	/* primary backlight */
	.pwm_id = 2,
	.max_brightness = 100,
	.dft_brightness = 50,
	.pwm_period_ns = 2000000,
	.init = thunderstonem_pwm_init,
};

static struct platform_device thunderstonem_lcd_backlight_devices = {
	.name = "pwm-backlight",
	.id = 2,
	.dev = {
		.platform_data = &thunderstonem_lcd_backlight_data,
	},
};

#if defined(CONFIG_TOUCHSCREEN_SIS9200)
/* TP_RST    : GPIO86 */
/* TP_VEN    : GPIO83 */
/* TOUCH_INT : GPIO101 */
static int sis9200_touch_io_power_onoff(int on)
{
	unsigned int touch_reset;
	unsigned int tp_ven;
	touch_reset = mfp_to_gpio(MFP_PIN_GPIO86);
	tp_ven = mfp_to_gpio(MFP_PIN_GPIO83);
	if (gpio_request(touch_reset, "TOUCH_RESET")) {
		pr_err("Failed to request GPIO for TOUCH pin!\n");
		goto out;
	}
	if (gpio_request(tp_ven, "TP_VEN")) {
		pr_err("Failed to request GPIO for TP_VEN pin!\n");
		goto out;
	}

	if (on) {
		/* Turn on SiS Chip*/
		gpio_direction_output(touch_reset, 0);
		printk(KERN_INFO "[MSI TOUCH] SiS Touch Reset Low\n");
		gpio_direction_output(tp_ven, 1);
		msleep(15);
		gpio_direction_output(touch_reset, 1);
		printk(KERN_INFO "[MSI TOUCH] SiS Touch Reset HI\n");
	}
	else {
		/* Turn off SiS Chip*/
		gpio_direction_output(tp_ven, 0);
		msleep(10);
		gpio_direction_output(touch_reset, 0);
		printk(KERN_INFO "[MSI TOUCH] SiS Touch Reset Low\n");
		gpio_direction_output(touch_reset, 0);
		printk(KERN_INFO "[MSI TOUCH] SiS Touch Power off\n");
	}
	gpio_free(touch_reset);
	gpio_free(tp_ven);
out:
	msleep(100);
	return 0;
}

static struct sis_i2c_platform_data sis9200_touch_data = {
	.power = sis9200_touch_io_power_onoff,
	/*
	TODO Adding other initial value
	*/
};
#endif

#if defined(CONFIG_BATTERY_BQ27X00_I2C)
struct bq27000_platform_data bq27x00_i2c_data = {
	.name = "bq27500",
	.read = NULL,
};
#endif

static struct i2c_board_info thunderstonem_twsi5_info[] = {
#if defined(CONFIG_TOUCHSCREEN_SIS9200)
	{
		.type = SIS_I2C_NAME,
		.addr = 0x5c,
		.irq  = gpio_to_irq(mfp_to_gpio(GPIO101_GPIO)),
		.platform_data  = &sis9200_touch_data,
	},
#endif

#if defined(CONFIG_BATTERY_BQ27X00_I2C)
	{
		.type           = "bq27500",
		.addr           = (0xAA>>1),
		.platform_data  = &bq27x00_i2c_data,
	},
#endif
};

#ifdef CONFIG_SENSORS_CM3218
static int __capella_cm3218_power(int on, uint8_t val){
		return 0;
}

static struct cm3218_platform_data cm3218_pdata = {
		.intr = GPIO93_GPIO,
		.levels = {
				0x0A, 0xA0, 0xE1, 0x140, 0x280,
				0x500, 0xA28, 0x16A8, 0x1F40, 0x2800
		},
		.power = __capella_cm3218_power,
		.ALS_slave_address = CM3218_ALS_cmd,
		.check_interrupt_add = CM3218_check_INI,
		.is_cmd = CM3218_ALS_SM_2 | CM3218_ALS_IT_250ms |
			CM3218_ALS_PERS_1 | CM3218_ALS_RES_1,
};
#endif

static struct i2c_board_info thunderstonem_twsi6_info[] = {
#ifdef CONFIG_SENSORS_CM3218
	        {
			.type           = CM3218_I2C_NAME,
			.addr           = CM3218_ALS_cmd,
			.irq            = gpio_to_irq(mfp_to_gpio(GPIO93_GPIO)),
			.platform_data  = &cm3218_pdata,
	        },
#endif
};

static struct pm80x_platform_data pm805_info = {
	.irq_mode = 0,
	.irq_base = IRQ_BOARD_START + PM800_MAX_IRQ,
	.headset = &pm80x_headset,
};

static struct i2c_board_info thunderstonem_twsi3_info[] = {
	{
		.type = "88PM80x",
		.addr = 0x39,
		.irq  = gpio_to_irq(mfp_to_gpio(GPIO23_GPIO)),
		.platform_data = &pm805_info,
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
	 * 1. WIFI_1V8 <-- V_1P8
	 * 2. WIFI_3V3 <-- V_WIFI_3V3
	 */

	static struct regulator *wifi_3v3, *wifi_1v8;
	static int f_enabled = 0;

	if (!wifi_1v8) {
		wifi_1v8 = regulator_get(NULL, "V_1P8");
		if (IS_ERR(wifi_1v8)) {
			wifi_1v8 = NULL;
			printk(KERN_ERR"get wifi_1v8 failed %s %d \n", __func__, __LINE__);
			return;
		}
	}
	if (!wifi_3v3) {
		wifi_3v3 = regulator_get(NULL, "V_WIFI_3V3");
		if (IS_ERR(wifi_3v3)) {
			wifi_3v3 = NULL;
			printk(KERN_ERR"get wifi_3v3 failed %s %d \n", __func__, __LINE__);
			return;
		}
	}

	if (on && (!f_enabled)) {
		regulator_set_voltage(wifi_1v8, 1800000, 1800000);
		regulator_enable(wifi_1v8);
		regulator_set_voltage(wifi_3v3, 3300000, 3300000);
		regulator_enable(wifi_3v3);
		f_enabled = 1;
		mfp_config(ARRAY_AND_SIZE(wifi_pin_config_on));
	}

	if (f_enabled && (!on)) {
		mfp_config(ARRAY_AND_SIZE(wifi_pin_config_off));
		regulator_disable(wifi_3v3);
		regulator_disable(wifi_1v8);
		f_enabled = 0;
	}
}
#endif


#include <linux/mmc/host.h>
static void tsm_sd_signal_1v8(int set)
{
	static struct regulator *v_sdmmc;
	int vol;

	v_sdmmc = regulator_get(NULL, "V_LDO13");
	if (IS_ERR(v_sdmmc)) {
		printk(KERN_ERR "Failed to get V_LDO13\n");
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
	.flags			= PXA_FLAG_ENABLE_CLOCK_GATING,
	.host_caps_disable	= MMC_CAP_UHS_SDR104 | MMC_CAP_UHS_SDR50,
	.signal_1v8		= tsm_sd_signal_1v8,
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc1 = {
	.flags	  = PXA_FLAG_CARD_PERMANENT |
			PXA_FLAG_WAKEUP_HOST,
	.pm_caps  = MMC_PM_KEEP_POWER,
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc2 = {
	.flags		= PXA_FLAG_SD_8_BIT_CAPABLE_SLOT |
				PXA_FLAG_CARD_PERMANENT |
				PXA_FLAG_ENABLE_CLOCK_GATING,
	.host_caps	= MMC_CAP_1_8V_DDR,
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


static void __init thunderstonem_init_mmc(void)
{
	int emmc_reset_n = mfp_to_gpio(GPIO149_GPIO);
	int wlan_wake = mfp_to_gpio(GPIO56_GPIO);

#ifdef CONFIG_SD8XXX_RFKILL
	int WIB_PDn = mfp_to_gpio(GPIO57_GPIO);
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
	static struct regulator *v_5v;
	int vbus_en = mfp_to_gpio(GPIO82_VBUS_EN);
	static bool enabled;
	bool on = !!vbus;

	pr_info("OTG vbus [%s]\n", on ? "on" : "off");

	if (!v_5v) {
		v_5v = regulator_get(NULL, "V_5V");
		if (IS_ERR(v_5v)) {
			pr_err("%s: Failed to get V_5V\n", __func__);
			v_5v = NULL;
			return -1;
		}
	}
	if (gpio_request(vbus_en, "OTG VBUS Enable")) {
		pr_err("%s: Failed to request gpio#%d\n", __func__, vbus_en);
		return -1;
	}
	/* Enable/disbale V_5V */
	if (on != enabled) {
		if (on)
			regulator_enable(v_5v);
		else
			regulator_disable(v_5v);
		enabled = on;
	}
	/* Enable/disable VBUS_EN */
	gpio_direction_output(vbus_en, on);
	gpio_free(vbus_en);
	mdelay(10);

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
#endif


#ifdef CONFIG_UIO_HDMI
static int hdmi_power(int on)
{
	static struct regulator *v_5v;
	int hdmi_ls_oe = mfp_to_gpio(GPIO94_HDMI_LS_OE);
	int hdmi_ct_hpd = mfp_to_gpio(GPIO95_HDMI_CT_HPD);
	int ret = 0;

	if (!v_5v) {
		v_5v = regulator_get(NULL, "V_5V");
		if (IS_ERR(v_5v)) {
			pr_err("%s: Failed to get V_5V\n", __func__);
			v_5v = NULL;
			return -1;
		}
	}
	if (gpio_request(hdmi_ls_oe, "hdmi_ls_oe")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d.\n", hdmi_ls_oe);
		return -1;
	}
	if (gpio_request(hdmi_ct_hpd, "hdmi_ct_hpd")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d.\n", hdmi_ct_hpd);
		ret = -1;
		goto out;
	}
	if (on) {
		regulator_enable(v_5v);
		gpio_direction_output(hdmi_ls_oe, 1);
		gpio_direction_output(hdmi_ct_hpd, 1);
	} else {
		gpio_direction_output(hdmi_ct_hpd, 0);
		gpio_direction_output(hdmi_ls_oe, 0);
		regulator_disable(v_5v);
	}

	gpio_free(hdmi_ct_hpd);

out:
	gpio_free(hdmi_ls_oe);
	return ret;
}

static struct uio_hdmi_platform_data mmp3_hdmi_info __initdata = {
	.sspa_reg_base = 0xD42A0C00,
	/* Fix me: gpio 59 lpm pull ? */
	.gpio = mfp_to_gpio(GPIO59_HDMI_DET),
	.edid_bus_num = 6,
	.hdmi_v5p_power = &hdmi_power,
	.hpd_val = 0x8000000,
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

/* Only for reboot routine */
extern int __raw_i2c_bus_reset(u8 bus_num);
extern int __raw_i2c_write_reg(u8 bus_num, u8 addr, u8 reg, u8 val);
extern int __raw_i2c_read_reg(u8 bus_num, u8 addr, u8 reg, u8 *buf, int len);
static int pm800_i2c_write_reg(u8 addr, u8 reg, u8 val)
{
	return __raw_i2c_write_reg(1, addr, reg, val);
}

static int pm800_i2c_read_reg(u8 addr, u8 reg, u8 *buf, int len)
{
	return __raw_i2c_read_reg(1, addr, reg, buf, len);
}

static int thunderstonem_board_reset(char mode, const char *cmd)
{
	u8 buf;
	/* Reset TWSI1 unit firstly */
	__raw_i2c_bus_reset(1);
	/* 1.Enable FAULT_WU_EN */
	pm800_i2c_read_reg(0x30, 0xE7, &buf, 1);
	buf |= (1 << 2);
	pm800_i2c_write_reg(0x30, 0xE7, buf);
	/* 2.Set FAULT_WU */
	buf |= (1 << 3);
	pm800_i2c_write_reg(0x30, 0xE7, buf);
	/* 3.Issue SW power down */
	pm800_i2c_write_reg(0x30, 0x0D, 0x20);
	mdelay(50);
	/* Rebooting... */
	return 1;
}

static void thunderstonem_poweroff(void)
{
	pm8xxx_system_poweroff();
}

/* Thunderstone GPIO Keyboard */
#define INIT_KEY(_code, _gpio, _active_low, _desc)      \
	{                                       \
		.code       = KEY_##_code,              \
		.gpio       = _gpio,                    \
		.active_low = _active_low,              \
		.desc       = _desc,                    \
		.type       = EV_KEY,                   \
		.wakeup     = 0,                        \
		.debounce_interval     = 0,                     \
	}

static struct gpio_keys_button gpio_keys_buttons[] = {
	INIT_KEY(VOLUMEUP, mfp_to_gpio(GPIO20_GPIO), 1, "VolumeUp button"),
	INIT_KEY(VOLUMEDOWN, mfp_to_gpio(GPIO22_GPIO), 1, "VolumeDown button"),
	INIT_KEY(HOME, mfp_to_gpio(GPIO2_GPIO), 1, "Home button"),
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

static int thunderstonem_get_board_version(void)
{
	int gpio_vers0 = mfp_to_gpio(GPIO16_BOARD_VERS0);
	int gpio_vers1 = mfp_to_gpio(GPIO17_BOARD_VERS1);
	int gpio_vers2 = mfp_to_gpio(GPIO18_BOARD_VERS2);

	if (gpio_request(gpio_vers0, "vers0")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_vers0);
		return -1;
	}

	if (gpio_request(gpio_vers1, "vers1")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_vers1);
		gpio_free(gpio_vers0);
		return -1;
	}

	if (gpio_request(gpio_vers2, "vers2")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_vers2);
		gpio_free(gpio_vers0);
		gpio_free(gpio_vers1);
		return -1;
	}

	gpio_direction_input(gpio_vers0);
	gpio_direction_input(gpio_vers1);
	gpio_direction_input(gpio_vers2);

	thunderstonem_board_version = ((!!gpio_get_value(gpio_vers2))<<2) |
		((!!gpio_get_value(gpio_vers1))<<1) |
		((!!gpio_get_value(gpio_vers0))<<0);

	thunderstonem_board_version = thunderstonem_board_version & 0x07;

	gpio_free(gpio_vers0);
	gpio_free(gpio_vers1);
	gpio_free(gpio_vers2);

	return 0;
}

#if 0
static inline int board_is_mmp3_thunderstonem_SA(void){
	return (machine_is_thunderstonem() && (0x5 == thunderstonem_board_version));
}

static inline int board_is_mmp3_thunderstonem_SB(void){
	return (machine_is_thunderstonem() && (0x6 == thunderstonem_board_version));
}

static inline int board_is_mmp3_thunderstonem_SC(void){
	return (machine_is_thunderstonem() && (0x7 == thunderstonem_board_version));
}
#endif

static void __init thunderstonem_init(void)
{
	extern int (*board_reset)(char mode, const char *cmd);
	board_reset = thunderstonem_board_reset;
	pm_power_off = thunderstonem_poweroff;
	mfp_config(ARRAY_AND_SIZE(thunderstonem_pin_config));

	thunderstonem_get_board_version();

	/* on-chip devices */
	mmp3_add_uart(3);

	/* for gps uart */
	mmp3_add_uart(2);


	mmp3_add_twsi(1, NULL, ARRAY_AND_SIZE(thunderstonem_twsi1_info));
	mmp3_add_twsi(3, NULL, ARRAY_AND_SIZE(thunderstonem_twsi3_info));
	mmp3_add_twsi(4, NULL, ARRAY_AND_SIZE(thunderstonem_twsi4_info));
	mmp3_add_twsi(5, NULL, ARRAY_AND_SIZE(thunderstonem_twsi5_info));
	mmp3_add_twsi(6, NULL, ARRAY_AND_SIZE(thunderstonem_twsi6_info));

	mmp3_add_videosram(&mmp3_videosram_info);
#ifdef CONFIG_FB_PXA168
	thunderstonem_add_lcd_mipi();
	mmp3_add_tv_out();
#endif

#ifdef CONFIG_UIO_HDMI
	mmp3_add_hdmi(&mmp3_hdmi_info);
#endif

	/* backlight */
	mmp3_add_pwm(3);
	platform_device_register(&thunderstonem_lcd_backlight_devices);

	platform_device_register(&gpio_keys);

	mmp3_add_thermal();
#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem();
#endif

#ifdef CONFIG_UIO_VMETA
	mmp_init_vmeta();
#endif
#ifdef CONFIG_MMC_SDHCI_PXAV3
	thunderstonem_init_mmc();
#endif /* CONFIG_MMC_SDHCI_PXAV3 */

	platform_device_register(&mmp3_device_rtc);

#if defined(CONFIG_TOUCHSCREEN_VNC)
	platform_device_register(&mmp3_device_vnc_touch);
#endif

	/* audio sspa support */
	mmp3_add_sspa(1);
	mmp3_add_sspa(2);
	mmp3_add_audiosram(&mmp3_audiosram_info);

	thunderstonem_fixed_power_init();

#ifdef CONFIG_USB_PXA_U2O
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

	pxa_u3d_phy_disable();
}

MACHINE_START(THUNDERSTONEM, "ThunderStoneM")
	.map_io		= mmp_map_io,
	.nr_irqs	= THUNDERSTONEM_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.reserve	= mmp3_reserve,
	.init_machine	= thunderstonem_init,
MACHINE_END
