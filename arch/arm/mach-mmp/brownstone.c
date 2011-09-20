/*
 *  linux/arch/arm/mach-mmp/brownstone.c
 *
 *  Support for the Marvell Brownstone Development Platform.
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
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/max8649.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max8925.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/pwm_backlight.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp2.h>
#include <mach/irqs.h>
#include <mach/tc35876x.h>
#include <mach/pxa168fb.h>
#include <mach/uio_hdmi.h>
#include <plat/usb.h>
#include <linux/i2c/tpk_r800.h>
#include <mach/axis_sensor.h>
#include "common.h"
#include "onboard.h"

#define BROWNSTONE_NR_IRQS	(IRQ_BOARD_START + 40)

#define GPIO_5V_ENABLE		(89)

static unsigned long brownstone_pin_config[] __initdata = {
	/* UART1 */
	GPIO29_UART1_RXD,
	GPIO30_UART1_TXD,

	/* UART3 */
	GPIO51_UART3_RXD,
	GPIO52_UART3_TXD,

	/* TWSI4 */
	TWSI4_SCL,
	TWSI4_SDA,

	/* TWSI5 */
	GPIO99_TWSI5_SCL,
	GPIO100_TWSI5_SDA,

	/* TWSI2 */
	GPIO43_TWSI2_SCL,
	GPIO44_TWSI2_SDA,

	/* TWSI6 */
	GPIO97_TWSI6_SCL,
	GPIO98_TWSI6_SDA,

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

	/* PMIC */
	PMIC_PMIC_INT | MFP_LPM_EDGE_FALL,
	GPIO45_WM8994_LDOEN,

	/* MMC0 */
	GPIO131_MMC1_DAT3 | MFP_PULL_HIGH,
	GPIO132_MMC1_DAT2 | MFP_PULL_HIGH,
	GPIO133_MMC1_DAT1 | MFP_PULL_HIGH,
	GPIO134_MMC1_DAT0 | MFP_PULL_HIGH,
	GPIO136_MMC1_CMD | MFP_PULL_HIGH,
	GPIO139_MMC1_CLK,
	GPIO140_MMC1_CD | MFP_PULL_LOW,
	GPIO141_MMC1_WP | MFP_PULL_LOW,

	/* MMC1 */
	GPIO37_MMC2_DAT3 | MFP_PULL_HIGH,
	GPIO38_MMC2_DAT2 | MFP_PULL_HIGH,
	GPIO39_MMC2_DAT1 | MFP_PULL_HIGH,
	GPIO40_MMC2_DAT0 | MFP_PULL_HIGH,
	GPIO41_MMC2_CMD | MFP_PULL_HIGH,
	GPIO42_MMC2_CLK,

	/* MMC2 */
	GPIO165_MMC3_DAT7 | MFP_PULL_HIGH,
	GPIO162_MMC3_DAT6 | MFP_PULL_HIGH,
	GPIO166_MMC3_DAT5 | MFP_PULL_HIGH,
	GPIO163_MMC3_DAT4 | MFP_PULL_HIGH,
	GPIO167_MMC3_DAT3 | MFP_PULL_HIGH,
	GPIO164_MMC3_DAT2 | MFP_PULL_HIGH,
	GPIO168_MMC3_DAT1 | MFP_PULL_HIGH,
	GPIO111_MMC3_DAT0 | MFP_PULL_HIGH,
	GPIO112_MMC3_CMD | MFP_PULL_HIGH,
	GPIO151_MMC3_CLK,

	/* SSPA1 */
	GPIO24_I2S_SYSCLK,
	GPIO25_I2S_BITCLK,
	GPIO26_I2S_SYNC,
	GPIO27_I2S_DATA_OUT,
	GPIO28_I2S_SDATA_IN,

	/* SSPA2 */
	GPIO33_SSPA2_CLK,
	GPIO34_SSPA2_FRM,
	GPIO35_SSPA2_TXD,
	GPIO36_SSPA2_RXD,

	/* 5V regulator */
	GPIO89_GPIO,

	/* Backlight */
	GPIO53_PWM3,

	/* LCD */
	GPIO83_LCD_RST,

	/* CM3623 INT */
	GPIO92_GPIO | MFP_PULL_HIGH,
};

static struct regulator_consumer_supply max8649_supply[] = {
	REGULATOR_SUPPLY("vcc_core", NULL),
};

static struct regulator_init_data max8649_init_data = {
	.constraints	= {
		.name		= "vcc_core range",
		.min_uV		= 1150000,
		.max_uV		= 1280000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8649_supply[0],
};

static struct max8649_platform_data brownstone_max8649_info = {
	.mode		= 2,	/* VID1 = 1, VID0 = 0 */
	.extclk		= 0,
	.ramp_timing	= MAX8649_RAMP_32MV,
	.regulator	= &max8649_init_data,
};

static struct regulator_consumer_supply brownstone_v_5vp_supplies[] = {
	REGULATOR_SUPPLY("v_5vp", NULL),
};

static struct regulator_init_data brownstone_v_5vp_data = {
	.constraints	= {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(brownstone_v_5vp_supplies),
	.consumer_supplies	= brownstone_v_5vp_supplies,
};

static struct fixed_voltage_config brownstone_v_5vp = {
	.supply_name		= "v_5vp",
	.microvolts		= 5000000,
	.gpio			= GPIO_5V_ENABLE,
	.enable_high		= 1,
	.enabled_at_boot	= 1,
	.init_data		= &brownstone_v_5vp_data,
};

static struct platform_device brownstone_v_5vp_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &brownstone_v_5vp,
	},
};

static struct regulator_consumer_supply regulator_supply[] = {
	[MAX8925_ID_SD1]	= REGULATOR_SUPPLY("v_sd1", NULL),
	[MAX8925_ID_SD2]	= REGULATOR_SUPPLY("v_sd2", NULL),
	[MAX8925_ID_SD3]	= REGULATOR_SUPPLY("v_sd3", NULL),
	[MAX8925_ID_LDO1]	= REGULATOR_SUPPLY("v_ldo1", NULL),
	[MAX8925_ID_LDO2]	= REGULATOR_SUPPLY("v_ldo2", NULL),
	[MAX8925_ID_LDO3]	= REGULATOR_SUPPLY("v_ldo3", NULL),
	[MAX8925_ID_LDO4]	= REGULATOR_SUPPLY("v_ldo4", NULL),
	[MAX8925_ID_LDO5]	= REGULATOR_SUPPLY("v_ldo5", NULL),
	[MAX8925_ID_LDO6]	= REGULATOR_SUPPLY("v_ldo6", NULL),
	[MAX8925_ID_LDO7]	= REGULATOR_SUPPLY("v_ldo7", NULL),
	[MAX8925_ID_LDO8]	= REGULATOR_SUPPLY("v_ldo8", NULL),
	[MAX8925_ID_LDO9]	= REGULATOR_SUPPLY("v_ldo9", NULL),
	[MAX8925_ID_LDO10]	= REGULATOR_SUPPLY("v_ldo10", NULL),
	[MAX8925_ID_LDO11]	= REGULATOR_SUPPLY("vmmc", "sdhci-pxa.0"),
	[MAX8925_ID_LDO12]	= REGULATOR_SUPPLY("v_ldo12", NULL),
	[MAX8925_ID_LDO13]	= REGULATOR_SUPPLY("v_ldo13", NULL),
	[MAX8925_ID_LDO14]	= REGULATOR_SUPPLY("v_ldo14", NULL),
	[MAX8925_ID_LDO15]	= REGULATOR_SUPPLY("v_ldo15", NULL),
	[MAX8925_ID_LDO16]	= REGULATOR_SUPPLY("v_ldo16", NULL),
	[MAX8925_ID_LDO17]	= REGULATOR_SUPPLY("v_ldo17", NULL),
	[MAX8925_ID_LDO18]	= REGULATOR_SUPPLY("v_ldo18", NULL),
	[MAX8925_ID_LDO19]	= REGULATOR_SUPPLY("v_ldo19", NULL),
	[MAX8925_ID_LDO20]	= REGULATOR_SUPPLY("v_ldo20", NULL),
};

#define REG_INIT(_name, _min, _max, _always, _boot)		\
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
	.consumer_supplies	= &regulator_supply[MAX8925_ID_##_name], \
}

static struct regulator_init_data regulator_data[] = {
	[MAX8925_ID_SD1] = REG_INIT(SD1, 637500, 1425000, 1, 1),
	[MAX8925_ID_SD2] = REG_INIT(SD2, 650000, 2225000, 1, 1),
	[MAX8925_ID_SD3] = REG_INIT(SD3, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO1] = REG_INIT(LDO1, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO2] = REG_INIT(LDO2, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO3] = REG_INIT(LDO3, 650000, 2250000, 0, 0),
	[MAX8925_ID_LDO4] = REG_INIT(LDO4, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO5] = REG_INIT(LDO5, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO6] = REG_INIT(LDO6, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO7] = REG_INIT(LDO7, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO8] = REG_INIT(LDO8, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO9] = REG_INIT(LDO9, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO10] = REG_INIT(LDO10, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO11] = REG_INIT(LDO11, 2800000, 2800000, 0, 0),
	[MAX8925_ID_LDO12] = REG_INIT(LDO12, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO13] = REG_INIT(LDO13, 750000, 1500000, 0, 0),
	[MAX8925_ID_LDO14] = REG_INIT(LDO14, 750000, 3000000, 0, 0),
	[MAX8925_ID_LDO15] = REG_INIT(LDO15, 750000, 2800000, 0, 0),
	[MAX8925_ID_LDO16] = REG_INIT(LDO16, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO17] = REG_INIT(LDO17, 650000, 2250000, 0, 0),
	[MAX8925_ID_LDO18] = REG_INIT(LDO18, 650000, 2250000, 0, 0),
	[MAX8925_ID_LDO19] = REG_INIT(LDO19, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO20] = REG_INIT(LDO20, 750000, 3900000, 0, 0),
};

static struct max8925_platform_data brownstone_max8925_info = {
	.irq_base		= IRQ_BOARD_START,

	.regulator[MAX8925_ID_SD1] = &regulator_data[MAX8925_ID_SD1],
	.regulator[MAX8925_ID_SD2] = &regulator_data[MAX8925_ID_SD2],
	.regulator[MAX8925_ID_SD3] = &regulator_data[MAX8925_ID_SD3],
	.regulator[MAX8925_ID_LDO1] = &regulator_data[MAX8925_ID_LDO1],
	.regulator[MAX8925_ID_LDO2] = &regulator_data[MAX8925_ID_LDO2],
	.regulator[MAX8925_ID_LDO3] = &regulator_data[MAX8925_ID_LDO3],
	.regulator[MAX8925_ID_LDO4] = &regulator_data[MAX8925_ID_LDO4],
	.regulator[MAX8925_ID_LDO5] = &regulator_data[MAX8925_ID_LDO5],
	.regulator[MAX8925_ID_LDO6] = &regulator_data[MAX8925_ID_LDO6],
	.regulator[MAX8925_ID_LDO7] = &regulator_data[MAX8925_ID_LDO7],
	.regulator[MAX8925_ID_LDO8] = &regulator_data[MAX8925_ID_LDO8],
	.regulator[MAX8925_ID_LDO9] = &regulator_data[MAX8925_ID_LDO9],
	.regulator[MAX8925_ID_LDO10] = &regulator_data[MAX8925_ID_LDO10],
	.regulator[MAX8925_ID_LDO11] = &regulator_data[MAX8925_ID_LDO11],
	.regulator[MAX8925_ID_LDO12] = &regulator_data[MAX8925_ID_LDO12],
	.regulator[MAX8925_ID_LDO13] = &regulator_data[MAX8925_ID_LDO13],
	.regulator[MAX8925_ID_LDO14] = &regulator_data[MAX8925_ID_LDO14],
	.regulator[MAX8925_ID_LDO15] = &regulator_data[MAX8925_ID_LDO15],
	.regulator[MAX8925_ID_LDO16] = &regulator_data[MAX8925_ID_LDO16],
	.regulator[MAX8925_ID_LDO17] = &regulator_data[MAX8925_ID_LDO17],
	.regulator[MAX8925_ID_LDO18] = &regulator_data[MAX8925_ID_LDO18],
	.regulator[MAX8925_ID_LDO19] = &regulator_data[MAX8925_ID_LDO19],
	.regulator[MAX8925_ID_LDO20] = &regulator_data[MAX8925_ID_LDO20],
};

static struct i2c_board_info brownstone_twsi1_info[] = {
	[0] = {
		.type		= "max8649",
		.addr		= 0x60,
		.platform_data	= &brownstone_max8649_info,
	},
	[1] = {
		.type		= "max8925",
		.addr		= 0x3c,
		.irq		= IRQ_MMP2_PMIC,
		.platform_data	= &brownstone_max8925_info,
	},
};

static struct sdhci_pxa_platdata mmp2_sdh_platdata_mmc0 = {
	.max_speed	= 25000000,
};

static struct regulator_consumer_supply wm8994_fixed_voltage0_supplies[] = {
	REGULATOR_SUPPLY("DBVDD", NULL),
	REGULATOR_SUPPLY("AVDD2", NULL),
	REGULATOR_SUPPLY("CPVDD", NULL),
};

static struct regulator_consumer_supply wm8994_fixed_voltage1_supplies[] = {
	REGULATOR_SUPPLY("SPKVDD1", NULL),
	REGULATOR_SUPPLY("SPKVDD2", NULL),
};

static struct regulator_init_data wm8994_fixed_voltage0_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(wm8994_fixed_voltage0_supplies),
	.consumer_supplies	= wm8994_fixed_voltage0_supplies,
};

static struct regulator_init_data wm8994_fixed_voltage1_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(wm8994_fixed_voltage1_supplies),
	.consumer_supplies	= wm8994_fixed_voltage1_supplies,
};

static struct fixed_voltage_config wm8994_fixed_voltage0_config = {
	.supply_name	= "VCC_1.8V",
	.microvolts	= 1800000,
	.gpio		= -EINVAL,
	.init_data	= &wm8994_fixed_voltage0_init_data,
};

static struct fixed_voltage_config wm8994_fixed_voltage1_config = {
	.supply_name	= "V_BAT",
	.microvolts	= 3700000,
	.gpio		= -EINVAL,
	.init_data	= &wm8994_fixed_voltage1_init_data,
};

static struct platform_device wm8994_fixed_voltage0 = {
	.name		= "reg-fixed-voltage",
	.id		= 2,
	.dev		= {
		.platform_data	= &wm8994_fixed_voltage0_config,
	},
};

static struct platform_device wm8994_fixed_voltage1 = {
	.name		= "reg-fixed-voltage",
	.id		= 3,
	.dev		= {
		.platform_data	= &wm8994_fixed_voltage1_config,
	},
};

static struct regulator_consumer_supply wm8994_avdd1_supply =
	REGULATOR_SUPPLY("AVDD1", NULL);

static struct regulator_consumer_supply wm8994_dcvdd_supply =
	REGULATOR_SUPPLY("DCVDD", NULL);

static struct regulator_init_data wm8994_ldo1_data = {
	.constraints	= {
		.name		= "AVDD1_3.0V",
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.min_uV		= 2400000,
		.max_uV		= 3100000,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8994_avdd1_supply,
};

static struct regulator_init_data wm8994_ldo2_data = {
	.constraints	= {
		.name		= "DCVDD_1.0V",
		.min_uV		= 900000,
		.max_uV		= 1200000,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &wm8994_dcvdd_supply,
};

static struct wm8994_pdata brownstone_wm8994_pdata = {
	.gpio_defaults[0] = 0x0003,
	/* AIF2 voice */
	.gpio_defaults[2] = 0x8100,
	.gpio_defaults[3] = 0x8100,
	.gpio_defaults[4] = 0x8100,
	.gpio_defaults[5] = 0x8100,
	.gpio_defaults[6] = 0x0100,
	/* AIF3 voice */
	.gpio_defaults[7] = 0x8100,
	.gpio_defaults[8] = 0x0100,
	.gpio_defaults[9] = 0x8100,
	.gpio_defaults[10] = 0x8100,

	.ldo[0]	= { mfp_to_gpio(GPIO45_WM8994_LDOEN), NULL, &wm8994_ldo1_data },
	.ldo[1]	= { 0, NULL, &wm8994_ldo2_data },
};

static struct i2c_board_info brownstone_twsi2_info[] = {
	{
		.type		= "wm8994",
		.addr		= 0x1a,
		.platform_data	= &brownstone_wm8994_pdata,
	},
};

static struct platform_device *fixed_rdev[] __initdata = {
	&wm8994_fixed_voltage0,
	&wm8994_fixed_voltage1,
};

static void __init brownstone_fixed_regulator(void)
{
	platform_add_devices(fixed_rdev, ARRAY_SIZE(fixed_rdev));
}

#ifdef CONFIG_USB_SUPPORT

#if defined(CONFIG_USB_PXA_U2O)
extern int pxa_usb_phy_init(unsigned int base);
static char *mmp2_usb_clock_name[] = {
	[0] = "U2OCLK",
};
static struct mv_usb_platform_data mmp2_usb_pdata = {
	.clknum		= 1,
	.clkname	= mmp2_usb_clock_name,
	.vbus		= NULL,
	.mode		= MV_USB_MODE_OTG,
	.phy_init	= pxa_usb_phy_init,
	.set_vbus	= NULL,
};
#endif

#endif

static struct platform_pwm_backlight_data brownstone_lcd_backlight_data = {
	/* primary backlight */
	.pwm_id			= 2,
	.max_brightness	= 100,
	.dft_brightness	= 50,
	.pwm_period_ns	= 2000000,
};

static struct platform_device brownstone_lcd_backlight_devices = {
	.name		= "pwm-backlight",
	.id			= 0,
	.dev		= {
		.platform_data = &brownstone_lcd_backlight_data,
	},
};

#if defined(CONFIG_TC35876X)
int tc358765_init(void)
{
	return 0;
}

static struct tc35876x_platform_data tc358765_data = {
	.platform_init = tc358765_init,
	.id = TC358765_CHIPID,
	.id_reg = TC358765_CHIPID_REG,
};
#endif
static int r800_set_power(int on)
{
	static struct regulator *v_ldo8;

	if (on) {
		v_ldo8 = regulator_get(NULL, "v_ldo8");
		if (IS_ERR(v_ldo8)) {
			v_ldo8 = NULL;
			pr_err("%s: enable ldo8 for touch fail!\n", __func__);
			return -EIO;
		} else {
			regulator_set_voltage(v_ldo8, 2800000, 2800000);
			regulator_enable(v_ldo8);
		}
	}
	else {
		regulator_disable(v_ldo8);
		regulator_put(v_ldo8);
		v_ldo8 = NULL;
	}
	msleep(100);
	return 1;
}

static int cm_set_power(int on)
{
	static struct regulator *v_ldo8;
	static int enabled;
	int changed = 0;

	if (on && (!enabled)) {
		v_ldo8 = regulator_get(NULL, "v_ldo8");
		if (IS_ERR(v_ldo8)) {
			v_ldo8 = NULL;
			return -EIO;
		} else {
			regulator_set_voltage(v_ldo8, 2800000, 2800000);
			regulator_enable(v_ldo8);
			enabled = 1;
			changed = 1;
		}
	}
	if ((!on) && enabled) {
		regulator_disable(v_ldo8);
		regulator_put(v_ldo8);
		v_ldo8 = NULL;
		enabled = 0;
		changed = 1;
	}
	if (changed)
		msleep(100);
	return 0;
}

static struct touchscreen_platform_data tpk_r800_data = {
	.set_power  = r800_set_power,
};

static struct axis_sensor_platform_data cm_platform_data = {
	.set_power  = cm_set_power,
};

static struct i2c_board_info brownstone_twsi4_info[] =
{
#if defined(CONFIG_SENSORS_CM3623)
	{
		.type       = "cm3623_ps",
		.addr       = (0xB0>>1),
		.platform_data  = &cm_platform_data,
	},
	{
		.type       = "cm3623_als_msb",
		.addr       = (0x20>>1),
		.platform_data  = &cm_platform_data,
	},
	{
		.type       = "cm3623_als_lsb",
		.addr       = (0x22>>1),
		.platform_data  = &cm_platform_data,
	},
	{
		.type       = "cm3623_int",
		.addr       = (0x18>>1),
		.platform_data  = &cm_platform_data,
	},
	{
		.type       = "cm3623_ps_threshold",
		.addr       = (0xB2>>1),
		.platform_data  = &cm_platform_data,
	},
#endif
#if defined(CONFIG_SENSORS_CM3213)
	{
		.type       = "cm3213_als_msb",
		.addr       = (0x20>>1),
		.platform_data  = &cm_platform_data,
	},
	{
		.type       = "cm3213_als_lsb",
		.addr       = (0x22>>1),
		.platform_data  = &cm_platform_data,
	},
	{
		.type       = "cm3213_int",
		.addr       = (0x18>>1),
		.platform_data  = &cm_platform_data,
	},
#endif
};

static struct i2c_board_info brownstone_twsi5_info[] = {
#if defined(CONFIG_TC35876X)
	{
		.type		= "tc35876x",
		.addr		= 0x0f,
		.platform_data	= &tc358765_data,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_TPK_R800)
	{
		.type           = "tpk_r800",
		.addr           = 0x10,
		.irq            = IRQ_GPIO(101),
		.platform_data  = &tpk_r800_data,
	},
#endif
};

static struct i2c_board_info brownstone_twsi6_info[] = {
	{
		.type			= "hdmi_edid",
		.addr			= 0x50,
	},
};

#ifdef CONFIG_UIO_HDMI
static struct uio_hdmi_platform_data mmp2_hdmi_info __initdata = {
	.sspa_reg_base = 0xD42A0C00,
	.gpio = mfp_to_gpio(GPIO46_HDMI_DET),
};
#endif

static struct sram_bank mmp2_audiosram_info = {
	.pool_name = "audio sram",
	.step = AUDIO_SRAM_GRANULARITY,
};

static void __init brownstone_init(void)
{
	mfp_config(ARRAY_AND_SIZE(brownstone_pin_config));

	/* on-chip devices */
	mmp2_add_uart(1);
	mmp2_add_uart(3);
	mmp2_add_twsi(1, NULL, ARRAY_AND_SIZE(brownstone_twsi1_info));
	mmp2_add_twsi(2, NULL, ARRAY_AND_SIZE(brownstone_twsi2_info));
	mmp2_add_twsi(4, NULL, ARRAY_AND_SIZE(brownstone_twsi4_info));
	mmp2_add_twsi(5, NULL, ARRAY_AND_SIZE(brownstone_twsi5_info));
	mmp2_add_twsi(6, NULL, ARRAY_AND_SIZE(brownstone_twsi6_info));
	mmp2_add_sdhost(0, &mmp2_sdh_platdata_mmc0); /* SD/MMC */
#ifdef CONFIG_USB_PXA_U2O
	pxa168_device_u2o.dev.platform_data = (void *)&mmp2_usb_pdata;
	platform_device_register(&pxa168_device_u2o);
#endif

	brownstone_fixed_regulator();

	mmp2_add_sspa(1);
	mmp2_add_sspa(2);
	mmp2_add_audiosram(&mmp2_audiosram_info);

	platform_device_register(&mmp_device_asoc_sspa1);
	platform_device_register(&mmp_device_asoc_sspa2);
	platform_device_register(&mmp_device_asoc_platform);

	/* enable 5v regulator */
	platform_device_register(&brownstone_v_5vp_device);

#ifdef CONFIG_FB_PXA168
	brownstone_add_lcd_mipi();
	mmp2_add_tv_out();
#endif

	/* backlight */
	mmp2_add_pwm(3);
	platform_device_register(&brownstone_lcd_backlight_devices);

#ifdef CONFIG_UIO_HDMI
	mmp2_add_hdmi(&mmp2_hdmi_info);
#endif
}

MACHINE_START(BROWNSTONE, "Brownstone Development Platform")
	/* Maintainer: Haojian Zhuang <haojian.zhuang@marvell.com> */
	.map_io		= mmp_map_io,
	.nr_irqs	= BROWNSTONE_NR_IRQS,
	.init_irq	= mmp2_init_irq,
	.timer		= &mmp2_timer,
	.init_machine	= brownstone_init,
MACHINE_END
