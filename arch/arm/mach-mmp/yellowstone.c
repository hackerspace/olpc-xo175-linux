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
#include <linux/mfd/max8925.h>
#include <linux/pwm_backlight.h>
#include <linux/regulator/machine.h>
#if defined(CONFIG_SENSORS_LSM303DLHC_ACC) || \
	defined(CONFIG_SENSORS_LSM303DLHC_MAG)
#include <linux/i2c/lsm303dlhc.h>
#endif
#if defined(CONFIG_SENSORS_L3G4200D_GYR)
#include <linux/i2c/l3g4200d.h>
#endif

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <mach/addr-map.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp3.h>
#include <mach/irqs.h>
#include <mach/regs-mpmu.h>
#include <mach/tc35876x.h>
#include <mach/uio_hdmi.h>
#include <plat/usb.h>
#include <media/soc_camera.h>

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

	/* TWSI3 */
	GPIO71_TWSI3_SCL,
	GPIO72_TWSI3_SDA,

	/* TWSI4 */
	TWSI4_SCL,
	TWSI4_SDA,

	/*PWM3*/
	GPIO53_PWM3,

	/* SSPA1 (I2S) */
	GPIO23_GPIO,
	GPIO24_I2S_SYSCLK,
	GPIO25_I2S_BITCLK,
	GPIO26_I2S_SYNC,
	GPIO27_I2S_DATA_OUT,
	GPIO28_I2S_SDATA_IN,

	/* camera */
	GPIO68_GPIO,
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

static unsigned long mmc3_pin_config[] __initdata = {
	MFP_CFG_DRV(GPIO161, AF2, SLOW), /* MMC3 DAT7 */
	MFP_CFG_DRV(GPIO145, AF2, SLOW), /* MMC3 DAT6 */
	MFP_CFG_DRV(GPIO162, AF2, SLOW), /* MMC3 DAT5 */
	MFP_CFG_DRV(GPIO146, AF2, SLOW), /* MMC3 DAT4 */
	MFP_CFG_DRV(GPIO163, AF2, SLOW), /* MMC3 DAT3 */
	MFP_CFG_DRV(GPIO108, AF2, SLOW), /* MMC3 DAT2 */
	MFP_CFG_DRV(GPIO164, AF2, SLOW), /* MMC3 DAT1 */
	MFP_CFG_DRV(GPIO109, AF2, SLOW), /* MMC3 DAT0 */
	MFP_CFG_DRV(GPIO111, AF2, SLOW), /* MMC3 CMD */
	MFP_CFG_DRV(GPIO110, AF2, SLOW), /* MMC3 CLK */
};

#if defined(CONFIG_VIDEO_MV)
/* soc  camera */
static int camera_sensor_power(struct device *dev, int on)
{
	static struct regulator *v_ldo14;
	static struct regulator *v_ldo15;
	static int f_enabled;
	int cam_enable = mfp_to_gpio(MFP_PIN_GPIO68), ret;

	if (gpio_request(cam_enable, "CAM_ENABLE_HI_SENSOR")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", cam_enable);
		return -EIO;
	}

	/* enable voltage for camera sensor OV5642 */
	/* v_ldo14 3.0v */
	/* v_ldo15 2.8v */
	if (on && (!f_enabled)) {
		v_ldo14 = regulator_get(NULL, "v_ldo14");
		if (IS_ERR(v_ldo14)) {
			v_ldo14 = NULL;
			return -EIO;
		} else {
			ret = regulator_set_voltage(v_ldo14, 3000000, 3000000);
			regulator_enable(v_ldo14);
			f_enabled = 1;
		}

		v_ldo15 = regulator_get(NULL, "v_ldo15");
		if (IS_ERR(v_ldo15)) {
			v_ldo15 = NULL;
			return -EIO;
		} else {
			regulator_set_voltage(v_ldo15, 2800000, 2800000);
			regulator_enable(v_ldo15);
			f_enabled = 1;
		}
	}

	if (f_enabled && (!on)) {
		if (v_ldo14) {
			regulator_disable(v_ldo14);
			regulator_put(v_ldo14);
			v_ldo14 = NULL;
		}
		if (v_ldo15) {
			regulator_disable(v_ldo15);
			regulator_put(v_ldo15);
			v_ldo15 = NULL;
		}
		f_enabled = 0;
	}	/* actually, no small power down pin needed */

	/* pull up camera pwdn pin to disable camera sensor */
	gpio_direction_output(cam_enable, 1);
	mdelay(1);
	/* pull down camera pwdn pin to enable camera sensor */
	gpio_direction_output(cam_enable, 0);

	gpio_free(cam_enable);
	msleep(100);

	return 0;
}

static struct i2c_board_info abilene_i2c_camera[] = {
	{
		I2C_BOARD_INFO("ov5642", 0x3c),
	},
};

static struct soc_camera_link iclink_ov5642 = {
	.bus_id         = 0,            /* Must match with the camera ID */
	.power          = camera_sensor_power,
	.board_info     = &abilene_i2c_camera[0],
	.i2c_adapter_id = 3,
	.flags = SOCAM_MIPI,
	.module_name    = "ov5642",
	.priv = "pxa2128-mipi",
};

static struct platform_device abilene_ov5642 = {
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
	struct mv_cam_pdata *data = dev->platform_data;
	if ((!data->clk_enabled) && init) {
		data->clk = clk_get(dev, "CCICRSTCLK");
		if (IS_ERR(data->clk)) {
			dev_err(dev, "Could not get rstclk\n");
			return PTR_ERR(data->clk);
		}
		data->clk_enabled = 1;

		return 0;
	}

	if (!init && data->clk_enabled) {
		clk_put(data->clk);
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

static struct regulator_consumer_supply max8925_regulator_supply[] = {
	[0] = {
		.supply = "DBVDD",
		},
	[1] = {
		.supply = "AVDD2",
		},
	[2] = {
		.supply = "CPVDD",
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
	[MAX8925_ID_LDO11]	= REGULATOR_SUPPLY("v_ldo11", NULL),
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
	[MAX8925_ID_SD1] = REG_INIT(SD1, 637500, 1425000, 0, 0),
	[MAX8925_ID_SD2] = {
				.constraints = {
						.name = "SD2",
						.min_uV = 650000,
						.max_uV = 2225000,
						.always_on = 1,
						.boot_on = 1,
						},
				.num_consumer_supplies =
				ARRAY_SIZE(max8925_regulator_supply),
				.consumer_supplies = max8925_regulator_supply,
				},
	[MAX8925_ID_SD3] = REG_INIT(SD3, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO1] = REG_INIT(LDO1, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO2] = REG_INIT(LDO2, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO3] = REG_INIT(LDO3, 1000000, 1500000, 0, 0),
	[MAX8925_ID_LDO4] = REG_INIT(LDO4, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO5] = REG_INIT(LDO5, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO6] = REG_INIT(LDO6, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO7] = REG_INIT(LDO7, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO8] = REG_INIT(LDO8, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO9] = REG_INIT(LDO9, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO10] = REG_INIT(LDO10, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO11] = REG_INIT(LDO11, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO12] = REG_INIT(LDO12, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO13] = REG_INIT(LDO13, 750000, 1500000, 0, 0),
	[MAX8925_ID_LDO14] = REG_INIT(LDO14, 750000, 3000000, 0, 0),
	[MAX8925_ID_LDO15] = REG_INIT(LDO15, 750000, 2800000, 0, 0),
	[MAX8925_ID_LDO16] = REG_INIT(LDO16, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO17] = REG_INIT(LDO17, 1000000, 1500000, 0, 0),
	[MAX8925_ID_LDO18] = REG_INIT(LDO18, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO19] = REG_INIT(LDO19, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO20] = REG_INIT(LDO20, 750000, 3900000, 1, 1),
};



static struct max8925_platform_data yellowstone_max8925_info = {
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

static struct i2c_board_info yellowstone_twsi1_info[] = {
	{
		.type		= "max8925",
		.addr		= 0x3c,
		.irq		= IRQ_MMP3_PMIC,
		.platform_data	= &yellowstone_max8925_info,
	},
};

#if defined(CONFIG_SENSORS_LSM303DLHC_ACC)
static struct lsm303dlhc_acc_platform_data lsm303dlhc_acc_data = {
	.poll_interval = 1000,
	.min_interval = 10,
	.g_range = LSM303DLHC_ACC_G_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.gpio_int1 = -EINVAL,
	.gpio_int2 = -EINVAL,
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

static struct platform_pwm_backlight_data yellowstone_lcd_backlight_data = {
	/* primary backlight */
	.pwm_id = 2,
	.max_brightness = 100,
	.dft_brightness = 50,
	.pwm_period_ns = 2000000,
};

static struct platform_device yellowstone_lcd_backlight_devices = {
	.name = "pwm-backlight",
	.id = 2,
	.dev = {
		.platform_data = &yellowstone_lcd_backlight_data,
	},
};

#if defined(CONFIG_TC35876X)
/* force LDO3 & LDO17 always on */
static int tc358765_init(void)
{
	struct regulator *vcc = NULL;

	/* enable LDO for MIPI bridge */
	vcc = regulator_get(NULL, "v_ldo17");
	if (IS_ERR(vcc))
		vcc = NULL;
	else {
		regulator_enable(vcc);
		regulator_set_voltage(vcc, 1200000, 1200000);
	}
	vcc = regulator_get(NULL, "v_ldo3");
	if (IS_ERR(vcc))
		vcc = NULL;
	else {
		regulator_enable(vcc);
		regulator_set_voltage(vcc, 1200000, 1200000);
	}

	return 0;
}

static struct tc35876x_platform_data tc358765_data = {
	.platform_init = tc358765_init,
	.id = TC358765_CHIPID,
	.id_reg = TC358765_CHIPID_REG,
};
#endif

static struct i2c_board_info yellowstone_twsi5_info[] = {
#if defined(CONFIG_TC35876X)
	{
		.type		= "tc35876x",
		.addr		= 0x0f,
		.platform_data	= &tc358765_data,
	},
#endif
};

static struct i2c_board_info yellowstone_twsi6_info[] = {
	{
	},
};

#ifdef CONFIG_MMC_SDHCI_PXAV3
#include <linux/mmc/host.h>
static void yellowstone_sd_signal_1v8(int set)
{
	static struct regulator *v_ldo_sd;
	int vol;

	v_ldo_sd = regulator_get(NULL, "v_ldo11");
	if (IS_ERR(v_ldo_sd)) {
		printk(KERN_ERR "Failed to get v_ldo11\n");
		return;
	}

	vol = set ? 1800000 : 3000000;
	regulator_set_voltage(v_ldo_sd, vol, vol);
	regulator_enable(v_ldo_sd);

	mmp3_io_domain_1v8(AIB_SDMMC_IO_REG, set);

	msleep(10);
	regulator_put(v_ldo_sd);
}

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc0 = {
	.clk_delay_cycles	= 0x1F,
	.signal_1v8		= yellowstone_sd_signal_1v8,
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc2 = {
	.flags		= PXA_FLAG_SD_8_BIT_CAPABLE_SLOT,
	.host_caps	= MMC_CAP_1_8V_DDR,
};

static void __init yellowstone_init_mmc(void)
{
	mfp_config(ARRAY_AND_SIZE(mmc3_pin_config));
	mmp3_add_sdh(2, &mmp3_sdh_platdata_mmc2); /* eMMC */

	mfp_config(ARRAY_AND_SIZE(mmc1_pin_config));
	mmp3_add_sdh(0, &mmp3_sdh_platdata_mmc0); /* SD/MMC */
}
#endif /* CONFIG_MMC_SDHCI_PXAV3 */


#ifdef CONFIG_USB_SUPPORT

#if defined(CONFIG_USB_PXA_U2O) || defined(CONFIG_USB_EHCI_PXA_U2O)
extern int pxa_usb_phy_init(unsigned int base);

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
	.set_vbus	= pxa_usb_set_vbus,
};
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H_HSIC
extern int mmp3_hsic_phy_init(unsigned int base);
extern int mmp3_hsic_private_init(struct mv_op_regs *opregs,
					unsigned int phyregs);

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

	gpio_free(reset);
	return 0;
}

static int mmp3_hsic1_set_vbus(unsigned int vbus)
{
	static struct regulator *ldo5;

	printk(KERN_INFO "%s: set %d\n", __func__, vbus);
	if (vbus) {
		if (!ldo5) {
			ldo5 = regulator_get(NULL, "v_ldo5");
			if (IS_ERR(ldo5)) {
				printk(KERN_INFO "ldo5 not found\n");
				return -EIO;
			}
			regulator_set_voltage(ldo5, 1200000, 1200000);
			regulator_enable(ldo5);
			printk(KERN_INFO "%s: enable regulator\n", __func__);
			udelay(2);
		}

		mmp3_hsic1_reset();
	} else {
		if (ldo5) {
			regulator_disable(ldo5);
			regulator_put(ldo5);
			ldo5 = NULL;
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

static void __init yellowstone_init(void)
{
	mfp_config(ARRAY_AND_SIZE(yellowstone_pin_config));

	/* on-chip devices */
	mmp3_add_uart(3);
	mmp3_add_twsi(1, NULL, ARRAY_AND_SIZE(yellowstone_twsi1_info));
	mmp3_add_twsi(4, NULL, ARRAY_AND_SIZE(yellowstone_twsi4_info));
	mmp3_add_twsi(5, NULL, ARRAY_AND_SIZE(yellowstone_twsi5_info));
	mmp3_add_twsi(6, NULL, ARRAY_AND_SIZE(yellowstone_twsi6_info));

	mmp3_add_keypad(&mmp3_keypad_info);

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

#ifdef CONFIG_MMC_SDHCI_PXAV3
	yellowstone_init_mmc();
#endif /* CONFIG_MMC_SDHCI_PXAV3 */

	platform_device_register(&mmp3_device_rtc);

#if defined(CONFIG_VIDEO_MV)
	platform_device_register(&abilene_ov5642);
	mmp3_add_cam(0, &mv_cam_data);
#endif

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

#ifdef CONFIG_USB_EHCI_PXA_U2H_HSIC
	mmp3_hsic1_device.dev.platform_data = (void *)&mmp3_hsic1_pdata;
	platform_device_register(&mmp3_hsic1_device);
#endif
}

MACHINE_START(YELLOWSTONE, "YellowStone")
	.map_io		= mmp_map_io,
	.nr_irqs	= YELLOWSTONE_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.init_machine	= yellowstone_init,
MACHINE_END
