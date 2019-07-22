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
#include <linux/regulator/driver.h>
#include <linux/regulator/ds4432.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/regulator/fixed.h>
#include <linux/switch.h>
#if defined(CONFIG_SENSORS_LSM303DLHC_ACC) || \
	defined(CONFIG_SENSORS_LSM303DLHC_MAG)
#include <linux/i2c/lsm303dlhc.h>
#endif
#if defined(CONFIG_SENSORS_L3G4200D_GYR)
#include <linux/i2c/l3g4200d.h>
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
	GPIO06_WM8994_LDOEN,

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
	static int f_enabled;
	int cam_pwdn = mfp_to_gpio(MFP_PIN_GPIO68);
	struct regulator *v_ldo3;

	/* We rely on mipi brige also connect the mipi signal */
	/* v_ldo3 MIPI BRIDGE CHIP PLL, 1.2V */
	v_ldo3 = regulator_get(NULL, "v_ldo3");
	if (IS_ERR(v_ldo3)) {
		v_ldo3 = NULL;
		return -EIO;
	}
	if (on) {
		regulator_set_voltage(v_ldo3, 1200000, 1200000);
		regulator_enable(v_ldo3);
	} else
		regulator_disable(v_ldo3);

	regulator_put(v_ldo3);

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

static struct i2c_board_info abilene_i2c_camera[] = {
	{
		I2C_BOARD_INFO("ov5642", 0x3c),
	},
};

static struct soc_camera_link iclink_ov5642 = {
	.bus_id         = 1,            /* Must match with the camera ID */
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
	static struct regulator *v_ldo14;
	static struct regulator *v_ldo15;
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

	v_ldo14 = regulator_get(NULL, "v_ldo14");
	if (IS_ERR(v_ldo14)) {
		v_ldo14 = NULL;
		return -EIO;
	}

	v_ldo15 = regulator_get(NULL, "v_ldo15");
	if (IS_ERR(v_ldo15)) {
		v_ldo15 = NULL;
		return -EIO;
	}
	if ((!data->clk_enabled) && init) {
		data->clk = clk_get(dev, "CCICRSTCLK");
		if (IS_ERR(data->clk)) {
			dev_err(dev, "Could not get rstclk\n");
			return PTR_ERR(data->clk);
		}
		data->clk_enabled = 1;
		regulator_set_voltage(v_ldo14, 3000000, 3000000);
		regulator_enable(v_ldo14);
		regulator_set_voltage(v_ldo15, 2800000, 2800000);
		regulator_enable(v_ldo15);

		gpio_direction_output(cam_enable, 1);
		gpio_free(cam_enable);
		return 0;
	}

	if (!init && data->clk_enabled) {
		clk_put(data->clk);
		regulator_disable(v_ldo14);
		regulator_put(v_ldo14);
		regulator_disable(v_ldo15);
		regulator_put(v_ldo15);
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

static struct i2c_pxa_platform_data twsi1_pdata = {
	.use_pio		 = 1,
};

static struct i2c_board_info yellowstone_twsi1_info[] = {
	{
		.type		= "max8925",
		.addr		= 0x3c,
		.irq		= IRQ_MMP3_PMIC,
		.platform_data	= &yellowstone_max8925_info,
	},
};

static int motion_sensor_set_power(int on, const char *device_name)
{
	static struct regulator *v_ldo8[3];
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
			v_ldo8[device_index] = regulator_get(NULL, "v_ldo8");
			if (IS_ERR(v_ldo8[device_index])) {
				v_ldo8[device_index] = NULL;
				return -ENODEV;
			} else {
				regulator_set_voltage(v_ldo8[device_index], 2800000, 2800000);
				regulator_enable(v_ldo8[device_index]);
				is_enabled[device_index] = 1;
			}
		}
		if ((!on) && is_enabled[device_index]) {
			regulator_disable(v_ldo8[device_index]);
			regulator_put(v_ldo8[device_index]);
			v_ldo8[device_index] = NULL;
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

static int max17083_ds4432_convert(int path, int mode,
			int iparam, int *oparam) {
	if (mode == DS4432_DCDC_VOLTAGE_TO_CURRENT)
		*oparam = (iparam - 1275000) / 100; /* vV -> 10 nA */
	else
		*oparam = iparam * 100 + 1275000; /* 10 nA -> vV */
	return 0;
}

static struct regulator_consumer_supply ds4432_supply[] = {
	REGULATOR_SUPPLY("vcc_main", NULL),
};

static struct ds4432_dac_data ds4432_data[] = {
	[0] = {
		.initdat = {
			.constraints    = {
				.name           = "vcc_main range",
				.min_uV         = 903000,
				.max_uV         = 1349400,
				.always_on      = 1,
				.boot_on        = 1,
				.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
			},
			.num_consumer_supplies  = 1,
			.consumer_supplies      = &ds4432_supply[0],
		},
		.name = "max17083+ds4432",
		.type = REGULATOR_VOLTAGE,
		.dac_path = 1,
		.cstep_10nA = 62, /* (0.997/(16*100000))*100000000 10nA */
		.param_convert = max17083_ds4432_convert,
	},
	/* ds4432 has two paths, we may register two here, however
	   the two seems to be tied together on current board. we need to
	   keep one unused and the other to do real control
	*/
};

static struct ds4432_platform_data yellowstone_ds4432_info = {
	.regulator_count = sizeof(ds4432_data)/sizeof(ds4432_data[0]),
	.regulators = ds4432_data,

};

static struct i2c_board_info yellowstone_twsi6_info[] = {
	{
		.type		= "ds4432",
		.addr		= 0x48,
		.platform_data	= &yellowstone_ds4432_info,
	},
};

static int wm8994_ldoen(void)
{
	int gpio = mfp_to_gpio(GPIO06_WM8994_LDOEN);

	if (gpio_request(gpio, "wm8994 ldoen gpio")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio);
		return -1;
	}

	gpio_direction_output(gpio, 1);
	mdelay(1);
	gpio_free(gpio);

	return 0;
}

static struct regulator_consumer_supply yellowstone_wm8994_regulator_supply[]
							= {
	[0] = {
		.supply = "AVDD1",
		},
	[1] = {
		.supply = "DCVDD",
		},
};

struct regulator_init_data yellowstone_wm8994_regulator_init_data[] = {
	[0] = {
		.constraints = {
				.name = "wm8994-ldo1",
				.min_uV = 2400000,
				.max_uV = 3100000,
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &yellowstone_wm8994_regulator_supply[0],
		},
	[1] = {
		.constraints = {
				.name = "wm8994-ldo2",
				.min_uV = 900000,
				.max_uV = 1200000,
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &yellowstone_wm8994_regulator_supply[1],
		},
};

struct wm8994_pdata yellowstone_wm8994_pdata = {
	.ldo[0] = {
			.enable = 0,
			.init_data = &yellowstone_wm8994_regulator_init_data[0],
			.supply = "AVDD1",

		},
	.ldo[1] = {
		.enable = 0,
		.init_data = &yellowstone_wm8994_regulator_init_data[1],
		.supply = "DCVDD",

		},
};

static struct regulator_consumer_supply yellowstone_fixed_regulator_supply[] = {
	[0] = {
		.supply = "SPKVDD1",
		},
	[1] = {
		.supply = "SPKVDD2",
		},
};

static struct i2c_board_info yellowstone_twsi3_info[] = {
	{
	 .type = "wm8994",
	 .addr = 0x1a,
	 .platform_data = &yellowstone_wm8994_pdata,
	 },
};

struct regulator_init_data yellowstone_fixed_regulator_init_data[] = {
	[0] = {
		.constraints = {
				.name = "wm8994-SPK1",
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &yellowstone_fixed_regulator_supply[0],
		},
	[1] = {
		.constraints = {
				.name = "wm8994-SPK2",
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &yellowstone_fixed_regulator_supply[1],
		},
};

struct fixed_voltage_config yellowstone_fixed_pdata[2] = {
	[0] = {
		.supply_name = "SPKVDD1",
		.microvolts = 3700000,
		.init_data = &yellowstone_fixed_regulator_init_data[0],
		.gpio = -1,
		},
	[1] = {
		.supply_name = "SPKVDD2",
		.microvolts = 3700000,
		.init_data = &yellowstone_fixed_regulator_init_data[1],
		.gpio = -1,
		},
};

static struct platform_device fixed_device[] = {
	[0] = {
		.name = "reg-fixed-voltage",
		.id = 0,
		.dev = {
			.platform_data = &yellowstone_fixed_pdata[0],
			},
		.num_resources = 0,
		},
	[1] = {
		.name = "reg-fixed-voltage",
		.id = 1,
		.dev = {
			.platform_data = &yellowstone_fixed_pdata[1],
			},
		.num_resources = 0,
		},
};

static struct platform_device *fixed_rdev[] __initdata = {
	&fixed_device[0],
	&fixed_device[1],
};

static void yellowstone_fixed_regulator(void)
{
	platform_add_devices(fixed_rdev, ARRAY_SIZE(fixed_rdev));
}

#if defined(CONFIG_SWITCH_HEADSET_HOST_GPIO)
static struct gpio_switch_platform_data headset_switch_device_data = {
	.name = "h2w",
	.gpio = mfp_to_gpio(GPIO23_GPIO),
	.name_on = NULL,
	.name_off = NULL,
	.state_on = NULL,
	.state_off = NULL,
};

static struct platform_device headset_switch_device = {
	.name            = "headset",
	.id              = 0,
	.dev             = {
		.platform_data = &headset_switch_device_data,
	},
};

static int wm8994_gpio_irq(void)
{
	int gpio = mfp_to_gpio(GPIO23_GPIO);

	if (gpio_request(gpio, "wm8994 irq")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio);
		return -1;
	}

	gpio_direction_input(gpio);
	mdelay(1);
	gpio_free(gpio);
	return 0;
}

static void __init yellowstone_init_headset(void)
{
	wm8994_gpio_irq();
	platform_device_register(&headset_switch_device);
}
#endif

#ifdef CONFIG_SD8XXX_RFKILL
static void mmp3_8787_set_power(unsigned int on)
{
	static struct regulator *v_ldo19;
	static struct regulator *v_ldo17;
	static int f_enabled = 0;
	/* v_ldo19 3.3v */
	/* v_ldo17 1.2v for 32k clk pull up*/
	if (on && (!f_enabled)) {
		v_ldo17 = regulator_get(NULL, "v_ldo17");
		if (IS_ERR(v_ldo17)) {
			v_ldo17 = NULL;
			printk(KERN_ERR"get v_ldo17 failed %s %d \n", __func__, __LINE__);
		} else {
			regulator_set_voltage(v_ldo17, 1200000, 1200000);
			regulator_enable(v_ldo17);
			f_enabled = 1;
		}

		v_ldo19 = regulator_get(NULL, "v_ldo19");
		if (IS_ERR(v_ldo19)) {
			v_ldo19 = NULL;
			printk(KERN_ERR"get v_ldo19 failed %s %d \n", __func__, __LINE__);
		} else {
			regulator_set_voltage(v_ldo19, 3300000, 3300000);
			regulator_enable(v_ldo19);
			f_enabled = 1;
		}
	}

	if (f_enabled && (!on)) {
		if (v_ldo17) {
			regulator_disable(v_ldo17);
			regulator_put(v_ldo17);
			v_ldo17 = NULL;
		}
		if (v_ldo19) {
			regulator_disable(v_ldo19);
			regulator_put(v_ldo19);
			v_ldo19 = NULL;
		}
		f_enabled = 0;
	}
}
#endif

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
	struct regulator *v_ldo16;
	v_ldo16 = regulator_get(NULL, "v_ldo16");
	if (IS_ERR(v_ldo16)) {
		v_ldo16 = NULL;
		pr_err("%s: enable ldo16 for touch fail!\n", __func__);
		return -EIO;
	}
	else {
		if (on) {
			regulator_set_voltage(v_ldo16, 3300000, 3300000);
			regulator_enable(v_ldo16);
		}
		else {
			regulator_disable(v_ldo16);
			v_ldo16 = NULL;
		}
		msleep(100);
		regulator_put(v_ldo16);
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

	gpio = mfp_to_gpio(GPIO85_GPIO);
	if (gpio_request(gpio, "N-trig Power")) {
		pr_err("gpio %d request failed\n", gpio);
		return -1;
	}

	gpio_direction_output(gpio, 0);
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

static int abilene_board_reset(char mode, const char *cmd)
{
#ifdef CONFIG_INPUT_MAX8925_ONKEY
	extern void max8925_system_restart(char mode, const char *cmd);
	max8925_system_restart(mode, cmd);
#endif
	return 1;
}

static void __init yellowstone_init(void)
{
	extern int (*board_reset)(char mode, const char *cmd);
	board_reset = abilene_board_reset;
	mfp_config(ARRAY_AND_SIZE(yellowstone_pin_config));

	/* on-chip devices */
	mmp3_add_uart(3);
	mmp3_add_twsi(1, &twsi1_pdata, ARRAY_AND_SIZE(yellowstone_twsi1_info));
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

	mmp3_add_twsi(3, NULL, ARRAY_AND_SIZE(yellowstone_twsi3_info));
	yellowstone_fixed_regulator();
	wm8994_ldoen();

	/* audio sspa support */
	mmp3_add_sspa(1);
	mmp3_add_sspa(2);
	mmp3_add_audiosram(&mmp3_audiosram_info);

#if defined(CONFIG_SWITCH_HEADSET_HOST_GPIO)
	yellowstone_init_headset();
#endif

#if defined(CONFIG_VIDEO_MV)
	platform_device_register(&abilene_ov5642);
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
}

MACHINE_START(YELLOWSTONE, "YellowStone")
	.map_io		= mmp_map_io,
	.nr_irqs	= YELLOWSTONE_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.reserve	= mmp3_reserve,
	.init_machine	= yellowstone_init,
MACHINE_END
