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
#include <linux/switch.h>
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

#include "common.h"
#include "onboard.h"

#define MK2_NR_IRQS		(IRQ_BOARD_START + 64)

static unsigned long mk2_pin_config[] __initdata = {
	/* UART3 */
	GPIO51_UART3_RXD,
	GPIO52_UART3_TXD,

	/* UART2 GPS*/
	GPIO47_UART2_RXD,
	GPIO48_UART2_TXD,
	GPIO49_UART2_CTS,
	GPIO50_UART2_RTS,

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

	/* CM3623 INT */
	GPIO138_GPIO | MFP_PULL_HIGH,

	/* Camera */
	GPIO67_GPIO,
	GPIO73_CAM_MCLK,

	GPIO0_GPIO,	/* OV5642_RST_N */
	GPIO64_GPIO,	/* OV5642_POWER_DOWN */
	GPIO69_GPIO,	/* OV5642 crystal enable */

	GPIO1_GPIO,	/* OV2656_RST_N */
	GPIO68_GPIO,	/* OV2656_POWER_DOWN */
	GPIO70_GPIO,	/* OV2656 crystal enable */

	/* Camera Flasher */
	GPIO12_GPIO, /* FLASH_EN */

	/* Gyroscope L3G4200D */
	GPIO65_GPIO, /* GYRO_INT_1 */
	GPIO66_GPIO, /* GYRO_INT_2 */

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
	GPIO160_GPIO,

	/* mk2 hp detect */
	GPIO130_GPIO,
	GPIO139_GPIO,

	/* mk2 gpio keypad */
	GPIO147_GPIO,
	GPIO148_GPIO,
	GPIO150_GPIO,
	GPIO154_GPIO,

	PMIC_PMIC_INT | MFP_LPM_EDGE_FALL,
	GPIO06_WM8994_LDOEN,
	/* LCD */
	GPIO152_VLCD_3V3,
	GPIO128_LCD_RST,
	/* backlight */
	GPIO17_BL_EN,

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
	GPIO63_BB_POWER_EN,

	/* SSP4 */
	GPIO78_SSP_CLK,
	GPIO79_SSP_FRM,
	GPIO80_SSP_TXD,
	GPIO81_SSP_RXD,
	GPIO101_GPIO, /* TS INT*/
	GPIO85_GPIO, /* TS_IO_EN */

	/* touch for mk2*/
	GPIO153_GPIO,
	GPIO101_GPIO,

	/* HDMI */
	GPIO54_HDMI_CEC,
	GPIO59_HDMI_DET,

	/* V_GSEN_EN  G-SENSOR POWER*/
	GPIO87_GPIO,
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
	int crystal_enable = mfp_to_gpio(GPIO69_GPIO);

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

	if (on) {
		regulator_set_voltage(dovdd_1v8, 1800000, 1800000);
		regulator_enable(dovdd_1v8);

		regulator_set_voltage(avdd_2v8, 2800000, 2800000);
		regulator_enable(avdd_2v8);

		gpio_direction_output(crystal_enable, 1);
		gpio_direction_output(cam_reset, 0);
		gpio_direction_output(cam_enable, 1);
		msleep(10);

		gpio_direction_output(cam_enable, 0);
		msleep(10);
		gpio_direction_output(cam_reset, 1);
		msleep(10);
	} else {
		gpio_direction_output(crystal_enable, 0);
		gpio_direction_output(cam_enable, 0);
		gpio_direction_output(cam_reset, 0);

		regulator_disable(avdd_2v8);
		regulator_disable(dovdd_1v8);

	}

	gpio_free(crystal_enable);
	gpio_free(cam_enable);
	gpio_free(cam_reset);

	return 0;

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
	.priv = "pxa2128-mipi",
};

static struct platform_device mk2_ov5642 = {
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
	.name = "MK2",
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

/* mk2 GPIO Keyboard */
#define INIT_KEY(_code, _gpio, _active_low, _desc)	\
	{						\
		.code       = KEY_##_code,		\
		.gpio       = _gpio,			\
		.active_low = _active_low,		\
		.desc       = _desc,			\
		.type       = EV_KEY,			\
		.wakeup     = 1,			\
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
	int gsen_pwr_en = mfp_to_gpio(GPIO87_GPIO);

	/* GPIO power enable */
	if (gpio_request(gsen_pwr_en, "GSENSOR Enable")) {
		printk(KERN_INFO "gpio %d request failed\n", gsen_pwr_en);
		return -1;
	}
#if defined(CONFIG_SENSORS_LSM303DLHC_ACC)
	if (!strcmp(device_name, LSM303DLHC_ACC_DEV_NAME))
		device_index = 0;
#endif
#if defined(CONFIG_SENSORS_LSM303DLHC_MAG)
	if (!strcmp(device_name, LSM303DLHC_MAG_DEV_NAME))
		device_index = 1;
#endif

	if ((device_index >= 0) && (device_index <= 2)) {
		if (on && (!is_enabled[device_index])) {
			pmic_2p8v_sens[device_index] = regulator_get(NULL, "pmic_2p8v_sens");
			if (IS_ERR(pmic_2p8v_sens[device_index])) {
				pmic_2p8v_sens[device_index] = NULL;
				return -ENODEV;
			} else {
				regulator_set_voltage(pmic_2p8v_sens[device_index], 2800000, 2800000);
				regulator_enable(pmic_2p8v_sens[device_index]);
				is_enabled[device_index] = 1;
			}

			gpio_direction_output(gsen_pwr_en, 1);
			gpio_free(gsen_pwr_en);
			/* printk( "LSM303 G-sensor/M-sensor power enable\r\n"); */
		}
		if ((!on) && is_enabled[device_index]) {
			regulator_disable(pmic_2p8v_sens[device_index]);
			regulator_put(pmic_2p8v_sens[device_index]);
			pmic_2p8v_sens[device_index] = NULL;
			is_enabled[device_index] = 0;

          gpio_direction_output(gsen_pwr_en, 0);
	       gpio_free(gsen_pwr_en);
			/* printk( "LSM303 G-sensor/M-sensor power disable\r\n"); */
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
	.negate_z = 1,
	.set_power = motion_sensor_set_power,
};
#endif

static int cm3623_set_power(int on)
{
	static struct regulator *pmic_2p8v_sens;
	static int enabled;
	int changed = 0;

	if (on && (!enabled)) {
		pmic_2p8v_sens = regulator_get(NULL, "pmic_2p8v_sens");
		if (IS_ERR(pmic_2p8v_sens)) {
			pmic_2p8v_sens = NULL;
			return -EIO;
		} else {
			regulator_set_voltage(pmic_2p8v_sens, 2800000, 2800000);
			regulator_enable(pmic_2p8v_sens);
			enabled = 1;
			changed = 1;
		}
	}
	if ((!on) && enabled) {
		regulator_disable(pmic_2p8v_sens);
		regulator_put(pmic_2p8v_sens);
		pmic_2p8v_sens = NULL;
		enabled = 0;
		changed = 1;
	}
	if (changed)
		msleep(100);

	return 0;
}

static struct axis_sensor_platform_data cm3623_platform_data = {
	.set_power	= cm3623_set_power,
};

static struct i2c_board_info mk2_twsi4_info[] = {
#if defined(CONFIG_SENSORS_CM3623)
	{
		.type		= "cm3623_ps",
		.addr		= (0xB0>>1),
		.platform_data	= &cm3623_platform_data,
	},
	{
		.type		= "cm3623_als_msb",
		.addr		= (0x20>>1),
		.platform_data	= &cm3623_platform_data,
	},
	{
		.type		= "cm3623_als_lsb",
		.addr		= (0x22>>1),
		.platform_data	= &cm3623_platform_data,
	},
	{
		.type		= "cm3623_int",
		.addr		= (0x18>>1),
		.platform_data	= &cm3623_platform_data,
	},
	{
		.type		= "cm3623_ps_threshold",
		.addr		= (0xB2>>1),
		.platform_data  = &cm3623_platform_data,
	},
#endif
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
	u8 data = 0x0;
	/* DVS related part */
	max77601_read(chip, MAX77601_AME_GPIO, &data, 1);
	if ((data & MAX77601_AME5_MASK) == MAX77601_AME5_MASK)
		printk(KERN_INFO "Max77601 SD0 is set to support DVS!\n");
	/* Set GPIO4 to alternative mode to enable ext_32K_in */
	max77601_set_bits(chip, MAX77601_AME_GPIO, \
		MAX77601_AME4_MASK, MAX77601_AME4_MASK);

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
	[MAX77601_ID_L0] = REG_INIT(L0, 800000, 2350000, 0, 0),
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

static struct i2c_board_info mk2_twsi1_info[] = {
#if defined(CONFIG_WIS_KBC_POWER)
	{
		.type		= "wis-kbc-power",
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
	.pwm_period_ns = 2000000,
};

static struct platform_device mk2_lcd_backlight_devices = {
	.name = "pwm-backlight",
	.id = 2,
	.dev = {
		.platform_data = &mk2_lcd_backlight_data,
	},
};

#ifdef CONFIG_MMC_SDHCI_PXAV3
#include <linux/mmc/host.h>
static void mk2_sd_signal_1v8(int set)
{
	static struct regulator *v_ldo_sd;
	int vol;

	v_ldo_sd = regulator_get(NULL, "pmic_sdmmc");
	if (IS_ERR(v_ldo_sd)) {
		printk(KERN_ERR "Failed to get pmic_sdmmc\n");
		return;
	}

	vol = set ? 1800000 : 3000000;
	regulator_set_voltage(v_ldo_sd, vol, vol);
	regulator_enable(v_ldo_sd);

	mmp3_io_domain_1v8(AIB_SDMMC_IO_REG, set);

	msleep(10);
	regulator_put(v_ldo_sd);
}

#ifdef CONFIG_SD8XXX_RFKILL
static void mmp3_8787_set_power(unsigned int on)
{
	/*
	 * FIXME: 32K_CLK is shared with pmic io interface power on B0 with max77601
	 * Don't touch this domain. But can be controlled on B0 with Ustica, add
	 * operation later for Ustica.
	 */
#if 0
	static struct regulator *v_ldo17;
	static int f_enabled = 0;
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
	}

	if (f_enabled && (!on)) {
		if (v_ldo17) {
			regulator_disable(v_ldo17);
			regulator_put(v_ldo17);
			v_ldo17 = NULL;
		}
		f_enabled = 0;
	}
#endif
}
#endif

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc0 = {
	.clk_delay_cycles	= 0x1F,
	.host_caps_disable	=
			MMC_CAP_UHS_SDR12 |
			MMC_CAP_UHS_SDR25 |
			MMC_CAP_UHS_SDR104 |
			MMC_CAP_UHS_SDR50 |
			MMC_CAP_UHS_DDR50,
	/*
	  * FIXME: pmic_sdmmc is fixed 2.8V on B0 with max77601,
	  * can not support 1.8V signal function
	  */
	/* .signal_1v8		= mk2_sd_signal_1v8, */
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc1 = {
	.flags          = PXA_FLAG_CARD_PERMANENT,
	.pm_caps	= MMC_PM_KEEP_POWER | MMC_PM_IRQ_ALWAYS_ON,
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc2 = {
	.flags		= PXA_FLAG_SD_8_BIT_CAPABLE_SLOT,
	.clk_delay_cycles	= 0xF,
};

static void __init mk2_init_mmc(void)
{
#ifdef CONFIG_SD8XXX_RFKILL
	int WIB_PDn = mfp_to_gpio(GPIO57_GPIO);
	int WIB_RESETn = mfp_to_gpio(GPIO58_GPIO);
	add_sd8x_rfkill_device(WIB_PDn, WIB_RESETn,\
			&mmp3_sdh_platdata_mmc1.pmmc, mmp3_8787_set_power);
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
	static struct regulator *pmic_1p2v_hsic;

	printk(KERN_INFO "%s: set %d\n", __func__, vbus);
	if (vbus) {
		if (!pmic_1p2v_hsic) {
			pmic_1p2v_hsic = regulator_get(NULL, "pmic_1p2v_hsic");
			if (IS_ERR(pmic_1p2v_hsic)) {
				printk(KERN_INFO "ldo5 not found\n");
				return -EIO;
			}
			regulator_set_voltage(pmic_1p2v_hsic, 1200000, 1200000);
			regulator_enable(pmic_1p2v_hsic);
			printk(KERN_INFO "%s: enable regulator\n", __func__);
			udelay(2);
		}

		mmp3_hsic1_reset();
	} else {
		if (pmic_1p2v_hsic) {
			regulator_disable(pmic_1p2v_hsic);
			regulator_put(pmic_1p2v_hsic);
			pmic_1p2v_hsic = NULL;
		}
	}

	return 0;
}

static char *mmp3_hsic1_clock_name[] = {
	[0] = "U2OCLK",
	[1] = "HSIC1CLK",
};

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
	.set_vbus	= mmp3_hsic1_set_vbus,
	.private_init	= mmp3_hsic_private_init,
};

#endif

#endif

#if defined(CONFIG_TC35876X)
/* force pmic_1p2v_mipi & pmic_1p2v_mipi_logic always on */
static int tc358765_init(void)
{
	struct regulator *vcc = NULL;
	int ret = 0, mipi_rst = mfp_to_gpio(GPIO128_LCD_RST);

	/* LVDS power enable */
	if (gpio_request(mipi_rst, "lcd reset gpio")) {
		printk(KERN_INFO "gpio %d request failed\n", mipi_rst);
		return -1;
	}
	gpio_direction_output(mipi_rst, 0);

	/* enable LDO1 & LDO7 for MIPI bridge */
	vcc = regulator_get(NULL, "PMIC_LDO1");
	if (IS_ERR(vcc))
		vcc = NULL;
	else {
		ret = regulator_set_voltage(vcc, 1200000, 1200000);
		regulator_enable(vcc);
		regulator_put(vcc);
	}
	vcc = regulator_get(NULL, "PMIC_LDO7");
	if (IS_ERR(vcc))
		vcc = NULL;
	else {
		ret = regulator_set_voltage(vcc, 1200000, 1200000);
		regulator_enable(vcc);
		regulator_put(vcc);
	}

	mdelay(5);
	gpio_direction_output(mipi_rst, 1);
	gpio_free(mipi_rst);

	return ret;
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

static struct regulator_consumer_supply mk2_wm8994_regulator_supply[] = {
	[0] = {
		.supply = "AVDD1",
		},
	[1] = {
		.supply = "DCVDD",
		},
};

struct regulator_init_data mk2_wm8994_regulator_init_data[] = {
	[0] = {
		.constraints = {
				.name = "wm8994-ldo1",
				.min_uV = 2400000,
				.max_uV = 3100000,
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &mk2_wm8994_regulator_supply[0],
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
		.consumer_supplies = &mk2_wm8994_regulator_supply[1],
		},
};

struct wm8994_pdata mk2_wm8994_pdata = {
	.ldo[0] = {
			.enable = 0,
			.init_data = &mk2_wm8994_regulator_init_data[0],
			.supply = "AVDD1",

		},
	.ldo[1] = {
		.enable = 0,
		.init_data = &mk2_wm8994_regulator_init_data[1],
		.supply = "DCVDD",

		},
};


static struct regulator_consumer_supply mk2_fixed_regulator_supply[] = {
	[0] = {
		.supply = "SPKVDD1",
		},
	[1] = {
		.supply = "SPKVDD2",
		},
};

struct regulator_init_data mk2_fixed_regulator_init_data[] = {
	[0] = {
		.constraints = {
				.name = "wm8994-SPK1",
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &mk2_fixed_regulator_supply[0],
		},
	[1] = {
		.constraints = {
				.name = "wm8994-SPK2",
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &mk2_fixed_regulator_supply[1],
		},
};

struct fixed_voltage_config mk2_fixed_pdata[2] = {
	[0] = {
		.supply_name = "SPKVDD1",
		.microvolts = 3700000,
		.init_data = &mk2_fixed_regulator_init_data[0],
		.gpio = -1,
		},
	[1] = {
		.supply_name = "SPKVDD2",
		.microvolts = 3700000,
		.init_data = &mk2_fixed_regulator_init_data[1],
		.gpio = -1,
		},
};

static struct platform_device fixed_device[] = {
	[0] = {
		.name = "reg-fixed-voltage",
		.id = 0,
		.dev = {
			.platform_data = &mk2_fixed_pdata[0],
			},
		.num_resources = 0,
		},
	[1] = {
		.name = "reg-fixed-voltage",
		.id = 1,
		.dev = {
			.platform_data = &mk2_fixed_pdata[1],
			},
		.num_resources = 0,
		},
};

static struct platform_device *fixed_rdev[] __initdata = {
	&fixed_device[0],
	&fixed_device[1],
};

static void mk2_fixed_regulator(void)
{
	platform_add_devices(fixed_rdev, ARRAY_SIZE(fixed_rdev));
}

#if defined(CONFIG_SWITCH_HEADSET_HOST_GPIO)
static struct gpio_switch_platform_data headset_switch_device_data = {
	.name = "h2w",
	.gpio = mfp_to_gpio(GPIO130_GPIO),
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
	int gpio = mfp_to_gpio(GPIO130_GPIO);

	if (gpio_request(gpio, "wm8994 irq")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio);
		return -1;
	}

	gpio_direction_input(gpio);
	mdelay(1);
	gpio_free(gpio);
	return 0;
}

static void __init mk2_init_headset(void)
{
	wm8994_gpio_irq();
	platform_device_register(&headset_switch_device);
}
#endif

#ifdef CONFIG_UIO_HDMI
static struct uio_hdmi_platform_data mmp3_hdmi_info __initdata = {
	.sspa_reg_base = 0xD42A0C00,
	/* Fix me: gpio 59 lpm pull ? */
	.gpio = mfp_to_gpio(GPIO59_HDMI_DET),
	.edid_bus_num = 6,
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
	},
#endif /* CONFIG_TOUCHSCREEN_EGALAX_I2C */
};

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

static int hdmi_power_on(void)
{
	int hdmi_pwr_en = mfp_to_gpio(GPIO160_GPIO);

	if (gpio_request(hdmi_pwr_en, "hdmi_pwr_en")){
		printk(KERN_ERR "Request GPIO failed, gpio: %d.\n", hdmi_pwr_en);
		return -1;
	}
	printk("%s: hdmi_pwr_en.\n", __FUNCTION__);
	gpio_direction_output(hdmi_pwr_en, 1);    /* power on */

	gpio_free(hdmi_pwr_en);
	return 0;
}

static int usb_hub_power_on (int onoff)
{
	/*int usbhub_pwr_en = mfp_to_gpio(GPIO166_GPIO);
	For B0 board hub power is controlled by LDO */
	int reset = mfp_to_gpio(GPIO96_HSIC_RESET);

	if (gpio_request(reset, "hsic reset")) {
		pr_err("Failed to request hsic reset gpio\n");
		return -EIO;
	}

	gpio_direction_output(reset, ~onoff);
	mdelay(20);
	gpio_direction_output(reset, onoff);
	gpio_free(reset);

	return 0;
}


static void __init mk2_init(void)
{
	mfp_config(ARRAY_AND_SIZE(mk2_pin_config));

	/* on-chip devices */
	mmp3_add_uart(3);
	mmp3_add_twsi(1, NULL, ARRAY_AND_SIZE(mk2_twsi1_info));
	mmp3_add_twsi(4, NULL, ARRAY_AND_SIZE(mk2_twsi4_info));
	mmp3_add_twsi(3, NULL, ARRAY_AND_SIZE(mk2_twsi3_info));
	mmp3_add_twsi(5, NULL, ARRAY_AND_SIZE(mk2_twsi5_info));
	/* for hdmi edid */
	/* mmp3_add_twsi(6, NULL, ARRAY_AND_SIZE(mk2_twsi6_info));*/

	platform_device_register(&gpio_keys);
	hdmi_power_on();
	usb_hub_power_on(1);


	mmp3_add_videosram(&mmp3_videosram_info);
#ifdef CONFIG_FB_PXA168
	mk2_add_lcd_mipi();
	mmp3_add_tv_out();
#endif

#ifdef CONFIG_UIO_HDMI
	/* mmp3_add_hdmi(&mmp3_hdmi_info); */
#endif
	/* backlight */
	mmp3_add_pwm(3);
	platform_device_register(&mk2_lcd_backlight_devices);

#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem();
#endif

#ifdef CONFIG_UIO_VMETA
	/* mmp_init_vmeta(); */
#endif

#ifdef CONFIG_MMC_SDHCI_PXAV3
	mk2_init_mmc();
#endif /* CONFIG_MMC_SDHCI_PXAV3 */

#if defined(CONFIG_VIDEO_MV)
	platform_device_register(&mk2_ov5642);
	mmp3_add_cam(0, &mv_cam_data);
#endif

	platform_device_register(&mmp3_device_rtc);

	mk2_fixed_regulator();
	wm8994_ldoen();

	/* audio sspa support */
	mmp3_add_sspa(1);
	mmp3_add_sspa(2);
	mmp3_add_audiosram(&mmp3_audiosram_info);

#if defined(CONFIG_SWITCH_HEADSET_HOST_GPIO)
	mk2_init_headset();
#endif

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
	/* platform_device_register(&mmp3_hsic2_device); */
#endif

	gsensor_power_en(1);
/* enable EETI EXC7200 touch controller */
#ifdef CONFIG_TOUCHSCREEN_EGALAX_I2C
	/* Configure irq for exc7200 touch panel */
	exc7200_config();
#endif /*CONFIG_TOUCHSCREEN_EGALAX_I2C */
	/* If we have a full configuration then disable any regulators
	 * which are not in use or always_on. */
	regulator_has_full_constraints();
}

MACHINE_START(MK2, "mk2")
	.map_io		= mmp_map_io,
	.nr_irqs	= MK2_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.reserve	= mmp3_reserve,
	.init_machine	= mk2_init,
MACHINE_END
