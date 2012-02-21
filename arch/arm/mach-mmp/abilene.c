/*
 *  linux/arch/arm/mach-mmp/abilene.c
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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/smc91x.h>
#include <linux/mfd/max8925.h>
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
#if defined(CONFIG_SENSORS_L3G4200D_GYR)
#include <linux/i2c/l3g4200d.h>
#endif
#if defined(CONFIG_SPI_PXA2XX)
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/ntrig_spi.h>
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
#include <plat/pmem.h>
#include <plat/usb.h>
#include <mach/sram.h>
#include <mach/axis_sensor.h>
#include <mach/uio_hdmi.h>
#include <media/soc_camera.h>
#include <linux/power/max8903_charger.h>

#include "common.h"
#include "onboard.h"

#define ABILENE_NR_IRQS		(IRQ_BOARD_START + 64)

static unsigned long abilene_pin_config[] __initdata = {
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

	/* CM3623 INT */
	GPIO138_GPIO | MFP_PULL_HIGH,

	/* camera */
	GPIO67_GPIO,
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
	GPIO62_VBUS_EN,

	/* Charger(max8903) control GPIOs */
	GPIO65_GPIO,	/* CHG_500MA */
	GPIO66_GPIO,	/* CHG_2A_N */

	/* HSIC1 reset pin*/
	GPIO96_HSIC_RESET,

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

	/* HDMI */
	GPIO54_HDMI_CEC,
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

#ifdef CONFIG_VIDEO_MVISP_OV8820
static int ov8820_sensor_power_on(int on, int flag)
{
	struct regulator *vcc_af;
	struct regulator *vcc_camera;
	struct regulator *pmic_1p2v_mipi;
	int sensor_power = mfp_to_gpio(MFP_PIN_GPIO67);

	if (gpio_request(sensor_power, "CAM_ENABLE_HI_SENSOR"))
		return -EIO;

	vcc_af = regulator_get(NULL, "vcc_af");
	if (IS_ERR(vcc_af)) {
		vcc_af = NULL;
		return -EIO;
	}
	vcc_camera = regulator_get(NULL, "vcc_camera");
	if (IS_ERR(vcc_camera)) {
		vcc_camera = NULL;
		return -EIO;
	}
	pmic_1p2v_mipi = regulator_get(NULL, "pmic_1p2v_mipi");
	if (IS_ERR(pmic_1p2v_mipi)) {
		pmic_1p2v_mipi = NULL;
		return -EIO;
	}

	/* Enable voltage for camera sensor OV8820 */
	/* vcc_af fixed 2.8v on B0 with max77601 (AFVCC 2.8 - 3.3V, 3.3V recommended) */
	/* vcc_camera 2.8v (AVDD 2.6 - 3.1V)*/
	/* pmic_1p2v_mipi MIPI BRIDGE CHIP PLL, 1.2V */
	if (on) {
		regulator_set_voltage(vcc_af, 2800000, 2800000);
		regulator_enable(vcc_af);
		regulator_set_voltage(vcc_camera, 2800000, 2800000);
		regulator_enable(vcc_camera);
		regulator_set_voltage(pmic_1p2v_mipi, 1200000, 1200000);
		regulator_enable(pmic_1p2v_mipi);
	} else {
		regulator_disable(vcc_af);
		regulator_disable(vcc_camera);
		regulator_disable(pmic_1p2v_mipi);
	}

       /* sensor_power is low active, reset the sensor now*/
	gpio_direction_output(sensor_power, 0);
	mdelay(20);
	/* sensor_power is low active, enable the sensor now*/
	gpio_direction_output(sensor_power, 1);
	gpio_free(sensor_power);

	regulator_put(vcc_af);
	regulator_put(vcc_camera);
	regulator_put(pmic_1p2v_mipi);
	msleep(100);

	return 0;
}

static struct sensor_platform_data ov8820_platdata = {
	.id = 0,
	.power_on = ov8820_sensor_power_on,
	.platform_set = NULL,
};

static struct i2c_board_info ov8820_info = {
	.type = "ov8820",
	.addr = 0x36,
	.platform_data = &ov8820_platdata,
};

static struct mvisp_subdev_i2c_board_info ov8820_isp_info[] = {
	[0] = {
		.board_info = &ov8820_info,
		.i2c_adapter_id = 2,
	},
	[1] = {
		.board_info = NULL,
		.i2c_adapter_id = 0,
	},
};

static struct mvisp_v4l2_subdevs_group dxoisp_subdevs_group[] = {
	[0] = {
		.i2c_board_info = ov8820_isp_info,
		.if_type = ISP_INTERFACE_CCIC_1,
	},
	[1] = {
		.i2c_board_info = NULL,
		.if_type = 0,
	},
};
#endif

#ifdef CONFIG_VIDEO_MVISP
#ifndef CONFIG_VIDEO_MVISP_OV8820
static struct mvisp_v4l2_subdevs_group dxoisp_subdevs_group[] = {
	[0] = {
		.i2c_board_info = NULL,
		.if_type = 0,
	},
};
#endif

static struct mvisp_platform_data mmp_dxoisp_plat_data = {
	.subdev_group = dxoisp_subdevs_group,
};

static void __init mmp_init_dxoisp(void)
{
	mmp_register_dxoisp(&mmp_dxoisp_plat_data);
}
#endif


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
	.oe_inverted = 0,/*magic number print from vendor's code*/
	.pwr_gpio = -1,
	.irq_flags = IRQF_DISABLED | IRQF_TRIGGER_RISING,
	.set_power = NULL,
};

static struct spi_board_info __initdata ntrig_spi_board_info[] = {
	{
		.modalias = "ntrig_spi",
		.bus_num = 5,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.max_speed_hz = 6500000,
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
	gpio_free(gpio);

	gpio = mfp_to_gpio(GPIO85_GPIO);
	if (gpio_request(gpio, "N-trig Power")) {
		pr_err("gpio %d request failed\n", gpio);
		return -1;
	}

	gpio_direction_output(gpio, 1);
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
#endif

#if defined(CONFIG_VIDEO_MV)
/* soc  camera */
static int camera_sensor_power(struct device *dev, int on)
{
	int cam_enable = mfp_to_gpio(MFP_PIN_GPIO67);
	struct regulator *pmic_1p2v_mipi;

	/* We rely on mipi brige also connect the mipi signal */
	/* pmic_1p2v_mipi MIPI BRIDGE CHIP PLL, 1.2V */
	pmic_1p2v_mipi = regulator_get(NULL, "pmic_1p2v_mipi");
	if (IS_ERR(pmic_1p2v_mipi)) {
		pmic_1p2v_mipi = NULL;
		return -EIO;
	}
	if (on) {
		regulator_set_voltage(pmic_1p2v_mipi, 1200000, 1200000);
		regulator_enable(pmic_1p2v_mipi);
	} else
		regulator_disable(pmic_1p2v_mipi);

	regulator_put(pmic_1p2v_mipi);

	if (gpio_request(cam_enable, "CAM_ENABLE_HI_SENSOR")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", cam_enable);
		return -EIO;
	}

	if (on)
		/* pull down camera pwdn pin to enable camera sensor */
		gpio_direction_output(cam_enable, 0);
	else
		/* pull up camera pwdn pin to disable camera sensor */
		gpio_direction_output(cam_enable, 1);

	gpio_free(cam_enable);
	mdelay(10);

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
	.i2c_adapter_id = 2,
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
	static struct regulator *vcc_af;
	static struct regulator *vcc_camera;
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
		vcc_af = regulator_get(NULL, "vcc_af");
		if (IS_ERR(vcc_af)) {
			vcc_af = NULL;
			return -EIO;
		} else {
			regulator_set_voltage(vcc_af, 2800000, 2800000);
			regulator_enable(vcc_af);
		}

		vcc_camera = regulator_get(NULL, "vcc_camera");
		if (IS_ERR(vcc_camera)) {
			vcc_camera = NULL;
			return -EIO;
		} else {
			regulator_set_voltage(vcc_camera, 2800000, 2800000);
			regulator_enable(vcc_camera);
		}
	} else {
		if (vcc_af) {
			regulator_disable(vcc_af);
			regulator_put(vcc_af);
			vcc_af = NULL;
		}
		if (vcc_camera) {
			regulator_disable(vcc_camera);
			regulator_put(vcc_camera);
			vcc_camera = NULL;
		}

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
	.direct_key_msk = 0x50, /* only KEY_VOLUMEUP and KEY_VOLUMEDOWN */
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
	[MAX8925_ID_LDO15] =
		{
			.constraints = {
				.name		= "LDO15",
				.min_uV		= 2800000,
				.max_uV		= 2800000,
				.always_on	= 1,
				.boot_on	= 0,
				},
			.num_consumer_supplies	= 1,
			.consumer_supplies	=
				&regulator_supply[MAX8925_ID_LDO15],
		},
	[MAX8925_ID_LDO16] = REG_INIT(LDO16, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO17] = REG_INIT(LDO17, 1000000, 1500000, 0, 0),
	[MAX8925_ID_LDO18] = REG_INIT(LDO18, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO19] = REG_INIT(LDO19, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO20] = REG_INIT(LDO20, 750000, 3900000, 1, 1),
};

/* max8925 power for battery/charger */
static struct max8925_power_pdata abilene_power_data = {
	.batt_detect		= 0,	/* can't detect battery by ID pin */
	.topoff_threshold	= MAX8925_TOPOFF_THR_10PER,
	.fast_charge		= MAX8925_FCHG_1000MA,
	.bat_max8925_en		= 1,	/* battery monitor en/disable */
	.chg_port_config	= CHG_PORT_WALL,	/* PMIC wired config */
};

static struct max8925_platform_data abilene_max8925_info = {
	.irq_base		= IRQ_BOARD_START,
	.power			= &abilene_power_data,

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

#ifdef CONFIG_CHARGER_MAX8903
#define MAX8903_PIN(_id, _gpio, _active_low, _desc)	\
{	\
	.id			= MAX8903_PIN_##_id,	\
	.gpio		= _gpio,				\
	.active_low	= _active_low,			\
	.desc		= _desc,				\
}

static struct max8903_gpio abilene_charger_gpios[] = {
	MAX8903_PIN(DCM,  mfp_to_gpio(GPIO66_GPIO), 1, "CHG_2A_N"),
	MAX8903_PIN(IUSB, mfp_to_gpio(GPIO65_GPIO), 0, "CHG_500MA"),
};

/* Batteries supplied to */
static char *max8903_supplied_to[] = {
	"max8925-battery",
};

static struct max8903_pdata abilene_charger_pdata = {
	.gpios		= abilene_charger_gpios,
	.gpio_nums	= ARRAY_SIZE(abilene_charger_gpios),
	.dc_valid	= 1,	/* DC-in is wired */
	.usb_valid	= 0,	/* USB-in is NOT wired */
	.supplied_to = max8903_supplied_to,
	.num_supplicants = ARRAY_SIZE(max8903_supplied_to),
};

static struct platform_device max8903_charger_device = {
	.name = "max8903-charger",
	.id = -1,
	.dev = {
		.platform_data = &abilene_charger_pdata,
	},
};
#endif	/* CONFIG_CHARGER_MAX8903 */

static int motion_sensor_set_power(int on, const char *device_name)
{
	static struct regulator *pmic_2p8v_sens[3];
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
			pmic_2p8v_sens[device_index] = regulator_get(NULL, "pmic_2p8v_sens");
			if (IS_ERR(pmic_2p8v_sens[device_index])) {
				pmic_2p8v_sens[device_index] = NULL;
				return -ENODEV;
			} else {
				regulator_set_voltage(pmic_2p8v_sens[device_index], 2800000, 2800000);
				regulator_enable(pmic_2p8v_sens[device_index]);
				is_enabled[device_index] = 1;
			}
		}
		if ((!on) && is_enabled[device_index]) {
			regulator_disable(pmic_2p8v_sens[device_index]);
			regulator_put(pmic_2p8v_sens[device_index]);
			pmic_2p8v_sens[device_index] = NULL;
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
	.negate_x = 0,
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

static struct i2c_board_info abilene_twsi4_info[] = {
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
#if defined(CONFIG_SENSORS_L3G4200D_GYR)
	{
		.type           = L3G4200D_GYR_DEV_NAME,
		.addr           = (0xD2>>1),
		.platform_data  = &l3g4200d_gyr_data,
	},
#endif
};

static struct i2c_pxa_platform_data twsi1_pdata = {
	.use_pio		 = 1,
};

static struct i2c_board_info abilene_twsi1_info[] = {
	{
		.type		= "max8925",
		.addr		= 0x3c,
		.irq		= IRQ_MMP3_PMIC,
		.platform_data	= &abilene_max8925_info,
	},
};

/*
 *  this array is shared by 2 pmic, Ustica or max77601
 *  macro PMIC_POWER_SUPPLY_MAX = max (PM800_ID_RG_MAX,MAX77601_RG_MAX)
 */
#define PMIC_POWER_SUPPLY_MAX MAX77601_VREG_MAX
static struct regulator_consumer_supply abilene_power_supply[PMIC_POWER_SUPPLY_MAX];
static struct regulator_init_data pmic_regulator_data[PMIC_POWER_SUPPLY_MAX];

#define REG_SUPPLY_INIT(_id, _name, _dev_name) \
{						\
	abilene_power_supply[_id].supply =  _name;  \
	abilene_power_supply[_id].dev_name = _dev_name; \
}

#define PMIC_REG_INIT(_id, _name, _min, _max, _always, _boot, _supply, _num) \
{		\
	pmic_regulator_data[_id].constraints.name = __stringify(_name);        \
	pmic_regulator_data[_id].constraints.min_uV = _min;    \
	pmic_regulator_data[_id].constraints.max_uV     = _max;       \
	pmic_regulator_data[_id].constraints.always_on = _always; \
	pmic_regulator_data[_id].constraints.boot_on = _boot; \
	pmic_regulator_data[_id].constraints.valid_ops_mask =  \
			REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS; \
	pmic_regulator_data[_id].consumer_supplies = _supply; \
	pmic_regulator_data[_id].num_consumer_supplies = _num;	\
}

static struct regulator_consumer_supply abilene_max77601_sd3_supply[] = {
	[0] = {
		.supply = "pmic_sdmmc",
	},
	[1] = {
		.supply = "vcc_af",
	},
	[2] = {
		.supply = "pmic_2p8v_sens",
	},
	[3] = {
		.supply = "pmic_lcd",
	},
	[4] = {
		.supply = "pmic_2p8v",
	},
};

/*
  * Use power domain name for supply name, instead of using name like "v_ldo3"
  * It will easily to support the case that driver use API
  * struct regulator *regulator_get(struct device *dev, const char *id)
  * with 2-pmic optional solution in B0
  */
static void abilene_power_supply_init(void)
{
	REG_SUPPLY_INIT(MAX77601_ID_SD0, "vcc_main_lpm", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_DVSSD0, "vcc_main", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_SD1, "v_ddr3", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_DVSSD1, "v_ddr3_dvs", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_SD2, "pmic_1p8v", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_SD3, "pmic_2p8v", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_SD4, "rsv_sd4", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_L0, "pmic_1p2v_hsic", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_L1, "pmic_1p2v_mipi", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_L2, "pmic_3p3v", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_L3, "vcc_camera", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_L4, "rsv_l4", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_L5, "pmic_bb", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_L6, "pmic_1p8v_ana", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_L7, "pmic_1p2v_mipi_logic", NULL);
	REG_SUPPLY_INIT(MAX77601_ID_L8, "pmic_1p2v_codec", NULL);

	PMIC_REG_INIT(MAX77601_ID_SD0, SD0, 600000, 3387500, 1, 1,
		&abilene_power_supply[MAX77601_ID_SD0], 1);
	PMIC_REG_INIT(MAX77601_ID_DVSSD0, DVSSD0, 600000, 3387500, 1, 1,
		&abilene_power_supply[MAX77601_ID_DVSSD0], 1);
	PMIC_REG_INIT(MAX77601_ID_SD1, SD1, 800000, 1587500, 1, 1,
		&abilene_power_supply[MAX77601_ID_SD1], 1);
	PMIC_REG_INIT(MAX77601_ID_DVSSD1, DVSSD1, 800000, 1587500, 1, 1,
		&abilene_power_supply[MAX77601_ID_DVSSD1], 1);
	PMIC_REG_INIT(MAX77601_ID_SD2, SD2, 600000, 3387500, 1, 1,
		&abilene_power_supply[MAX77601_ID_SD2], 1);
	/*
	  * max77601 SD3 is power supply of emmc/vcc_afp/sdmmc and some fixed 2.8V.
	  * It should be always on and kept at 2.8V B0 board with pmic max77601.
	  */
	PMIC_REG_INIT(MAX77601_ID_SD3, SD3, 600000, 3387500, 1, 1,
		&abilene_max77601_sd3_supply[0], ARRAY_SIZE(abilene_max77601_sd3_supply));
	PMIC_REG_INIT(MAX77601_ID_SD4, SD4, 600000, 3387500, 0, 0,
		&abilene_power_supply[MAX77601_ID_SD4], 1);

	PMIC_REG_INIT(MAX77601_ID_L0, LDO0, 800000, 2350000 , 0, 1,
		&abilene_power_supply[MAX77601_ID_L0], 1);
	PMIC_REG_INIT(MAX77601_ID_L1, LDO1, 800000, 2350000 , 0, 1,
		&abilene_power_supply[MAX77601_ID_L1], 1);
	PMIC_REG_INIT(MAX77601_ID_L2, LDO2, 800000, 3950000 , 1, 1,
		&abilene_power_supply[MAX77601_ID_L2], 1);
	PMIC_REG_INIT(MAX77601_ID_L3, LDO3, 800000, 3950000 , 0, 1,
		&abilene_power_supply[MAX77601_ID_L3], 1);
	PMIC_REG_INIT(MAX77601_ID_L4, LDO4, 800000, 1587500 , 0, 0,
		&abilene_power_supply[MAX77601_ID_L4], 1);
	PMIC_REG_INIT(MAX77601_ID_L5, LDO5, 800000, 3950000 , 1, 1,
		&abilene_power_supply[MAX77601_ID_L5], 1);
	PMIC_REG_INIT(MAX77601_ID_L6, LDO6, 800000, 3950000 , 1, 1,
		&abilene_power_supply[MAX77601_ID_L6], 1);
	PMIC_REG_INIT(MAX77601_ID_L7, LDO7, 800000, 3950000 , 0, 1,
		&abilene_power_supply[MAX77601_ID_L7], 1);
	PMIC_REG_INIT(MAX77601_ID_L8, LDO8, 800000, 3950000 , 0, 1,
		&abilene_power_supply[MAX77601_ID_L8], 1);
}

static int abilene_max77601_setup(struct max77601_chip *chip)
{
	u8 data = 0x0;
	/*
	  * Domain which will dynamic power on/off on abilene:
	  * pmic_1p2v_hsic(ldo0), pmic_1p2v_mipi(ldo1), vcc_camera(ldo3),
	  * pmic_bb(ldo5) ,pmic_1p2v_mipi_logic(ldo7),pmic_1p2v_codec(ldo8)
	  * should be set to Not_FPS mode
	  */
	max77601_set_bits(chip, MAX77601_FPS_L0, \
		MAX77601_FPSSRC_MASK, MAX77601_FPSSRC_NOTFPS);
	max77601_set_bits(chip, MAX77601_FPS_L1, \
		MAX77601_FPSSRC_MASK, MAX77601_FPSSRC_NOTFPS);
	max77601_set_bits(chip, MAX77601_FPS_L3, \
		MAX77601_FPSSRC_MASK, MAX77601_FPSSRC_NOTFPS);
	max77601_set_bits(chip, MAX77601_FPS_L5, \
		MAX77601_FPSSRC_MASK, MAX77601_FPSSRC_NOTFPS);
	max77601_set_bits(chip, MAX77601_FPS_L7, \
		MAX77601_FPSSRC_MASK, MAX77601_FPSSRC_NOTFPS);
	max77601_set_bits(chip, MAX77601_FPS_L8, \
		MAX77601_FPSSRC_MASK, MAX77601_FPSSRC_NOTFPS);

	/*
	  * Set pmic_1p2v_codec to 1.2V, for temp usage, will add to codec
	  * power framework if possible later
	  */
	data = (0x3 << 6 ) | 0x8;
	max77601_write(chip, 0x33, &data, 1);

	/* DVS related part */
	max77601_read(chip, MAX77601_AME_GPIO, &data, 1);
	if ((data & MAX77601_AME5_MASK) == MAX77601_AME5_MASK)
		printk(KERN_INFO "Max77601 SD0 is set to support DVS!\n");

	/* Set GPIO4 to alternative mode to enable ext_32K_in */
	max77601_set_bits(chip, MAX77601_AME_GPIO, \
		MAX77601_AME4_MASK, MAX77601_AME4_MASK);

	return 0;
};

static struct max77601_platform_data abilene_max77601_pdata = {
	.irq_base  = IRQ_BOARD_START,
	.regulator = pmic_regulator_data,
	.setup     = abilene_max77601_setup,
};

static struct i2c_board_info abilene_twsi1_max77601_info[] = {
	{
		.type		= "max77601",
		.addr		= 0x1c,
		.irq		= IRQ_MMP3_PMIC,
		.platform_data	= &abilene_max77601_pdata,
	},
};

static struct platform_pwm_backlight_data abilene_lcd_backlight_data = {
	/* primary backlight */
	.pwm_id = 2,
	.max_brightness = 100,
	.dft_brightness = 50,
	.pwm_period_ns = 2000000,
};

static struct platform_device abilene_lcd_backlight_devices = {
	.name = "pwm-backlight",
	.id = 2,
	.dev = {
		.platform_data = &abilene_lcd_backlight_data,
	},
};

#ifdef CONFIG_MMC_SDHCI_PXAV3
#include <linux/mmc/host.h>
static void abilene_sd_signal_1v8(int set)
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
	.quirks			= SDHCI_QUIRK_INVERTED_WRITE_PROTECT,
	/*
	  * FIXME: pmic_sdmmc is fixed 2.8V on B0 with max77601,
	  * can not support 1.8V signal function
	  */
	/* .signal_1v8		= abilene_sd_signal_1v8, */
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc1 = {
	.flags          = PXA_FLAG_CARD_PERMANENT,
	.pm_caps	= MMC_PM_KEEP_POWER | MMC_PM_IRQ_ALWAYS_ON,
	.host_caps	= MMC_CAP_POWER_OFF_CARD,
};

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc2 = {
	.flags		= PXA_FLAG_SD_8_BIT_CAPABLE_SLOT,
	.clk_delay_cycles	= 0xF,
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

static void __init abilene_init_mmc(void)
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

#if defined(CONFIG_TOUCHSCREEN_TPK_R800)
static int tpk_r800_set_power(int on)
{
	struct regulator *vcc = NULL;

	vcc = regulator_get(NULL, "pmic_2p8v");
	if (IS_ERR(vcc)) {
		pr_err("Failed to get regulator pmic_2p8v!\n");
		return -EIO;
	}

	if (on) {
		regulator_enable(vcc);
		regulator_set_voltage(vcc, 2800000, 2800000);
	} else
		regulator_disable(vcc);

	regulator_put(vcc);
	return 1;
}

static struct touchscreen_platform_data tpk_r800_data = {
	.set_power      = tpk_r800_set_power,
};
#endif

#ifdef CONFIG_USB_SUPPORT

#if defined(CONFIG_USB_PXA_U2O) || defined(CONFIG_USB_EHCI_PXA_U2O)

static char *mmp3_usb_clock_name[] = {
	[0] = "U2OCLK",
};

static int pxa_usb_set_vbus(unsigned int vbus)
{
	int gpio = mfp_to_gpio(GPIO62_VBUS_EN);

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

#if defined(CONFIG_TC35876X)
/* force LDO3 & LDO17 always on */
int tc358765_init(void)
{
	struct regulator *vcc = NULL;
	int ret = 0;
	/* enable LDO for MIPI bridge */
	vcc = regulator_get(NULL, "v_ldo17");
	if (IS_ERR(vcc))
		vcc = NULL;
	else {
		regulator_enable(vcc);
		ret = regulator_set_voltage(vcc, 1200000, 1200000);
	}
	vcc = regulator_get(NULL, "v_ldo3");
	if (IS_ERR(vcc))
		vcc = NULL;
	else {
		regulator_enable(vcc);
		ret = regulator_set_voltage(vcc, 1200000, 1200000);
	}

	return 0;
}

static struct tc35876x_platform_data tc358765_data = {
	.platform_init = tc358765_init,
	.id = TC358765_CHIPID,
	.id_reg = TC358765_CHIPID_REG,
};
#endif

static struct i2c_board_info abilene_twsi5_info[] = {
#if defined(CONFIG_TC35876X)
	{
		.type		= "tc35876x",
		.addr		= 0x0f,
		.platform_data	= &tc358765_data,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_TPK_R800)
	{
		.type		= "tpk_r800",
		.addr		= 0x10,
		.irq		= IRQ_GPIO(101),
		.platform_data	= &tpk_r800_data,
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

static struct ds4432_platform_data abilene_ds4432_info = {
	.regulator_count = sizeof(ds4432_data)/sizeof(ds4432_data[0]),
	.regulators = ds4432_data,

};

static struct i2c_board_info abilene_twsi6_info[] = {
	{
		.type		= "ds4432",
		.addr		= 0x48,
		.platform_data	= &abilene_ds4432_info,
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

static struct regulator_consumer_supply abilene_wm8994_regulator_supply[] = {
	[0] = {
		.supply = "AVDD1",
		},
	[1] = {
		.supply = "DCVDD",
		},
};

struct regulator_init_data abilene_wm8994_regulator_init_data[] = {
	[0] = {
		.constraints = {
				.name = "wm8994-ldo1",
				.min_uV = 2400000,
				.max_uV = 3100000,
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &abilene_wm8994_regulator_supply[0],
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
		.consumer_supplies = &abilene_wm8994_regulator_supply[1],
		},
};

struct wm8994_pdata abilene_wm8994_pdata = {
	.ldo[0] = {
			.enable = 0,
			.init_data = &abilene_wm8994_regulator_init_data[0],
			.supply = "AVDD1",

		},
	.ldo[1] = {
		.enable = 0,
		.init_data = &abilene_wm8994_regulator_init_data[1],
		.supply = "DCVDD",

		},
};

static struct regulator_consumer_supply abilene_fixed_regulator_supply[] = {
	[0] = {
		.supply = "SPKVDD1",
		},
	[1] = {
		.supply = "SPKVDD2",
		},
};

static struct i2c_board_info abilene_twsi3_info[] = {
	{
	 .type = "wm8994",
	 .addr = 0x1a,
	 .platform_data = &abilene_wm8994_pdata,
	 },
};

struct regulator_init_data abilene_fixed_regulator_init_data[] = {
	[0] = {
		.constraints = {
				.name = "wm8994-SPK1",
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &abilene_fixed_regulator_supply[0],
		},
	[1] = {
		.constraints = {
				.name = "wm8994-SPK2",
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &abilene_fixed_regulator_supply[1],
		},
};

struct fixed_voltage_config abilene_fixed_pdata[2] = {
	[0] = {
		.supply_name = "SPKVDD1",
		.microvolts = 3700000,
		.init_data = &abilene_fixed_regulator_init_data[0],
		.gpio = -1,
		},
	[1] = {
		.supply_name = "SPKVDD2",
		.microvolts = 3700000,
		.init_data = &abilene_fixed_regulator_init_data[1],
		.gpio = -1,
		},
};

static struct platform_device fixed_device[] = {
	[0] = {
		.name = "reg-fixed-voltage",
		.id = 0,
		.dev = {
			.platform_data = &abilene_fixed_pdata[0],
			},
		.num_resources = 0,
		},
	[1] = {
		.name = "reg-fixed-voltage",
		.id = 1,
		.dev = {
			.platform_data = &abilene_fixed_pdata[1],
			},
		.num_resources = 0,
		},
};

static struct platform_device *fixed_rdev[] __initdata = {
	&fixed_device[0],
	&fixed_device[1],
};

static void abilene_fixed_regulator(void)
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

static void __init abilene_init_headset(void)
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
};
#endif

static int abilene_board_reset(char mode, const char *cmd)
{
#ifdef CONFIG_INPUT_MAX8925_ONKEY
	extern void max8925_system_restart(char mode, const char *cmd);
	max8925_system_restart(mode, cmd);
#endif
	return 1;
}

static void __init abilene_init(void)
{
	extern int (*board_reset)(char mode, const char *cmd);
	board_reset = abilene_board_reset;
	mfp_config(ARRAY_AND_SIZE(abilene_pin_config));

	/* on-chip devices */
	mmp3_add_uart(3);

	abilene_power_supply_init();
	mmp3_add_twsi(1, &twsi1_pdata, ARRAY_AND_SIZE(abilene_twsi1_max77601_info));

	mmp3_add_twsi(4, NULL, ARRAY_AND_SIZE(abilene_twsi4_info));
	mmp3_add_twsi(5, NULL, ARRAY_AND_SIZE(abilene_twsi5_info));

	mmp3_add_keypad(&mmp3_keypad_info);

	mmp3_add_videosram(&mmp3_videosram_info);
#ifdef CONFIG_FB_PXA168
	abilene_add_lcd_mipi();
	mmp3_add_tv_out();
#endif

#ifdef CONFIG_UIO_HDMI
	mmp3_add_hdmi(&mmp3_hdmi_info);
#endif
	/* backlight */
	mmp3_add_pwm(3);
	platform_device_register(&abilene_lcd_backlight_devices);
	mmp3_add_thermal();

#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem();
#endif
#ifdef CONFIG_UIO_VMETA
	mmp_init_vmeta();
#endif

#ifdef CONFIG_MMC_SDHCI_PXAV3
	abilene_init_mmc();
#endif /* CONFIG_MMC_SDHCI_PXAV3 */

#if defined(CONFIG_VIDEO_MV)
	platform_device_register(&abilene_ov5642);
	mmp3_add_cam(0, &mv_cam_data);
#endif

#ifdef CONFIG_VIDEO_MVISP
	mmp_init_dxoisp();
#endif

#ifdef CONFIG_NTRIG_SPI
	mmp3_init_spi();
#endif
	platform_device_register(&mmp3_device_rtc);

#if defined(CONFIG_TOUCHSCREEN_VNC)
	platform_device_register(&mmp3_device_vnc_touch);
#endif


	mmp3_add_twsi(3, NULL, ARRAY_AND_SIZE(abilene_twsi3_info));

	abilene_fixed_regulator();
	wm8994_ldoen();

	/* audio sspa support */
	mmp3_add_sspa(1);
	mmp3_add_sspa(2);
	mmp3_add_audiosram(&mmp3_audiosram_info);

#if defined(CONFIG_SWITCH_HEADSET_HOST_GPIO)
	abilene_init_headset();
#endif

#ifdef CONFIG_CHARGER_MAX8903
	platform_device_register(&max8903_charger_device);
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

MACHINE_START(ABILENE, "Abilene")
	.map_io		= mmp_map_io,
	.nr_irqs	= ABILENE_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.reserve	= mmp3_reserve,
	.init_machine	= abilene_init,
MACHINE_END
