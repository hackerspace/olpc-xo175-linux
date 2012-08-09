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
#include <plat/devfreq.h>
#include <mach/sram.h>
#include <mach/axis_sensor.h>
#include <mach/uio_hdmi.h>
#include <media/soc_camera.h>
#include <mach/mmp3_pm.h>

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
	GPIO24_I2S_SYSCLK,
	GPIO25_I2S_BITCLK,
	GPIO26_I2S_SYNC,
	GPIO27_I2S_DATA_OUT,
	GPIO28_I2S_SDATA_IN,

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

	/* mk2 hp detect */
	GPIO23_GPIO,
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
	GPIO153_TOUCH_3V3,
	GPIO101_GPIO,
	GPIO91_TOUCH_SLP,

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
	/* V_SD_EN */
	GPIO138_GPIO | MFP_PULL_HIGH,
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

#if defined(CONFIG_DDR_DEVFREQ)
struct devfreq_frequency_table *mmp3_ddr_freq_table;
struct devfreq_pm_qos_table ddr_freq_qos_table[] = {
	{
		.freq = 355555,
		.qos_value = DDR_CONSTRAINT_LVL0,
	},
	{
		.freq = 533333,
		.qos_value = DDR_CONSTRAINT_LVL1,
	},
	{
		.freq = 800000,
		.qos_value = DDR_CONSTRAINT_LVL2,
	},
	{0, 0},
};

static struct devfreq_platform_data lpddr2_info = {
	.clk_name = "ddr",
	.interleave_is_on = 0,
};


static void ddr_devfreq_init(void)
{
	u32 i = 0;
	u32 ddr_freq_num = mmp3_get_pp_number();

	mmp3_ddr_freq_table = kmalloc(sizeof(struct devfreq_frequency_table) * \
					(ddr_freq_num + 1), GFP_KERNEL);

	if (!mmp3_ddr_freq_table)
		return;

	for (i = 0; i < ddr_freq_num; i++) {
		mmp3_ddr_freq_table[i].index = i;
		mmp3_ddr_freq_table[i].frequency = mmp3_get_pp_freq(i, MMP3_CLK_DDR_1);
	}
	mmp3_ddr_freq_table[i].index = i;
	mmp3_ddr_freq_table[i].frequency = DEVFREQ_TABLE_END;


	lpddr2_info.freq_table = mmp3_ddr_freq_table;
	lpddr2_info.hw_base[0] = DMCU_VIRT_BASE;
	lpddr2_info.hw_base[1] = DMCU_VIRT_BASE + 0x10000;
	lpddr2_info.qos_list = ddr_freq_qos_table;

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

#ifdef CONFIG_VMETA_DEVFREQ
extern int set_vmeta_freqs_table(struct devfreq *devfreq);
static struct devfreq_platform_data devfreq_vmeta_pdata = {
	.clk_name = "VMETA_CLK",
	.setup_freq_table = set_vmeta_freqs_table,
};

static void __init mmp_init_devfreq_vmeta(void)
{
	mmp_set_devfreq_vmeta_info(&devfreq_vmeta_pdata);
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
	.priv = "mk2-mipi",
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

	isppwr_power_control(on);

	if (on)
		clk_enable(data->clk);
	else
		clk_disable(data->clk);
}

static int get_mclk_src(struct device *dev)
{
	struct mv_cam_pdata *data = dev->platform_data;

	switch (data->mclk_src) {
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
	int gsen_pwr_en = mfp_to_gpio(GPIO87_GPIO);

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
		if (gpio_request(gsen_pwr_en, "GSENSOR Enable")) {
			printk(KERN_INFO "gpio %d request failed\n", gsen_pwr_en);
			return -1;
		}

		if (on && (!is_enabled[device_index])) {
			pmic_2p8v_sens[device_index] = regulator_get(NULL, "PMIC_V3_2V8");
			if (IS_ERR(pmic_2p8v_sens[device_index])) {
				pmic_2p8v_sens[device_index] = NULL;
				return -ENODEV;
			} else {
				regulator_set_voltage(pmic_2p8v_sens[device_index], 2800000, 2800000);
				regulator_enable(pmic_2p8v_sens[device_index]);
				is_enabled[device_index] = 1;

				gpio_direction_output(gsen_pwr_en, 1);
			}
		}
		if ((!on) && is_enabled[device_index]) {
			regulator_disable(pmic_2p8v_sens[device_index]);
			regulator_put(pmic_2p8v_sens[device_index]);
			pmic_2p8v_sens[device_index] = NULL;
			is_enabled[device_index] = 0;

			gpio_direction_output(gsen_pwr_en, 0);
		}

		gpio_free(gsen_pwr_en);
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
		pmic_2p8v_sens = regulator_get(NULL, "PMIC_V3_2V8"); /*Fix Me: conflict with schematics*/
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
	u8 data = 0;
	if (!chip)
		return -EINVAL;
	/* Set LDO0 and LDO1 work at NOT FPS mode
	 * which can be en/disabled dynamically */
	max77601_set_bits(chip, MAX77601_FPS_L0,
		MAX77601_FPSSRC_NOTFPS, MAX77601_FPSSRC_NOTFPS);
	max77601_set_bits(chip, MAX77601_FPS_L1,
		MAX77601_FPSSRC_NOTFPS, MAX77601_FPSSRC_NOTFPS);

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
	/* Set VCC_CORE(SD0) work at forced PWM mode */
	max77601_set_bits(chip, MAX77601_VREG_SD0_CFG, (1 << 2), (1 << 2));

	return 0;
};

/* Step-down regulators: SD[0..3] */
static struct regulator_consumer_supply regulator_supplies_sd0[] = {
	REGULATOR_SUPPLY("VCC_CORE", NULL),
};
static struct regulator_consumer_supply regulator_supplies_dvssd0[] = {
	REGULATOR_SUPPLY("VCC_CORE_DVS", NULL),
};
static struct regulator_consumer_supply regulator_supplies_sd1[] = {
	REGULATOR_SUPPLY("PMIC_V1", NULL),
};
static struct regulator_consumer_supply regulator_supplies_dvssd1[] = {
	REGULATOR_SUPPLY("PMIC_V1_DVS", NULL),
};
static struct regulator_consumer_supply regulator_supplies_sd2[] = {
	REGULATOR_SUPPLY("PMIC_V2_1V8", NULL),
};
static struct regulator_consumer_supply regulator_supplies_sd3[] = {
	/* sd/emmc/camera/sensor power */
	REGULATOR_SUPPLY("vmmc", "sdhci-pxa.0"),
	REGULATOR_SUPPLY("vmmc", "sdhci-pxa.2"),
	REGULATOR_SUPPLY("PMIC_V3_2V8", NULL),
};

/* Linear regulators: L[0..8] */
static struct regulator_consumer_supply regulator_supplies_ldo0[] = {
	REGULATOR_SUPPLY("PMIC_LDO0", NULL),
};
static struct regulator_consumer_supply regulator_supplies_ldo1[] = {
	REGULATOR_SUPPLY("PMIC_LDO1", NULL),
};
static struct regulator_consumer_supply regulator_supplies_ldo2[] = {
	REGULATOR_SUPPLY("PMIC_LDO2", NULL),
};
static struct regulator_consumer_supply regulator_supplies_ldo3[] = {
	REGULATOR_SUPPLY("PMIC_LDO3", NULL),
};
static struct regulator_consumer_supply regulator_supplies_ldo4[] = {
	REGULATOR_SUPPLY("PMIC_LDO4", NULL),
};
static struct regulator_consumer_supply regulator_supplies_ldo5[] = {
	REGULATOR_SUPPLY("PMIC_LDO5", NULL),
};
static struct regulator_consumer_supply regulator_supplies_ldo6[] = {
	REGULATOR_SUPPLY("PMIC_LDO6", NULL),
};
static struct regulator_consumer_supply regulator_supplies_ldo7[] = {
	REGULATOR_SUPPLY("PMIC_LDO7", NULL),
};
static struct regulator_consumer_supply regulator_supplies_ldo8[] = {
	REGULATOR_SUPPLY("PMIC_LDO8", NULL),
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
	.num_consumer_supplies	= ARRAY_SIZE(regulator_supplies_##_name),				\
	.consumer_supplies	= regulator_supplies_##_name, \
}

static struct regulator_init_data max77601_regulator_data[] = {
	/* Step-down regulators: SD[0..3] */
	[MAX77601_ID_SD0]	= REG_INIT(sd0, 600000, 3387500, 1, 1, 0),
	[MAX77601_ID_DVSSD0] 	= REG_INIT(dvssd0, 600000, 3387500, 1, 1, 0),
	[MAX77601_ID_SD1]	= REG_INIT(sd1, 800000, 1587500, 1, 1, 0),
	[MAX77601_ID_DVSSD1] 	= REG_INIT(dvssd1, 800000, 1587500, 1, 1, 0),
	[MAX77601_ID_SD2]	= REG_INIT(sd2, 600000, 3387500, 1, 1, 0),
	[MAX77601_ID_SD3]	= REG_INIT(sd3, 600000, 3387500, 1, 1, 0),
	/* Linear regulators: L[0..8] */
	[MAX77601_ID_L0] = REG_INIT(ldo0, 800000, 2350000, 0, 1, 0),
	[MAX77601_ID_L1] = REG_INIT(ldo1, 800000, 2350000, 0, 1, 0),
	[MAX77601_ID_L2] = REG_INIT(ldo2, 800000, 3950000, 1, 1, 0),
	[MAX77601_ID_L3] = REG_INIT(ldo3, 800000, 3950000, 0, 0, 0),
	[MAX77601_ID_L4] = REG_INIT(ldo4, 800000, 1587500, 0, 0, 0),
	[MAX77601_ID_L5] = REG_INIT(ldo5, 800000, 3950000, 1, 1, 1),
	[MAX77601_ID_L6] = REG_INIT(ldo6, 800000, 3950000, 1, 1, 0),
	[MAX77601_ID_L7] = REG_INIT(ldo7, 800000, 3950000, 0, 0, 0),
	[MAX77601_ID_L8] = REG_INIT(ldo8, 800000, 3950000, 0, 0, 0),
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
	ret = hsic2_pad_vdd_power(on);
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
	gpio = mfp_to_gpio(GPIO153_TOUCH_3V3);
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
	if (on)
		gpio_direction_output(hdmi_pwr_en, 1);
	else
		gpio_direction_output(hdmi_pwr_en, 0);
	gpio_free(hdmi_pwr_en);
	return 0;
}

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
	},
#endif /* CONFIG_TOUCHSCREEN_EGALAX_I2C */
};

static struct i2c_board_info mk2_twsi6_info[] = {
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
	u8 data;
	int i;

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

	/* Reset TWSI1 unit firstly */
	__raw_i2c_bus_reset(1);
	/* 1. Disable SW reset wake up */
	__raw_i2c_read_reg(1, 0x1c, MAX77601_ONOFFCNFG2, &data, 1);
	data &= ~MAX77601_SFT_RST_WK;
	__raw_i2c_write_reg(1, 0x1c, MAX77601_ONOFFCNFG2, data);
	/* 2. Issue Power down */
	__raw_i2c_read_reg(1, 0x1c, MAX77601_ONOFFCNFG1, &data, 1);
	data |= MAX77601_SFT_RST;
	__raw_i2c_write_reg(1, 0x1c, MAX77601_ONOFFCNFG1, data);

	mdelay(200);
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

static struct dmc_regtable_entry edb8132b3ma_2x133mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x48890065},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x32330125},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0x20131312},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x30125434},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x0A060081},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00005201},
	{DMCU_SDRAM_TIMING8, ALLBITS, 0x00000022},
};


static struct dmc_regtable_entry edb8132b3ma_2x177mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x488C0065},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x42430185},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0x20191912},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x30127046},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x0A080091},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00005201},
	{DMCU_SDRAM_TIMING8, ALLBITS, 0x0000002E},
};


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
		.dsrc = 3,
		.mode4x = 0,
		.pre_d = 3,
		.cas = 0x0008800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, edb8132b3ma_2x133mhz),
		},
	},

	{
		.dsrc = 3,
		.mode4x = 0,
		.pre_d = 2,
		.cas = 0x0008800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, edb8132b3ma_2x177mhz),
		},
	},

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


static void __init mk2_init(void)
{
	extern int (*board_reset)(char mode, const char *cmd);
	board_reset = mk2_board_reset;
	pm_power_off = mk2_power_off;
	mfp_config(ARRAY_AND_SIZE(mk2_pin_config));

	/* Reset TWSI1 unit firstly */
	__raw_i2c_bus_reset(1);
	/* clear recovery bit */
	if (max77601_rtc_raw_write(MAX77601_RTCSECA2, 0x00))
		pr_err("Recovery flag clear failed!\n");

	/* update ddr freq table & timing */
	mk2_update_ddr_info();

	/* on-chip devices */
	mmp3_add_uart(3);
	mmp3_add_twsi(1, NULL, ARRAY_AND_SIZE(mk2_twsi1_info));
	mmp3_add_twsi(4, NULL, ARRAY_AND_SIZE(mk2_twsi4_info));
	mmp3_add_twsi(3, NULL, ARRAY_AND_SIZE(mk2_twsi3_info));
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
#if defined(CONFIG_DDR_DEVFREQ)
	ddr_devfreq_init();
	mmp3_add_ddr_devfreq(&lpddr2_info);
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

#ifdef CONFIG_VMETA_DEVFREQ
	mmp_init_devfreq_vmeta();
#endif
#ifdef CONFIG_MMC_SDHCI_PXAV3
	mk2_init_mmc();
#endif /* CONFIG_MMC_SDHCI_PXAV3 */

#if defined(CONFIG_VIDEO_MV)
	platform_device_register(&mk2_ov5642);
	mmp3_add_cam(0, &mv_cam_data);
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

	gsensor_power_en(1);
/* enable EETI EXC7200 touch controller */
#ifdef CONFIG_TOUCHSCREEN_EGALAX_I2C
	/* Configure irq for exc7200 touch panel */
	exc7200_config();
#endif /*CONFIG_TOUCHSCREEN_EGALAX_I2C */
	/* If we have a full configuration then disable any regulators
	 * which are not in use or always_on. */
	regulator_has_full_constraints();

	mmp3_set_vcc_main_reg_id("VCC_CORE");
}

MACHINE_START(MK2, "mk2")
	.map_io		= mmp_map_io,
	.nr_irqs	= MK2_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.reserve	= mmp3_reserve,
	.init_machine	= mk2_init,
MACHINE_END
