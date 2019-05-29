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
#include <linux/mfd/88pm80x.h>
#include <linux/mfd/wm8994/pdata.h>
#include <linux/regulator/fixed.h>
#include <linux/switch_headset.h>
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
#include <mach/hsi_dev.h>
#include <plat/pmem.h>
#include <plat/usb.h>
#include <mach/sram.h>
#include <mach/axis_sensor.h>
#include <mach/uio_hdmi.h>
#include <media/soc_camera.h>
#include <mach/mmp3_pm.h>

#include "common.h"
#include "onboard.h"

#define ABILENE_NR_IRQS		(IRQ_BOARD_START + 64)

static unsigned long abilene_pin_config[] __initdata = {
	/* UART3 */
	GPIO51_UART3_RXD,
	GPIO52_UART3_TXD,

	/* UART2 GPS*/
	GPIO47_UART2_RXD,
	GPIO48_UART2_TXD,
	GPIO49_UART2_CTS,
	GPIO50_UART2_RTS,
	GPIO2_GPIO, /* PRES_INT_N GPS*/

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
	GPIO24_AUDPLL_SYSCLK,
	GPIO25_I2S_BITCLK,
	GPIO26_I2S_SYNC,
	GPIO27_I2S_DATA_OUT,
	GPIO28_I2S_SDATA_IN,

	/* CM3623 INT */
	GPIO138_GPIO | MFP_PULL_HIGH,

	/* camera */
	GPIO67_GPIO,
	GPIO73_CAM_MCLK,
	GPIO0_GPIO, /* CAM1_RST_N */
	GPIO1_GPIO, /* CAM2_RST_N */

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

	/* BB Power Enable: on b0, it changes to GPIO63 */
	GPIO63_BB_POWER_EN,

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

static int pcie_bb_power_on(int on)
{
	int pcie_power = mfp_to_gpio(GPIO63_BB_POWER_EN);

	printk ("PCIE_BB_POWER_ENABLE started \n");

	if (gpio_request(pcie_power, "PCIE_BB_POWER_ENABLE"))
		return -EIO;

	printk ("PCIE_BB_POWER_ENABLE started, step 1, on = %d\n", on);
	if (on)
		gpio_direction_output(pcie_power, 1);
	else
		gpio_direction_output(pcie_power, 0);

	gpio_free(pcie_power);

	return 0;
}





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

static int motion_sensor_set_power(int on, const char *device_name)
{
	static struct regulator *pmic_2p8v_sens[3];
	static int is_enabled[3] = {0, 0, 0};
	int device_index = -1;


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


}
#endif

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc0 = {
	.clk_delay_cycles	= 0x1F,
	.quirks			= SDHCI_QUIRK_INVERTED_WRITE_PROTECT,
	.host_caps_disable      =
				MMC_CAP_UHS_SDR12 |
				MMC_CAP_UHS_SDR25 |
				MMC_CAP_UHS_SDR104 |
				MMC_CAP_UHS_SDR50 |
				MMC_CAP_UHS_DDR50,
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



#endif


static struct i2c_board_info abilene_twsi5_info[] = {
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

static struct pm80x_platform_data pm805_info = {
	.irq_mode		= 0,
	.irq_base		= IRQ_BOARD_START + MAX77601_MAX_IRQ,
	.i2c_port		= PI2C_PORT,
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
	 .type = "88PM80x",
	 .addr = 0x38,
	 .irq = gpio_to_irq(mfp_to_gpio(GPIO23_GPIO)),
	 .platform_data = &pm805_info,
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

static struct switch_headset_pdata headset_pdata = {
	.name = "h2w",
	.gpio = mfp_to_gpio(GPIO23_GPIO),
};

static struct platform_device headset_switch_device = {
	.name = "headset",
	.id = 0,
	.dev = {
		.platform_data = &headset_pdata,
	},
};


extern int __raw_i2c_bus_reset(u8 bus_num);
extern int __raw_i2c_write_reg(u8 bus_num, u8 addr, u8 reg, u8 val);
extern int __raw_i2c_read_reg(u8 bus_num, u8 addr, u8 reg, u8 *buf, int len);

static int abilene_board_reset(char mode, const char *cmd)
{
	u8 data;

	/* Reset TWSI1 unit firstly */
	__raw_i2c_bus_reset(1);
	/* 1. Disable SW reset wake up */
	__raw_i2c_read_reg(1, 0x1c, MAX77601_ONOFFCNFG2, &data, 1);
	data |= MAX77601_SFT_RST_WK;
	__raw_i2c_write_reg(1, 0x1c, MAX77601_ONOFFCNFG2, data);
	/* 2. Issue Power down */
	__raw_i2c_read_reg(1, 0x1c, MAX77601_ONOFFCNFG1, &data, 1);
	data |= MAX77601_SFT_RST;
	__raw_i2c_write_reg(1, 0x1c, MAX77601_ONOFFCNFG1, data);

	mdelay(200);
	return 1;
}

#define DMCU_SDRAM_TIMING1 0x80
#define DMCU_SDRAM_TIMING2 0x84
#define DMCU_SDRAM_TIMING3 0x88
#define DMCU_SDRAM_TIMING4 0x8c
#define DMCU_SDRAM_TIMING5 0x90
#define DMCU_SDRAM_TIMING6 0x94
#define DMCU_SDRAM_TIMING7 0x98
#define DMCU_PHY_CTRL3 0x220
#define DMCU_PHY_DQ_BYTE_SEL 0x300
#define DMCU_PHY_DLL_CTRL_BYTE1 0x304
#define DMCU_PHY_DLL_WL_SEL 0x380
#define DMCU_PHY_DLL_WL_CTRL0 0x384
#define ALLBITS (0xFFFFFFFF)

static struct dmc_regtable_entry khx1600c9s3k_2x400mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x911403CF},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x64660414},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xC2003053},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x34F4A187},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x000F20C1},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00005501},
};

static struct dmc_regtable_entry khx1600c9s3k_2x533mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x911B03CF},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x74780564},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xC200406C},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x3694DA09},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x00142101},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00006601},
};

const struct dmc_regtable_entry khx1600c9s3k_2x600mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x955E03CF},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x84890614},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xC200487C},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x4762F24A},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x00162121},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00006601},
};

static struct dmc_regtable_entry khx1600c9s3k_4x400mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x59A803CF},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0xB5B88812},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0x610060A5},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x59D7430E},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x001D2181},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x02120501},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00008801},
};

static struct dmc_regtable_entry khx1600c9s3k_phy[] = {
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000000},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00001080},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000001},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00001080},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000002},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00001080},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000003},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00001080},
	{DMCU_PHY_CTRL3, ALLBITS, 0x00004055},
};

static struct dmc_regtable_entry khx1600c9s3k_wl[] = {
	{DMCU_PHY_DLL_WL_SEL, ALLBITS, 0x00000100},
	{DMCU_PHY_DLL_WL_CTRL0, ALLBITS, 0x001A001A},
	{DMCU_PHY_DLL_WL_SEL, ALLBITS, 0x00000101},
	{DMCU_PHY_DLL_WL_CTRL0, ALLBITS, 0x00160016},
	{DMCU_PHY_DLL_WL_SEL, ALLBITS, 0x00000102},
	{DMCU_PHY_DLL_WL_CTRL0, ALLBITS, 0x001A001A},
	{DMCU_PHY_DLL_WL_SEL, ALLBITS, 0x00000103},
	{DMCU_PHY_DLL_WL_CTRL0, ALLBITS, 0x00190019},

};
static struct dmc_timing_entry khx1600c9s3k_table[] = {
	{
		.dsrc = 1,
		.mode4x = 0,
		.pre_d = 0,
		.cas = 0x0008800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, khx1600c9s3k_2x400mhz),
			DEF_DMC_TAB_ENTRY(DMCRT_PH, khx1600c9s3k_phy),
			DEF_DMC_TAB_ENTRY(DMCRT_WL, khx1600c9s3k_wl),
		},
	},
	{
		.dsrc = 3,
		.mode4x = 0,
		.pre_d = 0,
		.cas = 0x0008800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, khx1600c9s3k_2x533mhz),
			DEF_DMC_TAB_ENTRY(DMCRT_PH, khx1600c9s3k_phy),
			DEF_DMC_TAB_ENTRY(DMCRT_WL, khx1600c9s3k_wl),
		},
	},
/*	{
		.dsrc = 2,
		.mode4x = 0,
		.pre_d = 0,
		.cas = 0x0008800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, khx1600c9s3k_2x600mhz),
			DEF_DMC_TAB_ENTRY(DMCRT_PH, khx1600c9s3k_phy),
			DEF_DMC_TAB_ENTRY(DMCRT_WL, khx1600c9s3k_wl),
		},
	},
	{
		.dsrc = 0,
		.mode4x = 1,
		.pre_d = 0,
		.cas = 0x0008800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, khx1600c9s3k_4x400mhz),
			DEF_DMC_TAB_ENTRY(DMCRT_PH, khx1600c9s3k_phy),
			DEF_DMC_TAB_ENTRY(DMCRT_WL, khx1600c9s3k_wl),
		},
	},
*/
};
static void abilene_update_ddr_info(void)
{
	mmp3_pm_update_dram_timing_table(ARRAY_SIZE(khx1600c9s3k_table),
						khx1600c9s3k_table);
}

static void __init abilene_init(void)
{
	extern int (*board_reset)(char mode, const char *cmd);
	board_reset = abilene_board_reset;
	mfp_config(ARRAY_AND_SIZE(abilene_pin_config));

	abilene_update_ddr_info();

	/* on-chip devices */
	mmp3_add_uart(3);


	mmp3_add_twsi(4, NULL, ARRAY_AND_SIZE(abilene_twsi4_info));
	mmp3_add_twsi(5, NULL, ARRAY_AND_SIZE(abilene_twsi5_info));

	mmp3_add_keypad(&mmp3_keypad_info);

	mmp3_add_videosram(&mmp3_videosram_info);

	mmp3_add_ddr_devfreq();
	/* backlight */
	mmp3_add_pwm(3);
	platform_device_register(&abilene_lcd_backlight_devices);
	mmp3_add_thermal();

#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem();
#endif






	platform_device_register(&mmp3_device_rtc);



	mmp3_add_twsi(3, NULL, ARRAY_AND_SIZE(abilene_twsi3_info));

	abilene_fixed_regulator();
	wm8994_ldoen();

	/* audio sspa support */
	mmp3_add_sspa(1);
	mmp3_add_sspa(2);
	mmp3_add_audiosram(&mmp3_audiosram_info);

	platform_device_register(&headset_switch_device);

#ifdef CONFIG_USB_PXA_U2O
	/* Place VBUS_EN low by default */
	pxa_usb_set_vbus(0);
	mmp3_device_u2o.dev.platform_data = (void *)&mmp3_usb_pdata;
	platform_device_register(&mmp3_device_u2o);
#endif



	pxa_u3d_phy_disable();
}

MACHINE_START(ABILENE, "Abilene")
	.map_io		= mmp_map_io,
	.nr_irqs	= ABILENE_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.reserve	= mmp3_reserve,
	.init_machine	= abilene_init,
MACHINE_END
