/*
 *  linux/arch/arm/mach-mmp/qseven.c
 *  Dipen Patel, dpatel@marvell.com
 *  Derived from abilene.c
 *  Support for the Marvell MMP3 Qseven Platform.
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

#include <linux/regulator/88pm867.h>
#include <linux/i2c/tsc2007.h>

#include <linux/pwm_backlight.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
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
#include <mach/hsi_dev.h>
#include <plat/pmem.h>
#include <plat/usb.h>
#include <mach/sram.h>
#include <mach/axis_sensor.h>
#include <mach/uio_hdmi.h>
#include <media/soc_camera.h>
#include <mach/mmp3_pm.h>
#include <mach/gpio.h>
#include <linux/i2c/eneec.h>
#include <linux/spi/flash.h>

#include "common.h"
#include "onboard.h"

#define QSEVEN_NR_IRQS		(IRQ_BOARD_START + 64)

static unsigned long qseven_pin_config[] __initdata = {
	/* UART3 */
	GPIO51_UART3_RXD,
	GPIO52_UART3_TXD,

	/* TWSI5 not connected in rev 1 carrier board*/
	GPIO41_TWSI5_SCL,
	GPIO42_TWSI5_SDA,

	/* TWSI6 for HDMI */
	GPIO47_TWSI6_SCL,
	GPIO48_TWSI6_SDA,

#if 0
	/* TWSI2 for camera */
	GPIO55_TWSI2_SCL,
	GPIO56_TWSI2_SDA,
	GPIO73_CAM_MCLK,
	GPIO72_GPIO,
#endif

	/* TWSI3 for audio codec on carrier card rev 2*/
//	GPIO95_TWSI3_SCL,
//	GPIO96_TWSI3_SDA,
	/* paul for Ariel2 */
		GPIO71_TWSI3_SCL,	
		GPIO72_TWSI3_SDA,
#if 0
	HDA_RST_N_GPIO_79, /*Not connected in carrier rev 1 board*/
#endif

	/* TWSI4 touch controller on carrier card*/
	TWSI4_SCL,
	TWSI4_SDA,

#if 0
	TSI_INT_N, /*also smb_int_n in the card schema*/
#endif

#if 0
	/*PWM3*/
	GPIO53_PWM3,
	/*PWM4 for Q7 board rev 1, not connected anywhere*/
	GPIO54_PWM4,
#endif

	/* SSPA1 (I2S) */
	GPIO25_I2S_BITCLK,
	GPIO26_I2S_SYNC,
	GPIO27_I2S_DATA_OUT,
	GPIO28_I2S_SDATA_IN,

        /* HSIC1 reset pin (output) for Ariel 2*/
        GPIO63_HSIC_RESET,

	/* Headphone Detection GPIO 62 pin (Input) for Ariel 2*/
	GPIO62_GPIO,

 	/* Microphone Detection GPIO 68 pin (Input) for Ariel 2*/
	GPIO68_GPIO,

#if 0
	/*ULPI QSEVEN rev 1*/
	ULPI_DATA_0_GPIO_66,
	ULPI_DATA_1_GPIO_65,
	ULPI_DATA_2_GPIO_64,
	ULPI_DATA_3_GPIO_63,
	ULPI_DATA_4_GPIO_62,
	ULPI_DATA_5_GPIO_61,
	ULPI_DATA_6_GPIO_60,
	ULPI_DATA_7_GPIO_59,
	ULPI_RST_N_GPIO_71,
	ULPI_CLK_GPIO_70,
	ULPI_DIR_GPIO_69,
	ULPI_NXT_GPIO_68,
	ULPI_STP_GPIO_67,
#endif

	/* SSP1 FOR NOR FLASH, NOT POPULTAED*/
	SSP1_RXD_GPIO_43,
	SSP1_TXD_GPIO_44,
	SSP1_CLK_GPIO_45,
	SSP1_FRM_GPIO_46,

	/*HDMI_HPD_N for hdmi detect, y no hdmi_cec*/
	GPIO59_GPIO,

#if 0
	/* SSP3 FOR CAN*/
	GPIO74_SSP_CLK,
	GPIO75_SSP_FRM,
	GPIO76_SSP_TXD,
	GPIO77_SSP_RXD,

	/*Power button not connected in rev 1 carrier board*/
	GPIO80_GPIO,
	/*HDMI_HPD_N for hdmi detect, y no hdmi_cec*/
	GPIO81_GPIO,
	GPIO82_GPIO,
	GPIO83_GPIO,

	/*LED*/
	GPIO84_GPIO,
	GPIO85_GPIO,
	GPIO86_GPIO,
	GPIO87_GPIO,
#endif
	/*LCD RGB*/
	GPIO74_LCD_FCLK,
	GPIO75_LCD_LCLK,
	GPIO76_LCD_PCLK,
	GPIO77_LCD_DENA,
	GPIO78_LCD_DD0,
	GPIO79_LCD_DD1,
	GPIO80_LCD_DD2,
	GPIO81_LCD_DD3,
	GPIO82_LCD_DD4,
	GPIO83_LCD_DD5,
	GPIO84_LCD_DD6,
	GPIO85_LCD_DD7,
	GPIO86_LCD_DD8,
	GPIO87_LCD_DD9,
	GPIO88_LCD_DD10,
	GPIO89_LCD_DD11,
	GPIO90_LCD_DD12,
	GPIO91_LCD_DD13,
	GPIO92_LCD_DD14,
	GPIO93_LCD_DD15,
	GPIO94_LCD_DD16,
	GPIO95_LCD_DD17,
	GPIO96_LCD_DD18,
	GPIO97_LCD_DD19,
	GPIO98_LCD_DD20,
	GPIO99_LCD_DD21,
	GPIO100_LCD_DD22,
	GPIO101_LCD_DD23,

	/* ENE EC */
	ENE_KB_INT_GPIO_60,
	SPI2_CLK_GPIO_55,
	SPI2_FRM_GPIO_56,
	SPI2_RXD_GPIO_57,
	SPI2_TXD_GPIO_58,
 	GPIO126_OFF1,
	GPIO127_OFF2,
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
	GPIO137_GPIO, /*drive low to enable power to card*/
};

#if 0
/* MMC2 is used for WIB card */
static unsigned long mmc2_pin_config[] __initdata = {
	GPIO37_MMC2_DAT3,
	GPIO38_MMC2_DAT2,
	GPIO39_MMC2_DAT1,
	GPIO40_MMC2_DAT0,
	GPIO41_MMC2_CMD,
	GPIO42_MMC2_CLK,

	/* GPIO used for power */
	GPIO57_GPIO, /* WLAN_PD_N */
	WIFI_32K_CLK_OUT,
};
#endif

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

#if defined(CONFIG_DDR_DEVFREQ)
struct devfreq_frequency_table *mmp3_ddr_freq_table;
struct devfreq_pm_qos_table ddr_freq_qos_table[] = {
	{
		.freq = 533333,
		.qos_value = DDR_CONSTRAINT_LVL0,
	},
	{
		.freq = 800000,
		.qos_value = DDR_CONSTRAINT_LVL1,
	},
	{
		.freq = 1066666,
		.qos_value = DDR_CONSTRAINT_LVL2,
	},
	{0, 0},
};

static struct devfreq_platform_data ddr_info = {
	.clk_name = "ddr",
	.interleave_is_on = 1,
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


	ddr_info.freq_table = mmp3_ddr_freq_table;
	ddr_info.hw_base[0] = DMCU_VIRT_BASE;
	ddr_info.hw_base[1] = DMCU_VIRT_BASE + 0x10000;
	ddr_info.qos_list = ddr_freq_qos_table;

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
#if 0

static struct pxa27x_keypad_platform_data mmp3_keypad_info = {
	.direct_key_map = {
		KEY_BACK,
		KEY_MENU,
		KEY_HOME,
		KEY_SEARCH,
		KEY_VOLUMEUP,
		KEY_VOLUMEDOWN,
	},
	.direct_key_num = 6,
	.debounce_interval = 30,
	.active_low = 1,
};

#if defined(CONFIG_TOUCHSCREEN_TSC2007)
static int tsc2007_init_gpio_irq(void)
{
	int gpio = mfp_to_gpio(TSI_INT_N);

	if (gpio_request(gpio, "TSC2007 GPIO irq")) {
			pr_err("gpio %d request failed\n", gpio);
			return -1;
	}
	gpio_direction_input(gpio);
	mdelay(100);
	gpio_free(gpio);
	return 0;
}

static int tsc_2007_pen_state(void)
{
	int temp = mfp_to_gpio(TSI_INT_N);
	int value;
	if (gpio_request(temp, "TSC2007 GPIO")) {
			pr_err("gpio %d request failed\n", temp);
			return -1;
	}
	gpio_direction_input(temp);
	value = gpio_get_value(temp);
	gpio_free(temp);
	return !value;
}

struct tsc2007_platform_data tsc_2007_data = {
	.model			= 2007,
	.x_plate_ohms		= 180,
	.init_platform_hw = NULL,
	.get_pendown_state = tsc_2007_pen_state,
	.clear_penirq = NULL,
	.exit_platform_hw = NULL,
	.poll_delay = 1,
	.poll_period = 1,

};
#endif
#endif

static struct i2c_board_info qseven_twsi4_info[] = {
	{
         .type = "KB39XX",
         .addr = 0x58,
	 .irq  = IRQ_GPIO(mfp_to_gpio(ENE_KB_INT_GPIO_60)),
	},
#if 0
#if defined(CONFIG_TOUCHSCREEN_TSC2007)
	{
		.type		= "tsc2007",
		.addr		= (0x90>>1),
		.irq		= IRQ_GPIO(mfp_to_gpio(TSI_INT_N)),
		.platform_data	= &tsc_2007_data,
	},
#endif
#endif
};

static struct i2c_board_info qseven_twsi3_info[] = {
	{
	 .type = "ce156",
	 .addr = 0x30,
	},
};
/*FIXME*/
#if 1 /* Enable for Ariel */
static struct i2c_board_info qseven_twsi2_info[] = {
	{
	},
};
static struct i2c_board_info qseven_twsi5_info[] = {
        {
        },
};
static struct i2c_board_info qseven_twsi6_info[] = {
	{
	},
};
#endif

#ifdef CONFIG_REGULATOR_88PM867

#define PMIC_POWER_MAX MAR88PM867_VREG_MAX
static struct regulator_consumer_supply qseven_power_supply[PMIC_POWER_MAX];
static struct regulator_init_data pmic_regulator_data[PMIC_POWER_MAX];

#define REG_SUPPLY_INIT(_id, _name, _dev_name) \
{						\
	qseven_power_supply[_id].supply =  _name;  \
	qseven_power_supply[_id].dev_name = _dev_name; \
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
	pmic_regulator_data[_id].constraints.valid_modes_mask = \
			REGULATOR_MODE_NORMAL; \
	pmic_regulator_data[_id].consumer_supplies = _supply; \
	pmic_regulator_data[_id].num_consumer_supplies = _num;	\
}

static void qseven_power_supply_init(void)
{
	REG_SUPPLY_INIT(MAR88PM867_ID_SD0, "vcc_main", NULL);
	PMIC_REG_INIT(MAR88PM867_ID_SD0, SD0, 775000, 1400000, 1, 1,
		&qseven_power_supply[MAR88PM867_ID_SD0], 1);
}

static struct mar88pm867_platform_data qseven_mar88pm867_pdata = {
	.regulator = pmic_regulator_data,
	.buck_id = MAR88PM867_ID_SD0,
};

static struct i2c_board_info qseven_twsi1_mar88pm867_info[] = {
	{
		.type		= "mar88pm867",
		.addr		= (0x32>>1),
		.platform_data	= &qseven_mar88pm867_pdata,
	},
	{
		.type	= "rtc_idt1338",
		.addr 	= 0x68,
	},
};

static struct regulator_consumer_supply qseven_fixed_regulator_supply[] = {
	[0] = {
		.supply = "v_1p8",
		},
};

struct regulator_init_data qseven_fixed_regulator_init_data[] = {
	[0] = {
		.constraints = {
				.name = "88pm867-v_1p8",
				.always_on = 1,
				.boot_on = 1,
				},
		.num_consumer_supplies = 1,
		.consumer_supplies = &qseven_fixed_regulator_supply[0],
		},
};

struct fixed_voltage_config qseven_fixed_pdata[] = {
	[0] = {
		.supply_name = "v_1p8",
		.microvolts = 1800000,
		.gpio = -EINVAL,
		.enabled_at_boot = 1,
		.init_data = &qseven_fixed_regulator_init_data[0],
		},
};

static struct platform_device fixed_device[] = {
	[0] = {
		.name = "reg-fixed-voltage",
		.id = 0,
		.dev = {
			.platform_data = &qseven_fixed_pdata[0],
			},
		.num_resources = 0,
		},
};

static struct platform_device *fixed_rdev[] __initdata = {
	&fixed_device[0],
};

static void qseven_regulators(void)
{
	platform_add_devices(fixed_rdev, ARRAY_SIZE(fixed_rdev));
}

#endif /*CONFIG_REGULATOR_88PM867*/


#if 0
static int qseven_pwm_init(struct device *dev)
{
	int lvds_blen, lvds_pplen;
	if (!cpu_is_mmp3_b1())/*replaced with b1*/
		return 0;

	lvds_blen = mfp_to_gpio(MFP_PIN_GPIO82);
	lvds_pplen = mfp_to_gpio(MFP_PIN_GPIO83);

	if (gpio_request(lvds_blen, "backlight enable")) {
		printk(KERN_INFO "gpio %d request failed\n", lvds_blen);
		return -1;
	}
	if (gpio_request(lvds_pplen, "lvds pplen")) {
		printk(KERN_INFO "gpio %d request failed\n", lvds_pplen);
		return -1;
	}

	gpio_direction_output(lvds_blen, 1);
	gpio_free(lvds_blen);

	gpio_direction_output(lvds_pplen, 1);
	gpio_free(lvds_pplen);

	return 0;
}

static struct platform_pwm_backlight_data qseven_lcd_backlight_data = {
	/* primary backlight */
	.pwm_id = 2,
	.max_brightness = 100,
	.dft_brightness = 50,
	.pwm_period_ns = 2000000,
	.init = qseven_pwm_init,
};

static struct platform_device qseven_lcd_backlight_devices = {
	.name = "pwm-backlight",
	.id = 2,
	.dev = {
		.platform_data = &qseven_lcd_backlight_data,
	},
};
#endif

#ifdef CONFIG_MMC_SDHCI_PXAV3
#include <linux/mmc/host.h>

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
	static int f_enabled = 0;
	if (on) {
		f_enabled = 1;
		pr_info("Turning wifi on\n");
		mfp_config(ARRAY_AND_SIZE(wifi_pin_config_on));
	}
	if ((!on)) {
		pr_info("Turning wifi off\n");
		mfp_config(ARRAY_AND_SIZE(wifi_pin_config_off));
		f_enabled = 0;
	}
}
static void mmp3_8787_set_power_1(unsigned int on)
{
	int wlan_pd_n = mfp_to_gpio(MFP_PIN_GPIO57);
	if (gpio_request(wlan_pd_n, "wifi card power")) {
		printk(KERN_INFO "gpio %d request failed\n", wlan_pd_n);
		return -1;
	}
	if (on) {
		pr_info("Turning wifi on\n");
		gpio_direction_output(wlan_pd_n, 1);
		gpio_free(wlan_pd_n);
	} else {
		pr_info("Turning wifi off\n");
		gpio_direction_output(wlan_pd_n, 0);
		gpio_free(wlan_pd_n);
	}
	mdelay(100);
	return 0;
}
#endif

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc0 = {
	.clk_delay_cycles	= 0x1F,
};

#if 0
static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc1 = {
	.flags          = PXA_FLAG_CARD_PERMANENT,
	.pm_caps	= MMC_PM_KEEP_POWER,
};
#endif

static struct sdhci_pxa_platdata mmp3_sdh_platdata_mmc2 = {
	.flags		= PXA_FLAG_SD_8_BIT_CAPABLE_SLOT,
};

static int __init qseven_init_mmc(void)
{
#if 0
        /* No MMC power enable for Ariel 2 */
	int sd_power_gpio = mfp_to_gpio(MFP_PIN_GPIO140);/* This Card detection on Ariel 2 */
	int wlan_pd_n = mfp_to_gpio(MFP_PIN_GPIO57);

	if (gpio_request(sd_power_gpio, "sd card power")) {
		printk(KERN_INFO "gpio %d request failed\n", sd_power_gpio);
		return -1;
	}

	gpio_direction_output(sd_power_gpio, 0);
	gpio_free(sd_power_gpio);
#endif

	mfp_config(ARRAY_AND_SIZE(mmc3_pin_config));
	mmp3_add_sdh(2, &mmp3_sdh_platdata_mmc2); /* eMMC */

	mfp_config(ARRAY_AND_SIZE(mmc1_pin_config));
#if 0 // disable 
	if (cpu_is_mmp3_b1())/*replaced with b1*/
		mmp3_sdh_platdata_mmc0.quirks =
					SDHCI_QUIRK_INVERTED_WRITE_PROTECT;
#endif
	mmp3_add_sdh(0, &mmp3_sdh_platdata_mmc0); /* SD/MMC */

#if 0
	/* SDIO for WIFI card */
	mfp_config(ARRAY_AND_SIZE(mmc2_pin_config));
	mmp3_add_sdh(1, &mmp3_sdh_platdata_mmc1);

	if (gpio_request(wlan_pd_n, "wifi card power")) {
		printk(KERN_INFO "gpio %d request failed\n", wlan_pd_n);
		return -1;
	}
	gpio_direction_output(wlan_pd_n, 0);
	mdelay(100);
	gpio_direction_output(wlan_pd_n, 1);
	gpio_free(wlan_pd_n);
#endif
	return 0;
}
#endif /* CONFIG_MMC_SDHCI_PXAV3 */


#ifdef CONFIG_USB_SUPPORT

#if defined(CONFIG_USB_PXA_U2O) || defined(CONFIG_USB_EHCI_PXA_U2O)

static char *mmp3_usb_clock_name[] = {
	[0] = "U2OCLK",
};

static struct mv_usb_platform_data mmp3_usb_pdata = {
	.clknum		= 1,
	.clkname	= mmp3_usb_clock_name,
	.vbus		= NULL,
	.mode		= MV_USB_MODE_OTG,
	.phy_init	= pxa_usb_phy_init,
	.phy_deinit	= pxa_usb_phy_deinit,
};
#endif /*CONFIG_USB_PXA_U20*/


#ifdef CONFIG_USB_EHCI_PXA_U2H_HSIC
static int mmp3_hsic1_reset(void)
{
	int reset;
	reset = mfp_to_gpio(GPIO63_HSIC_RESET);

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

#if 0
	static struct regulator *v_1p2_hsic;
	printk(KERN_INFO "%s: set %d\n", __func__, vbus);
#endif
	if (vbus) {

#if 0
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
#endif

		mmp3_hsic1_reset();
	} else {
#if 0
		if (v_1p2_hsic) {
			regulator_disable(v_1p2_hsic);
			regulator_put(v_1p2_hsic);
			v_1p2_hsic = NULL;
		}
#endif
	}

	return 0;
}

static char *mmp3_hsic1_clock_name[] = {
        [0] = "U2OCLK",
        [1] = "HSIC1CLK",
};

static struct mv_usb_platform_data mmp3_hsic1_pdata = {
        .clknum         = 2,
        .clkname        = mmp3_hsic1_clock_name,
        .vbus           = NULL,
        .mode           = MV_USB_MODE_HOST,
        .phy_init       = mmp3_hsic_phy_init,
        .phy_deinit     = mmp3_hsic_phy_deinit,
        .set_vbus       = mmp3_hsic1_set_vbus,
        .private_init   = mmp3_hsic_private_init,
};
#endif




#if 0
#ifdef CONFIG_USB_EHCI_PXA_U2H_FSIC /*Support for ulpi*/
static int mmp3_fsic_ulpi_phy_reset(void)
{
	int usb_ulpi = mfp_to_gpio(ULPI_RST_N_GPIO_71);
	if (gpio_request(usb_ulpi, "ulpi reset")) {
		printk(KERN_INFO "gpio %d request failed\n", usb_ulpi);
		return -1;
	}

	gpio_direction_output(usb_ulpi, 0);
	mdelay(100);
	gpio_direction_output(usb_ulpi, 1);
	mdelay(50);
	gpio_free(usb_ulpi);
	return 0;
}

static char *mmp3_fsic_clock_name[] = {
	[0] = "U2OCLK",
	[1] = "FSICCLK",
};

static struct mv_usb_platform_data mmp3_fsic_pdata = {
	.clknum		= 2,
	.clkname	= mmp3_fsic_clock_name,
	.vbus		= NULL,
	.mode		= MV_USB_MODE_HOST,
	.phy_init	= mmp3_fsic_phy_init,
	.p_init		= mmp3_fsic_p_init,
};

#endif
#endif

#endif /* USB support */
#ifdef CONFIG_UIO_HDMI
static struct uio_hdmi_platform_data mmp3_hdmi_info __initdata = {
	.sspa_reg_base = 0xD42A0C00,
	/* Fix me: gpio 81 lpm pull ? */
	.gpio = mfp_to_gpio(GPIO59_GPIO),
	.edid_bus_num = 6,
};
#endif

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

static struct dmc_regtable_entry khx1600c9s3k_2x133mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x911400CA},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x64660684},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xC2006C53},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x44F8A187},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x000E2101},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00008801},
};
static struct dmc_regtable_entry khx1600c9s3k_2x177mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x911400CA},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x64660684},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xC2006C53},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x44F8A187},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x000E2101},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00008801},
};
static struct dmc_regtable_entry khx1600c9s3k_2x266mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x911400CA},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x64660684},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xC2006C53},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x44F8A187},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x000E2101},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00008801},
};
static struct dmc_regtable_entry khx1600c9s3k_2x400mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x911400CA},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x64660684},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xC2006C53},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x44F8A187},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x000E2101},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00008801},
};
static struct dmc_regtable_entry khx1600c9s3k_2x533mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x911A00CA},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x848808B4},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xC200906C},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x4698DA09},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x00132161},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00008801},
};

static struct dmc_regtable_entry khx1600c9s3k_phy[] = {
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000000},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00002100},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000001},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00002100},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000002},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00002100},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000003},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00002100},
	{DMCU_PHY_CTRL3, ALLBITS, 0x20004044},
};

static struct dmc_regtable_entry khx1600c9s3k_wl[] = {
	{DMCU_PHY_DLL_WL_SEL, ALLBITS, 0x00000100},
	{DMCU_PHY_DLL_WL_CTRL0, ALLBITS, 0x00040004},
	{DMCU_PHY_DLL_WL_SEL, ALLBITS, 0x00000101},
	{DMCU_PHY_DLL_WL_CTRL0, ALLBITS, 0x00040004},
	{DMCU_PHY_DLL_WL_SEL, ALLBITS, 0x00000102},
	{DMCU_PHY_DLL_WL_CTRL0, ALLBITS, 0x00080008},
	{DMCU_PHY_DLL_WL_SEL, ALLBITS, 0x00000103},
	{DMCU_PHY_DLL_WL_CTRL0, ALLBITS, 0x00080008},

};
static struct dmc_timing_entry khx1600c9s3k_table[] = {
	{
		.dsrc = 3,
		.mode4x = 0,
		.pre_d = 3,
		.cas = 0x000c800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, khx1600c9s3k_2x133mhz),
		},
	},

	{
		.dsrc = 3,
		.mode4x = 0,
		.pre_d = 2,
		.cas = 0x000c800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, khx1600c9s3k_2x177mhz),
		},
	},

	{
		.dsrc = 3,
		.mode4x = 0,
		.pre_d = 1,
		.cas = 0x000c800,
		.table = {
			DEF_DMC_TAB_ENTRY(DMCRT_TM, khx1600c9s3k_2x266mhz),
		},
	},
	{
		.dsrc = 1,
		.mode4x = 0,
		.pre_d = 0,
		.cas = 0x000c800,
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
		.cas = 0x000c800,
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

static void set_ddr_dll(u32 val)
{
#ifdef CONFIG_DDR_DEVFREQ
	u32 tmp;
	if (val <= 0xf) {
		tmp = readl(ddr_info.hw_base[0] + 0x248);
		writel((tmp & ~(0xf << 28)) | (val << 28), \
			ddr_info.hw_base[0] + 0x248);
	} else {
		pr_err("dll timer should be lower than 0xf\n");
	}
#endif
}

static void qseven_update_ddr_info(void)
{
	mmp3_pm_update_dram_timing_table(ARRAY_SIZE(khx1600c9s3k_table),
						khx1600c9s3k_table);
}
#if 0
static struct i2c_board_info qseven_i2c_camera[] = {
	{
		I2C_BOARD_INFO("ov5642", 0x3c),
	},
};

static int camera_sensor_power(struct device *dev, int on)
{
	int cam_pwdn = mfp_to_gpio(MFP_PIN_GPIO72);

	if (gpio_request(cam_pwdn, "CAM_PWDN")) {
		printk(KERN_ERR"Request GPIO failed, gpio: %d\n", cam_pwdn);
		return -EIO;
	}

	/* pull up camera pwdn pin to disable camera sensor */
	/* pull down camera pwdn pin to enable camera sensor */
	if (on)
		gpio_direction_output(cam_pwdn, 1);
	else
		gpio_direction_output(cam_pwdn, 0);
	msleep(100);

	gpio_free(cam_pwdn);
	return 0;
}

static struct soc_camera_link iclink_ov5642 = {
	.bus_id         = 0,            /* Must match with the camera ID */
	.power          = camera_sensor_power,
	.board_info     = &qseven_i2c_camera[0],
	.i2c_adapter_id = 1,
	.flags = SOCAM_MIPI,
	.module_name    = "ov5642",
	.priv = "pxa2128-mipi",
};

static struct platform_device qseven_ov5642 = {
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

	if (cpu_is_mmp3_b1())
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
	.name = "qseven",
	.clk_enabled = 0,
	.dphy = {0x1b0b, 0x33, 0x1a03},
	.qos_req_min = 0,
	.dma_burst = 128,
	.bus_type = SOCAM_MIPI,
	.ccic_num_flag = 1,
	.mclk_min = 26,
	.mclk_src = 3,
	.controller_power = pxa2128_cam_ctrl_power,
	.init_clk = pxa2128_cam_clk_init,
	.enable_clk = pxa2128_cam_set_clk,
	.get_mclk_src = get_mclk_src,
};
#endif

/*
 *	Ariel Power control: ( Power off / Reset )
 *	Send 10MHz pulse from GPIO 126 to EC When system ready to power off or reboot.
 *	Power off    : GPIO 127 high
 * 	System reset : GPIO 127 low
*/
#define EC_PW_OFF 1
#define EC_PW_RESET 0
extern int (*board_reset)(char mode, const char *cmd);
static int ariel_board_reset(char mode, const char *cmd)
{
        int off_signal,off_control;
        off_signal  = mfp_to_gpio(MFP_PIN_GPIO126);
        off_control = mfp_to_gpio(MFP_PIN_GPIO127);
        if (gpio_request(off_signal, "halt signal")) {
                pr_err("Failed to request halt signal gpio\n");
                return -EIO;
        }
        if (gpio_request(off_control, "halt_reset pin")) {
                pr_err("Failed to request halt reset gpio\n");
                return -EIO;
        }
        gpio_direction_output(off_control, EC_PW_RESET);
        while(1){
                gpio_direction_output(off_signal, 0);
                mdelay(50);
                gpio_direction_output(off_signal, 1);
                mdelay(50);
        }
        /* Rebooting... */
        return 1;
}

static void ariel_poweroff(void)
{
        int off_signal,off_control;
        off_signal  = mfp_to_gpio(MFP_PIN_GPIO126);
        off_control = mfp_to_gpio(MFP_PIN_GPIO127);
        if (gpio_request(off_signal, "halt signal")) {
                pr_err("Failed to request halt signal gpio\n");
                return ;
        }
        if (gpio_request(off_control, "halt_reset pin")) {
                pr_err("Failed to request halt reset gpio\n");
                return ;
        }
        gpio_direction_output(off_control, EC_PW_OFF);
        while(1){
                gpio_direction_output(off_signal, 0);
                mdelay(50);
                gpio_direction_output(off_signal, 1);
                mdelay(50);
        }
        /* Power off... */
}

static void eneec_init_gpio_irq(void)
{
        int gpio = mfp_to_gpio(ENE_KB_INT_GPIO_60);

        if (gpio_request(gpio, "ENE EC irq")) {
                        pr_err("gpio %d request failed\n", gpio);
                        return ;
        }
        gpio_direction_input(gpio);
        mdelay(100);
        gpio_free(gpio);
        return ;

}
static struct pxa2xx_spi_chip flash_spi_device = {
        .tx_threshold = 8,
        .rx_threshold = 8,
        .gpio_cs = 46,
};
static struct pxa2xx_spi_chip eneec_spi_device = {
        .tx_threshold = 8,
        .rx_threshold = 8,
        .gpio_cs = 56,
};

static struct pxa2xx_spi_master pxa_ssp_master_info = {
				.clock_enable =1,
        .num_chipselect = -1,
        .enable_dma = 0,
};


/*****************************************************************************
 * SPI Devices:
 *     SPI0: 4M Flash MX25L3205D
 ****************************************************************************/


static struct mtd_partition Ariel2_spi_flash_partitions[] = {
	{
		.name = "bootloader(spi)",
		.size = 0x0100000,
		.offset = 0,
	},
};
static const struct flash_platform_data spi_flash_data = {
	.name						= "spi_flash",
	.parts					= Ariel2_spi_flash_partitions,
	.nr_parts 			= ARRAY_SIZE(Ariel2_spi_flash_partitions),
	.type           = "w25q64",
};

static struct spi_board_info __initdata ariel_board_spi_info[] = {
	{
    .modalias       = "m25p80",
		.bus_num = 2,
		.chip_select = -1,
		.mode = SPI_MODE_0,
	  .platform_data  = &spi_flash_data,
	  .irq            = -1,
	  .max_speed_hz   = 1000000,
	  .controller_data = &flash_spi_device,
	},
		{
	.modalias = "eneec_spi",
	.bus_num = 3,
        .chip_select = -1,
        .mode = SPI_MODE_0,
        .max_speed_hz = 1000000,
        .platform_data = NULL,//&ntrig_data,
        .irq = IRQ_GPIO(mfp_to_gpio(ENE_KB_INT_GPIO_60)),
        .controller_data = &eneec_spi_device,
     },
};



static void __init mmp3_init_spi(void)
{

        mmp3_add_ssp(1);
				mmp3_add_spi(2, &pxa_ssp_master_info);

		mmp3_add_ssp(2);
        mmp3_add_spi(3, &pxa_ssp_master_info);
        if ((spi_register_board_info(ariel_board_spi_info, ARRAY_SIZE(ariel_board_spi_info))) != 0)
		pr_err("%s: spi_register_board_info returned error\n", __func__);
}

static void __init qseven_init(void)
{

	mfp_config(ARRAY_AND_SIZE(qseven_pin_config));
	qseven_update_ddr_info();

	/* on-chip devices */
	mmp3_add_uart(3);
#if 1 /*Enable for Ariel*/
	mmp3_add_twsi(2, NULL, ARRAY_AND_SIZE(qseven_twsi2_info));
	mmp3_add_twsi(5, NULL, ARRAY_AND_SIZE(qseven_twsi5_info));
	mmp3_add_twsi(6, NULL, ARRAY_AND_SIZE(qseven_twsi6_info));
#endif
#ifdef CONFIG_REGULATOR_88PM867
	qseven_power_supply_init();
	mmp3_add_twsi(1, NULL, ARRAY_AND_SIZE(qseven_twsi1_mar88pm867_info));
#endif
        board_reset = ariel_board_reset;
        pm_power_off = ariel_poweroff;

#if 0
#if defined(CONFIG_TOUCHSCREEN_TSC2007)
	tsc2007_init_gpio_irq();
#endif
#endif

	mmp3_add_twsi(4, NULL, ARRAY_AND_SIZE(qseven_twsi4_info));
#if 0
	mmp3_add_keypad(&mmp3_keypad_info);
#endif

	mmp3_add_videosram(&mmp3_videosram_info);

#ifdef CONFIG_FB_PXA168
	abilene_add_lcd_mipi(); /* will keep same name */
	mmp3_add_tv_out();
#endif

#ifdef CONFIG_UIO_HDMI
	mmp3_add_hdmi(&mmp3_hdmi_info);
#endif

#if defined(CONFIG_DDR_DEVFREQ)
	ddr_devfreq_init();
	mmp3_add_ddr_devfreq(&ddr_info);
#endif
	/* Change DLL reset timer to 256 cycles
	set_ddr_dll(2);*/
#if 0
	/* backlight */
	mmp3_add_pwm(3);
	platform_device_register(&qseven_lcd_backlight_devices);
#endif
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
	qseven_init_mmc();
#endif /* CONFIG_MMC_SDHCI_PXAV3 */

	platform_device_register(&mmp3_device_rtc);

	/* audio sspa support */
	mmp3_add_twsi(3, NULL, ARRAY_AND_SIZE(qseven_twsi3_info));
	mmp3_add_sspa(1);
#if 0
	mmp3_add_sspa(2);
#endif
	mmp3_add_audiosram(&mmp3_audiosram_info);

	/* sensor ov5642 and ccic support */
#if 0
#if defined(CONFIG_VIDEO_MV)
	platform_device_register(&qseven_ov5642);
	mmp3_add_cam(0, &mv_cam_data);
#endif
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

#if 0
#ifdef CONFIG_USB_EHCI_PXA_U2H_FSIC
	mmp3_fsic_ulpi_phy_reset();
	mmp3_fsic_device.dev.platform_data = (void *)&mmp3_fsic_pdata;
	platform_device_register(&mmp3_fsic_device);
#endif
#endif

#ifdef CONFIG_REGULATOR_88PM867
	qseven_regulators();
#endif
//	pxa_u3d_phy_disable();		//paul disable due to Ariel2 disable USB3 Power
	 eneec_init_gpio_irq();
	 mmp3_init_spi();
}

MACHINE_START(QSEVEN, "Ariel")
	.map_io		= mmp_map_io,
	.nr_irqs	= QSEVEN_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.reserve	= mmp3_reserve,
	.init_machine	= qseven_init,
MACHINE_END
