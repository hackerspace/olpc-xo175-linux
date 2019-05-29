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
#include <mach/isp_dev.h>
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
	GPIO64_GPIO,
	GPIO68_GPIO,
	GPIO0_GPIO,
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
	if (chip->chip800_version == PM800_CHIP_B0)
		pm80x_set_bits(chip->base_page, PM800_WAKEUP2, (0xF << 4), 0);
	/* Select XO 32KHZ(USE_XO)
	 * Force all CLK32K_1/2/3 buffers to use the XO 32KHZ */
	pm80x_set_bits(chip->base_page, PM800_RTC_CONTROL, (1 << 7), (1 << 7));
	/*
	 * Enable 32k out1, out2 and out3 from XO
	 * CLK32K_1: EXT_32K_IN
	 * CLK32K_2: 32K_CLK for WIFI and PM805 bofore B0rev2
	 * CLK32K_3: 32K_CLK_GPS
	 * 	     32K_CLK for WIFI and PM805 since B0rev2
	 * Enable low jitter version for CLK32K_3 (0x21[5] = 0b1)
	 */
	pm80x_reg_write(chip->base_page, 0x21, 0x20);
	pm80x_set_bits(chip->base_page, PM800_RTC_MISC2,
			0x3f, (0x2 << 4) | (0x2 << 2) | 0x2);
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
	.headset = &pm80x_headset,
	.headset_flag = 1,
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

/* FAN53555 VOUT range:
 * Option 0/1/3/5:	600<-->1230mV
 * Option 4:		603<-->1411mV */
static struct regulator_init_data fan53555_regulator_data = {
	.constraints = {
		.name = "V_MAIN",
		.min_uV = 600000,
		.max_uV = 1411038,
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




static struct i2c_board_info yellowstone_twsi4_info[] = {
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
	.headset = &pm80x_headset,
	.headset_flag = 1,
};

static struct i2c_board_info yellowstone_twsi3_info[] = {
	{
		.type = "88PM80x",
		.addr = 0x38,
		.irq  = gpio_to_irq(mfp_to_gpio(GPIO23_GPIO)),
		.platform_data = &pm805_info,
	},
};




#ifdef CONFIG_USB_SUPPORT


#endif



static struct sram_bank mmp3_audiosram_info = {
	.pool_name = "audio sram",
	.step = AUDIO_SRAM_GRANULARITY,
};

static struct sram_bank mmp3_videosram_info = {
	.pool_name = "mmp-videosram",
	.step = VIDEO_SRAM_GRANULARITY,
};


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
	.gpio_cs = -1,
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

static int yellowstone_board_reset(char mode, const char *cmd)
{
	u8 buf;
	/* Reset TWSI1 unit firstly */
	__raw_i2c_bus_reset(1);
	/* 1.Enable FAULT_WU and FAULT_WU_EN */
	pm800_i2c_read_reg(0x30, 0xE7, &buf, 1);
	buf |= ((1 << 3) | (1 << 2));
	pm800_i2c_write_reg(0x30, 0xE7, buf);
	/* 2.Issue SW power down */
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


	/* backlight */
	mmp3_add_pwm(3);
	platform_device_register(&yellowstone_lcd_backlight_devices);

	mmp3_add_thermal();
#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem();
#endif

	mmp3_init_spi();

	platform_device_register(&mmp3_device_rtc);


	mmp3_add_twsi(3, NULL, ARRAY_AND_SIZE(yellowstone_twsi3_info));

	/* audio sspa support */
	mmp3_add_sspa(1);
	mmp3_add_sspa(2);
	mmp3_add_audiosram(&mmp3_audiosram_info);



#ifdef CONFIG_USB_PXA_U2O
	/* Place VBUS_EN low by default */
	pxa_usb_set_vbus(0);
	mmp3_device_u2o.dev.platform_data = (void *)&mmp3_usb_pdata;
	platform_device_register(&mmp3_device_u2o);
#endif


	/* If we have a full configuration then disable any regulators
	 * which are not in use or always_on. */
	regulator_has_full_constraints();

	pxa_u3d_phy_disable();
}

MACHINE_START(YELLOWSTONE, "YellowStone")
	.map_io		= mmp_map_io,
	.nr_irqs	= YELLOWSTONE_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.reserve	= mmp3_reserve,
	.init_machine	= yellowstone_init,
MACHINE_END
