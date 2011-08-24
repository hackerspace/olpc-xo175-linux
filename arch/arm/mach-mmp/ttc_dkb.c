/*
 *  linux/arch/arm/mach-mmp/ttc_dkb.c
 *
 *  Support for the Marvell PXA910-based TTC_DKB Development Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/onenand.h>
#include <linux/interrupt.h>
#include <linux/mfd/88pm860x.h>
#include <linux/i2c/pca9575.h>
#include <linux/i2c/pca953x.h>
#include <linux/i2c/elan_touch.h>
#include <linux/i2c/ft5306_touch.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/cmmb.h>
#include <linux/proc_fs.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>

#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa910.h>
#include <mach/pxa910.h>
#include <mach/regs-usb.h>
#include <mach/regs-rtc.h>

#include <plat/usb.h>
#include <mach/regs-rtc.h>

#include "common.h"
#include "onboard.h"

#define TTCDKB_NR_IRQS		(IRQ_BOARD_START + 24)

static int is_td_dkb;
static int __init td_dkb_setup(char *__unused)
{
	return is_td_dkb = 1;
}
__setup("td_dkb", td_dkb_setup);

static unsigned long ttc_dkb_pin_config[] __initdata = {
	/* GPS GPIO */
	GPIO45_GPIO, /*share with TPO reset*/

	/* UART2 GPS UART */
	GPIO43_UART2_RXD,
	GPIO44_UART2_TXD,

	/* UART0 FFUART */
	GPIO47_UART0_RXD,
	GPIO48_UART0_TXD,

	/* UART1 BT_UART */
	GPIO29_UART1_CTS,
	GPIO30_UART1_RTS,
	GPIO31_UART1_TXD,
	GPIO32_UART1_RXD,

	/* DFI */
	DF_IO0_ND_IO0,
	DF_IO1_ND_IO1,
	DF_IO2_ND_IO2,
	DF_IO3_ND_IO3,
	DF_IO4_ND_IO4,
	DF_IO5_ND_IO5,
	DF_IO6_ND_IO6,
	DF_IO7_ND_IO7,
	DF_IO8_ND_IO8,
	DF_IO9_ND_IO9,
	DF_IO10_ND_IO10,
	DF_IO11_ND_IO11,
	DF_IO12_ND_IO12,
	DF_IO13_ND_IO13,
	DF_IO14_ND_IO14,
	DF_IO15_ND_IO15,
	DF_nCS0_SM_nCS2_nCS0,
	DF_ALE_SM_WEn_ND_ALE,
	DF_CLE_SM_OEn_ND_CLE,
	DF_WEn_DF_WEn,
	DF_REn_DF_REn,
	DF_RDY0_DF_RDY0,

	/* I2C */
	GPIO53_CI2C_SCL,
	GPIO54_CI2C_SDA,

	/* mmc */
	MMC1_DAT7_MMC1_DAT7,
	MMC1_DAT6_MMC1_DAT6,
	MMC1_DAT5_MMC1_DAT5,
	MMC1_DAT4_MMC1_DAT4,
	MMC1_DAT3_MMC1_DAT3,
	MMC1_DAT2_MMC1_DAT2,
	MMC1_DAT1_MMC1_DAT1,
	MMC1_DAT0_MMC1_DAT0,
	MMC1_CMD_MMC1_CMD,
	MMC1_CLK_MMC1_CLK,
	MMC1_CD_MMC1_CD | MFP_PULL_HIGH,
	MMC1_WP_MMC1_WP | MFP_PULL_LOW,

	/* one wire */
	ONEWIRE_CLK_REQ,

	/*keypad*/
	GPIO00_KP_MKIN0,
	GPIO01_KP_MKOUT0,
	GPIO02_KP_MKIN1,
	GPIO03_KP_MKOUT1,
	GPIO04_KP_MKIN2,
	GPIO05_KP_MKOUT2,
	GPIO06_KP_MKIN3,
	GPIO07_KP_MKOUT3,
	GPIO08_KP_MKIN4,
	GPIO09_KP_MKOUT4,
	GPIO12_KP_MKIN6,

	/* AGPS GPIO */
	GPIO45_GPIO45, /*share with TPO reset*/
	/* RDA8207 XOUT2_EN enable signal for AGPS clock */
	GPIO113_GPS_CLKEN | MFP_PULL_HIGH,
};

static unsigned long lcd_tpo_pin_config[] __initdata = {
	GPIO81_LCD_FCLK,
	GPIO82_LCD_LCLK,
	GPIO83_LCD_PCLK,
	GPIO84_LCD_DENA,
	GPIO85_LCD_DD0,
	GPIO86_LCD_DD1,
	GPIO87_LCD_DD2,
	GPIO88_LCD_DD3,
	GPIO89_LCD_DD4,
	GPIO90_LCD_DD5,
	GPIO91_LCD_DD6,
	GPIO92_LCD_DD7,
	GPIO93_LCD_DD8,
	GPIO94_LCD_DD9,
	GPIO95_LCD_DD10,
	GPIO96_LCD_DD11,
	GPIO97_LCD_DD12,
	GPIO98_LCD_DD13,
	GPIO100_LCD_DD14,
	GPIO101_LCD_DD15,
	GPIO102_LCD_DD16,
	GPIO103_LCD_DD17,

	GPIO104_LCD_SPIDOUT,
	GPIO105_LCD_SPIDIN,
	GPIO106_LCD_RESET,
	GPIO107_LCD_CS1,
	GPIO108_LCD_DCLK,
};

static unsigned long ttc_rf_pin_config[] = {
	/* GSM */
	GPIO110_GPIO110 | MFP_PULL_LOW,
	GPIO111_GPIO111 | MFP_PULL_LOW,
	GPIO112_GPIO112 | MFP_PULL_LOW,
	GPIO113_GPIO113 | MFP_PULL_LOW,
	GPIO114_GPIO114 | MFP_PULL_LOW,
	GPIO115_GPIO115 | MFP_PULL_LOW,
	GPIO116_GPIO116 | MFP_PULL_LOW,
	/*TDS-CDMA*/
	GPIO60_GPIO60 | MFP_PULL_LOW,
	GPIO61_GPIO61,
	GPIO62_GPIO62,
	GPIO63_GPIO63,
	GPIO64_GPIO64,
	GPIO65_GPIO65,
	GPIO66_GPIO66,
};

static unsigned long tds_pin_config[] __initdata = {
	GPIO55_TDS_LNACTRL,
	GPIO57_TDS_TRXSW,
	GPIO58_TDS_RXREV,
	GPIO59_TDS_TXREV,
	GPIO60_TD_GPIO60 | MFP_PULL_HIGH,
};

static struct mtd_partition ttc_dkb_onenand_partitions[] = {
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= SZ_1M,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "reserved",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "reserved",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_8M,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= (SZ_2M + SZ_1M),
		.mask_flags	= 0,
	}, {
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_48M,
		.mask_flags	= 0,
	}
};

static struct onenand_platform_data ttc_dkb_onenand_info = {
	.parts		= ttc_dkb_onenand_partitions,
	.nr_parts	= ARRAY_SIZE(ttc_dkb_onenand_partitions),
};

static struct resource ttc_dkb_resource_onenand[] = {
	[0] = {
		.start	= SMC_CS0_PHYS_BASE,
		.end	= SMC_CS0_PHYS_BASE + SZ_1M,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ttc_dkb_device_onenand = {
	.name		= "onenand-flash",
	.id		= -1,
	.resource	= ttc_dkb_resource_onenand,
	.num_resources	= ARRAY_SIZE(ttc_dkb_resource_onenand),
	.dev		= {
		.platform_data	= &ttc_dkb_onenand_info,
	},
};

static unsigned int ttc_dkb_matrix_key_map[] = {
	KEY(0, 0, KEY_BACKSPACE),
	KEY(0, 1, KEY_END),
	KEY(0, 2, KEY_RIGHTCTRL),
	KEY(0, 3, KEY_0),
	KEY(0, 4, KEY_1),

	KEY(1, 0, KEY_MENU),
	KEY(1, 1, KEY_HOME),
	KEY(1, 2, KEY_SEND),
	KEY(1, 3, KEY_8),
	KEY(1, 4, KEY_9),

	KEY(2, 0, KEY_OK),
	KEY(2, 1, KEY_2),
	KEY(2, 2, KEY_3),
	KEY(2, 3, KEY_4),
	KEY(2, 4, KEY_5),

	KEY(3, 0, KEY_6),
	KEY(3, 1, KEY_VOLUMEUP),
	KEY(3, 2, KEY_7),
	KEY(3, 3, KEY_VOLUMEDOWN),
	KEY(3, 4, KEY_RECORD),

	KEY(4, 0, KEY_KPASTERISK),
	KEY(4, 1, KEY_KPDOT),
	KEY(4, 2, KEY_F2),
	KEY(4, 3, KEY_CAMERA),
	KEY(4, 4, KEY_CAMERA),

	KEY(6, 0, KEY_F1),
	KEY(6, 1, KEY_UP),
	KEY(6, 2, KEY_DOWN),
	KEY(6, 3, KEY_LEFT),
	KEY(6, 4, KEY_RIGHT),
};

static struct pxa3xx_nand_platform_data dkb_nand_info = {
	.attr		= ARBI_EN | NAKED_CMD,
	.num_cs		= 1,
};

static struct pxa27x_keypad_platform_data ttc_dkb_keypad_info __initdata = {
	.matrix_key_rows	= 7,
	.matrix_key_cols	= 5,
	.matrix_key_map		= ttc_dkb_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(ttc_dkb_matrix_key_map),
	.debounce_interval	= 30,
};

static int ttc_dkb_pm860x_fixup(struct pm860x_chip *chip,
			struct pm860x_platform_data *pdata)
{
	int data;
	/*
	Check testpage 0xD7:bit[0~1],if it is b00 or b11, that's to say
	2LSB of 0xD7 is maybe broken, will reset 0xD0~0xD7 to its default
	in test page by set 0xE1:b[7~6]=b00 for loading OTP;
	Besides, 0xE1:b[5~0] work as a counter to record times of D7 broken
	*/
	data = pm860x_page_reg_read(chip->client, 0xD7);
	data &= 0x3;
	if (data == 0x0 || data == 0x3) {
		data = pm860x_page_reg_read(chip->client, 0xE1);
		data &= 0x3F;
		if (data < 0x3F)
			data += 1;
		pm860x_page_reg_write(chip->client, 0xE1, data);
		data = pm860x_page_reg_read(chip->client, 0xE1);
		dev_dbg(chip->dev, "detect 0xD7 broken counter: %d", data);
	}
	/*confirm the interrupt mask*/
	pm860x_reg_write(chip->client, PM8607_INT_MASK_1, 0x00);
	pm860x_reg_write(chip->client, PM8607_INT_MASK_2, 0x00);
	pm860x_reg_write(chip->client, PM8607_INT_MASK_3, 0x00);

	pm860x_reg_write(chip->client, PM8607_INT_STATUS1, 0x3f);
	pm860x_reg_write(chip->client, PM8607_INT_STATUS1, 0xff);
	pm860x_reg_write(chip->client, PM8607_INT_STATUS1, 0xff);

	/* disable LDO5 turn on/off by LDO3_EN */
	pm860x_reg_write(chip->client, PM8607_MISC2,
	pm860x_reg_read(chip->client, PM8607_MISC2)|0x80);
	/* enable LDO5 for AVDD_USB */
	pm860x_reg_write(chip->client, PM8607_SUPPLIES_EN11,
	pm860x_reg_read(chip->client, PM8607_SUPPLIES_EN11)|0x80);

	/* init GPADC*/
	pm860x_reg_write(chip->client, PM8607_GPADC_MISC1, 0x0b);
	/* init power mode*/
	pm860x_reg_write(chip->client, PM8607_SLEEP_MODE1, 0xaa);
	pm860x_reg_write(chip->client, PM8607_SLEEP_MODE2, 0xaa);
	pm860x_reg_write(chip->client, PM8607_SLEEP_MODE3, 0xa2);
	/* set LDO14_SLP to be active in sleep mode */
	if (is_td_dkb)
		pm860x_reg_write(chip->client, PM8607_SLEEP_MODE4, 0x38);
	else
		pm860x_reg_write(chip->client, PM8607_SLEEP_MODE4, 0x3a);

	/* set vbuck1 0.9v in sleep*/
	pm860x_reg_write(chip->client, PM8607_SLEEP_BUCK1, 0x24);
	pm860x_reg_write(chip->client, PM8607_SLEEP_BUCK2, 0x24);
	/*RTC to use ext 32k clk*/
	pm860x_set_bits(chip->client, PM8607_RTC1, 1<<6, 1<<6);
	/*Enable RTC to use ext 32k clk*/
	pm860x_set_bits(chip->client, PM8607_RTC_MISC2, 0x7, 0x2);

	/* shut down LDO13 for no use on pxa920 */
	pm860x_reg_write(chip->client, PM8607_VIBRA_SET, 0x0c);
	/* audio save power */
	pm860x_reg_write(chip->client, PM8607_LP_CONFIG1, 0x40);
	/*to save pmic leakage*/
	pm860x_reg_write(chip->client, PM8607_LP_CONFIG3, 0x80);
	pm860x_reg_write(chip->client, PM8607_B0_MISC1, 0x80);
	pm860x_reg_write(chip->client, PM8607_MEAS_OFF_TIME1, 0x2);
	/* config sanremo Buck Controls Register to its default value
	to save 0.04mA in suspend. */
	pm860x_reg_write(chip->client, PM8607_BUCK_CONTROLS, 0x2b);
	pm860x_reg_write(chip->client, PM8607_LP_CONFIG2, 0x98);

	/* force LDO4 be active in sleep mode, required by CP */
	pm860x_set_bits(chip->client, PM8607_SLEEP_MODE2, 3 << 4, 3 << 4);
	return 0;
}

static struct pm860x_touch_pdata ttc_dkb_touch = {
	.gpadc_prebias	= 1,
	.slot_cycle	= 1,
	.tsi_prebias	= 6,
	.pen_prebias	= 16,
	.pen_prechg	= 2,
	.res_x		= 300,
};

static struct pm860x_backlight_pdata ttc_dkb_backlight[] = {
	{
		.id	= PM8606_ID_BACKLIGHT,
		.iset	= PM8606_WLED_CURRENT(4),
		.flags	= PM8606_BACKLIGHT1,
	},
};

static struct pm860x_led_pdata ttc_dkb_led[] = {
	{
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_RED,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_GREEN,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_BLUE,
	},
};

static struct regulator_consumer_supply ttc_dkb_regulator_supply[] = {
	[PM8607_ID_BUCK1]	= REGULATOR_SUPPLY("v_buck1", NULL),
	[PM8607_ID_BUCK3]	= REGULATOR_SUPPLY("v_buck3", NULL),
	[PM8607_ID_LDO1]	= REGULATOR_SUPPLY("v_ldo1", NULL),
	[PM8607_ID_LDO2]	= REGULATOR_SUPPLY("v_ldo2", NULL),
	[PM8607_ID_LDO3]	= REGULATOR_SUPPLY("v_ldo3", NULL),
	[PM8607_ID_LDO4]	= REGULATOR_SUPPLY("v_ldo4", NULL),
	[PM8607_ID_LDO5]	= REGULATOR_SUPPLY("v_ldo5", NULL),
	[PM8607_ID_LDO6]	= REGULATOR_SUPPLY("v_ldo6", NULL),
	[PM8607_ID_LDO7]	= REGULATOR_SUPPLY("v_ldo7", NULL),
	[PM8607_ID_LDO8]	= REGULATOR_SUPPLY("v_ldo8", NULL),
	[PM8607_ID_LDO9]	= REGULATOR_SUPPLY("v_ldo9", NULL),
	[PM8607_ID_LDO10]	= REGULATOR_SUPPLY("v_ldo10", NULL),
	[PM8607_ID_LDO12]	= REGULATOR_SUPPLY("v_ldo12", NULL),
	[PM8607_ID_LDO13]	= REGULATOR_SUPPLY("v_ldo13", NULL),
	[PM8607_ID_LDO14]	= REGULATOR_SUPPLY("v_ldo14", NULL),
};
static int regulator_index[] = {
	PM8607_ID_BUCK1,
	PM8607_ID_BUCK2,
	PM8607_ID_BUCK3,
	PM8607_ID_LDO1,
	PM8607_ID_LDO2,
	PM8607_ID_LDO3,
	PM8607_ID_LDO4,
	PM8607_ID_LDO5,
	PM8607_ID_LDO6,
	PM8607_ID_LDO7,
	PM8607_ID_LDO8,
	PM8607_ID_LDO9,
	PM8607_ID_LDO10,
	PM8607_ID_LDO11,
	PM8607_ID_LDO12,
	PM8607_ID_LDO13,
	PM8607_ID_LDO14,
	PM8607_ID_LDO15,
};
#define DKB_REG_INIT(_name, _min, _max, _always, _boot)			\
{									\
	.constraints = {						\
		.name		= __stringify(_name),			\
		.min_uV		= _min,					\
		.max_uV		= _max,					\
		.always_on	= _always,				\
		.boot_on	= _boot,				\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE		\
				| REGULATOR_CHANGE_STATUS,		\
	},								\
	.num_consumer_supplies	= 1,					\
	.consumer_supplies	=					\
			&ttc_dkb_regulator_supply[PM8607_ID_##_name],	\
	.driver_data		= &regulator_index[PM8607_ID_##_name],  \
}

static struct regulator_init_data ttc_dkb_regulator_init_data[] = {
	DKB_REG_INIT(BUCK1, 1000000, 1500000, 1, 1),
	DKB_REG_INIT(BUCK3, 1000000, 3000000, 1, 1),
	DKB_REG_INIT(LDO1, 1200000, 2800000, 1, 1),
	DKB_REG_INIT(LDO2, 1800000, 3300000, 1, 1),
	DKB_REG_INIT(LDO3, 1800000, 3300000, 1, 1),
	DKB_REG_INIT(LDO4, 1800000, 3300000, 1, 1),
	DKB_REG_INIT(LDO5, 2900000, 3300000, 1, 1),
	DKB_REG_INIT(LDO6, 1800000, 3300000, 1, 1),
	DKB_REG_INIT(LDO7, 1800000, 2900000, 1, 1),
	DKB_REG_INIT(LDO8, 1800000, 2900000, 1, 1),
	DKB_REG_INIT(LDO9, 1800000, 3300000, 1, 1),
	DKB_REG_INIT(LDO10, 1200000, 3300000, 1, 1),
	DKB_REG_INIT(LDO12, 1200000, 3300000, 0, 1),
	DKB_REG_INIT(LDO13, 1200000, 3000000, 0, 1),
	DKB_REG_INIT(LDO14, 1800000, 3300000, 0, 1),
};

/* RF has leak current when battery calibration */
static void ttc_disable_rf(void)
{
	/* disable rf */
	mfp_config(ARRAY_AND_SIZE(ttc_rf_pin_config));
}

struct pm860x_power_pdata ttc_dkb_power = {
	.disable_rf_fn  = ttc_disable_rf,
};

#ifdef CONFIG_RTC_DRV_MMP
static int sync_time_to_soc(unsigned int ticks)
{
	RCNR = ticks;
	return 0;
}
#endif

struct pm860x_rtc_pdata ttc_dkb_rtc = {
	.vrtc		= 1,
#ifdef CONFIG_RTC_DRV_MMP
	.sync		= sync_time_to_soc,
#endif
};

static struct pm860x_platform_data ttc_dkb_pm8607_info = {
	.backlight	= &ttc_dkb_backlight[0],
	.led		= &ttc_dkb_led[0],
	.touch		= &ttc_dkb_touch,
	.power		= &ttc_dkb_power,
	.rtc		= &ttc_dkb_rtc,
	.regulator	= &ttc_dkb_regulator_init_data[0],
	.fixup		= ttc_dkb_pm860x_fixup,
	.companion_addr	= 0x11,
	.irq_mode	= 0,
	.irq_base	= IRQ_BOARD_START,

	.i2c_port	= GI2C_PORT,
	.num_backlights	= ARRAY_SIZE(ttc_dkb_backlight),
	.num_leds	= ARRAY_SIZE(ttc_dkb_led),
	.num_regulators	= ARRAY_SIZE(ttc_dkb_regulator_init_data),
};

#if defined(CONFIG_GPIO_PCA9575)
static struct pca9575_platform_data pca9575_data[] = {
	[0] = {
		.gpio_base      = GPIO_EXT1(0),
	},
};
#endif

#if defined(CONFIG_GPIO_PCA953X)
static struct pca953x_platform_data max7312_data[] = {
	[0] = {
		.gpio_base      = GPIO_EXT0(0),
	},
};
#endif

#if defined(CONFIG_TOUCHSCREEN_FT5306)
static int ft5306_touch_io_power_onoff(int on)
{
	unsigned int tp_logic_en;
	tp_logic_en = GPIO_EXT0(MFP_PIN_GPIO15);

	if (gpio_request(tp_logic_en, "TP_LOGIC_EN")) {
		printk(KERN_ERR "Request GPIO failed,"
			"gpio: %d \n", tp_logic_en);
		return -EIO;
	}

	if (on)
		gpio_direction_output(tp_logic_en, 1);
	else
		gpio_direction_output(tp_logic_en, 0);

	gpio_free(tp_logic_en);
	return 0;
}
static struct ft5306_touch_platform_data ft5306_touch_data = {
	.power = ft5306_touch_io_power_onoff,
};
#endif

#if defined(CONFIG_TOUCHSCREEN_ELAN)
static int touch_io_power_onoff(int on)
{
	unsigned int tp_logic_en;
	tp_logic_en = GPIO_EXT0(MFP_PIN_GPIO15);

	if (gpio_request(tp_logic_en, "TP_LOGIC_EN")) {
		printk(KERN_ERR "Request GPIO failed,"
			"gpio: %d\n", tp_logic_en);
		return -EIO;
	}

	if (on)
		gpio_direction_output(tp_logic_en, 1);
	else
		gpio_direction_output(tp_logic_en, 0);

	gpio_free(tp_logic_en);
	return 0;
}
static struct elan_touch_platform_data elan_touch_data = {
	.power = touch_io_power_onoff,
};
#endif
static unsigned long lis33ldl_min_delay = 50;
static struct i2c_board_info ttc_dkb_i2c_info[] = {
	{
		.type		= "88PM860x",
		.addr		= 0x34,
		.platform_data	= &ttc_dkb_pm8607_info,
		.irq		= IRQ_PXA910_PMIC_INT,
	},
#if defined(CONFIG_GPIO_PCA9575)
	{
		.type           = "pca9575",
		.addr           = 0x20,
		.irq            = IRQ_GPIO(19),
		.platform_data  = &pca9575_data,
	},
#endif
#if defined(CONFIG_GPIO_PCA953X)
	{
		.type           = "max7312",
		.addr           = 0x23,
		.irq            = IRQ_GPIO(80),
		.platform_data  = &max7312_data,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_TPO)
	{
		.type		= "tpo_touch",
		.addr		= 0x18,
		.irq		= gpio_to_irq(45),
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_FT5306)
	{
		.type		= "ft5306_touch",
		.addr		=  0x3A,
		.irq		= gpio_to_irq(45),
		.platform_data	= &ft5306_touch_data,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_ELAN)
	{
		.type		= "elan_touch",
		.addr		= 0x8,
		.irq		= gpio_to_irq(45),
		.platform_data	= &elan_touch_data,
	},
#endif
#if defined(CONFIG_SENSORS_LIS331DL)
	{
		.type           = "lis331dl",
		.addr           =  0x1c,
		.platform_data  = &lis33ldl_min_delay,
	},
#endif
};

/* workaround for reset i2c bus by GPIO53 -SCL, GPIO54 -SDA */
static void i2c_pxa_bus_reset(void)
{
	unsigned long i2c_mfps[] = {
		GPIO53_GPIO53,		/* SCL */
		GPIO54_GPIO54,		/* SDA */
	};
	unsigned long mfp_pin[ARRAY_SIZE(i2c_mfps)];
	int ccnt;

	if (gpio_request(MFP_PIN_GPIO53, "SCL")) {
		pr_err("Failed to request GPIO for SCL pin!\n");
		goto out;
	}
	if (gpio_request(MFP_PIN_GPIO54, "SDA")) {
		pr_err("Failed to request GPIO for SDA pin!\n");
		goto out_sda;
	}
	pr_info("\t<<<i2c bus reseting>>>\n");
	/* set mfp pins to gpio */
	mfp_pin[0] = mfp_read(MFP_PIN_GPIO53);
	mfp_pin[1] = mfp_read(MFP_PIN_GPIO54);
	mfp_config(ARRAY_AND_SIZE(i2c_mfps));

	gpio_direction_input(MFP_PIN_GPIO54);
	for (ccnt = 20; ccnt; ccnt--) {
		gpio_direction_output(MFP_PIN_GPIO53, ccnt & 0x01);
		udelay(4);
	}
	gpio_direction_output(MFP_PIN_GPIO53, 0);
	udelay(4);
	gpio_direction_output(MFP_PIN_GPIO54, 0);
	udelay(4);
	/* stop signal */
	gpio_direction_output(MFP_PIN_GPIO53, 1);
	udelay(4);
	gpio_direction_output(MFP_PIN_GPIO54, 1);
	udelay(4);

	mfp_write(MFP_PIN_GPIO53, mfp_pin[0]);
	mfp_write(MFP_PIN_GPIO54, mfp_pin[1]);
	gpio_free(MFP_PIN_GPIO54);
out_sda:
	gpio_free(MFP_PIN_GPIO53);
out:
	return;
}

static struct i2c_pxa_platform_data dkb_i2c_pdata = {
	.fast_mode		 = 1,
	/* ilcr:fs mode b17~9=0x22,about 380K, standard mode b8~0=0x7E,100K */
	.ilcr			 = 0x082C447E,
	/* iwcr:b5~0=b01010 recommended value according to spec*/
	.hardware_lock		= pxa910_ripc_lock,
	.hardware_unlock	= pxa910_ripc_unlock,
	.hardware_trylock	= pxa910_ripc_trylock,
	.i2c_bus_reset		= i2c_pxa_bus_reset,
};

static struct i2c_pxa_platform_data ttc_dkb_pwr_i2c_pdata = {
	.fast_mode		 = 1,
	/* ilcr:fs mode b17~9=0x22,about 380K, standard mode b8~0=0x7E,100K */
	.ilcr			 = 0x082C447E,
	/* iwcr:b5~0=b01010 recommended value according to spec*/
	.iwcr			= 0x0000142A,
};

static struct platform_device *ttc_dkb_devices[] = {
	&ttc_dkb_device_onenand,
	&pxa910_device_rtc,
};

#if (defined CONFIG_CMMB)

static unsigned long cmmb_pin_config[] = {
	GPIO33_SSP0_CLK,
	GPIO35_SSP0_RXD,
	GPIO36_SSP0_TXD,
};

static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect = 1,
	.enable_dma = 1,
};

static int cmmb_power_reset(void)
{
	int cmmb_rst;

	cmmb_rst = GPIO_EXT1(7);

	if (gpio_request(cmmb_rst, "cmmb rst")) {
		pr_warning("failed to request GPIO for CMMB RST\n");
		return -EIO;
	}

	/* reset cmmb, keep low for about 1ms */
	gpio_direction_output(cmmb_rst, 0);
	msleep(100);

	/* get cmmb go out of reset state */
	gpio_direction_output(cmmb_rst, 1);
	gpio_free(cmmb_rst);

	return 0;
}

static int cmmb_power_on(void)
{
	int cmmb_en, cmmb_rst;

	cmmb_en = GPIO_EXT1(6);
	if (gpio_request(cmmb_en, "cmmb power")) {
		pr_warning("[ERROR] failed to request GPIO for CMMB POWER\n");
		return -EIO;
	}
	gpio_direction_output(cmmb_en, 0);
	msleep(100);

	gpio_direction_output(cmmb_en, 1);
	gpio_free(cmmb_en);

	msleep(100);

	cmmb_power_reset();

	return 0;
}

static int cmmb_power_off(void)
{
	int cmmb_en;

	cmmb_en = GPIO_EXT1(6);

	if (gpio_request(cmmb_en, "cmmb power")) {
		pr_warning("failed to request GPIO for CMMB POWER\n");
		return -EIO;
	}

	gpio_direction_output(cmmb_en, 0);
	gpio_free(cmmb_en);
	msleep(100);

	return 0;
}
/*.
 ** Add two functions: cmmb_cs_assert and cmmb_cs_deassert.
 ** Provide the capbility that
 ** cmmb driver can handle the SPI_CS by itself.
 **/
static int cmmb_cs_assert(void)
{
	int cs;
	cs = mfp_to_gpio(GPIO34_SSP0_FRM);
	gpio_direction_output(cs, 0);
	return 0;
}

static int cmmb_cs_deassert(void)
{
	int cs;
	cs = mfp_to_gpio(GPIO34_SSP0_FRM);
	gpio_direction_output(cs, 1);
	return 0;
}

static struct cmmb_platform_data cmmb_info = {
	.power_on = cmmb_power_on,
	.power_off = cmmb_power_off,
	.power_reset = cmmb_power_reset,
	.cs_assert = cmmb_cs_assert,
	.cs_deassert = cmmb_cs_deassert,

	.gpio_power = GPIO_EXT1(6),
	.gpio_reset = GPIO_EXT1(7),
	.gpio_cs = mfp_to_gpio(GPIO34_SSP0_FRM),
	.gpio_defined = 1,
};

static void cmmb_if101_cs(u32 cmd)
{
/* Because in CMMB read/write,the max data size is more than 8kB
 * 8k = max data length per dma transfer for pxaxxx
 * But till now,The spi_read/write driver doesn't support muti DMA cycles
 *
 * Here the spi_read/write will not affect the SPI_CS,but provides
 * cs_assert and cs_deassert in the struct cmmb_platform_data
 *
 * And cmmb driver can/should control SPI_CS by itself
 */
}

static struct pxa2xx_spi_chip cmmb_if101_chip = {
	.rx_threshold   = 1,
	.tx_threshold   = 1,
	.cs_control     = cmmb_if101_cs,
};

/* bus_num must match id in pxa2xx_set_spi_info() call */
static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias		= "cmmb_if",
		.platform_data	= &cmmb_info,
		.controller_data	= &cmmb_if101_chip,
		.irq			= gpio_to_irq(mfp_to_gpio(GPIO14_GPIO)),
		.max_speed_hz	= 8000000,
		.bus_num		= 1,
		.chip_select	= 0,
		.mode			= SPI_MODE_0,
	},
};

static void __init ttc_dkb_init_spi(void)
{
	int err;
	int cmmb_int, cmmb_cs;

	mfp_config(ARRAY_AND_SIZE(cmmb_pin_config));
	cmmb_cs = mfp_to_gpio(GPIO34_SSP0_FRM);
	err = gpio_request(cmmb_cs, "cmmb cs");
	if (err) {
		pr_warning("[ERROR] failed to request GPIO for CMMB CS\n");
		return;
	}
	gpio_direction_output(cmmb_cs, 1);

	cmmb_int = mfp_to_gpio(GPIO14_GPIO);

	err = gpio_request(cmmb_int, "cmmb irq");
	if (err) {
		pr_warning("[ERROR] failed to request GPIO for CMMB IRQ\n");
		return;
	}
	gpio_direction_input(cmmb_int);

	pxa910_add_ssp(0);
	pxa910_add_spi(1, &pxa_ssp_master_info);
	if (spi_register_board_info(spi_board_info,
			ARRAY_SIZE(spi_board_info))) {
		pr_warning("[ERROR] failed to register spi device.\n");
		return;
	}
}

#endif /*defined CONFIG_CMMB*/

/* MMC0 controller for SD-MMC */
static struct sdhci_pxa_platdata pxa910_sdh_platdata_mmc0 = {
	.flags			= PXA_FLAG_ENABLE_CLOCK_GATING,
	.clk_delay_sel		= 1,
	.clk_delay_cycles	= 2,
};

static void __init pxa910_init_mmc(void)
{
	unsigned long sd_pwr_cfg = GPIO15_MMC1_POWER;
	int sd_pwr_en = 0;

	if (is_td_dkb) {
		mfp_config(&sd_pwr_cfg, 1);
		sd_pwr_en = mfp_to_gpio(sd_pwr_cfg);

		if (gpio_request(sd_pwr_en, "SD Power Ctrl")) {
			printk(KERN_ERR "Failed to request SD_PWR_EN(gpio %d)\n", sd_pwr_en);
			sd_pwr_en = 0;
		} else {
			gpio_direction_output(sd_pwr_en, 1);
			gpio_free(sd_pwr_en);
		}
	}

	pxa910_add_sdh(0, &pxa910_sdh_platdata_mmc0); /* SD/MMC */
}

/* GPS: power on/off control */
static void gps_power_on(void)
{
	unsigned int gps_ldo, gps_rst_n;

	gps_ldo = (is_td_dkb) ? GPIO_EXT1(8) : GPIO_EXT1(7);
	if (gpio_request(gps_ldo, "gpio_gps_ldo")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_ldo);
		return;
	}

	gps_rst_n = (is_td_dkb) ? GPIO_EXT1(11) : mfp_to_gpio(MFP_PIN_GPIO15);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_rst_n);
		goto out;
	}

	gpio_direction_output(gps_ldo, 0);
	gpio_direction_output(gps_rst_n, 0);
	mdelay(1);
	gpio_direction_output(gps_ldo, 1);

	pr_info("sirf gps chip powered on\n");

	gpio_free(gps_rst_n);
out:
	gpio_free(gps_ldo);
	return;
}

static void gps_power_off(void)
{
	unsigned int gps_ldo, gps_rst_n, gps_on;

	gps_ldo = (is_td_dkb) ? GPIO_EXT1(8) : GPIO_EXT1(7);
	if (gpio_request(gps_ldo, "gpio_gps_ldo")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_ldo);
		return;
	}

	gps_on = (is_td_dkb) ? GPIO_EXT1(10) : GPIO_EXT1(1);
	if (gpio_request(gps_on, "gpio_gps_on")) {
		pr_err("Request GPIO failed,gpio: %d\n", gps_on);
		goto out1;
	}

	gps_rst_n = (is_td_dkb) ? GPIO_EXT1(11) : mfp_to_gpio(MFP_PIN_GPIO15);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		pr_debug("Request GPIO failed, gpio: %d\n", gps_rst_n);
		goto out2;
	}

	gpio_direction_output(gps_ldo, 0);
	gpio_direction_output(gps_rst_n, 0);
	gpio_direction_output(gps_on, 0);

	pr_info("sirf gps chip powered off\n");

	gpio_free(gps_rst_n);
out2:
	gpio_free(gps_on);
out1:
	gpio_free(gps_ldo);
	return;
}

static void gps_reset(int flag)
{
	unsigned int gps_rst_n;

	gps_rst_n = (is_td_dkb) ? GPIO_EXT1(11) : mfp_to_gpio(MFP_PIN_GPIO15);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_rst_n);
		return;
	}

	gpio_direction_output(gps_rst_n, flag);
	gpio_free(gps_rst_n);
	pr_info("sirf gps chip reset\n");
}

static void gps_on_off(int flag)
{
	unsigned int gps_on;

	gps_on = (is_td_dkb) ? GPIO_EXT1(10) : GPIO_EXT1(1);
	if (gpio_request(gps_on, "gpio_gps_on")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_on);
		return;
	}
	gpio_direction_output(gps_on, flag);
	gpio_free(gps_on);
	pr_info("sirf gps chip offon\n");
}

#if defined(CONFIG_PROC_FS)

#define SIRF_STATUS_LEN	16
static char sirf_status[SIRF_STATUS_LEN] = "off";

static ssize_t sirf_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = strlen(sirf_status);

	sprintf(page, "%s\n", sirf_status);
	return len + 1;
}

static ssize_t sirf_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	int flag, ret;
	char buffer[7];

	if (len > 255)
		len = 255;

	memset(messages, 0, sizeof(messages));

	if (!buff || copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strlen(messages) > (SIRF_STATUS_LEN - 1)) {
		pr_warning("[ERROR] messages too long! (%d) %s\n",
			strlen(messages), messages);
		return -EFAULT;
	}

	if (strncmp(messages, "off", 3) == 0) {
		strcpy(sirf_status, "off");
		gps_power_off();
	} else if (strncmp(messages, "on", 2) == 0) {
		strcpy(sirf_status, "on");
		gps_power_on();
	} else if (strncmp(messages, "reset", 5) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_reset(flag);
	} else if (strncmp(messages, "sirfon", 6) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_on_off(flag);
	} else
		pr_info("usage: echo {on/off} > /proc/driver/sirf\n");

	return len;
}

static void create_sirf_proc_file(void)
{
	struct proc_dir_entry *sirf_proc_file = NULL;

	sirf_proc_file = create_proc_entry("driver/sirf", 0644, NULL);
	if (!sirf_proc_file) {
		pr_err("sirf proc file create failed!\n");
		return;
	}

	sirf_proc_file->read_proc = sirf_read_proc;
	sirf_proc_file->write_proc = (write_proc_t  *)sirf_write_proc;
}
#endif

static void __init tds_mfp_init(void)
{
	if (is_td_dkb) {
		mfp_config(ARRAY_AND_SIZE(tds_pin_config));
	}
}

#ifdef CONFIG_USB_SUPPORT

#if defined(CONFIG_USB_PXA_U2O) || defined(CONFIG_USB_EHCI_PXA_U2O)

extern int pxa_usb_phy_init(unsigned int base);

static char *pxa910_usb_clock_name[] = {
	[0] = "U2OCLK",
};

#define STATUS2_VBUS	(1 << 4)
static int ttc_usb_vbus_poll(void)
{
	int ret = 0;

	ret = pm860x_codec_reg_read(PM8607_STATUS_2);
	if (ret < 0)
		return ret;

	if (ret & STATUS2_VBUS)
		ret = VBUS_HIGH;
	else
		ret = VBUS_LOW;

	return ret;
}

static struct mv_usb_addon_irq ttc_usb_vbus = {
	.irq	= IRQ_BOARD_START + PM8607_IRQ_CHG,
	.poll	= ttc_usb_vbus_poll,
};

#define MISC1_GPIO2_DIR		(1 << 5)
#define MISC1_GPIO2_VAL		(1 << 6)
static int ttc_usb_set_vbus(unsigned int on)
{
	int ret, data;

	ret = pm860x_codec_reg_read(PM8607_A1_MISC1);
	if (ret < 0)
		return ret;

	data = MISC1_GPIO2_DIR | ret;
	if (on)
		data |= MISC1_GPIO2_VAL;
	else
		data &= ~MISC1_GPIO2_VAL;

	ret = pm860x_codec_reg_write(PM8607_A1_MISC1, data);
	if (ret < 0)
		return ret;

	return 0;
}

static struct mv_usb_platform_data ttc_usb_pdata = {
	.clknum		= 1,
	.clkname	= pxa910_usb_clock_name,
	.vbus		= &ttc_usb_vbus,
	.mode		= MV_USB_MODE_OTG,
	.phy_init	= pxa_usb_phy_init,
	.set_vbus	= ttc_usb_set_vbus,
};
#endif
#endif

/*
 * for wvga panel:
 * 1: truly wvga panel
 * 2: sharp wvga panel
 */
#define TRULY_WVGA_PANEL 1
#define SHARP_WVGA_PANEL 2
static int wvga_lcd = 0;
static int __init wvga_lcd_setup(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	wvga_lcd = n;
	return 1;
}
__setup("wvga_lcd=", wvga_lcd_setup);

static int is_wvga_lcd(void)
{
	return wvga_lcd;
}

static void __init ttc_dkb_init(void)
{
	mfp_config(ARRAY_AND_SIZE(ttc_dkb_pin_config));
	tds_mfp_init();

	/* on-chip devices */
	pxa910_add_uart(0);
	pxa910_add_uart(1);
	pxa910_add_uart(2);
	pxa910_add_1wire();
	pxa910_add_nand(&dkb_nand_info);

	pxa910_add_keypad(&ttc_dkb_keypad_info);
	pxa910_add_cnm();
	pxa910_add_twsi(0, &dkb_i2c_pdata, ARRAY_AND_SIZE(ttc_dkb_i2c_info));

	pxa910_add_acipc();

#ifdef CONFIG_PXA910_IRE
	pxa910_add_ire();
#endif

#if defined(CONFIG_MMC_SDHCI_PXAV2)
	pxa910_init_mmc();
#endif

	/* off-chip devices */
	platform_add_devices(ARRAY_AND_SIZE(ttc_dkb_devices));

#ifdef CONFIG_FB_PXA168
	mfp_config(ARRAY_AND_SIZE(lcd_tpo_pin_config));
	if (TRULY_WVGA_PANEL == is_wvga_lcd()) {
		dkb_add_lcd_truly();
		pr_info("LCD: truly WVGA panel selected.\n");
	} else if (SHARP_WVGA_PANEL == is_wvga_lcd()) {
		dkb_add_lcd_sharp();
		pr_info("LCD: sharp WVGA panel selected.\n");
	} else
		dkb_add_lcd_tpo();
#endif

#if defined(CONFIG_PROC_FS)
	/* create proc for sirf GPS control */
	create_sirf_proc_file();
#endif

#if (defined CONFIG_CMMB)
	 /* spi device */
	if (is_td_dkb)
		ttc_dkb_init_spi();
#endif

#ifdef CONFIG_USB_PXA_U2O
	pxa168_device_u2o.dev.platform_data = &ttc_usb_pdata;
	platform_device_register(&pxa168_device_u2o);
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2O
	pxa168_device_u2oehci.dev.platform_data = &ttc_usb_pdata;
	platform_device_register(&pxa168_device_u2oehci);
#ifdef CONFIG_USB_PXA_U2O_OTG
	pxa168_device_u2ootg.dev.platform_data = &ttc_usb_pdata;
	platform_device_register(&pxa168_device_u2ootg);
#endif
#endif
}

MACHINE_START(TTC_DKB, "PXA910-based TTC_DKB Development Platform")
	.map_io		= mmp_map_io,
	.nr_irqs	= TTCDKB_NR_IRQS,
	.init_irq       = pxa910_init_irq,
	.timer          = &pxa910_timer,
	.reserve        = pxa910_reserve,
	.init_machine   = ttc_dkb_init,
MACHINE_END
