/*
 *  linux/arch/arm/mach-mmp/board-emeidkb.c
 *
 *  Support for the Marvell PXA988 Emei DKB Development Platform.
 *
 *  Copyright (C) 2012 Marvell International Ltd.
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
#include <linux/backlight.h>
#include <linux/i2c.h>
#include <linux/i2c/pxa-i2c.h>
#include <linux/mfd/88pm80x.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdhci.h>
#include <linux/platform_data/pxa_sdhci.h>
#include <linux/sd8x_rfkill.h>
#include <linux/regulator/machine.h>
#include <linux/i2c/elan_touch.h>
#include <linux/i2c/ft5306_touch.h>
#include <linux/mfd/88pm80x.h>
#include <linux/cwmi.h>
#include <linux/cwgd.h>
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/cmmb.h>
#include <linux/mfd/88pm80x.h>

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#endif

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa988.h>
#include <mach/pxa988.h>
#include <mach/irqs.h>
#include <mach/regs-mpmu.h>
#include <mach/sram.h>
#include <mach/regs-rtc.h>
#include <mach/soc_coda7542.h>
#include <mach/axis_sensor.h>
#include <mach/regs-apmu.h>
#include <mach/clock-pxa988.h>
#include <mach/regs-ciu.h>
#include <mach/system.h>

#ifdef CONFIG_PM_DEVFREQ
#include <plat/devfreq.h>
#endif
#include <plat/pmem.h>
#include <plat/pxa27x_keypad.h>
#include <plat/usb.h>

#include <media/soc_camera.h>
#include <mach/isp_dev.h>

#include "onboard.h"
#include "common.h"

#define EMEI_NR_IRQS		(IRQ_BOARD_START + 40)
#define PM8XXX_REGULATOR_MAX PM800_ID_RG_MAX

static unsigned long emeidkb_pin_config[] __initdata = {
	GPIO000_KP_MKIN0,	/* KP_MKIN[0] */
	GPIO001_KP_MKOUT0,	/* KP_MKOUT[0] */
	GPIO002_KP_MKIN1,	/* KP_MKIN[1] */
	GPIO003_KP_MKOUT1,	/* KP_MKOUT[1] */
	GPIO004_KP_MKIN2,	/* KP_MKIN[2] */
	GPIO005_KP_MKOUT2,	/* KP_MKOUT[2] */
	GPIO006_KP_MKIN3,	/* KP_MKIN[3] */

#define GPIO007_GPIO_WIB_PDn		GPIO007_GPIO_7
#define GPIO008_GPIO_WIB_WLAN		GPIO008_GPIO_8
#define GPIO009_GPIO_WIB_BT		GPIO009_GPIO_9
#define GPIO010_GPIO_RF_DCDC_EN		GPIO010_GPIO_10
#define GPIO011_GPIO_WIB_RESETn		GPIO011_GPIO_11
#define GPIO012_GPIO_TORCH_EN		GPIO012_GPIO_12
#define GPIO013_GPIO_FLASH_EN		GPIO013_GPIO_13
#define GPIO014_GPIO_PROX_IRQ		GPIO014_GPIO_14
#define GPIO015_GPIO_NFC_EN		GPIO015_GPIO_15
#define GPIO016_GPIO_TP_RESET		GPIO016_GPIO_16
#define GPIO017_GPIO_TP_INT		GPIO017_GPIO_17
#define GPIO018_GPIO_CMMB_EN		GPIO018_GPIO_18
#define GPIO019_GPIO_CMMB_RESET		GPIO019_GPIO_19
#define GPIO020_GPIO_CMMB_IRQ		GPIO020_GPIO_20
	GPIO007_GPIO_WIB_PDn,
	GPIO008_GPIO_WIB_WLAN,
	GPIO009_GPIO_WIB_BT,
	GPIO010_GPIO_RF_DCDC_EN,
	GPIO011_GPIO_WIB_RESETn,
	GPIO012_GPIO_TORCH_EN,
	GPIO013_GPIO_FLASH_EN,
	GPIO014_GPIO_PROX_IRQ | MFP_PULL_HIGH,
	GPIO015_GPIO_NFC_EN,
	GPIO016_GPIO_TP_RESET | MFP_PULL_FLOAT,
	GPIO017_GPIO_TP_INT | MFP_PULL_HIGH,
	GPIO018_GPIO_CMMB_EN,
	GPIO019_GPIO_CMMB_RESET,
	GPIO020_GPIO_CMMB_IRQ,

	GPIO021_I2S_BITCLK,	/* I2S_BITCLK */
	GPIO022_I2S_SYNC,	/* I2S_SYNC */
	GPIO023_I2S_DATA_OUT,	/* I2S_DATA_OUT */
	GPIO024_I2S_SDATA_IN,	/* I2S_DATA_IN */

	GPIO025_GSSP_SCLK,	/* PCM_CLK */
	GPIO026_GSSP_SFRM,	/* PCM_SYNC */
	GPIO027_GSSP_TXD,	/* PCM_TXD */
	GPIO028_GSSP_RXD,	/* PCM_RXD */

#define GPIO029_GPIO_CHARGER_IND1	GPIO029_GPIO_29
#define GPIO030_GPIO_CHARGER_IND2	GPIO030_GPIO_30
#define GPIO031_GPIO_CHARGER_EN		GPIO031_GPIO_31
#define GPIO032_GPIO_LCD_PWM		GPIO032_GPIO_32
	GPIO029_GPIO_CHARGER_IND1,
	GPIO030_GPIO_CHARGER_IND2,
	GPIO031_GPIO_CHARGER_EN,
	GPIO032_GPIO_LCD_PWM | MFP_PULL_FLOAT | MFP_LPM_FLOAT,

	GPIO033_SPI_DCLK,	/* CMMB_SPI_CLK */
	GPIO034_SPI_CS0,	/* CMMB_SPI_CS */
	GPIO035_SPI_DIN,	/* CMMB_SPI_DOUT */
	GPIO036_SPI_DOUT,	/* CMMB_SPI_DIN */

	/* MMC2 WIB */
	GPIO037_MMC2_DATA3,	/* WLAN_DAT3 */
	GPIO038_MMC2_DATA2,	/* WLAN_DAT2 */
	GPIO039_MMC2_DATA1,	/* WLAN_DAT1 */
	GPIO040_MMC2_DATA0,	/* WLAN_DAT0 */
	GPIO041_MMC2_CMD,	/* WLAN_CMD */
	GPIO042_MMC2_CLK,	/* WLAN_CLK */

#define GPIO043_GPIO_DVC1	GPIO043_GPIO_43
#define GPIO044_GPIO_DVC2	GPIO044_GPIO_44
	GPIO043_GPIO_DVC1,
	GPIO044_GPIO_DVC2,

	GPIO045_UART2_RXD,	/* GPS_UART_RXD */
	GPIO046_UART2_TXD,	/* GPS_UART_TXD */

	GPIO047_UART1_RXD,	/* AP_RXD */
	GPIO048_UART1_TXD,	/* AP_TXD */

#define GPIO049_GPIO_BARA_INT2	GPIO049_GPIO_49
#define GPIO050_GPIO_BARA_INT1	GPIO050_GPIO_50
	GPIO049_GPIO_BARA_INT2,
	GPIO050_GPIO_BARA_INT1,

	GPIO051_UART0_RXD,	/* CP_RXD */
	GPIO052_UART0_TXD,	/* CP_TXD */

	GPIO053_CI2C_SCL,	/* CI2C_SCL */
	GPIO054_CI2C_SDA,	/* CI2C_SDA */

	GPIO067_CCIC_IN7,	/* CAM_DATA<9> */
	GPIO068_CCIC_IN6,	/* CAM_DATA<8> */
	GPIO069_CCIC_IN5,	/* CAM_DATA<7> */
	GPIO070_CCIC_IN4,	/* CAM_DATA<6> */
	GPIO071_CCIC_IN3,	/* CAM_DATA<5> */
	GPIO072_CCIC_IN2,	/* CAM_DATA<4> */
	GPIO073_CCIC_IN1,	/* CAM_DATA<3> */
	GPIO074_CCIC_IN0,	/* CAM_DATA<2> */
	GPIO075_CAM_HSYNC,	/* CAM_HSYNC */
	GPIO076_CAM_VSYNC,	/* CAM_VSYNC */
	GPIO077_CAM_MCLK,	/* CAM_MCLK */
	GPIO078_CAM_PCLK,	/* CAM_PCLK */

#define GPIO079_GPIO_CAM_STROBE		GPIO079_GPIO_79
#define GPIO080_GPIO_CAM_PD_MAIN	GPIO080_GPIO_80
#define GPIO081_GPIO_CAM_RST_MAIN	GPIO081_GPIO_81
#define GPIO082_GPIO_CAM_PD_SUB		GPIO082_GPIO_82
#define GPIO083_GPIO_CAM_RST_SUB	GPIO083_GPIO_83
#define GPIO084_GPIO_GPS_RESET_N	GPIO084_GPIO_84
#define GPIO085_GPIO_GPS_CLK_EN		GPIO085_GPIO_85
#define GPIO086_GPIO_GPS_SEN_EN		GPIO086_GPIO_86
	GPIO079_GPIO_CAM_STROBE,
	GPIO080_GPIO_CAM_PD_MAIN,
	GPIO081_GPIO_CAM_RST_MAIN,
	GPIO082_GPIO_CAM_PD_SUB,
	GPIO083_GPIO_CAM_RST_SUB,
	GPIO084_GPIO_GPS_RESET_N,
	GPIO085_GPIO_GPS_CLK_EN,
	GPIO086_GPIO_GPS_SEN_EN,

	GPIO087_CI2C_SCL_2,	/* CI2C_SCL2 */
	GPIO088_CI2C_SDA_2,	/* CI2C_SDA2 */

	GPIO089_GPS_CLK,	/* GPS_ECLK */
	GPIO090_CMMB_CLK,	/* CMMB_26MHz */

#define GPIO091_GPIO_GYRO_INT		GPIO091_GPIO_91
#define GPIO092_GPIO_G_INT		GPIO092_GPIO_92
#define GPIO094_GPIO_NFC_IRQ		GPIO094_GPIO_94
#define GPIO095_GPIO_MOTION_INT		GPIO095_GPIO_95
#define GPIO096_GPIO_GPS_ON_OFF		GPIO096_GPIO_96
#define GPIO097_GPIO_GPS_PPS		GPIO097_GPIO_97
#define GPIO098_GPIO_PRESURE_DRDY	GPIO098_GPIO_98
#define GPIO124_GPIO_CODEC_INT		GPIO124_GPIO_124
	GPIO091_GPIO_GYRO_INT,
	GPIO092_GPIO_G_INT,
	GPIO094_GPIO_NFC_IRQ,
	GPIO095_GPIO_MOTION_INT,
	GPIO096_GPIO_GPS_ON_OFF,
	GPIO097_GPIO_GPS_PPS,
	GPIO098_GPIO_PRESURE_DRDY,
	GPIO124_GPIO_CODEC_INT,

	/* MMC1 Micro SD */
	MMC1_DAT3_MMC1_DAT3,
	MMC1_DAT2_MMC1_DAT2,
	MMC1_DAT1_MMC1_DAT1,
	MMC1_DAT0_MMC1_DAT0,
	MMC1_CMD_MMC1_CMD,
	MMC1_CLK_MMC1_CLK,
	MMC1_CD_MMC1_CD | MFP_PULL_HIGH,
	MMC1_WP_MMC1_WP | MFP_PULL_LOW,

	/* MMC3 16GB EMMC */
	ND_IO7_MMC3_DAT7,
	ND_IO6_MMC3_DAT6,
	ND_IO5_MMC3_DAT5,
	ND_IO4_MMC3_DAT4,
	ND_IO3_MMC3_DAT3,
	ND_IO2_MMC3_DAT2,
	ND_IO1_MMC3_DAT1,
	ND_IO0_MMC3_DAT0,
	ND_CLE_SM_OEN_MMC3_CMD,
	SM_SCLK_MMC3_CLK,

#define GPIO_GPS_TIMER_SYNC	ANT_SW4_GPIO_28
#define GPIO_RF_PDET_EN		SM_ADV_GPIO_0
#define GPIO_LCD_RESET_N	ND_RDY1_GPIO_1
#define GPIO_LED_B_CTRL		SM_ADVMUX_GPIO_2
#define GPIO_LED_R_CTRL		SM_BEN1_GPIO_127
#define GPIO_LED_G_CTRL		SM_CSN0_GPIO_103
#define GPIO_GPS_LDO_EN		SM_CSN1_GPIO_104
#define GPIO_VCM_PWDN		ND_CS1N3_GPIO_102
	GPIO_GPS_TIMER_SYNC,
	GPIO_RF_PDET_EN,
	GPIO_LCD_RESET_N,
	GPIO_LED_B_CTRL | MFP_PULL_FLOAT | MFP_LPM_FLOAT,
	GPIO_LED_R_CTRL | MFP_PULL_FLOAT | MFP_LPM_FLOAT,
	GPIO_LED_G_CTRL | MFP_PULL_FLOAT | MFP_LPM_FLOAT,
	GPIO_GPS_LDO_EN,
	GPIO_VCM_PWDN,
};

static unsigned long emeidkb_lcd_pin_config[] __initdata = {
	GPIO045_GPIO_45,

	GPIO081_LCD,
	GPIO082_LCD,
	GPIO083_LCD,
	GPIO084_LCD,
	GPIO085_LCD,
	GPIO086_LCD,
	GPIO087_LCD,
	GPIO088_LCD,
	GPIO089_LCD,
	GPIO090_LCD,
	GPIO091_LCD,
	GPIO092_LCD,
	GPIO093_LCD,
	GPIO094_LCD,
	GPIO095_LCD,
	GPIO096_LCD,
/* camera power control pin */
#define GPIO018_GPIO_CAM_RST_MAIN	GPIO018_GPIO_18
#define GPIO019_GPIO_CAM_PD_SUB		GPIO019_GPIO_19
#define GPIO020_GPIO_CAM_RST_SUB	GPIO020_GPIO_20
	GPIO018_GPIO_18,
	GPIO019_GPIO_19,
	GPIO020_GPIO_20,
};

static unsigned int emei_dkb_matrix_key_map[] = {
	KEY(0, 0, KEY_BACKSPACE),
	KEY(0, 1, KEY_MENU),
	KEY(0, 2, KEY_CAMERA), /* 1st camera */

	KEY(1, 0, KEY_OK),
	KEY(1, 1, KEY_HOMEPAGE),
	KEY(1, 2, KEY_CAMERA), /* 2nd camera */

	KEY(2, 0, KEY_LEFT),
	KEY(2, 1, KEY_RIGHT),
	KEY(2, 2, KEY_UNKNOWN), /* unused key */

	KEY(3, 0, KEY_UP),
	KEY(3, 1, KEY_DOWN),
	KEY(3, 2, KEY_UNKNOWN), /* unused key */
};

static struct pxa27x_keypad_platform_data emei_dkb_keypad_info __initdata = {
	.matrix_key_rows	= 4,
	.matrix_key_cols	= 3,
	.matrix_key_map		= emei_dkb_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(emei_dkb_matrix_key_map),
	.debounce_interval	= 30,
};

static int emei_i2c_read_reg(u8 addr, u8 reg, u8 *buf, int len)
{
	return __raw_i2c_read_reg(3, addr, reg, buf, len);
}
/*
 * for HVGA panel:
 */
static int is_HVGA_lcd;
#if defined(CONFIG_TOUCHSCREEN_ELAN)
static int touch_io_power_onoff(int on);
#endif
static void ft5306_touch_reset(void);
static void lcd_HVGA_setup(void)
{
	u8 buf[4] = {0,};

	ft5306_touch_reset();
#if defined(CONFIG_TOUCHSCREEN_ELAN)
	touch_io_power_onoff(1);
#endif
	/* Reset TWSI3 pwr_i2c unit firstly */
	__raw_i2c_bus_reset(3);
	emei_i2c_read_reg(0x8, 0x00, buf, 4);

	if (buf[0] == 0x55)
		is_HVGA_lcd = 1;
	printk(KERN_INFO "emei dkb touch : 0x%x, HVGA_lcd: %d\n",
			buf[0], is_HVGA_lcd);
}

static void emei_dkb_set_bl(int intensity)
{
	int gpio_bl, bl_level, p_num;
	static int bl_level_last;

	gpio_bl = mfp_to_gpio(GPIO032_GPIO_LCD_PWM);
	if (gpio_request(gpio_bl, "lcd backlight")) {
		pr_err("gpio %d request failed\n", gpio_bl);
		return;
	}

	/*
	 * Brightness is controlled by a series of pulses
	 * generated by gpio. It has 32 leves and level 1
	 * is the brightest. Pull low for 3ms makes
	 * backlight shutdown
	 */
	bl_level = (100 - intensity) * 32 / 100 + 1;

	if (bl_level == bl_level_last)
		goto set_bl_return;

	if (bl_level == 33) {
		/* shutdown backlight */
		gpio_direction_output(gpio_bl, 0);
		goto set_bl_return;
	}

	if (bl_level > bl_level_last)
		p_num = bl_level - bl_level_last;
	else
		p_num = bl_level + 32 - bl_level_last;

	while (p_num--) {
		gpio_direction_output(gpio_bl, 0);
		udelay(1);
		gpio_direction_output(gpio_bl, 1);
		udelay(1);
	}

set_bl_return:
	if (bl_level == 33)
		bl_level_last = 0;
	else
		bl_level_last = bl_level;
	gpio_free(gpio_bl);
	pr_debug("%s, intensity:%d\n", __func__, intensity);
}

static struct generic_bl_info emei_dkb_lcd_backlight_data = {
	.name = "emei-bl",
	.max_intensity = 100,
	.default_intensity = 50,
	.set_bl_intensity = emei_dkb_set_bl,
};

static struct platform_device emei_dkb_lcd_backlight_devices = {
	.name = "generic-bl",
	.dev = {
		.platform_data = &emei_dkb_lcd_backlight_data,
	},
};

static struct regulator_consumer_supply regulator_supply[PM8XXX_REGULATOR_MAX];
static struct regulator_init_data regulator_data[PM8XXX_REGULATOR_MAX];

static int PM800_ID_regulator_index[] = {
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

#define REG_SUPPLY_INIT(_id, _name, _dev_name)		\
{							\
	int _i = _id;				\
	regulator_supply[_i].supply		= _name;		\
	regulator_supply[_i].dev_name	= _dev_name;	\
}

/* notes: apply_uV which means proper voltage value (latest set value or min)
* would be applied first time when enabled. So it would be set 1 if min voltage
* == max voltage*/
#define REG_INIT(_id, _chip, _name, _min, _max, _always, _boot)	\
{									\
	int _i = _id;				\
	regulator_data[_i].constraints.name	= __stringify(_name);	\
	regulator_data[_i].constraints.min_uV	= _min;	\
	regulator_data[_i].constraints.max_uV	= _max;	\
	regulator_data[_i].constraints.always_on	= _always;	\
	regulator_data[_i].constraints.boot_on	= _boot;	\
	regulator_data[_i].constraints.valid_ops_mask	=	\
		REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS;	\
	regulator_data[_i].num_consumer_supplies	= 1;	\
	regulator_data[_i].consumer_supplies	=	\
		&regulator_supply[_chip##_##_name];	\
	regulator_data[_i].driver_data	=	\
		&_chip##_regulator_index[_chip##_##_name];	\
	regulator_data[_i].constraints.apply_uV = (_min == _max);	\
}

static void mic_set_power(int on)
{
	struct regulator *v_ldo = regulator_get(NULL, "v_micbias");
	if (IS_ERR(v_ldo)) {
		v_ldo = NULL;
		pr_err("Get regulator error\n");
		return;
	}
	if (on)
		regulator_enable(v_ldo);
	else
		regulator_disable(v_ldo);

	regulator_put(v_ldo);
	v_ldo = NULL;
}

static struct pm80x_headset_pdata pm80x_headset = {
	.mic_set_power = mic_set_power,
};

#ifdef CONFIG_RTC_DRV_MMP
static int sync_time_to_soc(unsigned int ticks)
{
	RCNR = ticks;
	return 0;
}
#endif

static struct pm80x_rtc_pdata pm80x_rtc = {
	/* FIXME: NOT verified */
	.vrtc           = 1,
#ifdef CONFIG_RTC_DRV_MMP
	.sync		= sync_time_to_soc,
#endif

};

static struct pm80x_dvc_pdata pm80x_dvc = {
	/*FIXME: need to be added */
};

static int pm800_plat_config(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata)
{
	if (!chip || !pdata || chip->id != CHIP_PM800 || !chip->base_page) {
		pr_err("%s:chip or pdata is not availiable!\n", __func__);
		return -EINVAL;
	}
	/* RESET_OUTn, RTC_RESET_MODE =0 */
	pm80x_reg_write(chip->base_page, PM800_RTC_MISC1, 0xb0);
	/* Enable 32Khz-out-3 low jitter XO_LJ = 1 */
	pm80x_reg_write(chip->base_page, PM800_LOW_POWER2, 0x20);
	/*
	 * Enable 32Khz-out-from XO 1, 2, 3
	 * all enabled
	 */
	pm80x_reg_write(chip->base_page, PM800_RTC_MISC2, 0x2a);
	/* Set XO CAP to 22pF to avoid speaker noise, XO_CAP_SEL = 7 */
	pm80x_reg_write(chip->base_page, PM800_USER_DATA1, 0x70);
	/* Enable voltage change in pmic, POWER_HOLD = 1 */
	pm80x_reg_write(chip->base_page, PM800_WAKEUP1, 0x80);
	/* BUCK enable 0x50, BUCK1, 2, 3, 4 */
	pm80x_reg_write(chip->power_page, PM800_BUCK_ENA, 0x0f);
	/* LDO enable 0x51, 0x52, 0x53, LDO1, 3, 5, 7 */
	pm80x_reg_write(chip->power_page, PM800_LDO_ENA1_1, 0x54);
	pm80x_reg_write(chip->power_page, PM800_LDO_ENA1_2, 0x20); /* LDO 14 */
	pm80x_reg_write(chip->power_page, PM800_LDO_ENA1_3, 0x02); /* LDO 18 */

	return 0;
}

static struct pm80x_platform_data pm800_info = {
	.headset                = &pm80x_headset,
	.headset_flag		= 1,
	.regulator		= regulator_data,
	.rtc			= &pm80x_rtc,
	.dvc			= &pm80x_dvc,
	.companion_addr		= 0x38,		/* AUDIO */
	.base_page_addr		= 0x30,
	.power_page_addr	= 0x31,
	.gpadc_page_addr	= 0x32,
	/* .test_page_addr	= 0x37, */
	.irq_mode		= 0,
	.irq_base		= IRQ_BOARD_START,
	/*PM805 has it's own interrupt line (GPIO124)*/
	.irq_companion		= gpio_to_irq(124),
	/* FIXME: need to be update */
	.i2c_port		= PI2C_PORT,
	.pm800_plat_config	= pm800_plat_config,
};

static void regulator_init_pm800(void)
{
	int i = 0;
	REG_SUPPLY_INIT(PM800_ID_BUCK1, "vcc_main", NULL);
	REG_INIT(i++, PM800_ID, BUCK1, 600000, 3950000, 1, 1);

	REG_SUPPLY_INIT(PM800_ID_BUCK2, "v_buck2", NULL);
	REG_INIT(i++, PM800_ID, BUCK2, 600000, 3950000, 1, 1);

	REG_SUPPLY_INIT(PM800_ID_BUCK3, "v_buck3", NULL);
	REG_INIT(i++, PM800_ID, BUCK3, 600000, 3950000, 1, 1);

	REG_SUPPLY_INIT(PM800_ID_BUCK4, "v_rf_vdd", NULL);
	REG_INIT(i++, PM800_ID, BUCK4, 600000, 3950000, 1, 1);

	REG_SUPPLY_INIT(PM800_ID_BUCK5, "v_rf_pa", NULL);
	REG_INIT(i++, PM800_ID, BUCK5, 600000, 3950000, 1, 0);

	REG_SUPPLY_INIT(PM800_ID_LDO1, "v_gps_1v1", NULL);
	REG_INIT(i++, PM800_ID, LDO1, 600000, 1500000, 0, 0);

	REG_SUPPLY_INIT(PM800_ID_LDO2, "v_micbias", NULL);
	REG_INIT(i++, PM800_ID, LDO2, 1700000, 2800000, 0, 1);

	REG_SUPPLY_INIT(PM800_ID_LDO3, "v_ramp", NULL);
	REG_INIT(i++, PM800_ID, LDO3, 1200000, 3300000, 0, 1);

	REG_SUPPLY_INIT(PM800_ID_LDO4, "v_usim1", NULL);
	REG_INIT(i++, PM800_ID, LDO4, 1200000, 3300000, 0, 0);

	REG_SUPPLY_INIT(PM800_ID_LDO5, "v_ldo5", NULL);
	REG_INIT(i++, PM800_ID, LDO5, 1200000, 3300000, 0, 1);

	REG_SUPPLY_INIT(PM800_ID_LDO6, "v_wib_3v3", NULL);
	REG_INIT(i++, PM800_ID, LDO6, 1200000, 3300000, 0, 0);

	REG_SUPPLY_INIT(PM800_ID_LDO7, "v_vctcxo", NULL);
	REG_INIT(i++, PM800_ID, LDO7, 1200000, 3300000, 0, 1);

	REG_SUPPLY_INIT(PM800_ID_LDO8, "v_ldo8", NULL);
	REG_INIT(i++, PM800_ID, LDO8, 1200000, 3300000, 0, 1);

	REG_SUPPLY_INIT(PM800_ID_LDO9, "v_wib_1v8", NULL);
	REG_INIT(i++, PM800_ID, LDO9, 1200000, 3300000, 0, 0);

	REG_SUPPLY_INIT(PM800_ID_LDO10, "v_usim2", NULL);
	REG_INIT(i++, PM800_ID, LDO10, 1200000, 3300000, 0, 0);

	REG_SUPPLY_INIT(PM800_ID_LDO11, "v_sensor", NULL);
	REG_INIT(i++, PM800_ID, LDO11, 1200000, 3300000, 0, 0);

	REG_SUPPLY_INIT(PM800_ID_LDO12, "vmmc_io", "sdhci-pxa.0");
	REG_INIT(i++, PM800_ID, LDO12, 1200000, 3300000, 0, 0);

	REG_SUPPLY_INIT(PM800_ID_LDO13, "vmmc", "sdhci-pxa.0");
	REG_INIT(i++, PM800_ID, LDO13, 1200000, 3300000, 0, 0);

	REG_SUPPLY_INIT(PM800_ID_LDO14, "vmmc", "sdhci-pxa.2");
	REG_INIT(i++, PM800_ID, LDO14, 1200000, 3300000, 0, 1);

	REG_SUPPLY_INIT(PM800_ID_LDO15, "v_ldo15", NULL);
	REG_INIT(i++, PM800_ID, LDO15, 1200000, 3300000, 0, 0);

	REG_SUPPLY_INIT(PM800_ID_LDO16, "v_cam_avdd", NULL);
	REG_INIT(i++, PM800_ID, LDO16, 1200000, 3300000, 0, 0);

	REG_SUPPLY_INIT(PM800_ID_LDO17, "v_cam_af", NULL);
	REG_INIT(i++, PM800_ID, LDO17, 1200000, 3300000, 0, 0);

	REG_SUPPLY_INIT(PM800_ID_LDO18, "v_ldo18", NULL);
	REG_INIT(i++, PM800_ID, LDO18, 1700000, 3300000, 1, 1);

	REG_SUPPLY_INIT(PM800_ID_LDO19, "v_gps_3v", NULL);
	REG_INIT(i++, PM800_ID, LDO19, 1700000, 3300000, 0, 0);

	pr_info("%s: select emeidkb ldo map\n", __func__);
	pm800_info.num_regulators = i;
}

#if defined(CONFIG_SENSORS_CWMI) || defined(CONFIG_SENSORS_CWGD) \
	|| defined(CONFIG_SENSORS_ISL29043)
static int sensor_set_power(int on)
{
	static struct regulator *v_sensor;

	if (!v_sensor) {
		v_sensor = regulator_get(NULL, "v_sensor");
		if (IS_ERR(v_sensor)) {
			v_sensor = NULL;
			return -EIO;
		}
	}

	if (on) {
		regulator_set_voltage(v_sensor, 2800000, 2800000);
		regulator_enable(v_sensor);
	} else {
		regulator_disable(v_sensor);
	}
	msleep(100);
	return 0;
}
#endif

#if defined(CONFIG_SENSORS_CWMI)
static struct cwmi_platform_data cwmi_acc_data = {
	.set_power = sensor_set_power,
	.axes = {
		1, 0, 0,
		0, 1, 0,
		0, 0, 1},
};

static struct cwmi_platform_data cwmi_mag_data = {
	.set_power = sensor_set_power,
	.axes = {
		1, 0, 0,
		0, 1, 0,
		0, 0, 1},
};
#endif

#if defined(CONFIG_SENSORS_CWGD)
static struct cwgd_platform_data cwgd_plat_data = {
	.set_power = sensor_set_power,
	.axes = {
		0, 1, 0,
		-1, 0, 0,
		0, 0, 1},
};
#endif

#if defined(CONFIG_SENSORS_ISL29043)
static struct axis_sensor_platform_data isl29043_plat_data = {
	.set_power = sensor_set_power,
};
#endif

static struct i2c_board_info dkb_i2c_camera[] = {
#if defined(CONFIG_SOC_CAMERA_OV2659)
	{
		I2C_BOARD_INFO("ov2659", 0x30),
	},
#endif
};

#if defined(CONFIG_SOC_CAMERA_OV2659)
static int camera_sensor_power(struct device *dev, int on)
{
	unsigned int cam_pwr;
	unsigned int cam_reset;
	static struct regulator *v_sensor;

	if (!v_sensor) {
		v_sensor = regulator_get(NULL, "v_cam_avdd");
		if (IS_ERR(v_sensor)) {
			v_sensor = NULL;
			pr_err(KERN_ERR "Enable v_ldo16 failed!\n");
			return -EIO;
		}
	}

	if (is_HVGA_lcd) {
		cam_pwr = mfp_to_gpio(GPIO019_GPIO_CAM_PD_SUB);
		cam_reset = mfp_to_gpio(GPIO020_GPIO_CAM_RST_SUB);
	} else {
		cam_pwr = mfp_to_gpio(GPIO082_GPIO_CAM_PD_SUB);
		cam_reset = mfp_to_gpio(GPIO083_GPIO_CAM_RST_SUB);
	}

	if (cam_pwr) {
		if (gpio_request(cam_pwr, "CAM_PWR")) {
			printk(KERN_ERR "Request GPIO failed,"
					"gpio: %d\n", cam_pwr);
			return -EIO;
		}
	}
	if (gpio_request(cam_reset, "CAM_RESET")) {
		printk(KERN_ERR "Request GPIO failed,"
			"gpio: %d\n", cam_reset);
		return -EIO;
	}

	if (on) {
		regulator_set_voltage(v_sensor, 2800000, 2800000);
		regulator_enable(v_sensor);
		msleep(20);
		gpio_direction_output(cam_pwr, 0);
		mdelay(1);
		gpio_direction_output(cam_reset, 0);
		mdelay(1);
		gpio_direction_output(cam_reset, 1);
		mdelay(1);
	} else {
		gpio_direction_output(cam_reset, 0);
		mdelay(1);
		gpio_direction_output(cam_reset, 1);
		gpio_direction_output(cam_pwr, 1);
		regulator_disable(v_sensor);
	}
	gpio_free(cam_pwr);
	gpio_free(cam_reset);
	return 0;
}

static struct soc_camera_link iclink_ov2659_dvp = {
	.bus_id         = 0,            /* Must match with the camera ID */
	.power          = camera_sensor_power,
	.board_info     = &dkb_i2c_camera[0],
	.i2c_adapter_id = 0,
	.module_name    = "ov2659",
};

static struct platform_device dkb_ov2659_dvp = {
	.name   = "soc-camera-pdrv",
	.id     = 0,
	.dev    = {
		.platform_data = &iclink_ov2659_dvp,
	},
};
#endif

static int pxa988_cam_clk_init(struct device *dev, int init)
{
	struct mv_cam_pdata *data = dev->platform_data;

	if ((!data->clk_enabled) && init) {
		data->clk[0] = clk_get(dev, "CCICFUNCLK");
		if (IS_ERR(data->clk[0])) {
			dev_err(dev, "Could not get function clk\n");
			goto out_clk0;
		}
		data->clk[1] = clk_get(dev, "CCICAXICLK");
		if (IS_ERR(data->clk[1])) {
			dev_err(dev, "Could not get AXI clk\n");
			goto out_clk1;
		}
		data->clk[2] = clk_get(dev, "LCDCIHCLK");
		if (IS_ERR(data->clk[2])) {
			dev_err(dev, "Could not get lcd/ccic AHB clk\n");
			goto out_clk2;
		}
		if (data->bus_type == SOCAM_MIPI) {
			data->clk[3] = clk_get(dev, "CCICPHYCLK");
			if (IS_ERR(data->clk[3])) {
				dev_err(dev, "Could not get PHY clk\n");
				goto out_clk3;
			}
		}
		data->clk_enabled = 1;
		return 0;
	}

	if (!init && data->clk_enabled) {
		clk_put(data->clk[0]);
		clk_put(data->clk[1]);
		clk_put(data->clk[2]);
		if (data->bus_type == SOCAM_MIPI)
			clk_put(data->clk[3]);
		data->clk_enabled = 0;
		return 0;
	}
	return -EFAULT;

out_clk0:
		return PTR_ERR(data->clk[0]);
out_clk1:
		clk_put(data->clk[0]);
		return PTR_ERR(data->clk[1]);
out_clk2:
		clk_put(data->clk[0]);
		clk_put(data->clk[1]);
		return PTR_ERR(data->clk[2]);
out_clk3:
		clk_put(data->clk[0]);
		clk_put(data->clk[1]);
		clk_put(data->clk[2]);
		return PTR_ERR(data->clk[3]);
}

static void pxa988_cam_set_clk(struct device *dev, int on)
{
	struct mv_cam_pdata *data = dev->platform_data;

	if (data->clk_enabled) {
		if (on == 1) {
			clk_enable(data->clk[1]);
			clk_enable(data->clk[0]);
			if (data->bus_type == SOCAM_MIPI)
				clk_enable(data->clk[3]);
			clk_enable(data->clk[2]);
		} else {
			clk_disable(data->clk[0]);
			if (data->bus_type == SOCAM_MIPI)
				clk_disable(data->clk[3]);
			clk_disable(data->clk[1]);
			clk_disable(data->clk[2]);
		}
	}
}

struct mv_cam_pdata mv_cam_data;
/* TODO reserve src parameter temporary */
static int pxa988_cam_get_mclk_src(struct device *dev)
{
	int rate = 0;
	struct mv_cam_pdata *data = dev->platform_data;

	if (data->clk_enabled)
		rate = clk_get_rate(data->clk[0]) / 1000000;

	return rate;
}

struct mv_cam_pdata mv_cam_data = {
	.name = "EMEI",
	.clk_enabled = 0,
	.qos_req_min = 624,
	.dma_burst = 64,
	.mipi_enabled = 0,
	.mclk_min = 24,
	.mclk_src = 3,
	.init_clk = pxa988_cam_clk_init,
	.enable_clk = pxa988_cam_set_clk,
	.get_mclk_src = pxa988_cam_get_mclk_src,
};

static struct platform_device *dkb_platform_devices[] = {
#if defined(CONFIG_SOC_CAMERA_OV2659)
	&dkb_ov2659_dvp,
#endif
	&pxa988_device_rtc,
};


#ifdef CONFIG_VIDEO_MVISP_OV8825
static int ov8825_sensor_power_on(int on, int flag)
{
	static struct regulator *af_vcc;
	static struct regulator *avdd;
	int rst;
	int pwdn = mfp_to_gpio(GPIO080_GPIO_CAM_PD_MAIN);
	int ret = 0;
	if (is_HVGA_lcd)
		rst = mfp_to_gpio(GPIO018_GPIO_CAM_RST_MAIN);
	else
		rst = mfp_to_gpio(GPIO081_GPIO_CAM_RST_MAIN);

	if (gpio_request(pwdn, "CAM_ENABLE_LOW")) {
		ret = -EIO;
		goto out;
	}

	if (gpio_request(rst, "CAM_RESET_LOW")) {
		ret = -EIO;
		goto out_rst;
	}

	if (!af_vcc) {
		af_vcc = regulator_get(NULL, "v_cam_af");
		if (IS_ERR(af_vcc)) {
			ret = -EIO;
			goto out_af_vcc;
		}
	}

	if (!avdd) {
		avdd = regulator_get(NULL, "v_cam_avdd");
		if (IS_ERR(avdd)) {
			ret =  -EIO;
			goto out_avdd;
		}
	}

	/* Enable voltage for camera sensor OV8825 */
	if (on) {
		regulator_set_voltage(af_vcc, 2800000, 2800000);
		regulator_enable(af_vcc);
		regulator_set_voltage(avdd, 2800000, 2800000);
		regulator_enable(avdd);
		mdelay(5);
		/* enable the sensor now*/
		gpio_direction_output(pwdn, 1);
		mdelay(1);
		gpio_direction_output(rst, 1);
		mdelay(20);
	} else {
		gpio_direction_output(rst, 0);

		regulator_disable(avdd);
		regulator_disable(af_vcc);

		gpio_direction_output(pwdn, 0);
	}

	gpio_free(rst);
	gpio_free(pwdn);
	return 0;

out_avdd:
	avdd = NULL;
	regulator_put(af_vcc);
out_af_vcc:
	af_vcc = NULL;
	gpio_free(rst);
out_rst:
	gpio_free(pwdn);
out:
	return ret;
}

static struct sensor_platform_data ov8825_platdata = {
	.id = 0,
	.power_on = ov8825_sensor_power_on,
	.platform_set = NULL,
};

static struct i2c_board_info ov8825_info = {
	.type = "ov8825",
	.addr = 0x36,
	.platform_data = &ov8825_platdata,
};

static struct mvisp_subdev_i2c_board_info ov8825_isp_info[] = {
	[0] = {
		.board_info = &ov8825_info,
		.i2c_adapter_id = 0,
	},
	[1] = {
		.board_info = NULL,
		.i2c_adapter_id = 0,
	},
};

static struct mvisp_v4l2_subdevs_group dxoisp_subdevs_group[] = {
	[0] = {
		.i2c_board_info = ov8825_isp_info,
		.if_type = ISP_INTERFACE_CCIC_1,
	},
	[1] = {
		.i2c_board_info = NULL,
		.if_type = 0,
	},
};
#endif


#ifdef CONFIG_VIDEO_MVISP
#ifndef CONFIG_VIDEO_MVISP_OV8825
static struct mvisp_v4l2_subdevs_group dxoisp_subdevs_group[] = {
	[0] = {
		.i2c_board_info = NULL,
		.if_type = 0,
	},
};
#endif

static char *pxa988_isp_ccic_clk_name[] = {
	[0] = "ISP-CLK",
	[1] = "CCICPHYCLK",
	[2] = "CCICFUNCLK",
};

static struct mvisp_platform_data pxa988_dxoisp_pdata = {
	.isp_clknum       = 1,
	.ccic_clknum      = 2,
	.clkname          = pxa988_isp_ccic_clk_name,
	.mvisp_reset      = pxa988_isp_reset_hw,
	.isp_pwr_ctrl     = pxa988_isp_power_control,
	.subdev_group     = dxoisp_subdevs_group,
	.ccic_dummy_ena   = false,
	.ispdma_dummy_ena = false,
};

static void __init pxa988_init_dxoisp(void)
{
	pxa988_register_dxoisp(&pxa988_dxoisp_pdata);
}
#endif


static struct i2c_board_info emeidkb_i2c_info[] = {

};

#if defined(CONFIG_TOUCHSCREEN_ELAN)
static int touch_io_power_onoff(int on)
{
	unsigned int tp_logic_en;
	tp_logic_en = mfp_to_gpio(MFP_PIN_GPIO45);

	if (gpio_request(tp_logic_en, "TP_LOGIC_EN")) {
		printk(KERN_ERR "Request GPIO tp_logic_en failed,"
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

#if defined(CONFIG_TOUCHSCREEN_FT5306)
static int ft5306_touch_io_power_onoff(int on)
{
	static struct regulator *v_ldo8;

	if (!v_ldo8) {
		v_ldo8 = regulator_get(NULL, "v_ldo8");
		if (IS_ERR(v_ldo8)) {
			v_ldo8 = NULL;
			pr_err("%s: enable v_ldo8 for touch fail!\n", __func__);
			return -EIO;
		}
	}

	if (on) {
		regulator_set_voltage(v_ldo8, 3100000, 3100000);
		regulator_enable(v_ldo8);
	} else
		regulator_disable(v_ldo8);

	msleep(100);
	return 0;
}

static void ft5306_touch_reset(void)
{
	unsigned int touch_reset;

	touch_reset = mfp_to_gpio(GPIO016_GPIO_TP_RESET);

	if (gpio_request(touch_reset, "ft5306_reset")) {
		pr_err("Failed to request GPIO for ft5306_reset pin!\n");
		goto out;
	}

	gpio_direction_output(touch_reset, 1);
	msleep(5);
	gpio_direction_output(touch_reset, 0);
	msleep(5);
	gpio_direction_output(touch_reset, 1);
	msleep(300);
	printk(KERN_INFO "ft5306_touch reset done.\n");
	gpio_free(touch_reset);
out:
	return;
}

static u32 ft5306_virtual_key_code[4] = {
	KEY_MENU,
	KEY_HOMEPAGE,
	KEY_SEARCH,
	KEY_BACK,
};

static u32 ft5306_get_virtual_key(u16 x_pos, u16 y_pos, u16 x_max, u16 y_max)
{
	int unit = (x_max / 13);

	if ((unit < x_pos) && (x_pos < 3 * unit))
		return ft5306_virtual_key_code[0];
	else if ((4 * unit < x_pos) && (x_pos < 6 * unit))
		return ft5306_virtual_key_code[1];
	else if ((7 * unit < x_pos) && (x_pos < 9 * unit))
		return ft5306_virtual_key_code[2];
	else if ((10 * unit < x_pos) && (x_pos < 13 * unit))
		return ft5306_virtual_key_code[3];
	else
		return KEY_RESERVED;
}

static int ft5306_set_virtual_key(struct input_dev *idev)
{
	__set_bit(EV_KEY, idev->evbit);
	__set_bit(KEY_MENU, idev->keybit);
	__set_bit(KEY_HOMEPAGE, idev->keybit);
	__set_bit(KEY_SEARCH, idev->keybit);
	__set_bit(KEY_BACK, idev->keybit);

	return 0;
}

static struct ft5306_touch_platform_data ft5306_touch_data = {
	.power = ft5306_touch_io_power_onoff,
	.reset = ft5306_touch_reset,
	.keypad = ft5306_get_virtual_key, /* get virtual key code */
	.abs_x_max = 540,
	.abs_y_max = 960,
	.abs_flag = 0,
	.virtual_key = 1,	/* enable virtual key for android */
	.set_virtual_key = ft5306_set_virtual_key,
};
#endif

static struct i2c_board_info emeidkb_pwr_i2c_info[] = {
	{
		.type		= "88PM80x",
		.addr		= 0x34,
		.platform_data	= &pm800_info,
		.irq		= IRQ_PXA988_PMIC,
	},

#if defined(CONFIG_CHARGER_ISL9226)
	{
		.type		= "isl9226",
		.addr		= 0x59,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_ELAN)
	{
		.type		= "elan_touch",
		.addr		= 0x8,
		.irq  = gpio_to_irq(17),
		.platform_data	= &elan_touch_data,
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_FT5306)
	{
		.type = "ft5306_touch",
		.addr = 0x39,
		.irq  = gpio_to_irq(mfp_to_gpio(GPIO017_GPIO_TP_INT)),
		.platform_data	= &ft5306_touch_data,
	},
#endif

};

static struct i2c_board_info emeidkb_i2c1_info[] = {
#if defined(CONFIG_SENSORS_CWMI)
	{
		.type       = "cwmi_acc",
		.addr       = 0x19,
		.irq = gpio_to_irq(mfp_to_gpio(GPIO092_GPIO_G_INT)),
		.platform_data  = &cwmi_acc_data,
	},
	{
		.type       = "cwmi_mag",
		.addr       = 0x1e,
		.platform_data  = &cwmi_mag_data,
	},
#endif
#if defined(CONFIG_SENSORS_CWGD)
	{
		.type       = "cwgd",
		.addr       = 0x69,
		.irq = gpio_to_irq(mfp_to_gpio(GPIO091_GPIO_GYRO_INT)),
		.platform_data  = &cwgd_plat_data,
	},
#endif
#if defined(CONFIG_SENSORS_ISL29043)
	{
		.type		= "isl29043",
		.addr		= 0x44,
		.irq = gpio_to_irq(mfp_to_gpio(GPIO014_GPIO_PROX_IRQ)),
		.platform_data	= &isl29043_plat_data,
	},
#endif
};

static struct i2c_pxa_platform_data emeidkb_pwr_i2c_pdata = {
	.hardware_lock		= pxa988_ripc_lock,
	.hardware_unlock	= pxa988_ripc_unlock,
	.hardware_trylock	= pxa988_ripc_trylock,
};

static struct sram_bank pxa988_asram_info = {
	.pool_name = "audio sram",
	.step = AUDIO_SRAM_GRANULARITY,
};

#ifdef CONFIG_USB_PXA_U2O
static char *pxa988_usb_clock_name[] = {
	[0] = "UDCCLK",
};

static struct mv_usb_addon_irq emeidkb_usb_vbus = {
	.irq	= IRQ_BOARD_START + PM800_IRQ_CHG,
	.poll	= pm80x_read_vbus_val,
};

static struct mv_usb_platform_data emeidkb_usb_pdata = {
	.clknum		= 1,
	.clkname	= pxa988_usb_clock_name,
	.vbus		= &emeidkb_usb_vbus,
	.mode		= MV_USB_MODE_DEVICE,
	.phy_init	= pxa_usb_phy_init,
	.phy_deinit	= pxa_usb_phy_deinit,
};
#endif /* CONFIG_USB_PXA_U2O */

#ifdef CONFIG_MMC_SDHCI_PXAV3
#define MFP_WIB_PDn		GPIO007_GPIO_7
#define MFP_WIB_RESETn		GPIO011_GPIO_11
#define POWER_OFF_SD_SIGNAL_IN_SUSPEND 0

static void emeidkb_set_sdcard_signal_level(int vol)
{
	static struct regulator *vcc_signal;
	static int signal_vol;
	static int enabled_signal;
	struct device *dev_host;

	if (!vcc_signal) {
		dev_host = bus_find_device_by_name(&platform_bus_type,
					NULL, "sdhci-pxa.0");
		if (dev_host == NULL) {
			printk(KERN_ERR "sdhci-pxa host failed %s %d\n",
				__func__, __LINE__);
			return;
		}

		/* LDO12 = 2.8V/300mA Off by default */
		vcc_signal = regulator_get(dev_host, "vmmc_io");
		if (IS_ERR(vcc_signal)) {
			vcc_signal = NULL;
			printk(KERN_ERR "get vmmc_io failed %s %d\n",
				__func__, __LINE__);
			return;
		}
	}

	if (vol) {
		if (signal_vol != vol)
			regulator_set_voltage(vcc_signal, vol, vol);

		if (!enabled_signal) {
			regulator_enable(vcc_signal);
			enabled_signal = 1;
		}
	} else {
		if (enabled_signal) {
			regulator_disable(vcc_signal);
			enabled_signal = 0;
		}
	}

	signal_vol = vol;
}

static void emeidkb_sdcard_signal_1v8(int set)
{
	int vol;

	vol = set ? 1800000 : 3000000;
	emeidkb_set_sdcard_signal_level(vol);

	pxa988_aib_mmc1_iodomain(vol);
}

static int mmc1_lp_switch(unsigned int low_power, int with_card)
{
	int power_on;

	/*
	 * Clock pin is float by default, it will cause current leak
	 * during suspend for some SD cards if SD power is always on.
	 * So enable internal PULL-UP of clock pin during suspend to
	 * fix the issue.
	 */
	mfp_cfg_t mfp_cfg_mmc0_clk = MMC1_CLK_MMC1_CLK;
	mfp_cfg_t mfp_cfg_mmc0_clk_sleep = MMC1_CLK_MMC1_CLK | MFP_PULL_HIGH;

	if (low_power)
		mfp_config(&mfp_cfg_mmc0_clk_sleep, 1);
	else
		mfp_config(&mfp_cfg_mmc0_clk, 1);

	power_on = !low_power;

#if POWER_OFF_SD_SIGNAL_IN_SUSPEND
	emeidkb_set_sdcard_signal_level(power_on ? 2800000 : 0);
#endif
	return 0;
}
#undef POWER_OFF_SD_SIGNAL_IN_SUSPEND

#ifdef CONFIG_SD8XXX_RFKILL
static void emeidkb_8787_set_power(unsigned int on)
{
	static struct regulator *wib_1v8;
	static struct regulator *wib_3v3;
	static int enabled;

	/* LDO9 = 1.8V/300mA Off by default; LDO6 = 2.8V/300mA Off by default */
	if (!wib_1v8) {
		wib_1v8 = regulator_get(NULL, "v_wib_1v8");
		if (IS_ERR(wib_1v8)) {
			wib_1v8 = NULL;
			printk(KERN_ERR "get v_wib_1v8 failed %s %d\n",
				__func__, __LINE__);
			return;
		}
	}

	if (!wib_3v3) {
		wib_3v3 = regulator_get(NULL, "v_wib_3v3");
		if (IS_ERR(wib_3v3)) {
			wib_3v3 = NULL;
			printk(KERN_ERR "get v_wib_3v3 failed %s %d\n",
				__func__, __LINE__);
			return;
		}
	}

	if (on && !enabled) {
		regulator_set_voltage(wib_1v8, 1800000, 1800000);
		regulator_enable(wib_1v8);
		regulator_set_voltage(wib_3v3, 2800000, 2800000);
		regulator_enable(wib_3v3);
		enabled = 1;
	}

	if (!on && enabled) {
		regulator_disable(wib_1v8);
		regulator_disable(wib_3v3);
		enabled = 0;
	}
}
#endif

/* For emeiDKB, MMC1(SDH1) used for SD/MMC Card slot */
static struct sdhci_pxa_platdata pxa988_sdh_platdata_mmc1 = {
	.clk_delay_cycles	= 0x1F,
	.quirks			= SDHCI_QUIRK_INVERTED_WRITE_PROTECT,
	.signal_1v8		= emeidkb_sdcard_signal_1v8,
	.lp_switch		= mmc1_lp_switch,
};

/* For emeiDKB, MMC2(SDH2) used for WIB card */
static struct sdhci_pxa_platdata pxa988_sdh_platdata_mmc2 = {
	.flags          = PXA_FLAG_CARD_PERMANENT | PXA_FLAG_WAKEUP_HOST,
	.pm_caps	= MMC_PM_KEEP_POWER | MMC_PM_IRQ_ALWAYS_ON,
};

/* For emeiDKB, MMC3(SDH3) used for eMMC */
static struct sdhci_pxa_platdata pxa988_sdh_platdata_mmc3 = {
	.flags		= PXA_FLAG_SD_8_BIT_CAPABLE_SLOT,
	.clk_delay_cycles	= 0xF,
};

static void __init emeidkb_init_mmc(void)
{
#ifdef CONFIG_SD8XXX_RFKILL
	int WIB_PDn = mfp_to_gpio(MFP_WIB_PDn);
	int WIB_RESETn = mfp_to_gpio(MFP_WIB_RESETn);
	add_sd8x_rfkill_device(WIB_PDn, WIB_RESETn,
			&pxa988_sdh_platdata_mmc2.pmmc,
			emeidkb_8787_set_power);
#endif

	/*
	 * Note!!
	 *  The regulator can't be used here, as this is called in arch_init
	 */

	/* HW MMC3(sdh3) used for eMMC, and register first */
	pxa988_add_sdh(3, &pxa988_sdh_platdata_mmc3);

	/* HW MMC1(sdh1) used for SD/MMC card */
	pxa988_add_sdh(1, &pxa988_sdh_platdata_mmc1);

	/* HW MMC2(sdh2) used for SDIO(WIFI/BT/FM module), and register last */
	pxa988_add_sdh(2, &pxa988_sdh_platdata_mmc2);
}
#else
static void __init emeidkb_init_mmc(void)
{

}
#endif /* CONFIG_MMC_SDHCI_PXAV3 */

#if (defined CONFIG_CMMB)

#define CMMB_SPI_CLK		GPIO033_SPI_DCLK
#define CMMB_SPI_CSn		GPIO034_SPI_CS0
#define CMMB_SPI_DIN		GPIO035_SPI_DIN
#define CMMB_SPI_DOUT		GPIO036_SPI_DOUT
#define CMMB_POWER_EN		GPIO018_GPIO_18
#define CMMB_POWER_RESETn	GPIO019_GPIO_19
#define CMMB_IRQ_GPIO		GPIO013_GPIO_13
#define CMMB_EXT_CLK_EN	GPIO090_CMMB_CLK

static unsigned long cmmb_pin_config[] = {
	CMMB_SPI_CLK,
	CMMB_SPI_CSn,
	CMMB_SPI_DIN,
	CMMB_SPI_DOUT,
};

static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect = 1,
	.enable_dma = 1,
};

/*
 * FIXME:Need to fine tune the delay time for the below 3 functions
 * Maybe we can short the time
 */
static int cmmb_power_reset(void)
{
	int cmmb_rst;

	cmmb_rst = mfp_to_gpio(CMMB_POWER_RESETn);

	if (gpio_request(cmmb_rst, "cmmb rst")) {
		pr_warning("failed to request GPIO for CMMB RST\n");
		return -EIO;
	}

	gpio_direction_output(cmmb_rst, 0);
	msleep(100);

	/* get cmmb go out of reset state */
	gpio_direction_output(cmmb_rst, 1);
	gpio_free(cmmb_rst);

	return 0;
}

static int cmmb_power_on(void)
{
	int cmmb_en;

	cmmb_en = mfp_to_gpio(CMMB_POWER_EN);
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

	cmmb_en = mfp_to_gpio(CMMB_POWER_EN);

	if (gpio_request(cmmb_en, "cmmb power")) {
		pr_warning("failed to request GPIO for CMMB POWER\n");
		return -EIO;
	}

	gpio_direction_output(cmmb_en, 0);
	gpio_free(cmmb_en);
	msleep(100);

	return 0;
}

/*
 * Add two functions: cmmb_cs_assert and cmmb_cs_deassert.
 * Provide the capbility that
 * cmmb driver can handle the SPI_CS by itself.
 */
static int cmmb_cs_assert(void)
{
	int cs;
	cs = mfp_to_gpio(CMMB_SPI_CSn);
	gpio_direction_output(cs, 0);
	return 0;
}

static int cmmb_cs_deassert(void)
{
	int cs;
	cs = mfp_to_gpio(CMMB_SPI_CSn);
	gpio_direction_output(cs, 1);
	return 0;
}

static struct cmmb_platform_data cmmb_info = {
	.power_on = cmmb_power_on,
	.power_off = cmmb_power_off,
	.power_reset = cmmb_power_reset,
	.cs_assert = cmmb_cs_assert,
	.cs_deassert = cmmb_cs_deassert,

	.gpio_power = mfp_to_gpio(CMMB_POWER_EN),
	.gpio_reset = mfp_to_gpio(CMMB_POWER_RESETn),
	.gpio_cs = mfp_to_gpio(CMMB_SPI_CSn),
	.gpio_defined = 1,
};

static void cmmb_dummy_cs(u32 cmd)
{
/*
 * Because in CMMB read/write,the max data size is more than 8kB
 * 8k = max data length per dma transfer for pxaxxx
 * But till now,The spi_read/write driver doesn't support muti DMA cycles
 *
 * Here the spi_read/write will not affect the SPI_CS,but provides
 * cs_assert and cs_deassert in the struct cmmb_platform_data
 *
 * And cmmb driver can/should control SPI_CS by itself
 */
}

static struct pxa2xx_spi_chip cmmb_spi_chip = {
	.rx_threshold   = 1,
	.tx_threshold   = 1,
	.cs_control     = cmmb_dummy_cs,
};

/* bus_num must match id in pxa2xx_set_spi_info() call */
static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias		= "cmmb_if",
		.platform_data	= &cmmb_info,
		.controller_data	= &cmmb_spi_chip,
		.irq		= gpio_to_irq(mfp_to_gpio(CMMB_IRQ_GPIO)),
		.max_speed_hz	= 8000000,
		.bus_num		= 1,
		.chip_select	= 0,
		.mode			= SPI_MODE_0,
	},
};

static void __init emeidkb_init_spi(void)
{
	int err;
	int cmmb_int, cmmb_cs;

	mfp_config(ARRAY_AND_SIZE(cmmb_pin_config));
	cmmb_cs = mfp_to_gpio(CMMB_SPI_CSn);
	err = gpio_request(cmmb_cs, "cmmb cs");
	if (err) {
		pr_warning("[ERROR] failed to request GPIO for CMMB CS\n");
		return;
	}
	gpio_direction_output(cmmb_cs, 1);

	cmmb_int = mfp_to_gpio(CMMB_IRQ_GPIO);

	err = gpio_request(cmmb_int, "cmmb irq");
	if (err) {
		pr_warning("[ERROR] failed to request GPIO for CMMB IRQ\n");
		return;
	}
	gpio_direction_input(cmmb_int);
	gpio_free(cmmb_int);

	pxa988_add_ssp(0);
	pxa988_add_spi(1, &pxa_ssp_master_info);
	if (spi_register_board_info(spi_board_info,
			ARRAY_SIZE(spi_board_info))) {
		pr_warning("[ERROR] failed to register spi device.\n");
		return;
	}
}
#endif /* defined CONFIG_CMMB */

#ifdef CONFIG_DDR_DEVFREQ
static struct devfreq_frequency_table *ddr_freq_table;

static struct devfreq_platform_data devfreq_ddr_pdata = {
	.clk_name = "ddr",
	.interleave_is_on = 0,	/* only one mc */
};

static struct platform_device pxa988_device_ddrdevfreq = {
	.name = "devfreq-ddr",
	.id = -1,
};

static void __init pxa988_init_device_ddrdevfreq(void)
{
	u32 i = 0;
	u32 ddr_freq_num = pxa988_get_ddr_op_num();

	ddr_freq_table = kmalloc(sizeof(struct devfreq_frequency_table) * \
					(ddr_freq_num + 1), GFP_KERNEL);
	if (!ddr_freq_table)
		return;

	for (i = 0; i < ddr_freq_num; i++) {
		ddr_freq_table[i].index = i;
		ddr_freq_table[i].frequency = pxa988_get_ddr_op_rate(i);
	}
	ddr_freq_table[i].index = i;
	ddr_freq_table[i].frequency = DEVFREQ_TABLE_END;

	devfreq_ddr_pdata.freq_table = ddr_freq_table;
	devfreq_ddr_pdata.hw_base[0] =  DMCU_VIRT_BASE;
	devfreq_ddr_pdata.hw_base[1] =  DMCU_VIRT_BASE;

	pxa988_device_ddrdevfreq.dev.platform_data = (void *)&devfreq_ddr_pdata;
	platform_device_register(&pxa988_device_ddrdevfreq);
}
#endif

static void __init emeidkb_init_smc(void)
{
	/*
	 * emeidkb doesn't use SMC,
	 * just turn off SMC clock to save power.
	 */
	__raw_writel(0x3, APMU_SMC_CLK_RES_CTRL);
}

#ifdef CONFIG_VMETA_DEVFREQ
static struct devfreq_frequency_table *vpu_freq_table;

static struct devfreq_platform_data devfreq_vpu_pdata = {
	.clk_name = "VPUCLK",
};

static struct platform_device pxa988_device_vpudevfreq = {
	.name = "devfreq-vMeta",
	.id = -1,
};

static void __init pxa988_init_device_vpudevfreq(void)
{
	u32 i = 0;
	u32 vpu_freq_num = pxa988_get_vpu_op_num();

	vpu_freq_table = kmalloc(sizeof(struct devfreq_frequency_table) * \
					(vpu_freq_num + 1), GFP_KERNEL);
	if (!vpu_freq_table)
		return;

	for (i = 0; i < vpu_freq_num; i++) {
		vpu_freq_table[i].index = i;
		vpu_freq_table[i].frequency = pxa988_get_vpu_op_rate(i);
	}
	vpu_freq_table[i].index = i;
	vpu_freq_table[i].frequency = DEVFREQ_TABLE_END;

	devfreq_vpu_pdata.freq_table = vpu_freq_table;

	pxa988_device_vpudevfreq.dev.platform_data = (void *)&devfreq_vpu_pdata;
	platform_device_register(&pxa988_device_vpudevfreq);
}
#endif

#ifdef CONFIG_PROC_FS
/* GPS: power on/off control */
static void gps_power_on(void)
{
	unsigned int gps_ldo, gps_rst_n;

	gps_ldo = mfp_to_gpio(GPIO_GPS_LDO_EN);
	if (gpio_request(gps_ldo, "gpio_gps_ldo")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_ldo);
		return;
	}

	gps_rst_n = mfp_to_gpio(GPIO084_GPIO_GPS_RESET_N);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_rst_n);
		goto out;
	}

	gpio_direction_output(gps_ldo, 0);
	gpio_direction_output(gps_rst_n, 0);
	mdelay(1);
	gpio_direction_output(gps_ldo, 1);

	pr_info("gps chip powered on\n");

	gpio_free(gps_rst_n);
out:
	gpio_free(gps_ldo);
	return;
}

static void gps_power_off(void)
{
	unsigned int gps_ldo, gps_rst_n, gps_on;

	gps_ldo = mfp_to_gpio(GPIO_GPS_LDO_EN);
	if (gpio_request(gps_ldo, "gpio_gps_ldo")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_ldo);
		return;
	}

	gps_on = mfp_to_gpio(GPIO096_GPIO_GPS_ON_OFF);
	if (gpio_request(gps_on, "gpio_gps_on")) {
		pr_err("Request GPIO failed,gpio: %d\n", gps_on);
		goto out1;
	}

	gps_rst_n = mfp_to_gpio(GPIO084_GPIO_GPS_RESET_N);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		pr_debug("Request GPIO failed, gpio: %d\n", gps_rst_n);
		goto out2;
	}

	gpio_direction_output(gps_ldo, 0);
	gpio_direction_output(gps_rst_n, 0);
	gpio_direction_output(gps_on, 0);

	pr_info("gps chip powered off\n");

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

	gps_rst_n = mfp_to_gpio(GPIO084_GPIO_GPS_RESET_N);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_rst_n);
		return;
	}

	gpio_direction_output(gps_rst_n, flag);
	gpio_free(gps_rst_n);
}

static void gps_on_off(int flag)
{
	unsigned int gps_on;

	gps_on = mfp_to_gpio(GPIO096_GPIO_GPS_ON_OFF);
	if (gpio_request(gps_on, "gpio_gps_on")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_on);
		return;
	}

	gpio_direction_output(gps_on, flag);
	gpio_free(gps_on);
}

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

	/*
	 * CSR and Marvell GPS lib will both use this file
	 * "/proc/drver/sirf" may be modified in future
	 */
	sirf_proc_file = create_proc_entry("driver/sirf", 0644, NULL);
	if (!sirf_proc_file) {
		pr_err("sirf proc file create failed!\n");
		return;
	}

	sirf_proc_file->read_proc = sirf_read_proc;
	sirf_proc_file->write_proc = (write_proc_t  *)sirf_write_proc;
}
#endif

#define PM800_SW_PDOWN			(1 << 5)
static void emei_dkb_poweroff(void)
{
	printk(KERN_INFO"turning off power....\n");
	pm80x_codec_reg_set_bits((PM80X_BASE_PAGE << 8) | PM800_WAKEUP1,
				 PM800_SW_PDOWN, PM800_SW_PDOWN);
}

static void __init emeidkb_init(void)
{
	mfp_config(ARRAY_AND_SIZE(emeidkb_pin_config));

	pm_power_off = emei_dkb_poweroff;

	lcd_HVGA_setup();
	if (is_HVGA_lcd)
		mfp_config(ARRAY_AND_SIZE(emeidkb_lcd_pin_config));

	/* backlight */
	platform_device_register(&emei_dkb_lcd_backlight_devices);

	pxa988_add_thermal();

#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem();
#endif
	pxa988_add_keypad(&emei_dkb_keypad_info);
	regulator_init_pm800();

	/* For AP debug */
	pxa988_add_uart(1);
	/* For GPS */
	pxa988_add_uart(2);
	/* FIXME: add i2c_pxa_platform_data */
	pxa988_add_twsi(0, NULL, ARRAY_AND_SIZE(emeidkb_i2c_info));
	if (!is_HVGA_lcd)
		pxa988_add_twsi(1, NULL, ARRAY_AND_SIZE(emeidkb_i2c1_info));
	pxa988_add_twsi(2, &emeidkb_pwr_i2c_pdata,
			ARRAY_AND_SIZE(emeidkb_pwr_i2c_info));

	/* add ssp1 for hifi audio */
	pxa988_add_ssp(1);
	/* add audio sram */
	pxa988_add_asram(&pxa988_asram_info);

#ifdef CONFIG_FB_PXA168
	if (is_HVGA_lcd) {
		emeidkb_add_lcd_pl();
		emeidkb_add_tv_out();
		printk(KERN_INFO "LCD: Parallel tpo panel selected.\n");
	} else {
		emeidkb_add_lcd_mipi();
		printk(KERN_INFO "LCD: qHD panel selected.\n");
	}
#endif

#ifdef CONFIG_PROC_FS
	/* create proc for sirf GPS control */
	create_sirf_proc_file();
#endif
	emeidkb_init_mmc();

	emeidkb_init_smc();

#ifdef CONFIG_UIO_CODA7542
	pxa_register_coda7542();
#endif

#ifdef CONFIG_VMETA_DEVFREQ
	pxa988_init_device_vpudevfreq();
#endif

#ifdef CONFIG_USB_PXA_U2O
	pxa988_device_udc.dev.platform_data = &emeidkb_usb_pdata;
	platform_device_register(&pxa988_device_udc);
#endif

#ifdef CONFIG_PXA9XX_ACIPC
	platform_device_register(&pxa9xx_device_acipc);
#endif

	/* off-chip devices */
	platform_add_devices(ARRAY_AND_SIZE(dkb_platform_devices));

#if defined(CONFIG_VIDEO_MV)
	pxa988_add_cam(&mv_cam_data);
#endif

#ifdef CONFIG_VIDEO_MVISP
	pxa988_init_dxoisp();
#endif

#if (defined CONFIG_CMMB)
	if (!is_HVGA_lcd)
		emeidkb_init_spi();
#endif

#ifdef CONFIG_DDR_DEVFREQ
	pxa988_init_device_ddrdevfreq();
#endif
}

MACHINE_START(EMEIDKB, "PXA988")
	.map_io		= mmp_map_io,
	.nr_irqs	= EMEI_NR_IRQS,
	.init_irq	= pxa988_init_irq,
	.timer		= &pxa988_timer,
	.reserve	= pxa988_reserve,
	.init_machine	= emeidkb_init,
MACHINE_END
