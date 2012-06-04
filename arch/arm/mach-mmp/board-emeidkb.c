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

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa988.h>
#include <mach/pxa988.h>
#include <mach/irqs.h>
#include <mach/regs-mpmu.h>
#include <plat/pmem.h>
#include <plat/pxa27x_keypad.h>

#include "common.h"

#define EMEI_NR_IRQS		(IRQ_BOARD_START + 24)

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
	GPIO014_GPIO_PROX_IRQ,
	GPIO015_GPIO_NFC_EN,
	GPIO016_GPIO_TP_RESET,
	GPIO017_GPIO_TP_INT,
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
	GPIO032_GPIO_LCD_PWM,

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

	GPIO045_UART3_RXD,	/* GPS_UART_RXD */
	GPIO046_UART3_TXD,	/* GPS_UART_TXD */

	GPIO047_UART2_RXD,	/* AP_RXD */
	GPIO048_UART2_TXD,	/* AP_TXD */

#define GPIO049_GPIO_BARA_INT2	GPIO049_GPIO_49
#define GPIO050_GPIO_BARA_INT1	GPIO050_GPIO_50
	GPIO049_GPIO_BARA_INT2,
	GPIO050_GPIO_BARA_INT1,

	GPIO051_UART1_RXD,	/* CP_RXD */
	GPIO052_UART1_TXD,	/* CP_TXD */

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
#define GPIO092_GPIO_COMPASS_INT	GPIO092_GPIO_92
#define GPIO093_GPIO_G_INT		GPIO093_GPIO_93
#define GPIO094_GPIO_NFC_IRQ		GPIO094_GPIO_94
#define GPIO095_GPIO_MOTION_INT		GPIO095_GPIO_95
#define GPIO096_GPIO_GPS_ON_OFF		GPIO096_GPIO_96
#define GPIO097_GPIO_GPS_PPS		GPIO097_GPIO_97
#define GPIO098_GPIO_PRESURE_DRDY	GPIO098_GPIO_98
#define GPIO124_GPIO_CODEC_INT		GPIO124_GPIO_124
	GPIO091_GPIO_GYRO_INT,
	GPIO092_GPIO_COMPASS_INT,
	GPIO093_GPIO_G_INT,
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
	GPIO_LED_B_CTRL,
	GPIO_LED_R_CTRL,
	GPIO_LED_G_CTRL,
	GPIO_GPS_LDO_EN,
	GPIO_VCM_PWDN,
};

static unsigned int emei_dkb_matrix_key_map[] = {
	KEY(0, 0, KEY_BACKSPACE),
	KEY(0, 1, KEY_END),
	KEY(0, 2, KEY_CAMERA), /* 1st camera */

	KEY(1, 0, KEY_OK),
	KEY(1, 1, KEY_HOME),
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

static void __init emeidkb_init(void)
{
	mfp_config(ARRAY_AND_SIZE(emeidkb_pin_config));

	/* backlight */
	platform_device_register(&emei_dkb_lcd_backlight_devices);

#ifdef CONFIG_ANDROID_PMEM
	pxa_add_pmem();
#endif
	pxa988_add_keypad(&emei_dkb_keypad_info);
}

MACHINE_START(EMEIDKB, "PXA988-Based")
	.map_io		= mmp_map_io,
	.nr_irqs	= EMEI_NR_IRQS,
	.init_irq	= pxa988_init_irq,
	.timer		= &pxa988_timer,
	.reserve	= pxa988_reserve,
	.init_machine	= emeidkb_init,
MACHINE_END
