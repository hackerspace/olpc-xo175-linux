/*
 *  linux/arch/arm/mach-mmp/ttc_dkb.c
 *
 *  Support for the Marvell PXA910-based TTC_DKB Development Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

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
#include <linux/gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa910.h>
#include <mach/pxa910.h>

#include "common.h"

#define TTCDKB_NR_IRQS		(IRQ_BOARD_START + 24)

static unsigned long ttc_dkb_pin_config[] __initdata = {
	/* UART2 */
	GPIO47_UART2_RXD,
	GPIO48_UART2_TXD,

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

static struct pxa27x_keypad_platform_data ttc_dkb_keypad_info __initdata = {
	.matrix_key_rows	= 7,
	.matrix_key_cols	= 5,
	.matrix_key_map		= ttc_dkb_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(ttc_dkb_matrix_key_map),
	.debounce_interval	= 30,
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

static struct pm860x_platform_data ttc_dkb_pm8607_info = {
	.backlight	= &ttc_dkb_backlight[0],
	.led		= &ttc_dkb_led[0],
	.companion_addr	= 0x11,
	.irq_mode	= 0,
	.irq_base	= IRQ_BOARD_START,

	.i2c_port	= GI2C_PORT,
	.num_backlights	= ARRAY_SIZE(ttc_dkb_backlight),
	.num_leds	= ARRAY_SIZE(ttc_dkb_led),
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
};

static struct platform_device *ttc_dkb_devices[] = {
	&ttc_dkb_device_onenand,
};

static void __init ttc_dkb_init(void)
{
	mfp_config(ARRAY_AND_SIZE(ttc_dkb_pin_config));

	/* on-chip devices */
	pxa910_add_uart(1);

	pxa910_add_keypad(&ttc_dkb_keypad_info);

	pxa910_add_twsi(0, NULL, ARRAY_AND_SIZE(ttc_dkb_i2c_info));

	/* off-chip devices */
	platform_add_devices(ARRAY_AND_SIZE(ttc_dkb_devices));
}

MACHINE_START(TTC_DKB, "PXA910-based TTC_DKB Development Platform")
	.map_io		= mmp_map_io,
	.nr_irqs	= TTCDKB_NR_IRQS,
	.init_irq       = pxa910_init_irq,
	.timer          = &pxa910_timer,
	.init_machine   = ttc_dkb_init,
MACHINE_END
