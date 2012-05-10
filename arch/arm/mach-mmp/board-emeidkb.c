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

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa988.h>
#include <mach/pxa988.h>
#include <mach/irqs.h>
#include <mach/regs-mpmu.h>

#include "common.h"

#define EMEI_NR_IRQS		(IRQ_BOARD_START + 24)

static unsigned long emeidkb_pin_config[] __initdata = {

};

static void __init emeidkb_init(void)
{
	mfp_config(ARRAY_AND_SIZE(emeidkb_pin_config));
}

MACHINE_START(EMEIDKB, "PXA988-Based")
	.map_io		= mmp_map_io,
	.nr_irqs	= EMEI_NR_IRQS,
	.init_irq	= pxa988_init_irq,
	.timer		= &pxa988_timer,
	.reserve	= pxa988_reserve,
	.init_machine	= emeidkb_init,
MACHINE_END
