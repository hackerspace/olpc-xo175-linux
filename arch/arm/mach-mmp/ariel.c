/*
 *  linux/arch/arm/mach-mmp/ariel.c
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

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <mach/addr-map.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp3.h>
#include <mach/irqs.h>

#include "common.h"
#include "onboard.h"

#define YELLOWSTONE_NR_IRQS		(IRQ_BOARD_START + 64)

static void __init ariel_init(void)
{
	/* on-chip devices */
	mmp3_add_uart(3);

	platform_device_register(&mmp3_device_rtc);
}

MACHINE_START(ARIEL, "Ariel")
	.map_io		= mmp_map_io,
	.nr_irqs	= YELLOWSTONE_NR_IRQS,
	.init_irq	= mmp3_init_irq,
	.timer		= &mmp3_timer,
	.reserve	= mmp3_reserve,
	.init_machine	= ariel_init,
MACHINE_END
