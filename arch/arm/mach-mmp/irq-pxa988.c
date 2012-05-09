/*
 * linux/arch/arm/mach-mmp/irq-pxa988.c
 *
 * Generic IRQ handling, GPIO IRQ demultiplexing, etc.
 *
 * Copyright (C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/hardware/gic.h>
#include <mach/regs-icu.h>

#include "common.h"

void __init pxa988_init_gic(void)
{
	/* disable global irq/fiq of icu for cp, ca9_0 and ca9_1 */
	__raw_writel(0x3, PXA988_ICU_CP_GBL_INT_MSK);
	__raw_writel(0x3, PXA988_ICU_A9C0_GBL_INT_MSK);
	__raw_writel(0x3, PXA988_ICU_A9C1_GBL_INT_MSK);
}
