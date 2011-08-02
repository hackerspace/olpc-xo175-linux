/*
 * arch/arm/mach-pxa/include/mach/pxa95x_pm.c
 *
 * PXA95x Power Management Routines
 *
 * Copyright (C) 2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#undef DEBUG
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/kobject.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/pxa3xx-regs.h>
#include <mach/regs-rtc.h>
#include <mach/regs-intc.h>
#include <mach/regs-ost.h>
#include <mach/mfp.h>
#include <mach/mfp-pxa3xx.h>
#include <mach/gpio.h>
#include <mach/pxa95x_pm.h>
#include <mach/soc_vmeta.h>
#ifdef CONFIG_ARMV7_OS_SAVE_AND_RESTORE
#include <asm/hardware/armv7_jtag.h>
#endif

static struct clk *clk_pout;

unsigned int user_index;
void vmeta_pwr(unsigned int enableDisable)
{
	unsigned int vmpwr = 0;
	static unsigned int onetime;
	vmpwr = VMPWR;

	if (onetime == 0) {
		onetime = 1;
		dvfm_enable_op_name("208M_HF", user_index);
		dvfm_enable_op_name("416M_VGA", user_index);
	}
	if (VMETA_PWR_ENABLE == enableDisable) {
		if (vmpwr & VMPWR_PWR_ST)
			return;	/*Pwr is already on */
		VMPWR = VMPWR_SETALLWAYS | VMPWR_PWON;
		do {
			vmpwr = VMPWR;
		} while ((vmpwr & VMPWR_PWR_ST) != VMPWR_PWR_ST);
	} else if (VMETA_PWR_DISABLE == enableDisable) {
		if ((vmpwr & VMPWR_PWR_ST) != VMPWR_PWR_ST)
			return;	/*Pwr is already off */
		VMPWR = VMPWR_SETALLWAYS;
	}
}

void gc_pwr(int enableDisable)
{
	unsigned int gcpwr = 0;

	gcpwr = GCPWR;
	if (GC_PWR_ENABLE == enableDisable) {
		if (gcpwr & GCPWR_PWR_ST)
			return;	/*Pwr is already on */
		GCPWR = GCPWR_SETALLWAYS | GCPWR_PWON;
		do {
			gcpwr = GCPWR;
		} while ((gcpwr & GCPWR_PWR_ST) != GCPWR_PWR_ST);
		gcpwr = GCPWR;
		gcpwr |= GCPWR_RST_N;
		GCPWR = gcpwr;
	} else if (GC_PWR_DISABLE == enableDisable) {
		if ((gcpwr & GCPWR_PWR_ST) != GCPWR_PWR_ST)
			return;	/*Pwr is already off */
		/* GCPWR_RST_N,GCPWR_PWON = 0 */
		GCPWR = GCPWR_SETALLWAYS;
		do {
			gcpwr = GCPWR;
		} while ((gcpwr & GCPWR_PWR_ST) == GCPWR_PWR_ST);
	}
}
EXPORT_SYMBOL(gc_pwr);

static int __init pxa95x_pm_init(void)
{
	clk_pout = clk_get(NULL, "CLK_POUT");
	if (IS_ERR(clk_pout)) {
		pr_err("pxa95x_pm: get CLK_POUT failed\n");
		clk_pout = NULL;
	}

	if (clk_pout)
		clk_enable(clk_pout);
	return 0;
}
late_initcall(pxa95x_pm_init);
