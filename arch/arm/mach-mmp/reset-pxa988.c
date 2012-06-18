/*
 * linux/arch/arm/mach-mmp/reset-pxa988.c
 *
 * Author:	Neil Zhang <zhangwm@marvell.com>
 * Copyright:	(C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/smp.h>

#include <asm/io.h>
#include <asm/hardware/gic.h>

#include <mach/regs-ciu.h>
#include <mach/regs-apmu.h>
#include <mach/reset-pxa988.h>

#ifdef CONFIG_SMP
/*
 * This function is called from boot_secondary to bootup the secondary cpu.
 * It maybe called when system bootup or add a plugged cpu into system.
 */
void pxa_cpu_reset(u32 cpu)
{
	static bool bootup = true;
	u32 tmp;

	if (bootup) {
		/* When system bootup, we need to release core from reset */
		pxa988_set_sreset_flag(cpu);

		tmp = readl(PMU_CC2_AP);
		if (cpu)
			tmp &= ~(CPU1_CORE_RST | CPU1_DBG_RST | CPU1_WDOG_RST);
		else
			tmp &= ~(CPU0_CORE_RST | CPU0_DBG_RST | CPU0_WDOG_RST);
		writel(tmp, PMU_CC2_AP);
		wmb();

		bootup = false;
	} else
		gic_raise_softirq(cpumask_of(cpu), 1);
}
#endif

void __init pxa_cpu_reset_handler_init(void)
{
	/* We will reset from DDR directly by default */
#ifdef CONFIG_SMP
	writel(virt_to_phys(pxa988_secondary_startup), &secondary_cpu_handler);
	/* For smp, we use pxa_cpu_reset_handler as the reset handler */
	writel(virt_to_phys(pxa988_cpu_reset_handler),
		CIU_REG(CIU_WARM_RESET_VECTOR));
#else
	/* For amp, we will use pxa988_cpu_resume as the reset handler */
#endif
}
