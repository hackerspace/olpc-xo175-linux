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
#include <asm/cacheflush.h>

#include <mach/regs-ciu.h>
#include <mach/regs-apmu.h>
#include <mach/reset-pxa988.h>

#ifdef CONFIG_SMP
static void pxa988_set_sreset_flag(u32 cpu)
{
	sw_reset_flag[cpu] = 1;
	smp_wmb();
	flush_cache_all();
	outer_clean_range(__pa(&sw_reset_flag[cpu]),
		__pa(&sw_reset_flag[cpu] + sizeof(sw_reset_flag[cpu])));
}

/*
 * This function is called from boot_secondary to bootup the secondary cpu.
 * It maybe called when system bootup or add a plugged cpu into system.
 *
 * cpu here can only be 1 since we only have two cores.
 */
void pxa_cpu_reset(u32 cpu)
{
	u32 tmp;

	BUG_ON(cpu != 1);

	tmp = readl(PMU_CC2_AP);
	if (tmp & CPU1_CORE_RST) {
		/* Core 1 first bootup, we need to release core from reset */
		pxa988_set_sreset_flag(cpu);
		tmp &= ~(CPU1_CORE_RST | CPU1_DBG_RST | CPU1_WDOG_RST);
		writel(tmp, PMU_CC2_AP);
		smp_wmb();
	} else
		gic_raise_softirq(cpumask_of(cpu), 1);
}
#endif

void __init pxa_cpu_reset_handler_init(void)
{
	/* We will reset from DDR directly by default */
#ifdef CONFIG_SMP
	writel(__pa(pxa988_secondary_startup), &secondary_cpu_handler);
	flush_cache_all();
	outer_clean_range(__pa(&secondary_cpu_handler),
		__pa(&secondary_cpu_handler + sizeof(secondary_cpu_handler)));
	/* For smp, we use pxa_cpu_reset_handler as the reset handler */
	writel(virt_to_phys(pxa988_cpu_reset_handler),
		CIU_WARM_RESET_VECTOR);
#elif defined(CONFIG_PM)
	/* For amp, we will use pxa988_cpu_resume as the C2 exit handler */
	writel(__pa(pxa988_cpu_resume_handler), CIU_WARM_RESET_VECTOR);
#endif
}
