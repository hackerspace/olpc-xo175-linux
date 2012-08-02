/*
 * linux/arch/arm/mach-mmp/ca9-hotplug.c
 *
 * Author:      Hong Feng <hongfeng@marvell.com>
 * Copyright:   (C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/completion.h>
#include <linux/jiffies.h>

#include <asm/cacheflush.h>
#include <asm/delay.h>
#include <asm/io.h>
#include <mach/smp.h>
#include <mach/regs-apmu.h>
#include <mach/pxa988_lowpower.h>


static int pxa988_powergate_is_powered(u32 cpu)
{
	return !(__raw_readl(APMU_CORE_STATUS) & (1 << (3 + 2 * cpu)));
}

/*
 * called from the request cpu,
 * make sure the target cpu is actully down after return
 */
int platform_cpu_kill(unsigned int cpu)
{
	unsigned long timeout = jiffies + HZ;

	while (pxa988_powergate_is_powered(cpu)) {
		if (time_after(jiffies, timeout))
			panic("Core power down timeout, panic for debug\n");
		udelay(10);
	}
	printk(KERN_NOTICE "CPU%u: real shutdown\n", cpu);
	return 1;
}

/*
 * platform-specific code to shutdown a CPU,
 * Called with IRQs disabled
 */
void platform_cpu_die(unsigned int cpu)
{
	/* we're ready for shutdown now, so do it */
	pxa988_hotplug_enter(cpu, PXA988_LPM_D2_UDR);
	/* Never get here, Reset from pxa988_secondary_startup */
	panic("core reset should never get here");
}

int platform_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}
