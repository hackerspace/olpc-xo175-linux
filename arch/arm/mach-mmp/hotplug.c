/*
 *  linux/arch/arm/mach-realview/hotplug.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/completion.h>

#include <asm/cacheflush.h>
#include <mach/smp.h>
#include <mach/mmp3_pm.h>

extern volatile int pen_release;

static DECLARE_COMPLETION(cpu_killed);

static inline void platform_do_lowpower(unsigned int cpu)
{
	int hardid = smp_hardid[cpu];

	flush_cache_all();
	dsb();
	/*
	 * there is no power-control hardware on this platform, so all
	 * we can do is put the core into WFI; this is safe as the calling
	 * code will have already disabled interrupts
	 */
	for (;;) {
		/*
		 * here's the WFI
		 */
		mmp3_pm_enter_idle(hardid);

		if (pen_release == hardid) {
			/*
			 * OK, proper wakeup, we're done
			 */
			break;
		}

		/*
		 * getting here, means that we have come out of WFI without
		 * having been woken up - this shouldn't happen
		 *
		 * The trouble is, letting people know about this is not really
		 * possible, since we are currently running incoherently, and
		 * therefore cannot safely call printk() or anything else
		 */
#ifdef DEBUG
		printk("CPU%u: spurious wakeup call\n", cpu);
#endif
	}
}

int platform_cpu_kill(unsigned int cpu)
{
	return wait_for_completion_timeout(&cpu_killed, 5000);
}

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void platform_cpu_die(unsigned int cpu)
{
#ifdef DEBUG
	unsigned int this_cpu = hard_smp_processor_id();

	if (smp_hardid[cpu] != this_cpu) {
		printk(KERN_CRIT "Eek! platform_cpu_die running on %u, should be %u\n",
			   this_cpu, hardid[cpu]);
		BUG();
	}
#endif

	printk(KERN_NOTICE "CPU%u: shutdown\n", cpu);
	complete(&cpu_killed);

	/*
	 * we're ready for shutdown now, so do it
	 */
	platform_do_lowpower(cpu);

	/*
	 * bring this CPU back into the world of cache
	 * coherency, and then restore interrupts
	 */
}

int platform_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	return cpu == 0 ? -EPERM : 0;
}
