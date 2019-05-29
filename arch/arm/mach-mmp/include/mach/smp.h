#ifndef ASMARM_ARCH_SMP_H
#define ASMARM_ARCH_SMP_H

#include <linux/smp.h>
#include <asm/hardware/gic.h>

#define hard_smp_processor_id()			\
	({						\
		unsigned int cpunum;			\
		__asm__("mrc p15, 0, %0, c0, c0, 5"	\
			: "=r" (cpunum));		\
		cpunum &= 0x0F;				\
	})

/*
 * In MMP3, there are 3 cores: PJ4mm = 2, PJ4mp1 = 0 , PJ4mp2 = 1
 * Use hard ID in order to support booting from mm core.
 */
extern unsigned int smp_hardid[NR_CPUS];

#endif
