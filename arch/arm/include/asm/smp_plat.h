/*
 * ARM specific SMP header, this contains our implementation
 * details.
 */
#ifndef __ASMARM_SMP_PLAT_H
#define __ASMARM_SMP_PLAT_H

#include <asm/cputype.h>

/*
 * Return true if we are running on a SMP platform
 */
static inline bool is_smp(void)
{
#ifndef CONFIG_SMP
	return false;
#elif defined(CONFIG_SMP_ON_UP)
	extern unsigned int smp_on_up;
	return !!smp_on_up;
#else
	return true;
#endif
}

/* all SMP configurations have the extended CPUID registers */


#endif
