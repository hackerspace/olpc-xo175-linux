/*
 * linux/arch/arm/mach-mmp/include/mach/regs-pmu.h
 *
 *   Power Management Unit
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_PMU_H
#define __ASM_MACH_REGS_PMU_H

#include <mach/addr-map.h>

#define PMUM_VIRT_BASE		(APB_VIRT_BASE + 0x50000)
#define PMUM_REG(x)		(PMUM_VIRT_BASE + (x))

#define PMUA_VIRT_BASE		(AXI_VIRT_BASE + 0x82800)
#define PMUA_REG(x)		(PMUA_VIRT_BASE + (x))

#endif	/* __ASM_MACH_REGS_PMU_H */
