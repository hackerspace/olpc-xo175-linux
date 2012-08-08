/*
 * linux/arch/arm/mach-mmp/include/mach/system.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_SYSTEM_H
#define __ASM_MACH_SYSTEM_H

#include <mach/cputype.h>

static inline void arch_idle(void)
{
	cpu_do_idle();
}

extern void (*arch_reset)(char, const char *);

/* hacked i2c routine */
extern int __raw_i2c_bus_reset(u8 bus_num);
extern int __raw_i2c_read_reg(u8 bus_num, u8 addr, u8 reg, u8 *buf, int len);

#endif /* __ASM_MACH_SYSTEM_H */
