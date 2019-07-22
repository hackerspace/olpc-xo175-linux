/*
 * linux/arch/arm/mach-mmp/include/mach/reset-pxa988.h
 *
 * Author:	Neil Zhang <zhangwm@marvell.com>
 * Copyright:	(C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __RESET_PXA988_H__
#define __RESET_PXA988_H__

#define CPU0_CORE_RST	(1 << 16)
#define CPU0_DBG_RST	(1 << 18)
#define CPU0_WDOG_RST	(1 << 19)

#define CPU1_CORE_RST	(1 << 20)
#define CPU1_DBG_RST	(1 << 22)
#define CPU1_WDOG_RST	(1 << 23)

#ifdef CONFIG_SMP
extern u32 secondary_cpu_handler;
extern void pxa988_secondary_startup(void);
extern void pxa988_set_sreset_flag(u32 cpu);
extern void pxa988_cpu_reset_handler(void);

extern void pxa_cpu_reset(u32 cpu);
#endif

extern void __init pxa_cpu_reset_handler_init(void);

#endif /* __RESET_PXA988_H__ */
