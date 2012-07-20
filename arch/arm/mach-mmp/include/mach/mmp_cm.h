/*
 * Driver for mmp platform's core morphing
 *
 * Copyright (c) 2011-2012 Marvell Semiconductors Inc.
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef MMP_CM_H
#define MMP_CM_H

#include <mach/smp.h>
#include <mach/addr-map.h>

#define MMP_CM_CPU_ID_MP1	0
#define MMP_CM_CPU_ID_MM	1

#define MMP_CM_REG_DIS_MP1	(PGU_VIRT_BASE + 0xc80)
#define MMP_CM_REG_DIS_MP1_MP2	(PGU_VIRT_BASE + 0xc84)
#define MMP_CM_REG		MMP_CM_REG_DIS_MP1_MP2

#ifdef CONFIG_CORE_MORPHING
extern int cm_get_active_core_id(void);
extern int cm_vote_mp1(void);
extern int cm_cancel_vote_mp1(void);
extern void cm_enable(void);
#else
#define cm_get_active_core_id()	(0)
#define cm_vote_mp1()		do { } while (0)
#define cm_cancel_vote_mp1()	do { } while (0)
#define cm_enable()		do { } while (0)
#endif

#endif /* MMP_CM_H */
