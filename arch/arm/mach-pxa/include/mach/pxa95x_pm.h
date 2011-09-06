/*
 * arch/arm/mach-pxa/include/mach/pxa95x_pm.h
 *
 * PXA95x Power Management Routines Head File
 *
 * Copyright (C) 2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __PXA95X_PM_H__
#define __PXA95X_PM_H__

#define GC_PWR_ENABLE           (1)
#define GC_PWR_DISABLE          (0)

extern void gc_pwr(int enableDisable);

#endif
