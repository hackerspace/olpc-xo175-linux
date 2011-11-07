/*
 * MMP Power Management Routines
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2011 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __MMP_PM_H__
#define __MMP_PM_H__

#include <linux/pm_qos_params.h>

#ifdef CONFIG_CPU_MMP2
#include <mach/mmp2_pm.h>
#define PM_QOS_CONSTRAINT EXIT_LATENCY_CORE_EXTIDLE
#else
#define PM_QOS_CONSTRAINT PM_QOS_DEFAULT_VALUE
#endif

#endif
