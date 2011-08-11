/*
 * MMP3 Power Management Routines
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2011 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __MMP3_PM_H__
#define __MMP3_PM_H__

enum {
	MMP3_CLK_MP1 = 0,
	MMP3_CLK_MP2,
	MMP3_CLK_MM,
	MMP3_CLK_ACLK,
	MMP3_CLK_DDR_1,
	MMP3_CLK_DDR_2,
	MMP3_CLK_AXI_1,
	MMP3_CLK_AXI_2,
	MMP3_CLK_TOP_NUM,
};

void mmp3_setfreq(int clkid, unsigned long khz);
unsigned long mmp3_getfreq(int clkid);
int mmp3_get_pp_number(void);
unsigned long mmp3_get_pp_freq(int ppidx, int clkid);
void mmp3_pm_enter_idle(int cpu);


enum {
	TRACE_DFC_MARKER_CORE = (1u << 0),
	TRACE_DFC_MARKER_DRAM = (1u << 1),
	TRACE_DFC_MARKER_AXI = (1u << 2),
	TRACE_DFC_MARKER_ERR = (1u << 7),
};

union trace_dfc_log {
	struct {
		u32 marker:8;
		u32 pp_core:8;
		u32 pp_dram:8;
		u32 pp_axi:8;
	};
	u32 val;
};

#include <mach/mmp_events.h>

#endif
