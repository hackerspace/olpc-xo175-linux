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

struct dmc_regtable_entry {
	u32 reg;
	u32 b2c;
	u32 b2s;
};
struct dmc_regtable {
	u32 count;
	struct dmc_regtable_entry *entry;
};

enum {
	DMCRT_TM = 0,
	DMCRT_PH,
	DMCRT_RL,
	DMCRT_WL,
	DMCRT_TYPECNT,

	DMC_MODE_4X = (1u << 0),
};

struct dmc_timing_entry {
	u32 dsrc;
	u32 mode4x;
	u32 pre_d;
	u32 cas;
	struct dmc_regtable table[DMCRT_TYPECNT];
};
#define DEF_DMC_TAB_ENTRY(label, a)		\
	[label] = {ARRAY_SIZE(a), a}

#define DEF_DMC_TAB(tm, ph, rl, wl)			\
	{						\
		DEF_DMC_TAB_ENTRY(DMCRT_TM, tm),	\
		DEF_DMC_TAB_ENTRY(DMCRT_PH, ph),	\
		DEF_DMC_TAB_ENTRY(DMCRT_RL, rl),	\
		DEF_DMC_TAB_ENTRY(DMCRT_WL, wl),	\
	}


void mmp3_setfreq(int clkid, unsigned long khz);
unsigned long mmp3_getfreq(int clkid);
int mmp3_get_pp_number(void);
unsigned long mmp3_get_pp_freq(int ppidx, int clkid);
void mmp3_pm_enter_idle(int cpu);
void mmp3_pm_enter_c2(int cpu, int hpg);
void mmp3_pm_enter_d2(void);
void mmp3_set_vcc_main_reg_id(const char *name);
int mmp3_get_core_clk(int cpu);

void mmp3_pm_update_dram_timing_table(int count, struct dmc_timing_entry *tab);

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

#endif
