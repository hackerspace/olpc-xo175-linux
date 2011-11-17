/*
 * linux/arch/arm/mach-mmp/pm-mmp3.c
 *
 *
 * Copyright (C) 2011 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clocksource.h>
#include <linux/time.h>
#include <asm/proc-fns.h>
#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include <mach/addr-map.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/system.h>
#ifdef CONFIG_TRACEPOINTS
#define CREATE_TRACE_POINTS
#endif
#include <mach/mmp3_pm.h>
#include <mach/smp.h>

static inline int mmp3_smpid(void)
{
	return hard_smp_processor_id();
}

#define pm_print(args...) printk(args)

/* Dynamic Frequency Change Part */
enum {
	MMP3_FREQ_OP_GET	= 0,
	MMP3_FREQ_OP_UPDATE	= 1,
	MMP3_FREQ_OP_SHOW	= 2,

	MMP3_FREQ_PH_D_BY_4	= 1,
	MMP3_FREQ_PH_D_BY_6	= 2,
	MMP3_FREQ_PH_D_BY_8	= 3,
	MMP3_FREQ_PH_D_BY_10	= 4,
	MMP3_FREQ_PH_D_BY_12	= 5,
	MMP3_FREQ_PH_D_BY_14	= 6,
	MMP3_FREQ_PH_D_BY_15	= 7,

	MMP3_FREQCH_VOLTING	= (1u << 27),
	MMP3_FREQCH_CORE	= (1u << 24),
	MMP3_FREQCH_DRAM_CH1	= (8u << 22),
	MMP3_FREQCH_DRAM_CH2	= (1u << 22),
	MMP3_FREQCH_DRAM	= MMP3_FREQCH_DRAM_CH1 | MMP3_FREQCH_DRAM_CH2,
	MMP3_FREQCH_AXI		= (1u << 26),

	MMP3_FREQCH_START	= 0,
	MMP3_FREQCH_COMPLETE	= 1,
};

#define FIELD2DIV(x) ((x) + 1)
#define GET_FIELD(val, shft, bits) (((val) >> (shft)) & ((1u << (bits)) - 1))
#define MAK_FIELD(val, shft, bits) (((val) & ((1u << (bits)) - 1)) << (shft))
#define MOD_FIELD(old, fld, shft, bits) \
	(((old) & ~(MAK_FIELD(-1, shft, bits))) | MAK_FIELD(fld, shft, bits))

/* FCCR 29-31 */
#define MMP3_FREQ_PSRC_SET(old, fld) MOD_FIELD(old, fld, 29, 3)
/* FCCR 29-31 */
#define MMP3_FREQ_PSRC_GET(val) GET_FIELD(val, 29, 3)
/* FCCR 23-25 */
#define MMP3_FREQ_DSRC_SET(old, fld) MOD_FIELD(old, fld, 23, 3)
/* FCCR 23-25 */
#define MMP3_FREQ_DSRC_GET(val) GET_FIELD(val, 23, 3)
/* BUS_CLKRST 6-8 */
#define MMP3_FREQ_ASRC_SET(old, fld) MOD_FIELD(old, fld, 6, 3)
/* BUS_CLKRST 6-8*/
#define MMP3_FREQ_ASRC_GET(val) GET_FIELD(val, 6, 3)
/* CC 3-5 */
#define MMP3_FREQ_AT_SET(old, fld) MOD_FIELD(old, fld, 3, 3)
/* DM_CC 3-5 */
#define MMP3_FREQ_AT_GET(val) GET_FIELD(val, 3, 3)
/* CC 9-11 */
#define MMP3_FREQ_PH_SET(old, fld) MOD_FIELD(old, fld, 9, 3)
/* DM2_CC 25-27 */
#define MMP3_FREQ_PH_GET(val) GET_FIELD(val, 25, 3)
/* CC 0-2 */
#define MMP3_FREQ_PJ_SET(old, fld) MOD_FIELD(old, fld, 0, 3)
/* DM_CC 0-2 */
#define MMP3_FREQ_PJ_GET(val) GET_FIELD(val, 0, 3)
/* CC2 17-20 */
#define MMP3_FREQ_MM_SET(old, fld) MOD_FIELD(old, fld, 17, 4)
/* DM2_CC 17-20 */
#define MMP3_FREQ_MM_GET(val) GET_FIELD(val, 17, 4)
/* CC2 9-12 */
#define MMP3_FREQ_MP1_SET(old, fld) MOD_FIELD(old, fld, 9, 4)
/* DM2_CC 9-12 */
#define MMP3_FREQ_MP1_GET(val) GET_FIELD(val, 9, 4)
/* CC2 13-16 */
#define MMP3_FREQ_MP2_SET(old, fld) MOD_FIELD(old, fld, 13, 4)
/* DM2_CC 13-16 */
#define MMP3_FREQ_MP2_GET(val) GET_FIELD(val, 13, 4)
/* CC2 21-24 */
#define MMP3_FREQ_ACLK_SET(old, fld) MOD_FIELD(old, fld, 21, 4)
/* DM2_CC 21-24 */
#define MMP3_FREQ_ACLK_GET(val) GET_FIELD(val, 21, 4)
/* CC 12-14 */
#define MMP3_FREQ_DDR1_SET(old, fld) MOD_FIELD(old, fld, 12, 3)
/* DM_CC 12-14 */
#define MMP3_FREQ_DDR1_GET(val) GET_FIELD(val, 12, 3)
/* CC3 17-19 */
#define MMP3_FREQ_DDR2_SET(old, fld) MOD_FIELD(old, fld, 17, 3)
/* DM_CC 9-11 */
#define MMP3_FREQ_DDR2_GET(val) GET_FIELD(val, 9, 3)
/* CC 15-17 */
#define MMP3_FREQ_AXI1_SET(old, fld) MOD_FIELD(old, fld, 15, 3)
/* DM_CC 15-17 */
#define MMP3_FREQ_AXI1_GET(val) GET_FIELD(val, 15, 3)
/* CC2 0-2 */
#define MMP3_FREQ_AXI2_SET(old, fld) MOD_FIELD(old, fld, 0, 3)
/* DM2_CC 0-2 */
#define MMP3_FREQ_AXI2_GET(val) GET_FIELD(val, 0, 3)

#define MMP3_PROTECT_CC(x) (((x) & 0x0003fe3f) | 0x00bc0000)
#define MMP3_PROTECT_CC2(x) ((x) & 0xfffffe07)
#define MMP3_PROTECT_CC3(x) ((x) & 0x0effff1f)
#define MMP3_PROTECT_BUSCLKRST(x) ((x) & 0x000001c3)
#define MMP3_PROTECT_FCCR(x) ((x) & 0xff83ffff)


struct mmp3_clock_source {
	char *name;
	u32 frequency;
};

struct mmp3_freq_plan {
	char *name;
	union {
		struct {
			u32 op:4;
			u32 psrc:3;
			u32 pj_d:3;
			u32 mp1_d:4;
			u32 mp2_d:4;
			u32 mm_d:4;
			u32 aclk_d:4;
			u32 ph_d:3;
			u32 at_d:3;
		};
		u32 val;
	} core;
	union {
		struct {
			u32 op:4;
			u32 dsrc:3;
			u32 ch1_d:3;
			u32 ch2_d:3;
		};
		u32 val;
	} dram;
	union {
		struct {
			u32 op:4;
			u32 asrc:3;
			u32 aclk1_d:3;
			u32 aclk2_d:3;
		};
		u32 val;
	} axi;
	u32 khz[MMP3_CLK_TOP_NUM];
};

struct mmp3_pmu {
	u32 *fccr;
	u32 *cc;
	u32 *cc2;
	u32 *cc3;
	u32 *bus;
	u32 *dm_cc;
	u32 *dm2_cc;
	u32 *mc_interleave;
	u32 *mc_slp_req;
	u32 *swstat;
	u32 *dmcu[2];

	u32 ddrdfc_trigger;
	u32 swstat_store;
	spinlock_t mmp3_fc_spin;
	struct mutex mmp3_fc_lock;
	struct mmp3_clock_source *sources;
	struct mmp3_freq_plan *pps;
	int ppscnt;
	void (*setrate)(struct mmp3_pmu *pmu, int id, u32 khz);
	u32 pp_lowbar[MMP3_CLK_TOP_NUM];
	int pp_curr;
	struct mmp3_freq_plan pl_curr;

	u32 trigge_stat[MMP3_CLK_TOP_NUM];
};

#define MMP3_PLL1_VCO			(1600000)
#define MMP3_PLL1_FREQ			(MMP3_PLL1_VCO / 2)
#define MMP3_PLL1_CLKOUTP_FREQ		((MMP3_PLL1_VCO * 2) / 3)
#define MMP3_PLL2_FREQ			(1334000)
#define MMP3_VCXO_FREQ			(26000)

static struct mmp3_clock_source mmp3_fc_sources[] = {
	[0] = {
		.name = "PLL1/2",
		.frequency = MMP3_PLL1_FREQ / 2,
	},
	[1] = {
		.name = "PLL1",
		.frequency = MMP3_PLL1_FREQ,
	},
	[2] = {
		.name = "PLL2",
		.frequency = MMP3_PLL2_FREQ,
	},
	[3] = {
		.name = "PLL1_CLKOUTP",
		.frequency = MMP3_PLL1_CLKOUTP_FREQ,
	},
	[4] = {
		.name = "VCXO",
		.frequency = MMP3_VCXO_FREQ,
	},
};

#define MMP3_TEST_PP 0

enum {
#if MMP3_TEST_PP
	MMP3_PP_26,
	MMP3_PP_50,
#endif
	MMP3_PP_100,
	MMP3_PP_200,
	MMP3_PP_400,
	MMP3_PP_800,
	MMP3_PP_1066,
};
static struct mmp3_freq_plan mmp3_pps[] = {
	/* DDR settings will be fixed upon init according to DDR module used */
#if MMP3_TEST_PP
	[MMP3_PP_26] = {
		.name = "MMP3_PP_26",
		.core = {{MMP3_FREQ_OP_GET, 4, 0, 0, 0, 0, 0, 0, 0},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 7, 7},},
		.dram = {{MMP3_FREQ_OP_GET, 0, 1, 1},},
	},
	[MMP3_PP_50] = {
		.name = "MMP3_PP_50",
		.core = {{MMP3_FREQ_OP_GET, 0, 7, 0, 0, 0, 0, 7, 7},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 3, 3},},
		.dram = {{MMP3_FREQ_OP_GET, 0, 0, 0},},
	},
#endif
	[MMP3_PP_100] = {
		.name = "MMP3_PP_100",
		.core = {{MMP3_FREQ_OP_GET, 0, 3, 0, 0, 0, 0, 3, 3},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 3, 3},},
		.dram = {{MMP3_FREQ_OP_GET, 0, 0, 0},},
	},
	[MMP3_PP_200] = {
		.name = "MMP3_PP_200",
		.core = {{MMP3_FREQ_OP_GET, 0, 1, 0, 0, 0, 0, 1, 1},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 1, 3},},
		.dram = {{MMP3_FREQ_OP_GET, 0, 0, 0},},
	},
	[MMP3_PP_400] = {
		.name = "MMP3_PP_400",
		.core = {{MMP3_FREQ_OP_GET, 1, 1, 0, 0, 0, 0, 1, 1},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 0, 1},},
		.dram = {{MMP3_FREQ_OP_GET, 1, 0, 0},},
	},
	[MMP3_PP_800] = {
		.name = "MMP3_PP_800",
		.core = {{MMP3_FREQ_OP_GET, 1, 0, 0, 0, 1, 1, 1, 1},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 0, 1},},
		.dram = {{MMP3_FREQ_OP_GET, 1, 0, 0},},
	},
	[MMP3_PP_1066] = {
		.name = "MMP3_PP_1066",
		.core = {{MMP3_FREQ_OP_GET, 3, 0, 0, 0, 1, 1, 2, 2},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 0, 1},},
		.dram = {{MMP3_FREQ_OP_GET, 1, 0, 0},},
	},
};

static struct mmp3_pmu mmp3_pmu_config = {
	.fccr = (u32 *)MPMU_FCCR,
	.cc = (u32 *)APMU_REG(0x4),
	.cc2 = (u32 *)APMU_REG(0x150),
	.cc3 = (u32 *)APMU_REG(0x188),
	.bus = (u32 *)APMU_REG(0x6c),
	.dm_cc = (u32 *)APMU_REG(0xc),
	.dm2_cc = (u32 *)APMU_REG(0x158),
	.mc_interleave = (u32 *)APMU_REG(0x184),
	.mc_slp_req = (u32 *)APMU_REG(0xb4),
	.dmcu = {
		[0] = (u32 *)(DMCU_VIRT_BASE + 0x0),
		[1] = (u32 *)(DMCU_VIRT_BASE + 0x10000),
	},

	.sources = mmp3_fc_sources,
	.pps = mmp3_pps,
	.ppscnt = ARRAY_SIZE(mmp3_pps),
};

/* low power mode Part */
struct mmp3_cpu_idle_config {
	u32 idle_config_valid_mask;
	u32 idle_config_keep_mask;
	u32 *idle_config;
	u32 *wake_status;
	u32 *intr_status;
	u32 *freq_status;
};
#define INTR_STAT ((u32 *)(GIC_CPU_VIRT_BASE + 0x18))
#define WAKE_STAT ((u32 *)(MPMU_AWUCRS))
#define MK_APMU_REG_VADDR(x) ((u32 *)APMU_REG(x))
static struct mmp3_percpu_config {
	struct mmp3_cpu_idle_config cic;
} mmp3_percpu[] = {
	[0] = {
		.cic = {
			.idle_config_valid_mask = 0x000000e2,
			.idle_config_keep_mask = 0x8deffc1d,
			.idle_config = MK_APMU_REG_VADDR(0x18),
			.wake_status = WAKE_STAT,
			.intr_status = INTR_STAT,
			.freq_status = &(mmp3_pmu_config.swstat_store),
		},
	},
	[1] = {
		.cic = {
			.idle_config_valid_mask = 0x00000062,
			.idle_config_keep_mask = 0x8deffc1d,
			.idle_config = MK_APMU_REG_VADDR(0x200),
			.wake_status = WAKE_STAT,
			.intr_status = INTR_STAT,
			.freq_status = &(mmp3_pmu_config.swstat_store),
		},
	},
	[2] = {
		.cic = {
			.idle_config_valid_mask = 0x00000062,
			.idle_config_keep_mask = 0x8deffc1d,
			.idle_config = MK_APMU_REG_VADDR(0x204),
			.wake_status = WAKE_STAT,
			.intr_status = INTR_STAT,
			.freq_status = &(mmp3_pmu_config.swstat_store),
		},
	},
};

/* DFC related functions */
static void mmp3_freq_plan_cal(struct mmp3_pmu *pmu,
					struct mmp3_freq_plan *pl)
{
	u32 basefreq;
	basefreq = pmu->sources[pl->core.psrc].frequency;
	basefreq = basefreq / FIELD2DIV(pl->core.pj_d);
	pl->khz[MMP3_CLK_MP1] = basefreq / FIELD2DIV(pl->core.mp1_d);
	pl->khz[MMP3_CLK_MP2] = basefreq / FIELD2DIV(pl->core.mp2_d);
	pl->khz[MMP3_CLK_MM] = basefreq / FIELD2DIV(pl->core.mm_d);
	pl->khz[MMP3_CLK_ACLK] = basefreq / FIELD2DIV(pl->core.aclk_d);

	basefreq = pmu->sources[pl->dram.dsrc].frequency / 2;
	pl->khz[MMP3_CLK_DDR_1] = basefreq / FIELD2DIV(pl->dram.ch1_d);
	pl->khz[MMP3_CLK_DDR_2] = basefreq / FIELD2DIV(pl->dram.ch1_d);

	basefreq = pmu->sources[pl->axi.asrc].frequency;
	pl->khz[MMP3_CLK_AXI_1] = basefreq / FIELD2DIV(pl->axi.aclk1_d);
	pl->khz[MMP3_CLK_AXI_2] = basefreq / FIELD2DIV(pl->axi.aclk2_d);
}

static void mmp3_freq_plan_print(struct mmp3_pmu *pmu,
	struct mmp3_freq_plan *pl, char * tag, u32 tmr)
{
	int proid = mmp3_smpid();
	if (pl->core.op == MMP3_FREQ_OP_SHOW) {
		pm_print("%s<PM:%d>:CPU@[%12s]|"
				"MM: %7d|MP1:%7d|MP2:%7d|FAB:%7d|"
				"%2d us|%d\n"
			, tag
			, proid
			, pmu->sources[pl->core.psrc].name
			, pl->khz[MMP3_CLK_MM]
			, pl->khz[MMP3_CLK_MP1]
			, pl->khz[MMP3_CLK_MP2]
			, pl->khz[MMP3_CLK_ACLK]
			, tmr
			, pmu->trigge_stat[MMP3_CLK_MP1]
			);
	}
	if (pl->dram.op == MMP3_FREQ_OP_SHOW) {
		pm_print("%s<PM:%d>:DDR@[%12s]|CH1:%7d|CH2:%7d|"
				"           |           |"
				"%2d us|%d\n"
			, tag
			, proid
			, pmu->sources[pl->dram.dsrc].name
			, pl->khz[MMP3_CLK_DDR_1]
			, pl->khz[MMP3_CLK_DDR_2]
			, tmr
			, pmu->trigge_stat[MMP3_CLK_DDR_1]
			);
	}
	if (pl->axi.op == MMP3_FREQ_OP_SHOW) {
		pm_print("%s<PM:%d>:AXI@[%12s]|A1: %7d|A2: %7d|"
				"           |           |"
				"%2d us|%d\n"
			, tag
			, proid
			, pmu->sources[pl->axi.asrc].name
			, pl->khz[MMP3_CLK_AXI_1]
			, pl->khz[MMP3_CLK_AXI_2]
			, tmr
			, pmu->trigge_stat[MMP3_CLK_AXI_1]
			);
	}
}
#define mmp3_freq_plan_print_info(pmu, pl, tmr) \
	mmp3_freq_plan_print(pmu, pl, KERN_INFO, tmr)
#define mmp3_freq_plan_print_dbg(pmu, pl, tmr) \
	mmp3_freq_plan_print(pmu, pl, KERN_DEBUG, tmr)

static void mmp3_clear_PJ_RD_STATUS(struct mmp3_pmu *pmu)
{
	u32 val;
	__raw_readl(pmu->cc);
	val = __raw_readl(pmu->cc);
	val = MMP3_PROTECT_CC(val); /* set reserved, SB0, SB1, bits*/
	val = val | (1u << 31);
	__raw_writel(val, pmu->cc);
	val = val & (~(1u << 31));
	__raw_writel(val, pmu->cc);
}

static void mmp3_get_freq_plan(struct mmp3_pmu *pmu,
				struct mmp3_freq_plan *pl, bool crs)
{
	u32 val;
	/* fccr */
	val = __raw_readl(pmu->fccr);
	pl->core.psrc = MMP3_FREQ_PSRC_GET(val);
	pl->dram.dsrc = MMP3_FREQ_DSRC_GET(val);
	/* bus clkrst */
	val = __raw_readl(pmu->bus);
	pl->axi.asrc = MMP3_FREQ_ASRC_GET(val);
	/* dm_cc */
	val = __raw_readl(pmu->dm_cc);
	pl->core.at_d = MMP3_FREQ_AT_GET(val);
	pl->core.pj_d = MMP3_FREQ_PJ_GET(val);
	pl->dram.ch1_d = MMP3_FREQ_DDR1_GET(val);
	pl->dram.ch2_d = MMP3_FREQ_DDR2_GET(val);
	pl->axi.aclk1_d = MMP3_FREQ_AXI1_GET(val);
	/* dm2_cc */
	val = __raw_readl(pmu->dm2_cc);
	pl->core.ph_d = MMP3_FREQ_PH_GET(val);
	pl->core.aclk_d = MMP3_FREQ_ACLK_GET(val);
	pl->core.mm_d = MMP3_FREQ_MM_GET(val);
	pl->core.mp1_d = MMP3_FREQ_MP1_GET(val);
	pl->core.mp2_d = MMP3_FREQ_MP2_GET(val);
	pl->axi.aclk2_d = MMP3_FREQ_AXI2_GET(val);

	if (crs)
		mmp3_clear_PJ_RD_STATUS(pmu);

	mmp3_freq_plan_cal(pmu, pl);
}

#define NONEOP_MASK (~(0xf))
#define EXTRACTDIV(val) ((val) & NONEOP_MASK)
static u32 mmp3_compare_freq_plan(struct mmp3_freq_plan *p1,
					struct mmp3_freq_plan *p2)
{
	u32 mark = 0;
	if ((EXTRACTDIV(p1->core.val) != EXTRACTDIV(p2->core.val))) {
		printk(KERN_DEBUG "<PM> CORE not same: 0x%08x != 0x%08x\n",
			p1->core.val, p2->core.val);
		mark |= TRACE_DFC_MARKER_CORE;
	}
	if ((EXTRACTDIV(p1->dram.val) != EXTRACTDIV(p2->dram.val))) {
		printk(KERN_DEBUG "<PM> DRAM not same: 0x%08x != 0x%08x\n",
			p1->dram.val, p2->dram.val);
		mark |= TRACE_DFC_MARKER_DRAM;
	}
	if ((EXTRACTDIV(p1->axi.val) != EXTRACTDIV(p2->axi.val))) {
		printk(KERN_DEBUG "<PM> AXI not same: 0x%08x != 0x%08x\n",
			p1->axi.val, p2->axi.val);
		mark |= TRACE_DFC_MARKER_AXI;
	}
	return mark;
}

static void mmp3_update_freq_plan(struct mmp3_pmu *pmu,
				struct mmp3_freq_plan *pl)
{
	u32 val;
	/* cc */
	val = __raw_readl(pmu->cc);
	val = MMP3_PROTECT_CC(val);	/*
					 * clear trigger bits
					 * clear read ignor and SB0 bits
					 * set SB1 bits
					 */
	val = MMP3_FREQ_PJ_SET(val, pl->core.pj_d);
	val = MMP3_FREQ_AT_SET(val, pl->core.at_d);
	val = MMP3_FREQ_PH_SET(val, pl->core.ph_d);
	val = MMP3_FREQ_DDR1_SET(val, pl->dram.ch1_d);
	val = MMP3_FREQ_AXI1_SET(val, pl->axi.aclk1_d);
	__raw_writel(val, pmu->cc);
	/* cc2 */
	val = __raw_readl(pmu->cc2);
	val = MMP3_PROTECT_CC2(val); /* clear SB0 bits */
	val = MMP3_FREQ_ACLK_SET(val, pl->core.aclk_d);
	val = MMP3_FREQ_MM_SET(val, pl->core.mm_d);
	val = MMP3_FREQ_MP1_SET(val, pl->core.mp1_d);
	val = MMP3_FREQ_MP2_SET(val, pl->core.mp2_d);
	val = MMP3_FREQ_AXI2_SET(val, pl->axi.aclk2_d);
	__raw_writel(val, pmu->cc2);
	/* cc3 */
	val = __raw_readl(pmu->cc3);
	val = MMP3_PROTECT_CC3(val); /* clear SB0 bits */
	val = MMP3_FREQ_DDR2_SET(val, pl->dram.ch2_d);
	__raw_writel(val, pmu->cc3);
	/* bus clkrst*/
	val = __raw_readl(pmu->bus);
	val = MMP3_PROTECT_BUSCLKRST(val); /* clear SB0 bits */
	val = MMP3_FREQ_ASRC_SET(val, pl->axi.asrc);
	__raw_writel(val, pmu->bus);
	/* fccr */
	val = __raw_readl(pmu->fccr);
	val = MMP3_PROTECT_FCCR(val); /* clear SB0 bits */
	val = MMP3_FREQ_PSRC_SET(val, pl->core.psrc);
	val = MMP3_FREQ_DSRC_SET(val, pl->dram.dsrc);
	__raw_writel(val, pmu->fccr);
}

#define FIXADDR(base, offset) ((u32 *)(((u32)base)+offset))
#define DMCU_HWTCTRL(base) FIXADDR(base, 0x1c0)
#define DMCU_HWTDAT0(base) FIXADDR(base, 0x1c8)
#define DMCU_HWTDAT1(base) FIXADDR(base, 0x1cc)
#define DMCU_MAP_CS0 0x10
#define DMCU_MAP_CS1 0x14
#define DMCU_MAP_VALID (1u << 0)
#define DMCU_CMD_CSSEL_CS0 (1u << 24)
#define DMCU_CMD_CSSEL_CS1 (1u << 25)
#define DMCU_SDRAM_TIMING1 0x80
#define DMCU_SDRAM_TIMING2 0x84
#define DMCU_SDRAM_TIMING3 0x88
#define DMCU_SDRAM_TIMING4 0x8c
#define DMCU_SDRAM_TIMING5 0x90
#define DMCU_SDRAM_TIMING6 0x94
#define DMCU_SDRAM_TIMING7 0x98
#define DMCU_SDRAM_CTRL1 0x50
#define DMCU_SDRAM_CTRL4 0x58
#define DMCU_SDRAM_CTRL14 0x68
#define DMCU_PHY_CTRL3 0x220
#define DMCU_PHY_CTRL14 0x24c
#define DMCU_PHY_DQ_BYTE_SEL 0x300
#define DMCU_PHY_DLL_CTRL_BYTE1 0x304
#define DMCU_USER_COMMAND0 0x160
#define DMCU_USER_COMMAND1 0x164
#define DMCU_HWTPAUSE 0x00010000
#define DMCU_HWTEND 0x00020000
#define DMCU_HWTWRITE 0x80000000
#define DMCU_SDRAM_TYPE_MASK (7u << 2)
#define DMCU_SDRAM_TYPE_DDR3 (2u << 2)
#define DMCU_SDRAM_TYPE_LPDDR2 (5u << 2)

struct ddr_config {
	u32 reg;
	u32 b2c;
	u32 b2s;
};
struct ddrdfc_param {
	u32 oldkhz;
	u32 newkhz;
	u32 newcas;
	u32 newtiming_cnt;
	struct ddr_config *newtiming;
	u32 phytuning_cnt;
	struct ddr_config *phytuning;
};

#define COMMON_DEF(param)						\
	u32 tmpvalue, tab, ent, map;					\
	u32 *base = (u32 *)param;					\
	do {								\
		tab = 0;						\
		ent = 0;						\
		map = 0;						\
		if (__raw_readl(FIXADDR(base, DMCU_MAP_CS0))		\
					& DMCU_MAP_VALID)		\
			map |= DMCU_CMD_CSSEL_CS0;			\
		if (__raw_readl(FIXADDR(base, DMCU_MAP_CS1))		\
					& DMCU_MAP_VALID)		\
			map |= DMCU_CMD_CSSEL_CS1;			\
	} while (0)

#define BEGIN_TABLE(tabidx)						\
	do {								\
		tab = tabidx;						\
		ent = 0;						\
	} while (0)

#define UPDATE_REG(val, reg) __raw_writel(val, reg);

#define INSERT_ENTRY_EX(reg, b2c, b2s, pause, last)			\
	do {								\
		if (b2c == 0xFFFFFFFF) {				\
			tmpvalue = b2s;					\
		} else {						\
			tmpvalue = __raw_readl(FIXADDR(base, reg));	\
			tmpvalue = (((tmpvalue) & (~(b2c))) | (b2s));	\
		}							\
		UPDATE_REG(tmpvalue, DMCU_HWTDAT0(base));		\
		tmpvalue = reg;						\
		if (pause)						\
			tmpvalue |= DMCU_HWTPAUSE;			\
		if (last)						\
			tmpvalue |= DMCU_HWTEND;			\
		UPDATE_REG(tmpvalue, DMCU_HWTDAT1(base));		\
		tmpvalue = (((tab) & 0x3) << 5)				\
				| ((ent) & 0xf) | DMCU_HWTWRITE;	\
		UPDATE_REG(tmpvalue, DMCU_HWTCTRL(base));		\
		ent++;							\
		if (last) {						\
			tab++;						\
			ent = 0;					\
		}							\
	} while (0)

#define INSERT_ENTRIES(entries, entcount, marklast)			\
	do {								\
		u32 li, emk;						\
		if (marklast) {						\
			emk = 0;					\
			for (li = 0; li < entcount; li++) {		\
				if ((li + 1) == entcount)		\
					emk = 1;			\
				INSERT_ENTRY_EX(entries[li].reg,	\
						entries[li].b2c,	\
						entries[li].b2s,	\
						0, emk);		\
			}						\
		} else {						\
			for (li = 0; li < entcount; li++) {		\
				INSERT_ENTRY_EX(entries[li].reg,	\
						entries[li].b2c,	\
						entries[li].b2s,	\
						0, 0);			\
			}						\
		}							\
	} while (0)

#define INSERT_ENTRY(reg, b2c, b2s) INSERT_ENTRY_EX(reg, b2c, b2s, 0, 0)
#define LAST_ENTRY(reg, b2c, b2s) INSERT_ENTRY_EX(reg, b2c, b2s, 0, 1)
#define ALLBITS (0xFFFFFFFF)
#define NONBITS (0x00000000)

#define USE_PHYTUNING 0

static void mmp3_ddrhwt_lpddr2_h2l(u32 *dmcu, struct ddrdfc_param *param)
{
	COMMON_DEF(dmcu);
	/*
	 * 1 PMU asserts 'mc_sleep_req' on type 'mc_sleep_type'; MC4 enters
	 *    self-refresh mode and hold scheduler for DDR access
	 * 2 Frequency change
	 *
	 * Step 1-2 should be triggered by PMU upon DDR change request
	 *
	 * Step 3-6 programmed in the 1st table
	 */
	BEGIN_TABLE(0);
	/* Halt MC4 scheduler*/
	INSERT_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x2);
	/* 3 update timing, we use the boot timing which is for high clock */
	INSERT_ENTRY(DMCU_SDRAM_CTRL4, 0x0001FC00,
				(0x0001FC00 & param->newcas));
	INSERT_ENTRIES(param->newtiming, param->newtiming_cnt, 0);
#if USE_PHYTUNING
	INSERT_ENTRIES(param->phytuning, param->phytuning_cnt, 0);
#endif

	/* 4 reset master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x20000000);
	/* 5 update master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x40000000);
	/* 6. synchronize 2x clock */
	LAST_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x80000000);

	/*
	 * 7 wake up SDRAM; when first table done (acked) PMU will de-assert
	 *    mc_sleep_req to wake up SDRAM from self-refresh
	 *
	 * 8 update SDRAM mode register, programmed in 2nd table
	 */
	BEGIN_TABLE(1);
	INSERT_ENTRY(DMCU_SDRAM_CTRL1, 0x40, 0x40);
	/* 9 do a long ZQ Cal */
	INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, (map | 0x1000));
	INSERT_ENTRY(DMCU_USER_COMMAND1, ALLBITS, (map | 0x20001));
	INSERT_ENTRY(DMCU_USER_COMMAND1, ALLBITS, (map | 0x20002));
	INSERT_ENTRY(DMCU_USER_COMMAND1, ALLBITS, (map | 0x20003));
	/* resume scheduler*/
	LAST_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0);
}

static void mmp3_ddrhwt_lpddr2_l2h(u32 *dmcu, struct ddrdfc_param *param)
{
	COMMON_DEF(dmcu);
	/*
	 * 1 PMU asserts 'mc_sleep_req' on type 'mc_sleep_type'; MC4 enters
	 *    self-refresh mode and hold scheduler for DDR access
	 *
	 * 2 update timing, programmed in 1st table
	 *    we just use the boot timing which is for high clock, no change
	 */
	BEGIN_TABLE(0);
	/* Halt MC4 scheduler*/
	INSERT_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x2);
	/* Update CAS*/
	INSERT_ENTRY(DMCU_SDRAM_CTRL4, 0x0001FC00,
				(0x0001FC00 & param->newcas));
#if USE_PHYTUNING
	INSERT_ENTRIES(param->newtiming, param->newtiming_cnt, 0);
	INSERT_ENTRIES(param->phytuning, param->phytuning_cnt, 1);
#else
	INSERT_ENTRIES(param->newtiming, param->newtiming_cnt, 1);
#endif
	/*
	 * 3 Frequency change upon 1st table done
	 *
	 * Step 4-6 programmed in the 2nd table
	 */
	BEGIN_TABLE(1);
	/* 4 reset master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x20000000);
	/* 5 update master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x40000000);
	/* 6. synchronize 2x clock */
	LAST_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x80000000);

	/*
	 * 7 wake up SDRAM; when first table done (acked) PMU will de-assert
	 *    mc_sleep_req to wake up SDRAM from self-refresh
	 *
	 * 8 update SDRAM mode register, programmed in 2nd table
	 */
	BEGIN_TABLE(2);
	INSERT_ENTRY(DMCU_SDRAM_CTRL1, 0x40, 0x40);
	/* 9 do a long ZQ Cal */
	INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, (map | 0x1000));
	INSERT_ENTRY(DMCU_USER_COMMAND1, ALLBITS, (map | 0x20001));
	INSERT_ENTRY(DMCU_USER_COMMAND1, ALLBITS, (map | 0x20002));
	INSERT_ENTRY(DMCU_USER_COMMAND1, ALLBITS, (map | 0x20003));
	/* resume scheduler*/
	LAST_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0);
}

static void mmp3_ddrhwt_ddr3_h2l(u32 *dmcu, struct ddrdfc_param *param)
{
	COMMON_DEF(dmcu);
	/*
	 * 1 Halt MC4 and disable SDRAM DLL, programmed in 1st table
	 */
	BEGIN_TABLE(0);
	/* Halt MC4 scheduler*/
	INSERT_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x2);
	if (param->newkhz < 125000) {
		/* disable PLL*/
		INSERT_ENTRY(DMCU_SDRAM_CTRL4, NONBITS, 0x80000000);
		/* LMR1*/
		INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, (map | 0x200));
		/* CAS, keep PLL disabled*/
		LAST_ENTRY(DMCU_SDRAM_CTRL4, 0x0001FC00,
				(0x80000000 | (0x0001FC00 & param->newcas)));
	} else {
		/* Update CAS*/
		LAST_ENTRY(DMCU_SDRAM_CTRL4, 0x0001FC00,
				(0x0001FC00 & param->newcas));
	}

	/*
	 * 2 PMU asserts 'mc_sleep_req' on type 'mc_sleep_type'; MC4 enters
	 *    self-refresh mode and hold scheduler for DDR access
	 * 3 Frequency change
	 *
	 * Step 4-7 programmed in the 2nd table
	 */
	BEGIN_TABLE(1);
	/* 4 update timing, do phy tuning*/
	INSERT_ENTRIES(param->newtiming, param->newtiming_cnt, 0);
#if USE_PHYTUNING
	INSERT_ENTRIES(param->phytuning, param->phytuning_cnt, 0);
#endif
	/* 5 reset master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x20000000);
	/* 6 update master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x40000000);
	/* 7. synchronize 2x clock */
	LAST_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x80000000);

	/*
	 * 8 wake up SDRAM; when first table done (acked) PMU will de-assert
	 *    mc_sleep_req to wake up SDRAM from self-refresh
	 *
	 * 9 update SDRAM mode register and do ZQ Cal, programmed in 3nd table
	 */
	BEGIN_TABLE(2);
	INSERT_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x2);
	/* 9.1 reset SDRAM DLL */
	INSERT_ENTRY(DMCU_SDRAM_CTRL1, 0x40, NONBITS);
	/* 9.2 update mode register, LMR0/LMR2 */
	INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, (map | 0x100));
	INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, (map | 0x400));
	/* 9.3 do a long ZQ Cal */
	INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, (map | 0x1000));
	/* resume scheduler*/
	LAST_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0);
}

static void mmp3_ddrhwt_ddr3_l2h(u32 *dmcu, struct ddrdfc_param *param)
{
	COMMON_DEF(dmcu);
	/*
	 * 1 PMU asserts 'mc_sleep_req' on type 'mc_sleep_type'; MC4 enters
	 *    self-refresh mode and hold scheduler for DDR access
	 *
	 * 2 update timing, programmed in 1st table
	 *    we just use the boot timing which is for high clock, no change
	 */
	BEGIN_TABLE(0);
	/* Halt MC4 scheduler*/
	INSERT_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x2);
	if (param->oldkhz >= 125000) {
		/* Update CAS*/
		INSERT_ENTRY(DMCU_SDRAM_CTRL4, 0x0001FC00,
				(0x0001FC00 & param->newcas));
	}

#if USE_PHYTUNING
	INSERT_ENTRIES(param->newtiming, param->newtiming_cnt, 0);
	INSERT_ENTRIES(param->phytuning, param->phytuning_cnt, 1);
#else
	INSERT_ENTRIES(param->newtiming, param->newtiming_cnt, 1);
#endif
	/*
	 * 3 Frequency change
	 *
	 * Step 4-6 programmed in the 2nd table
	 */
	BEGIN_TABLE(1);
	/* 4 reset master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x20000000);
	/* 5 update master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x40000000);
	/* 6. synchronize 2x clock */
	LAST_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x80000000);

	/*
	 * 7 wake up SDRAM; when first table done (acked) PMU will de-assert
	 *    mc_sleep_req to wake up SDRAM from self-refresh
	 *
	 * 8 enable and reset SDRAM DLL
	 * 9 do ZQ Cal
	 * 10 unhalt MC4 scheduler
	 * 8-10 programmed in 3nd table
	 */
	BEGIN_TABLE(2);
	INSERT_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x2);
	if (param->newkhz >= 125000) {
		/* update CAS, enable PLL*/
		INSERT_ENTRY(DMCU_SDRAM_CTRL4, 0x8001FC00,
				(0x0001FC00 & param->newcas));
		/* LMR1*/
		INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, (map | 0x200));
	}
	/* 8.1 reset SDRAM DLL */
	INSERT_ENTRY(DMCU_SDRAM_CTRL1, 0x40, 0x40);
	/* 8.3 update mode register, LMR0/LMR2 */
	INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, (map | 0x100));
	INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, (map | 0x400));
	/* 9 do a long ZQ Cal */
	INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, (map | 0x1000));
	/* resume scheduler*/
	LAST_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0);
}
#if 1
static struct ddr_config ddr3_400mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x911403cf},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x64660404},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xc2004453},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x34f8a187},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x000f2121},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00005501},
};
#else
static struct ddr_config ddr3_400mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x911b00cb},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x748803b4},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xc208406c},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x4698da09},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x00140181},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00005501},
};
#endif

static struct ddr_config ddr3_533mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x911b03cf},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x74780564},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xc2005b6c},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x3698da09},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x00142181},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00006601},
};

static struct ddr_config ddr3_667mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x99a103cf},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x969b06b4},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0xc200728d},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x48390e8c},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x001921f1},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x04040200},
	{DMCU_SDRAM_TIMING7, ALLBITS, 0x00007701},
};

static struct ddr_config ddr3_phy[] = {
	{DMCU_PHY_CTRL3, ALLBITS, 0x20004044},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000000},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00002100},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000001},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00002100},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000002},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00002100},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000003},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00002100},
};

static struct ddr_config ddr3_phyhi[] = {
	{DMCU_PHY_CTRL3, ALLBITS, 0x20004044},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000000},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00001080},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000001},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00001080},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000002},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00001080},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000003},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00001080},
};

static struct ddr_config lpddr2_400mhz[] = {
	{DMCU_SDRAM_TIMING1, ALLBITS, 0x4cda00c5},
	{DMCU_SDRAM_TIMING2, ALLBITS, 0x94860342},
	{DMCU_SDRAM_TIMING3, ALLBITS, 0x2000381b},
	{DMCU_SDRAM_TIMING4, ALLBITS, 0x3023009d},
	{DMCU_SDRAM_TIMING5, ALLBITS, 0x20110142},
	{DMCU_SDRAM_TIMING6, ALLBITS, 0x02424190},
};

static struct ddr_config lpddr2_phy[] = {
	{DMCU_PHY_CTRL3, ALLBITS, 0x20004444},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000000},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00002100},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000001},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00002100},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000002},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00002100},
	{DMCU_PHY_DQ_BYTE_SEL, ALLBITS, 0x00000003},
	{DMCU_PHY_DLL_CTRL_BYTE1, ALLBITS, 0x00002100},
};

static void mmp3_udpate_ddr_parameter_table(struct mmp3_pmu *pmu,
						struct mmp3_freq_plan *pl)
{
	u32 *base;
	u32 type, index, bit, tmpval, l2h;
	struct ddrdfc_param param;

	mmp3_freq_plan_cal(pmu, pl);
	bit = 4;
	for (index = 0; index < 2; index++) {
		base = pmu->dmcu[index];
		bit += index * 2;

		param.oldkhz = pmu->pl_curr.khz[MMP3_CLK_DDR_1 + index];
		param.newkhz = pl->khz[MMP3_CLK_DDR_1 + index];
		if (param.oldkhz < param.newkhz)
			l2h = 1;
		else
			l2h = 0;

		type = __raw_readl(FIXADDR(base, DMCU_SDRAM_CTRL4))
						& DMCU_SDRAM_TYPE_MASK;
		tmpval = __raw_readl(pmu->mc_slp_req) & ~(3u << bit);
		switch (type) {
		case DMCU_SDRAM_TYPE_DDR3:
			if (param.newkhz < 125000)
				param.newcas = 0x0008400;
			else
				param.newcas = 0x0008800;
			if (param.newkhz <= 400000) {
				param.newtiming = ddr3_400mhz;
				param.newtiming_cnt = ARRAY_SIZE(ddr3_400mhz);
				param.phytuning = ddr3_phy;
				param.phytuning_cnt = ARRAY_SIZE(ddr3_phy);
			} else if (param.newkhz <= 533000) {
				param.newtiming = ddr3_533mhz;
				param.newtiming_cnt = ARRAY_SIZE(ddr3_533mhz);
				param.phytuning = ddr3_phy;
				param.phytuning_cnt = ARRAY_SIZE(ddr3_phy);
			} else {
				param.newtiming = ddr3_667mhz;
				param.newtiming_cnt = ARRAY_SIZE(ddr3_667mhz);
				param.phytuning = ddr3_phyhi;
				param.phytuning_cnt = ARRAY_SIZE(ddr3_phyhi);
			}
			if (l2h) {
				tmpval |=  (3u << bit);
				UPDATE_REG(tmpval, pmu->mc_slp_req);
				mmp3_ddrhwt_ddr3_l2h(base, &param);
			} else {
				tmpval |=  (2u << bit);
				UPDATE_REG(tmpval, pmu->mc_slp_req);
				mmp3_ddrhwt_ddr3_h2l(base, &param);
			}
			break;
		case DMCU_SDRAM_TYPE_LPDDR2:
			param.newcas = 0x0008000;
			param.newtiming = lpddr2_400mhz;
			param.newtiming_cnt = ARRAY_SIZE(lpddr2_400mhz);
			param.phytuning = lpddr2_phy;
			param.phytuning_cnt = ARRAY_SIZE(lpddr2_phy);
			if (l2h) {
				tmpval |= (1u << bit);
				UPDATE_REG(tmpval, pmu->mc_slp_req);
				mmp3_ddrhwt_lpddr2_l2h(base, &param);
			} else {
				tmpval |= (0u << bit);
				UPDATE_REG(tmpval, pmu->mc_slp_req);
				mmp3_ddrhwt_lpddr2_h2l(base, &param);
			}
			break;
		default:
			pr_err("<PM> unsupported DDR type in DFC\n");
			break;
		}
	}
}

#define WFINOP_WFENOP ((1u << 24) | (1u << 13))
static void mmp3_disable_idle(void *p)
{
	register unsigned long regval;
	__asm__ __volatile__ ("mrc p15, 1, %0, c15, c1, 1" : "=r" (regval));
	regval |= WFINOP_WFENOP;
	__asm__ __volatile__ ("mcr p15, 1, %0, c15, c1, 1" : : "r" (regval));
}

static void mmp3_enable_idle(void *p)
{
	register unsigned long regval;
	__asm__ __volatile__ ("mrc p15, 1, %0, c15, c1, 1" : "=r" (regval));
	regval &= ~WFINOP_WFENOP;
	__asm__ __volatile__ ("mcr p15, 1, %0, c15, c1, 1" : : "r" (regval));
}

static void mmp3_smp_notify_dfc(int stat)
{
	struct cpumask mask;

	/*
	 * A0 need to all cores run, we implement this by set IPI broadcase
	 * and use WFIasNOP and WFEasNOP, to control idle behavior on each
	 * core
	 */
	cpumask_setall(&mask);

	if (stat == MMP3_FREQCH_START) {
		mmp3_disable_idle(NULL);
		smp_call_function_many(&mask, mmp3_disable_idle, NULL, 1);
	} else {
		smp_call_function_many(&mask, mmp3_enable_idle, NULL, 1);
		mmp3_enable_idle(NULL);
	}
}

static inline u32 cyc2us(u32 cycle)
{
	return (cycle >> 2) + (cycle >> 5) + (cycle >> 6)
		+ (cycle >> 7) + (cycle >> 9) + (cycle >> 10);
}

static inline u32 read_timestamp(void)
{
	unsigned long r1, r2;
	__asm__ __volatile__ ("mrrc p15, 0, %0, %1, c14"
		: "=r" (r1), "=r" (r2) : : "cc");
	return r1;
}

static u32 mmp3_prepare_freqch(struct mmp3_pmu *pmu,
	struct mmp3_freq_plan *pl, union trace_dfc_log *logentry)
{
	u32 change, retry;
	struct mmp3_freq_plan curpl;

	/* obtain FC onwership, should always pass */
	retry = 1000;
	while ((__raw_readl(pmu->dm_cc) & (1u << 24)) != 0) {
		retry--;
		if (retry <= 0) {
			pr_err("<PM> Cannot gain owner of PMU DFC\n");
			return 0;
		}
	}

	/* now get old configuration to see if change is necessary */
	memcpy(&curpl, pl, sizeof(curpl));
	mmp3_get_freq_plan(pmu, &curpl, false);

	logentry->marker = 0;
	change = 0;
	pmu->pl_curr.core.op = MMP3_FREQ_OP_GET;
	pmu->pl_curr.dram.op = MMP3_FREQ_OP_GET;
	pmu->pl_curr.axi.op = MMP3_FREQ_OP_GET;
	if (curpl.core.val != pl->core.val) {
		if (pl->core.op == MMP3_FREQ_OP_UPDATE) {
			pmu->pl_curr.core.op = MMP3_FREQ_OP_SHOW;
			change |= MMP3_FREQCH_CORE;
			logentry->marker |= TRACE_DFC_MARKER_CORE;
			logentry->pp_core = (unsigned char) pmu->pp_curr;
			pmu->trigge_stat[MMP3_CLK_MP1]++;
		} else
			pl->core.val = curpl.core.val;
	}
	if (curpl.axi.val != pl->axi.val) {
		if (pl->axi.op == MMP3_FREQ_OP_UPDATE) {
			pmu->pl_curr.axi.op = MMP3_FREQ_OP_SHOW;
			change |= MMP3_FREQCH_AXI;
			logentry->marker |= TRACE_DFC_MARKER_AXI;
			logentry->pp_axi = (unsigned char) pmu->pp_curr;
			pmu->trigge_stat[MMP3_CLK_AXI_1]++;
		} else
			pl->axi.val = curpl.axi.val;
	}
	if (curpl.dram.val != pl->dram.val) {
		if (pl->dram.op == MMP3_FREQ_OP_UPDATE) {
			pmu->pl_curr.dram.op = MMP3_FREQ_OP_SHOW;
			/*
			 * Ax Workaround to do PJ DFC with DDR DFC to
			 * avoid incorrect update of MM/ACLK divider update
			 * by DDR DFC.
			 */
			change |= MMP3_FREQCH_CORE | pmu->ddrdfc_trigger;
			logentry->marker |= TRACE_DFC_MARKER_DRAM;
			logentry->pp_dram = (unsigned char) pmu->pp_curr;
			pmu->trigge_stat[MMP3_CLK_DDR_1]++;
		} else
			pl->dram.val = curpl.dram.val;
	}

	if (change != 0) {
		/* change required */
		mmp3_update_freq_plan(pmu, pl);

		/* Update DDR register table*/
		if (change & MMP3_FREQCH_DRAM)
			mmp3_udpate_ddr_parameter_table(pmu, pl);

	} else {
		/* no change, need to clear PJ_RD_STATUS */
		mmp3_clear_PJ_RD_STATUS(pmu);
	}

	return change;
}


static void mmp3_dfc_trigger(struct mmp3_pmu *pmu, struct mmp3_freq_plan *pl,
	union trace_dfc_log *logentry, u32 change, u32 *time, u32*same)
{
	unsigned long flags;
	u32 val, samex, timex;

	if (change == 0)
		return;

	/* compose trigger val */
	val = __raw_readl(pmu->cc);
	val = MMP3_PROTECT_CC(val); /* set reserved */
	val = val | change | (MMP3_FREQCH_VOLTING);

	spin_lock_irqsave(&(pmu->mmp3_fc_spin), flags);
	timex = read_timestamp();

	dsb();
	/* trigger change*/
	__raw_writel(val, pmu->cc);

	timex = read_timestamp() - timex;
	spin_unlock_irqrestore(&(pmu->mmp3_fc_spin), flags);

	/* done, PJ_RD_STATUS should have been cleared by HW*/

	/* now read back current frequency settings, for query*/
	mmp3_get_freq_plan(pmu, &pmu->pl_curr, true);

	samex = mmp3_compare_freq_plan(&pmu->pl_curr, pl);

	if (samex != 0)
		logentry->marker |= TRACE_DFC_MARKER_ERR;

	trace_dfc(logentry);

	if (time != NULL)
		*time = timex;
	if (same != NULL)
		*same = samex;

	/* done, PJ_RD_STATUS should have been cleared by HW*/
}

static void mmp3_dfc_postchange(struct mmp3_pmu *pmu, struct mmp3_freq_plan *pl,
	union trace_dfc_log *logentry, u32 time, u32 same)
{
	if (same != 0) {
		/* usually should not be here, log a message here */
		pr_err("<PM> DFC result is not the requested one\n");
		mmp3_freq_plan_print_info(pmu, &pmu->pl_curr, time);
	} else {
		mmp3_freq_plan_print_dbg(pmu, &pmu->pl_curr, time);
	}
}

struct dfc_schedule {
	void (*trigger)(struct mmp3_pmu *pmu, struct mmp3_freq_plan *pl,
			union trace_dfc_log *logentry, u32 change,
			u32 *time, u32 *same);
	struct mmp3_pmu *pmu;
	struct mmp3_freq_plan *pl;
	union trace_dfc_log *logentry;
	u32 change;
	u32 time;
	u32 same;
	struct completion comp;
};

static struct dfc_schedule *dfcsch;
static int dfcsch_active;
static DEFINE_SPINLOCK(dfcsch_lock);
static void __maybe_unused trigger_scheduler(u32 flag)
{
	unsigned long savedflags;

	spin_lock_irqsave(&dfcsch_lock, savedflags);
	if (flag == 0x80000000)
		dfcsch_active = 0;
	else
		dfcsch_active = 1;

	if (dfcsch != NULL) {
		dfcsch->trigger(dfcsch->pmu, dfcsch->pl, dfcsch->logentry,
				dfcsch->change, &dfcsch->time, &dfcsch->same);
		complete(&dfcsch->comp);
		dfcsch = NULL;
	}
	spin_unlock_irqrestore(&dfcsch_lock, savedflags);
}

static void mmp3_schedule_freqch_loose(struct mmp3_pmu *pmu,
			struct mmp3_freq_plan *pl)
{
	unsigned long savedflags;
	union trace_dfc_log logentry;
	u32 change, time, same;

	change = mmp3_prepare_freqch(pmu, pl, &logentry);
	if (change == 0)
		return;

	time = 0;
	same = 0;
	__raw_writel(1, pmu->swstat);
	mmp3_smp_notify_dfc(MMP3_FREQCH_START);

	spin_lock_irqsave(&dfcsch_lock, savedflags);
	if (dfcsch_active && (pl->dram.op == MMP3_FREQ_OP_UPDATE)) {
		struct dfc_schedule sch;
		int ret;
		sch.trigger = mmp3_dfc_trigger;
		sch.pmu = pmu;
		sch.pl = pl;
		sch.logentry = &logentry;
		sch.change = change;
		sch.time = 0;
		sch.same = 0;
		init_completion(&sch.comp);
		dfcsch = &sch;
		spin_unlock_irqrestore(&dfcsch_lock, savedflags);
		ret = wait_for_completion_timeout(&sch.comp,
						msecs_to_jiffies(500));
		time = sch.time;
		same = sch.same;
		if (ret <= 0) {
			/* timeout, which indicates that the trigger
			 * is not triggering any more
			 */
			spin_lock_irqsave(&dfcsch_lock, savedflags);
			if (dfcsch != NULL) {
				dfcsch_active = 0;
				dfcsch = NULL;
				mmp3_dfc_trigger(pmu, pl, &logentry,
							change, &time, &same);
			}
			spin_unlock_irqrestore(&dfcsch_lock, savedflags);
		}
	} else {
		mmp3_dfc_trigger(pmu, pl, &logentry, change, &time, &same);
		spin_unlock_irqrestore(&dfcsch_lock, savedflags);
	}

	mmp3_dfc_postchange(pmu, pl, &logentry, time, same);

	__raw_writel(0, pmu->swstat);
	mmp3_smp_notify_dfc(MMP3_FREQCH_COMPLETE);
}

static int mmp3_pp_find_bound(struct mmp3_pmu *pmu, int id, u32 khz)
{
	int i;
	for (i = 0; i < pmu->ppscnt; i++) {
		if (pmu->pps[i].khz[id] >= khz)
			return i;
	}
	return -1;
}

static int mmp3_find_lowbar(struct mmp3_pmu *pmu, int start, int end)
{
	int i, lowbar;
	lowbar = 0;
	for (i = start; i <= end; i++) {
		if (lowbar < pmu->pp_lowbar[i])
			lowbar = pmu->pp_lowbar[i];
	}
	return lowbar;
}

/*
 * This function implements the modle of change to lowest PP that can
 * satisfy the request from core, ddr and axi.
 * Lowbar means the low point of PP that meets everyone's reqirement
 */
static void mmp3_clk_setrate_lowbar(struct mmp3_pmu *pmu, int id, u32 khz)
{
	struct mmp3_freq_plan pl;
	int ppidx;

	ppidx = mmp3_pp_find_bound(pmu, id, khz);
	if (ppidx < 0)
		return;

	mutex_lock(&(pmu->mmp3_fc_lock));
	pmu->pp_lowbar[id] = ppidx;
	ppidx = mmp3_find_lowbar(pmu, 0, ARRAY_SIZE(pmu->pp_lowbar) - 1);
	if (pmu->pp_curr != ppidx) {
		pmu->pp_curr = ppidx;
		memcpy(&pl, &pmu->pps[ppidx], sizeof(pl));
		/*
		 * operation default to MMP3_FREQ_OP_GET
		 * here assume every change, mmp3_do_freqch will omit
		 * unnecessary part
		 */
		pl.core.op = MMP3_FREQ_OP_UPDATE;
		pl.dram.op = MMP3_FREQ_OP_UPDATE;
		pl.axi.op = MMP3_FREQ_OP_UPDATE;
		mmp3_schedule_freqch_loose(pmu, &pl);
	}
	mutex_unlock(&(pmu->mmp3_fc_lock));
}

/*
 * This function implements the modle of change core, ddr and axi frequency
 * indenpendently. Lowbar modle still utilized in each domain, since in one
 * domain, all individuals share the same clock source. For example when change
 * CPU clock, the choose from the part of the PP table of the lowest one
 * that can meets every core's requirements.
 */
static void mmp3_clk_setrate_localized(struct mmp3_pmu *pmu, int id, u32 khz)
{
	struct mmp3_freq_plan pl;
	int ppidx;

	ppidx = mmp3_pp_find_bound(pmu, id, khz);
	if (ppidx < 0)
		return;

	mutex_lock(&(pmu->mmp3_fc_lock));
	pmu->pp_lowbar[id] = ppidx;

	if (id <= MMP3_CLK_ACLK) {
		/* change MP */
		ppidx = mmp3_find_lowbar(pmu, MMP3_CLK_MP1, MMP3_CLK_ACLK);
		pmu->pp_curr = ppidx;
		memcpy(&pl, &pmu->pps[ppidx], sizeof(pl));
		pl.core.op = MMP3_FREQ_OP_UPDATE;
	} else if (id <= MMP3_CLK_DDR_2) {
		/* change DDR */
		ppidx = mmp3_find_lowbar(pmu, MMP3_CLK_DDR_1, MMP3_CLK_DDR_2);
		pmu->pp_curr = ppidx;
		memcpy(&pl, &pmu->pps[ppidx], sizeof(pl));
		pl.dram.op = MMP3_FREQ_OP_UPDATE;
	} else if (id <= MMP3_CLK_AXI_2) {
		/* change AXI */
		ppidx = mmp3_find_lowbar(pmu, MMP3_CLK_AXI_1, MMP3_CLK_AXI_2);
		pmu->pp_curr = ppidx;
		memcpy(&pl, &pmu->pps[ppidx], sizeof(pl));
		pl.axi.op = MMP3_FREQ_OP_UPDATE;
	}
	mmp3_schedule_freqch_loose(pmu, &pl);
	mutex_unlock(&(pmu->mmp3_fc_lock));
}

/*
 *    This function is not currently used by anyone, but it is an interface
 * that can trigger combined CORE/DDR/AXI change to a PP. It is now mainly for
 * debugging and testing purpose. In the future it might be used in some
 * special scenario where we need to quickly switch to a certain PP in kernel
 * directly without going through upper level policy makers.
 */
void mmp3_apply_pp(int ppidx)
{
	struct mmp3_freq_plan pl;
	struct mmp3_pmu *pmu = &mmp3_pmu_config;
	if (ppidx < pmu->ppscnt) {
		memcpy(&pl, &pmu->pps[ppidx], sizeof(pl));
		pl.core.op = MMP3_FREQ_OP_UPDATE;
		pl.dram.op = MMP3_FREQ_OP_UPDATE;
		pl.axi.op = MMP3_FREQ_OP_UPDATE;
		mutex_lock(&(pmu->mmp3_fc_lock));
		mmp3_schedule_freqch_loose(pmu, &pl);
		mutex_unlock(&(pmu->mmp3_fc_lock));
	}
}
EXPORT_SYMBOL_GPL(mmp3_apply_pp);

void mmp3_setfreq(int clkid, unsigned long khz)
{
	struct mmp3_pmu *pmu = &mmp3_pmu_config;
	if (clkid < ARRAY_SIZE(pmu->pps[0].khz)) {
		if (pmu->setrate)
			pmu->setrate(pmu, clkid, khz);
	}
}
EXPORT_SYMBOL_GPL(mmp3_setfreq);

unsigned long mmp3_getfreq(int clkid)
{
	struct mmp3_pmu *pmu = &mmp3_pmu_config;
	if (clkid < ARRAY_SIZE(pmu->pps[0].khz))
		return pmu->pl_curr.khz[clkid];
	return 0;
}
EXPORT_SYMBOL_GPL(mmp3_getfreq);

int mmp3_get_pp_number(void)
{
	struct mmp3_pmu *pmu = &mmp3_pmu_config;
	return pmu->ppscnt;
}
EXPORT_SYMBOL_GPL(mmp3_get_pp_number);

unsigned long mmp3_get_pp_freq(int ppidx, int clkid)
{
	struct mmp3_pmu *pmu = &mmp3_pmu_config;
	if ((ppidx >= pmu->ppscnt) ||
		(clkid >= ARRAY_SIZE(pmu->pps[ppidx].khz)))
		return 0;
	return pmu->pps[ppidx].khz[clkid];
}
EXPORT_SYMBOL_GPL(mmp3_get_pp_freq);

/* Low Power IDLE related function */
static inline void mmp3_mod_idle_config(struct mmp3_cpu_idle_config *cic,
			u32 target_value) {
	u32 the_value;
	the_value = readl(cic->idle_config);
	the_value &= cic->idle_config_keep_mask;
	the_value |= (target_value & cic->idle_config_valid_mask);
	writel(the_value, cic->idle_config);
}

#define MMP3_IDLE_CHECK_DVM_EVENTS
#define MMP3_PM_C1_INCG 0x0
#define MMP3_PM_C1_EXCG 0x2
#define MMP3_PM_C2_LPPD 0x62
#define MMP3_PM_C2_MPPD 0x72

void mmp3_pm_enter_idle(int cpu)
{
	u32 state = MMP3_PM_C1_INCG;
	struct mmp3_cpu_idle_config *cic;

	cic = &(mmp3_percpu[mmp3_smpid()].cic);

	mmp3_mod_idle_config(cic, state);
	/* no DFC in process*/
	trace_idle_entry(state);

	/* Still need to check DFC flag for case where IPI call will not
	 * be processed, for example hotplug.
	 */
	if (likely(__raw_readl(cic->freq_status) == 0)) {
#if defined(CONFIG_SMP) && defined(MMP3_IDLE_CHECK_DVM_EVENTS)
		while (1) {
			__asm__ __volatile__ ("wfi");
			if ((__raw_readl(cic->freq_status) != 0) ||
#ifdef CONFIG_ARM_GIC
			((__raw_readl(cic->intr_status) & 0x3ff) != 1023) ||
#endif
			((__raw_readl(cic->wake_status) & 0x7fffc7ff) != 0)) {
				/* real wake up, break loop to handle*/
				break;
			}
		}
#else
		arch_idle();
#endif
	}

	trace_idle_exit(__raw_readl(cic->wake_status));
}

void mmp3_pm_enter_c2(int cpu)
{
	u32 state = MMP3_PM_C2_LPPD;
	struct mmp3_cpu_idle_config *cic;
	int core_id = mmp3_smpid();

	cic = &(mmp3_percpu[core_id].cic);

	__raw_writel(readl(APMU_ISLD_CPUMC_PDWN_CTRL) & ~(0x1 << 6),
			 APMU_ISLD_CPUMC_PDWN_CTRL);

	if (likely(__raw_readl(cic->freq_status) == 0)) {

		mmp3_mod_idle_config(cic, state);
		/* no DFC in process*/
		trace_idle_entry(state);

#if defined(CONFIG_SMP) && defined(MMP3_IDLE_CHECK_DVM_EVENTS)
		while (1) {
			flush_cache_all();
			dsb();
			__asm__ __volatile__ ("wfi");

			if ((__raw_readl(cic->freq_status) != 0) ||
#ifdef CONFIG_ARM_GIC
			((__raw_readl(cic->intr_status) & 0x3ff) != 1023) ||
#endif
			((__raw_readl(cic->wake_status) & 0x7fffc7ff) != 0)) {
				/* real wake up, break loop to handle*/
				break;
			}
		}
#else
		flush_cache_all();
		dsb();
		arch_idle();
#endif
		trace_idle_exit(__raw_readl(cic->wake_status));
	}
	__raw_writel(readl(APMU_ISLD_CPUMC_PDWN_CTRL) | (0x1 << 6),
			APMU_ISLD_CPUMC_PDWN_CTRL);
}


static void mmp3_do_idle(void)
{
	if (!need_resched())
		mmp3_pm_enter_idle(smp_processor_id());
	local_irq_enable();
}

static int __init mmp3_pm_init(void)
{
	int i, j;
	u32 ddrt;
	struct mmp3_pmu *pmu = &mmp3_pmu_config;

	pmu->swstat_store = 0;
	pmu->swstat = &(pmu->swstat_store);
	spin_lock_init(&(pmu->mmp3_fc_spin));
	mutex_init(&(pmu->mmp3_fc_lock));

	/*
	 * wake from IPI -- It's the workaround of MMP3 A0,
	 * suppose it'll be fixed on B0. Need remove it then
	 */
	__raw_writel(0x00004838, APMU_DEBUG2);
	__raw_writel(0x00000200, APMU_DEBUG);

	pm_idle = mmp3_do_idle;

	/* vote SPSD*/
	__raw_writel(__raw_readl(MPMU_APCR) | (1u << 28), MPMU_APCR);

	/* fix DDR settings according to DDR module */
	ddrt = __raw_readl(FIXADDR(pmu->dmcu[0], DMCU_SDRAM_CTRL4))
					& DMCU_SDRAM_TYPE_MASK;
	if (ddrt == DMCU_SDRAM_TYPE_DDR3) {
		mmp3_pps[MMP3_PP_100].dram.dsrc = 1;
		mmp3_pps[MMP3_PP_200].dram.dsrc = 1;
		mmp3_pps[MMP3_PP_400].dram.dsrc = 1;
		mmp3_pps[MMP3_PP_800].dram.dsrc = 3;
		mmp3_pps[MMP3_PP_1066].dram.dsrc = 3;
	}

	if ((__raw_readl(pmu->mc_interleave) & (0x3f << 16)) == 0) {
#if MMP3_TEST_PP
		mmp3_pps[MMP3_PP_26].dram.ch2_d = 0;
#endif
		pmu->ddrdfc_trigger = MMP3_FREQCH_DRAM_CH1;
	} else {
		pmu->ddrdfc_trigger = MMP3_FREQCH_DRAM;
	}

	for (i = 0; i < pmu->ppscnt; i++)
		mmp3_freq_plan_cal(pmu, &(pmu->pps[i]));

	/* process same speed, assume khz goes up only and real diff is big */
	for (j = 0; j < ARRAY_SIZE(pmu->pps[0].khz); j++) {
		u32 curkhz = pmu->pps[0].khz[j];
		for (i = 1; i < pmu->ppscnt; i++) {
			if (curkhz == pmu->pps[i].khz[j])
				pmu->pps[i].khz[j] += i;
			else
				curkhz = pmu->pps[i].khz[j];
		}
	}

	if (0)
		pmu->setrate = mmp3_clk_setrate_lowbar;
	else
		pmu->setrate = mmp3_clk_setrate_localized;

	mmp3_get_freq_plan(pmu, &pmu->pl_curr, true);
	pmu->pl_curr.core.op = MMP3_FREQ_OP_SHOW;
	pmu->pl_curr.dram.op = MMP3_FREQ_OP_SHOW;
	pmu->pl_curr.axi.op = MMP3_FREQ_OP_SHOW;
	mmp3_freq_plan_print_info(pmu, &pmu->pl_curr, 0);

	dfcsch = NULL;
	dfcsch_active = 0;

	return 0;
}

core_initcall(mmp3_pm_init);
