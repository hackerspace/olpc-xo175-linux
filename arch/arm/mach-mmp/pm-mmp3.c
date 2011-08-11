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
	MMP3_FREQCH_DRAM	= (9u << 22), /* both channel*/
	MMP3_FREQCH_AXI		= (1u << 26),

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
	u32 mhz[MMP3_CLK_TOP_NUM];
};

struct mmp3_pmu {
	u32 *fccr;
	u32 *cc;
	u32 *cc2;
	u32 *cc3;
	u32 *bus;
	u32 *dm_cc;
	u32 *dm2_cc;
	u32 *swstat;

	u32 swstat_store;
	spinlock_t mmp3_fc_spin;
	struct mutex mmp3_fc_lock;
	struct mmp3_clock_source *sources;
	struct mmp3_freq_plan *pps;
	int ppscnt;
	void (*setrate)(struct mmp3_pmu *pmu, int id, u32 mhz);
	u32 pp_lowbar[MMP3_CLK_TOP_NUM];
	int pp_curr;
	struct mmp3_freq_plan pl_curr;
};

static struct mmp3_clock_source mmp3_fc_sources[] = {
	[0] = {
		.name = "PLL1/2",
		.frequency = 399,
	},
	[1] = {
		.name = "PLL1",
		.frequency = 797,
	},
	[2] = {
		.name = "PLL2",
		.frequency = 1334,
	},
	[3] = {
		.name = "PLL1_CLKOUTP",
		.frequency = 1063,
	},
	[4] = {
		.name = "VCXO",
		.frequency = 26,
	},
};

enum {
	MMP3_PP_ULPA = 0,
	MMP3_PP_FOOT,
	MMP3_PP_SIDE,
	MMP3_PP_PEAK,
	MMP3_PP_NUM,
};
static struct mmp3_freq_plan mmp3_pps[MMP3_PP_NUM] = {
	/* DDR settings will be fixed upon init according to DDR module used */
	[MMP3_PP_ULPA] = {
		.name = "ULPA",
		.core = {{MMP3_FREQ_OP_GET, 0, 1, 0, 0, 0, 0, 1, 1},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 1, 3},},
	},
	[MMP3_PP_FOOT] = {
		.name = "FOOT",
		.core = {{MMP3_FREQ_OP_GET, 1, 1, 0, 0, 0, 0, 1, 1},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 0, 1},},
	},
	[MMP3_PP_SIDE] = {
		.name = "SIDE",
		.core = {{MMP3_FREQ_OP_GET, 1, 0, 0, 0, 1, 1, 1, 1},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 0, 1},},
	},
	[MMP3_PP_PEAK] = {
		.name = "PEAK",
		.core = {{MMP3_FREQ_OP_GET, 3, 0, 0, 0, 1, 1, 2, 2},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 0, 1},},
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
			.idle_config_keep_mask = 0xb6305c01,
			.idle_config = MK_APMU_REG_VADDR(0x18),
			.wake_status = WAKE_STAT,
			.intr_status = INTR_STAT,
			.freq_status = &(mmp3_pmu_config.swstat_store),
		},
	},
	[1] = {
		.cic = {
			.idle_config_valid_mask = 0x00000062,
			.idle_config_keep_mask = 0x36105801,
			.idle_config = MK_APMU_REG_VADDR(0x200),
			.wake_status = WAKE_STAT,
			.intr_status = INTR_STAT,
			.freq_status = &(mmp3_pmu_config.swstat_store),
		},
	},
	[2] = {
		.cic = {
			.idle_config_valid_mask = 0x00000062,
			.idle_config_keep_mask = 0x36105801,
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
	pl->mhz[MMP3_CLK_MP1] = basefreq / FIELD2DIV(pl->core.mp1_d);
	pl->mhz[MMP3_CLK_MP2] = basefreq / FIELD2DIV(pl->core.mp2_d);
	pl->mhz[MMP3_CLK_MM] = basefreq / FIELD2DIV(pl->core.mm_d);
	pl->mhz[MMP3_CLK_ACLK] = basefreq / FIELD2DIV(pl->core.aclk_d);

	basefreq = pmu->sources[pl->dram.dsrc].frequency / 2;
	pl->mhz[MMP3_CLK_DDR_1] = basefreq / FIELD2DIV(pl->dram.ch1_d);
	pl->mhz[MMP3_CLK_DDR_2] = basefreq / FIELD2DIV(pl->dram.ch1_d);

	basefreq = pmu->sources[pl->axi.asrc].frequency;
	pl->mhz[MMP3_CLK_AXI_1] = basefreq / FIELD2DIV(pl->axi.aclk1_d);
	pl->mhz[MMP3_CLK_AXI_2] = basefreq / FIELD2DIV(pl->axi.aclk2_d);
}

static void mmp3_freq_plan_print(struct mmp3_pmu *pmu,
					struct mmp3_freq_plan *pl, char * tag)
{
	int proid = mmp3_smpid();
	if (pl->core.op == MMP3_FREQ_OP_SHOW) {
		pm_print("%s <PM>:FROM[%d]:CPU: SRC[%12s]|"
				"MP1:%4d|MP2:%4d|MM:%4d\n"
			, tag
			, proid
			, pmu->sources[pl->core.psrc].name
			, pl->mhz[MMP3_CLK_MP1]
			, pl->mhz[MMP3_CLK_MP2]
			, pl->mhz[MMP3_CLK_MM]
			);
	}
	if (pl->dram.op == MMP3_FREQ_OP_SHOW) {
		pm_print("%s <PM>:FROM[%d]:DDR: SRC[%12s]|"
				"CH1:%4d|CH2:%4d\n"
			, tag
			, proid
			, pmu->sources[pl->dram.dsrc].name
			, pl->mhz[MMP3_CLK_DDR_1]
			, pl->mhz[MMP3_CLK_DDR_2]
			);
	}
	if (pl->axi.op == MMP3_FREQ_OP_SHOW) {
		pm_print("%s <PM>:FROM[%d]:AXI: SRC[%12s]|"
				"ACLK1:%4d|ACLK2:%4d\n"
			, tag
			, proid
			, pmu->sources[pl->axi.asrc].name
			, pl->mhz[MMP3_CLK_AXI_1]
			, pl->mhz[MMP3_CLK_AXI_2]
			);
	}
}
#define mmp3_freq_plan_print_info(pmu, pl) \
	mmp3_freq_plan_print(pmu, pl, KERN_INFO)
#define mmp3_freq_plan_print_dbg(pmu, pl) \
	mmp3_freq_plan_print(pmu, pl, KERN_DEBUG)

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
	if ((EXTRACTDIV(p1->core.val) != EXTRACTDIV(p2->core.val)))
		mark |= TRACE_DFC_MARKER_CORE;
	if ((EXTRACTDIV(p1->dram.val) != EXTRACTDIV(p2->dram.val)))
		mark |= TRACE_DFC_MARKER_DRAM;
	if ((EXTRACTDIV(p1->axi.val) != EXTRACTDIV(p2->axi.val)))
		mark |= TRACE_DFC_MARKER_AXI;
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

static void mmp3_udpate_ddr_parameter_table(struct mmp3_pmu *pmu,
			struct mmp3_freq_plan *pl)
{
	/* TODO: add DDR parameter table programming here*/
}

static void mmp3_do_freqch_loose(struct mmp3_pmu *pmu,
			struct mmp3_freq_plan *pl)
{
	unsigned long flags;
	u32 val, change, retry, same;
	union trace_dfc_log logentry;
	struct mmp3_freq_plan curpl;

	spin_lock_irqsave(&(pmu->mmp3_fc_spin), flags);

	/* obtain FC onwership, should always pass */
	retry = 1000;
	while ((__raw_readl(pmu->dm_cc) & (1u << 24)) != 0) {
		retry--;
		if (retry <= 0) {
			spin_unlock_irqrestore(&(pmu->mmp3_fc_spin), flags);
			pr_err("<PM> Cannot gain owner of PMU DFC\n");
			return;
		}
	}

	__raw_writel(1, pmu->swstat);

	/* now get old configuration to see if change is necessary */
	memcpy(&curpl, pl, sizeof(curpl));
	mmp3_get_freq_plan(pmu, &curpl, false);

	logentry.marker = 0;
	same = 0;
	change = 0;
	pmu->pl_curr.core.op = MMP3_FREQ_OP_GET;
	pmu->pl_curr.dram.op = MMP3_FREQ_OP_GET;
	pmu->pl_curr.axi.op = MMP3_FREQ_OP_GET;
	if (curpl.core.val != pl->core.val) {
		if (pl->core.op == MMP3_FREQ_OP_UPDATE) {
			pmu->pl_curr.core.op = MMP3_FREQ_OP_SHOW;
			change |= MMP3_FREQCH_CORE;
			logentry.marker |= TRACE_DFC_MARKER_CORE;
			logentry.pp_core = (unsigned char) pmu->pp_curr;
		} else
			pl->core.val = curpl.core.val;
	}
	if (curpl.axi.val != pl->axi.val) {
		if (pl->axi.op == MMP3_FREQ_OP_UPDATE) {
			pmu->pl_curr.axi.op = MMP3_FREQ_OP_SHOW;
			change |= MMP3_FREQCH_AXI;
			logentry.marker |= TRACE_DFC_MARKER_AXI;
			logentry.pp_axi = (unsigned char) pmu->pp_curr;
		} else
			pl->axi.val = curpl.axi.val;
	}
	if (curpl.dram.val != pl->dram.val) {
		if (pl->dram.op == MMP3_FREQ_OP_UPDATE) {
			pmu->pl_curr.dram.op = MMP3_FREQ_OP_SHOW;
			change |= MMP3_FREQCH_DRAM;
			logentry.marker |= TRACE_DFC_MARKER_DRAM;
			logentry.pp_dram = (unsigned char) pmu->pp_curr;
		} else
			pl->dram.val = curpl.dram.val;
	}

	if (change != 0) {
		/* change required */
		mmp3_update_freq_plan(pmu, pl);
		val = __raw_readl(pmu->cc);
		val = MMP3_PROTECT_CC(val); /* set reserved */
		val = val | change | (MMP3_FREQCH_VOLTING);
		if (val & MMP3_FREQCH_DRAM) {
			/* DDR change, need to program DLL update sequence*/
			mmp3_udpate_ddr_parameter_table(pmu, pl);
		}

		/* A0 need to all cores run, we use coherent broadcasts*/
		dsb();

		/* trigger change*/
		__raw_writel(val, pmu->cc);
		/* done, PJ_RD_STATUS should have been cleared by HW*/

		/* now read back current frequency settings, for query*/
		mmp3_get_freq_plan(pmu, &pmu->pl_curr, true);

		same = mmp3_compare_freq_plan(&pmu->pl_curr, pl);

		if (same != 0)
			logentry.marker |= TRACE_DFC_MARKER_ERR;

		trace_dfc(&logentry);

	} else {
		/* no change, need to clear PJ_RD_STATUS */
		mmp3_clear_PJ_RD_STATUS(pmu);
	}

	__raw_writel(0, pmu->swstat);
	spin_unlock_irqrestore(&(pmu->mmp3_fc_spin), flags);

	if (change != 0) {
		if (same != 0) {
			/* usually should not be here, log a message here */
			pr_err("<PM> DFC result is not the requested one\n");
			mmp3_freq_plan_print_info(pmu, &pmu->pl_curr);
		} else
			mmp3_freq_plan_print_dbg(pmu, &pmu->pl_curr);
	}
}

static int mmp3_pp_find_bound(struct mmp3_pmu *pmu, int id, u32 mhz)
{
	int i;
	for (i = 0; i < pmu->ppscnt; i++) {
		if (pmu->pps[i].mhz[id] >= mhz)
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
static void mmp3_clk_setrate_lowbar(struct mmp3_pmu *pmu, int id, u32 mhz)
{
	struct mmp3_freq_plan pl;
	int ppidx;

	ppidx = mmp3_pp_find_bound(pmu, id, mhz);
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
		mmp3_do_freqch_loose(pmu, &pl);
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
static void mmp3_clk_setrate_localized(struct mmp3_pmu *pmu, int id, u32 mhz)
{
	struct mmp3_freq_plan pl;
	int ppidx;

	ppidx = mmp3_pp_find_bound(pmu, id, mhz);
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
	mmp3_do_freqch_loose(pmu, &pl);
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
		mmp3_do_freqch_loose(pmu, &pl);
		mutex_unlock(&(pmu->mmp3_fc_lock));
	}
}
EXPORT_SYMBOL_GPL(mmp3_apply_pp);

void mmp3_setfreq(int clkid, unsigned long khz)
{
	struct mmp3_pmu *pmu = &mmp3_pmu_config;
	if (clkid < ARRAY_SIZE(pmu->pps[0].mhz)) {
		if (pmu->setrate) {
			/* internal interface use mhz, need to covert */
			pmu->setrate(pmu, clkid, khz / 1000);
		}
	}
}
EXPORT_SYMBOL_GPL(mmp3_setfreq);

unsigned long mmp3_getfreq(int clkid)
{
	struct mmp3_pmu *pmu = &mmp3_pmu_config;
	if (clkid < ARRAY_SIZE(pmu->pps[0].mhz)) {
		/* internal use mhz while interface is khz */
		return pmu->pl_curr.mhz[clkid] * 1000;
	}
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
		(clkid >= ARRAY_SIZE(pmu->pps[ppidx].mhz)))
		return 0;
	return pmu->pps[ppidx].mhz[clkid] * 1000;
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

	if (likely(__raw_readl(cic->freq_status) == 0)) {

		mmp3_mod_idle_config(cic, state);
		/* no DFC in process*/
		trace_idle_entry(state);

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
		trace_idle_exit(__raw_readl(cic->wake_status));
	}
}


static void mmp3_do_idle(void)
{
	if (!need_resched())
		mmp3_pm_enter_idle(smp_processor_id());
	local_irq_enable();
}

static int __init mmp3_pm_init(void)
{
	int i;
	struct mmp3_pmu *pmu = &mmp3_pmu_config;

	pmu->swstat_store = 0;
	pmu->swstat = &(pmu->swstat_store);
	spin_lock_init(&(pmu->mmp3_fc_spin));
	mutex_init(&(pmu->mmp3_fc_lock));

	pm_idle = mmp3_do_idle;

	/* TODO: fix DDR settings according to DDR module */

	for (i = 0; i < pmu->ppscnt; i++)
		mmp3_freq_plan_cal(pmu, &(pmu->pps[i]));

	if (0)
		pmu->setrate = mmp3_clk_setrate_lowbar;
	else
		pmu->setrate = mmp3_clk_setrate_localized;

	mmp3_get_freq_plan(pmu, &pmu->pl_curr, true);
	pmu->pl_curr.core.op = MMP3_FREQ_OP_SHOW;
	pmu->pl_curr.dram.op = MMP3_FREQ_OP_SHOW;
	pmu->pl_curr.axi.op = MMP3_FREQ_OP_SHOW;
	mmp3_freq_plan_print_info(pmu, &pmu->pl_curr);

	return 0;
}

core_initcall(mmp3_pm_init);
