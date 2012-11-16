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
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clocksource.h>
#include <linux/time.h>
#include <linux/cpu.h>
#include <plat/clock.h>
#include <asm/mach/map.h>
#include <asm/proc-fns.h>
#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include <mach/addr-map.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-ciu.h>
#include <mach/regs-icu.h>
#include <mach/regs-mcu.h>
#include <mach/system.h>
#ifdef CONFIG_TRACEPOINTS
#define CREATE_TRACE_POINTS
#endif
#include <mach/mmp3_pm.h>
#include <mach/smp.h>
#include <mach/mmp_cm.h>
#include <linux/fb.h>

static DEFINE_SPINLOCK(dfcsch_lock);
atomic_t mmp3_fb_is_suspended = ATOMIC_INIT(0);

static inline int mmp3_smpid(void)
{
#ifdef CONFIG_CORE_MORPHING
	/*
	 * after set morph bit, mm core will return back core id
	 * is 0 from CP15 CPUID register; so need check core
	 * morphing's module and get the active id number.
	 *
	 * if core is mm core, the need force to return core id = 2.
	 * so idle functions can set idle_cfg and cc4 registers for
	 * mm core correctly.
	 */
	if (cm_get_active_core_id() == MMP_CM_CPU_ID_MM)
		return 2;
#endif

	return hard_smp_processor_id();
}

#ifdef CONFIG_SMP
enum ipi_msg_type {
	IPI_TIMER = 2,
	IPI_RESCHEDULE,
	IPI_CALL_FUNC,
	IPI_CALL_FUNC_SINGLE,
	IPI_CPU_STOP,
	IPI_CPU_BACKTRACE,
	IPI_CPU_SYNC_COHERENCY,
};

#define MP1_COHT_STATE_OFFSET	0x0
#define MP2_COHT_STATE_OFFSET	0x4
#define MP1_HANDSHAKE_OFFSET	0x8
#define MP2_HANDSHAKE_OFFSET	0xc

#define SEM4_OFFSET		0x20

#define MP1_C1_CFG_OFFSET	0x100
#define MP1_C2_CFG_OFFSET	0x104
#define MP2_C1_CFG_OFFSET	0x108
#define MP2_C2_CFG_OFFSET	0x10c
#define MM_C1_CFG_OFFSET	0x110
#define MM_C2_CFG_OFFSET	0x114
#define CM_CID_OFFSET		0x118

#define INT_STATUS_ADDR_OFFSET	0x200
#define WAKE_STATUS_ADDR_OFFSET	0x204
#define SGIR_ADDR_OFFSET	0x208
#define APMU_BASE_ADDR_OFFSET	0x20c

#define CC_OFFSET		0x300
#define CC_ADDR_OFFSET		0x304
#define CC2_OFFSET		0x308
#define CC2_ADDR_OFFSET		0x30c
#define CC3_OFFSET		0x310
#define CC3_ADDR_OFFSET		0x314
#define BUS_OFFSET		0x318
#define BUS_ADDR_OFFSET		0x31c
#define FCCR_OFFSET		0x320
#define FCCR_ADDR_OFFSET	0x324

static u32 num_cpus;
static char *sync_buf;
static volatile int *coherent_state;

extern int mmp3_trigger_dfc_ll(u32 dfc_val, u32 buf_base);
extern int mmp3_trigger_dfc_ll_b0p(u32 dfc_val, u32 buf_base);
extern int mmp3_coherent_handler(u32 buf_base);
extern int mmp3_enter_c2(u32 buf_base);

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

static void set_sync_buf(u32 offset, u32 val)
{
	u32 *data;

	data = (u32 *)(sync_buf + offset);
	*data = val;
	return;
}

static u32 get_sync_buf(u32 offset)
{
	return *(u32 *)(sync_buf + offset);
}

void handle_coherency_maint_req(void *p)
{
	mmp3_coherent_handler((u32)sync_buf);
}

static void do_nothing(void *unused)
{
	int cpu = smp_processor_id();
	printk(KERN_INFO "%s: cpu %d\n", __func__, cpu);
}

static atomic_t glb_hp_lock = ATOMIC_INIT(0);

static int __cpuinit mmp3_pm_cpu_callback(struct notifier_block *nfb,
				     unsigned long action, void *hcpu)
{
	int err = 0;

	spin_lock(&dfcsch_lock);
	switch (action) {
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
		cm_vote_mp1();
		atomic_inc(&glb_hp_lock);
		smp_mb();
		break;
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
		while(!(readl(APMU_CORE_STATUS) & 0x200))
			nop();
		atomic_dec(&glb_hp_lock);
		smp_mb();
		break;
	case CPU_UP_CANCELED:
	case CPU_UP_CANCELED_FROZEN:
		atomic_dec(&glb_hp_lock);
		cm_cancel_vote_mp1();
		break;
#ifdef CONFIG_HOTPLUG_CPU
	case CPU_DOWN_PREPARE:
	case CPU_DOWN_PREPARE_FROZEN:
		atomic_inc(&glb_hp_lock);
		smp_mb();
		/* kick all the CPUs so that they exit out of pm_idle */
		smp_call_function(do_nothing, NULL, 1);
		break;
	case CPU_DOWN_FAILED:
	case CPU_DOWN_FAILED_FROZEN:
		atomic_dec(&glb_hp_lock);
		break;
	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		while(readl(APMU_CORE_STATUS) & 0x200)
			nop();
		atomic_dec(&glb_hp_lock);
		smp_mb();
		cm_cancel_vote_mp1();
		break;
#endif
	}
	spin_unlock(&dfcsch_lock);
	return notifier_from_errno(err);
}

static struct notifier_block __cpuinitdata mmp3_pm_cpu_notifier = {
	&mmp3_pm_cpu_callback, NULL, 0
};
#endif

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

};

#define FIELD2DIV(x) ((x) + 1)
#define GET_FIELD(val, shft, bits) (((val) >> (shft)) & ((1u << (bits)) - 1))
#define MAK_FIELD(val, shft, bits) (((val) & ((1u << (bits)) - 1)) << (shft))
#define MOD_FIELD(old, fld, shft, bits) \
	(((old) & ~(MAK_FIELD(-1, shft, bits))) | MAK_FIELD(fld, shft, bits))

/* FCCR 29-31 */
#define MMP3_FREQ_PSRC_SET(old, fld) MOD_FIELD(old, fld, 29, 3)
/* FCCR 29-31 */
#define MMP3_FREQ_PSRC_GET(val) GET_FIELD(val, 3, 3)
/* FCCR 23-25 */
#define MMP3_FREQ_DSRC_SET(old, fld) MOD_FIELD(old, fld, 23, 3)
/* FCCR 23-25 */
#define MMP3_FREQ_DSRC_GET(val) GET_FIELD(val, 6, 3)
/* BUS_CLKRST 6-8 */
#define MMP3_FREQ_ASRC_SET(old, fld) MOD_FIELD(old, fld, 6, 3)
/* BUS_CLKRST 6-8*/
#define MMP3_FREQ_ASRC_GET(val) GET_FIELD(val, 9, 3)
/* BUS_CLKRST 9-11 */
#define MMP3_FREQ_DSRC2_SET(old, fld) MOD_FIELD(old, fld, 9, 3)
/* BUS_CLKRST 9-11*/
#define MMP3_FREQ_DSRC2_GET(val) GET_FIELD(val, 9, 3)
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
/* DOUBLER_GENERIC_CTRL 2-3 */
#define MMP3_FREQ_DDR1_DBBYPASS_GET(val) GET_FIELD(val, 2, 1)
#define MMP3_FREQ_DDR2_DBBYPASS_GET(val) GET_FIELD(val, 3, 1)

#define MMP3_PROTECT_CC(x) (((x) & 0x0003fe3f) | 0x00bc0000)
#define MMP3_PROTECT_CC2(x) ((x) & 0xfffffe07)
#define MMP3_PROTECT_CC3(x) ((x) & 0x0effff1f)
#define MMP3_PROTECT_BUSCLKRST(x) ((x) & 0x0000ffc3)
#define MMP3_PROTECT_FCCR(x) ((x) & 0xff83ffff)

struct mmp3_fc_source {
	char *name;
	u32 frequency;
	struct clk *source;
};

struct mmp3_freq_plan {
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
			u32 mode4x:1;
			u32 pre_d:3;
			struct dmc_timing_entry *timing;
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
	u32 *dbc;
	u32 *db[2];
	u32 *bus;
	u32 *dm_cc;
	u32 *dm2_cc;
	u32 *pll_sel_status;
	u32 *mc_interleave;
	u32 *mc_slp_req;
	u32 *mc_par_ctrl;
	u32 *swstat;
	u32 *dmcu[2];

	u32 ddrdfc_trigger;
	u32 swstat_store;
	spinlock_t mmp3_fc_spin;
	struct mutex mmp3_fc_lock;
	struct mmp3_fc_source *sources;
	struct mmp3_freq_plan *pps;
	int ppscnt;
	void (*setrate)(struct mmp3_pmu *pmu, int id, u32 khz);
	u32 pp_lowbar[MMP3_CLK_TOP_NUM];
	int pp_curr;
	struct mmp3_freq_plan pl_curr;

	u32 trigge_stat[MMP3_CLK_TOP_NUM];
	u32 source_status[MMP3_CLK_TOP_NUM];
};

static struct mmp3_fc_source mmp3_fccs[] = {
	[0] = {
		.name = "pll1_d_2",
	},
	[1] = {
		.name = "pll1",
	},
	[2] = {
		.name = "pll2",
	},
	[3] = {
		.name = "pll1_clkoutp",
	},
	[4] = {
		.name = "vctcxo",
	},
};

static inline void mmp3_fccs_enable(int idx)
{
	clk_enable(mmp3_fccs[idx].source);
}

static inline void mmp3_fccs_disable(int idx)
{
	clk_disable(mmp3_fccs[idx].source);
}

static inline u32 mmp3_fccs_speed(int idx)
{
	return mmp3_fccs[idx].frequency;
}

#define MMP3_TEST_PP 0
#define MMP3_PP_TABLE_DIFF_BYIDX 0

static struct mmp3_freq_plan mmp3_pps[] = {
	/* DDR will be fixed upon platform init according to DDR module */
#if MMP3_TEST_PP
	{
		.core = {{MMP3_FREQ_OP_GET, 4, 0, 0, 0, 0, 0, 0, 0},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 7, 7},},	/* 50/50 */
	},
	{
		.core = {{MMP3_FREQ_OP_GET, 0, 7, 0, 0, 0, 0, 7, 7},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 5, 5},},	/* 66/66 */
	},
#endif
	{
		.core = {{MMP3_FREQ_OP_GET, 0, 3, 0, 0, 0, 0, 3, 3},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 3, 3},},	/* 100/100 */
	},
	{
		.core = {{MMP3_FREQ_OP_GET, 0, 1, 0, 0, 0, 0, 1, 1},},
		.axi  = {{MMP3_FREQ_OP_GET, 0, 1, 3},},	/* 200/100 */
	},
	{
		.core = {{MMP3_FREQ_OP_GET, 1, 1, 0, 0, 0, 0, 1, 1},},
		.axi  = {{MMP3_FREQ_OP_GET, 1, 2, 3},},	/* 266/200 */
	},
	{
		.core = {{MMP3_FREQ_OP_GET, 1, 0, 0, 0, 1, 1, 1, 1},},
		.axi  = {{MMP3_FREQ_OP_GET, 3, 2, 4},},	/* 354/213 */
	},
	{
		.core = {{MMP3_FREQ_OP_GET, 3, 0, 0, 0, 1, 1, 2, 2},},
		.axi  = {{MMP3_FREQ_OP_GET, 1, 1, 3},},	/* 400/200 */
	},
#if MMP3_TEST_PP
	{
		.core = {{MMP3_FREQ_OP_GET, 2, 0, 0, 0, 1, 1, 2, 2},},
		.axi  = {{MMP3_FREQ_OP_GET, 1, 1, 3},},	/* 400/200 */
	},
#endif
};

static struct mmp3_pmu mmp3_pmu_config = {
	.fccr = (u32 *)MPMU_FCCR,
	.cc = (u32 *)APMU_REG(0x4),
	.cc2 = (u32 *)APMU_REG(0x150),
	.cc3 = (u32 *)APMU_REG(0x188),
	.bus = (u32 *)APMU_REG(0x6c),
	.db = {
		[0] = (u32 *)APMU_REG(0x258),
		[1] = (u32 *)APMU_REG(0x25c),
	},
	.dbc = (u32 *)APMU_REG(0x260),
	.dm_cc = (u32 *)APMU_REG(0xc),
	.dm2_cc = (u32 *)APMU_REG(0x158),
	.pll_sel_status = (u32 *)APMU_REG(0xc4),
	.mc_interleave = (u32 *)CIU_REG(0xa0),
	.mc_slp_req = (u32 *)APMU_REG(0xb4),
	.mc_par_ctrl = (u32 *)APMU_REG(0x11c),
	.dmcu = {
		[0] = (u32 *)(DMCU_VIRT_BASE + 0x0),
		[1] = (u32 *)(DMCU_VIRT_BASE + 0x10000),
	},

	.sources = mmp3_fccs,
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
			.idle_config_keep_mask = 0x0deffc1d,
			.idle_config = MK_APMU_REG_VADDR(0x18),
			.wake_status = WAKE_STAT,
			.intr_status = INTR_STAT,
			.freq_status = &(mmp3_pmu_config.swstat_store),
		},
	},
	[1] = {
		.cic = {
			.idle_config_valid_mask = 0x00000062,
			.idle_config_keep_mask = 0x0deffc1d,
			.idle_config = MK_APMU_REG_VADDR(0x200),
			.wake_status = WAKE_STAT,
			.intr_status = INTR_STAT,
			.freq_status = &(mmp3_pmu_config.swstat_store),
		},
	},
	[2] = {
		.cic = {
			.idle_config_valid_mask = 0x00000062,
			.idle_config_keep_mask = 0x0deffc1d,
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

	basefreq = pmu->sources[pl->dram.dsrc].frequency;
	pl->khz[MMP3_CLK_DDR_1] = basefreq / FIELD2DIV(pl->dram.pre_d);
	pl->khz[MMP3_CLK_DDR_2] = basefreq / FIELD2DIV(pl->dram.pre_d);
	if (pl->dram.mode4x) {
		pl->khz[MMP3_CLK_DDR_1] *= 4;
		pl->khz[MMP3_CLK_DDR_2] *= 4;
	}

	basefreq = pmu->sources[pl->axi.asrc].frequency;
	pl->khz[MMP3_CLK_AXI_1] = basefreq / FIELD2DIV(pl->axi.aclk1_d);
	pl->khz[MMP3_CLK_AXI_2] = basefreq / FIELD2DIV(pl->axi.aclk2_d);
}

static void mmp3_freq_plan_print(struct mmp3_pmu *pmu,
	struct mmp3_freq_plan *pl, char * tag, u32 tmr)
{
	int proid = mmp3_smpid();
	tmr = cyc2us(tmr);
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
		pm_print("%s<PM:%d>:DDR@[%12s]|CH1:%7d (x%d mode)  |"
				"CH2:%7d (x%d mode)  |"
				"%2d us|%d\n"
			, tag
			, proid
			, pmu->sources[pl->dram.dsrc].name
			, pl->khz[MMP3_CLK_DDR_1]
			, (pl->dram.mode4x) ? 4 : 2
			, pl->khz[MMP3_CLK_DDR_2]
			, (pl->dram.mode4x) ? 4 : 2
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

static int mmp3_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *evdata = data;

	if ((event == FB_EVENT_SUSPEND) || ((event == FB_EVENT_BLANK) &&
			(*(int *)evdata->data != FB_BLANK_UNBLANK)))
		atomic_set(&mmp3_fb_is_suspended, 1);
	else if ((event == FB_EVENT_RESUME) || ((event == FB_EVENT_BLANK) &&
			(*(int *)evdata->data == FB_BLANK_UNBLANK)))
		atomic_set(&mmp3_fb_is_suspended, 0);

	return 0;
}

static struct notifier_block mmp3_fb_notif = {
	.notifier_call = mmp3_fb_notifier_callback,
};


static void mmp3_get_freq_plan(struct mmp3_pmu *pmu,
				struct mmp3_freq_plan *pl, bool crs)
{
	u32 val;
	/* pll_sel_status */
	val = __raw_readl(pmu->pll_sel_status);
	pl->core.psrc = MMP3_FREQ_PSRC_GET(val);
	pl->dram.dsrc = MMP3_FREQ_DSRC_GET(val);
	pl->axi.asrc = MMP3_FREQ_ASRC_GET(val);
	/* dm_cc */
	val = __raw_readl(pmu->dm_cc);
	pl->core.at_d = MMP3_FREQ_AT_GET(val);
	pl->core.pj_d = MMP3_FREQ_PJ_GET(val);
	pl->dram.pre_d = MMP3_FREQ_DDR1_GET(val);
	pl->axi.aclk1_d = MMP3_FREQ_AXI1_GET(val);
	/* dm2_cc */
	val = __raw_readl(pmu->dm2_cc);
	pl->core.ph_d = MMP3_FREQ_PH_GET(val);
	pl->core.aclk_d = MMP3_FREQ_ACLK_GET(val);
	pl->core.mm_d = MMP3_FREQ_MM_GET(val);
	pl->core.mp1_d = MMP3_FREQ_MP1_GET(val);
	pl->core.mp2_d = MMP3_FREQ_MP2_GET(val);
	pl->axi.aclk2_d = MMP3_FREQ_AXI2_GET(val);
	/* doubler */
	val = __raw_readl(pmu->dbc);
	pl->dram.mode4x = (MMP3_FREQ_DDR1_DBBYPASS_GET(val) != 0) ? 0 : 1;

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
		printk(KERN_WARNING "<PM> CORE not same: 0x%08x != 0x%08x\n",
			p1->core.val, p2->core.val);
		mark |= TRACE_DFC_MARKER_CORE;
	}
	if ((EXTRACTDIV(p1->dram.val) != EXTRACTDIV(p2->dram.val))) {
		printk(KERN_WARNING "<PM> DRAM not same: 0x%08x != 0x%08x\n",
			p1->dram.val, p2->dram.val);
		mark |= TRACE_DFC_MARKER_DRAM;
	}
	if ((EXTRACTDIV(p1->axi.val) != EXTRACTDIV(p2->axi.val))) {
		printk(KERN_WARNING "<PM> AXI not same: 0x%08x != 0x%08x\n",
			p1->axi.val, p2->axi.val);
		mark |= TRACE_DFC_MARKER_AXI;
	}
	return mark;
}

static void mmp3_dpll_off(struct mmp3_pmu *pmu, u32 idx)
{
	u32 regval;
	if (idx > 1) {
		pr_err("<PM> %s: invalid doubler index %d\n"
				, __func__, idx);
		return;
	}
	writel(0, pmu->db[idx]);
	udelay(5);

	/* set bypass mode */
	regval = readl(pmu->dbc);
	regval |= (1u << (2 + idx));
	writel(regval, pmu->dbc);
}

static u32 mmp3_dpll_on(struct mmp3_pmu *pmu, u32 idx,
				u32 mode4x, u32 srckhz)
{
	static const u32 ICP = 0x7; /* 42.5uA */
	static const u32 CTUNE = 0x2; /* 2 unit cap loading */
	static const u32 VDDL = 0x4; /* 0.9v */
	static const u32 VREG_IVREF = 0x0;
	static const u32 VDDM = 0x1;
	u32 outkhz, refdiv, fbdiv, vco_pre_div, kvco, regval;

	if (idx > 1) {
		pr_err("<PM> %s: invalid doubler index %d\n"
				, __func__, idx);
		return 0;
	}

	if ((srckhz < 100000) || (srckhz > 600000)) {
		pr_err("<PM> %s: invalid source freq %dkhz\n"
				, __func__, srckhz);
		return 0; /* invalid */
	}

	refdiv = srckhz / 100000;
	fbdiv = refdiv * 4;
	vco_pre_div = 1;
	while (fbdiv < 9) {
		fbdiv *= 2;
		vco_pre_div *= 2;
	}
	outkhz = (srckhz / refdiv) * fbdiv;

	if ((outkhz < 1200000) || (outkhz > 2400000)) {
		pr_err("<PM> %s: invalid target VCO freq %dkhz\n"
				, __func__, outkhz);
		return 0; /* out of range */
	}

	kvco = ((outkhz - 1200000) / 200000) + 1;

	regval = (mode4x == 0) ? 0 : (1u << 31); /* 4x mode */
	regval |= vco_pre_div << 29;
	regval |= CTUNE << 27;
	regval |= kvco << 24;
	regval |= ICP << 20;
	regval |= VREG_IVREF << 18;
	regval |= VDDL << 14;
	regval |= VDDM << 12;
	regval |= fbdiv << 7;
	regval |= refdiv << 2;

	writel(regval, pmu->db[idx]);
	udelay(5);

	regval = readl(pmu->db[idx]);
	regval |= (1u << 1); /* power up */
	writel(regval, pmu->db[idx]);
	udelay(5);

	/* set bypass mode */
	regval = readl(pmu->dbc);
	regval &= ~(1u << (2 + idx));
	writel(regval, pmu->dbc);

	return outkhz;
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
	val = MMP3_FREQ_DDR1_SET(val, pl->dram.pre_d);
	val = MMP3_FREQ_AXI1_SET(val, pl->axi.aclk1_d);

	set_sync_buf(CC_OFFSET, (u32)val);
	set_sync_buf(CC_ADDR_OFFSET, (u32)pmu->cc);

	/* cc2 */
	val = __raw_readl(pmu->cc2);
	val = MMP3_PROTECT_CC2(val); /* clear SB0 bits */
	val = MMP3_FREQ_ACLK_SET(val, pl->core.aclk_d);
	val = MMP3_FREQ_MM_SET(val, pl->core.mm_d);
	val = MMP3_FREQ_MP1_SET(val, pl->core.mp1_d);
	val = MMP3_FREQ_MP2_SET(val, pl->core.mp2_d);
	val = MMP3_FREQ_AXI2_SET(val, pl->axi.aclk2_d);

	set_sync_buf(CC2_OFFSET, (u32)val);
	set_sync_buf(CC2_ADDR_OFFSET, (u32)pmu->cc2);

	/* cc3 */
	val = __raw_readl(pmu->cc3);
	val = MMP3_PROTECT_CC3(val); /* clear SB0 bits */
	val = MMP3_FREQ_DDR2_SET(val, pl->dram.pre_d);

	set_sync_buf(CC3_OFFSET, (u32)val);
	set_sync_buf(CC3_ADDR_OFFSET, (u32)pmu->cc3);

	/* bus clkrst*/
	val = __raw_readl(pmu->bus);
	val = MMP3_PROTECT_BUSCLKRST(val); /* clear SB0 bits */
	val = MMP3_FREQ_ASRC_SET(val, pl->axi.asrc);
	val = MMP3_FREQ_DSRC2_SET(val, pl->dram.dsrc); /* always same as ch1 */

	set_sync_buf(BUS_OFFSET, (u32)val);
	set_sync_buf(BUS_ADDR_OFFSET, (u32)pmu->bus);

	/* fccr */
	val = __raw_readl(pmu->fccr);
	val = MMP3_PROTECT_FCCR(val); /* clear SB0 bits */
	val = MMP3_FREQ_PSRC_SET(val, pl->core.psrc);
	val = MMP3_FREQ_DSRC_SET(val, pl->dram.dsrc);

	set_sync_buf(FCCR_OFFSET, (u32)val);
	set_sync_buf(FCCR_ADDR_OFFSET, (u32)pmu->fccr);
}

struct ddrdfc_param {
	u32 oldkhz;
	u32 newkhz;
	u32 newcas;
	u32 newtiming_cnt;
	struct dmc_regtable_entry *newtiming;
	u32 phytuning_cnt;
	struct dmc_regtable_entry *phytuning;
	u32 rl_cnt;
	struct dmc_regtable_entry *rl;
	u32 wl_cnt;
	struct dmc_regtable_entry *wl;
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

#define UPDATE_REG(val, reg)						\
	do {								\
		__raw_writel(val, reg);					\
		/*printk(KERN_INFO" %08x ==> [%p]\n", val, reg);*/	\
	} while (0)

#define INSERT_ENTRY_EX(reg, b2c, b2s, pause, last)			\
	do {								\
		if (ent >= 32) {					\
			pr_err("INSERT_ENTRY_EX too much entry\n");	\
		}							\
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
		tmpvalue = (((tab << 5) + ent) & 0x7f) | DMCU_HWTWRITE;	\
		UPDATE_REG(tmpvalue, DMCU_HWTCTRL(base));		\
		ent++;							\
	} while (0)

#define INSERT_ENTRIES(entries, entcount, pause, last)			\
	do {								\
		u32 li;							\
		for (li = 0; li < entcount; li++) {			\
			if ((li + 1) == entcount) {			\
				INSERT_ENTRY_EX(entries[li].reg,	\
						entries[li].b2c,	\
						entries[li].b2s,	\
						pause, last);		\
			} else {					\
				INSERT_ENTRY_EX(entries[li].reg,	\
						entries[li].b2c,	\
						entries[li].b2s,	\
						0, 0);			\
			}						\
		}							\
	} while (0)

#define INSERT_ENTRY(reg, b2c, b2s) INSERT_ENTRY_EX(reg, b2c, b2s, 0, 0)
#define PAUSE_ENTRY(reg, b2c, b2s) INSERT_ENTRY_EX(reg, b2c, b2s, 1, 0)
#define LAST_ENTRY(reg, b2c, b2s) INSERT_ENTRY_EX(reg, b2c, b2s, 1, 1)
#define ALLBITS (0xFFFFFFFF)
#define NONBITS (0x00000000)

#define USE_PHYTUNING 0
#define USE_RL 0
#define USE_WL 0

static void mmp3_ddrhwt_lpddr2_h2l(u32 *dmcu, struct ddrdfc_param *param,
				u32 table_index)
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
	BEGIN_TABLE(table_index);
	/* Halt MC4 scheduler*/
	INSERT_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x2);
	/* 3 update timing, we use the boot timing which is for high clock */
	INSERT_ENTRY(DMCU_SDRAM_CTRL4, 0, 0);
	INSERT_ENTRIES(param->newtiming, param->newtiming_cnt, 0, 0);
#if USE_PHYTUNING
	INSERT_ENTRIES(param->phytuning, param->phytuning_cnt, 0, 0);
#endif

	/* 4 reset master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x20000000);
	/* 5 update master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x40000000);
	/* 6. synchronize 2x clock */
	PAUSE_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x80000000);

	/*
	 * 7 wake up SDRAM; when first table done (acked) PMU will de-assert
	 *    mc_sleep_req to wake up SDRAM from self-refresh
	 *
	 * 8 update SDRAM mode register, programmed in 2nd table
	 */
	INSERT_ENTRY(DMCU_SDRAM_CTRL1, 0x40, 0x40);
	/* 9 do a long ZQ Cal */
	INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, (map | 0x1000));
	INSERT_ENTRY(DMCU_USER_COMMAND1, ALLBITS, (map | 0x20001));
	INSERT_ENTRY(DMCU_USER_COMMAND1, ALLBITS, (map | 0x20002));
	INSERT_ENTRY(DMCU_USER_COMMAND1, ALLBITS, (map | 0x20003));
	/* resume scheduler*/
	LAST_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0);
}

static void mmp3_ddrhwt_lpddr2_l2h(u32 *dmcu, struct ddrdfc_param *param,
				u32 table_index)
{
	COMMON_DEF(dmcu);
	/*
	 * 1 PMU asserts 'mc_sleep_req' on type 'mc_sleep_type'; MC4 enters
	 *    self-refresh mode and hold scheduler for DDR access
	 *
	 * 2 update timing, programmed in 1st table
	 *    we just use the boot timing which is for high clock, no change
	 */
	BEGIN_TABLE(table_index);
	/* Halt MC4 scheduler*/
	INSERT_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x2);
	/* Update CAS*/
	INSERT_ENTRY(DMCU_SDRAM_CTRL4, 0, 0);
#if USE_PHYTUNING
	INSERT_ENTRIES(param->newtiming, param->newtiming_cnt, 0, 0);
	INSERT_ENTRIES(param->phytuning, param->phytuning_cnt, 1, 0);
#else
	INSERT_ENTRIES(param->newtiming, param->newtiming_cnt, 1, 0);
#endif
	/*
	 * 3 Frequency change upon 1st table done
	 *
	 * Step 4-6 programmed in the 2nd table
	 */
	/* 4 reset master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x20000000);
	/* 5 update master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x40000000);
	/* 6. synchronize 2x clock */
	PAUSE_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x80000000);

	/*
	 * 7 wake up SDRAM; when first table done (acked) PMU will de-assert
	 *    mc_sleep_req to wake up SDRAM from self-refresh
	 *
	 * 8 update SDRAM mode register, programmed in 2nd table
	 */
	INSERT_ENTRY(DMCU_SDRAM_CTRL1, 0x40, 0x40);
	/* 9 do a long ZQ Cal */
	INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, (map | 0x1000));
	INSERT_ENTRY(DMCU_USER_COMMAND1, ALLBITS, (map | 0x20001));
	INSERT_ENTRY(DMCU_USER_COMMAND1, ALLBITS, (map | 0x20002));
	INSERT_ENTRY(DMCU_USER_COMMAND1, ALLBITS, (map | 0x20003));
	/* resume scheduler*/
	LAST_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0);
}

static void mmp3_ddrhwt_ddr3_h2l(u32 *dmcu, struct ddrdfc_param *param,
				u32 table_index)
{
	COMMON_DEF(dmcu);
	/*
	 * 1 Halt MC4 and disable SDRAM DLL, programmed in 1st table
	 */
	BEGIN_TABLE(table_index);
	/* Halt MC4 scheduler*/
	PAUSE_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x2);
	/*
	 * 2 PMU asserts 'mc_sleep_req' on type 'mc_sleep_type'; MC4 enters
	 *    self-refresh mode and hold scheduler for DDR access
	 * 3 Frequency change
	 *
	 * Step 4-7 programmed in the 2nd table
	 */
	/* 4 update timing, do phy tuning*/
	/* Update CAS*/
	INSERT_ENTRY(DMCU_SDRAM_CTRL4, 0, 0);
	INSERT_ENTRIES(param->newtiming, param->newtiming_cnt, 0, 0);
#if USE_PHYTUNING
	INSERT_ENTRIES(param->phytuning, param->phytuning_cnt, 0, 0);
#endif
#if USE_RL
	INSERT_ENTRIES(param->rl, param->rl_cnt, 0, 0);
#endif
#if USE_WL
	INSERT_ENTRIES(param->wl, param->wl_cnt, 0, 0);
#endif
	/* 5 reset master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x20000000);
	/* 6 update master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x40000000);
	/* 7. synchronize 2x clock */
	PAUSE_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x80000000);

	/*
	 * 8 wake up SDRAM; when first table done (acked) PMU will de-assert
	 *    mc_sleep_req to wake up SDRAM from self-refresh
	 *
	 * 9 update SDRAM mode register and do ZQ Cal, programmed in 3nd table
	 */
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

static void mmp3_ddrhwt_ddr3_l2h(u32 *dmcu, struct ddrdfc_param *param,
				u32 table_index)
{
	COMMON_DEF(dmcu);
	/*
	 * 1 PMU asserts 'mc_sleep_req' on type 'mc_sleep_type'; MC4 enters
	 *    self-refresh mode and hold scheduler for DDR access
	 *
	 * 2 update timing, programmed in 1st table
	 *    we just use the boot timing which is for high clock, no change
	 */
	BEGIN_TABLE(table_index);
	/* Halt MC4 scheduler*/
	INSERT_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x2);
	/* Update CAS*/
	INSERT_ENTRY(DMCU_SDRAM_CTRL4, 0, 0);
#if USE_PHYTUNING
	INSERT_ENTRIES(param->newtiming, param->newtiming_cnt, 0, 0);
	INSERT_ENTRIES(param->phytuning, param->phytuning_cnt, 1, 0);
#else
	INSERT_ENTRIES(param->newtiming, param->newtiming_cnt, 1, 0);
#endif
#if USE_RL
	INSERT_ENTRIES(param->rl, param->rl_cnt, 0, 0);
#endif
#if USE_WL
	INSERT_ENTRIES(param->wl, param->wl_cnt, 0, 0);
#endif
	/*
	 * 3 Frequency change
	 *
	 * Step 4-6 programmed in the 2nd table
	 */
	/* 4 reset master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x20000000);
	/* 5 update master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x40000000);
	/* 6. synchronize 2x clock */
	PAUSE_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x80000000);

	/*
	 * 7 wake up SDRAM; when first table done (acked) PMU will de-assert
	 *    mc_sleep_req to wake up SDRAM from self-refresh
	 *
	 * 8 enable and reset SDRAM DLL
	 * 9 do ZQ Cal
	 * 10 unhalt MC4 scheduler
	 * 8-10 programmed in 3nd table
	 */
	INSERT_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x2);
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

static void mmp3_udpate_ddr_parameter_table(struct mmp3_pmu *pmu,
						struct mmp3_freq_plan *pl)
{
	u32 *base;
	u32 type, index, bit, tmpval, l2h, tabidx;
	struct ddrdfc_param param;

	mmp3_freq_plan_cal(pmu, pl);
	tabidx = 0; /* only use table 0*/
	UPDATE_REG((tabidx << 5) | (tabidx << 8) | 0x4, pmu->mc_par_ctrl);
	for (index = 0; index < 2; index++) {
		base = pmu->dmcu[index];
		bit = 4 + index * 2;

		if ((((pmu->ddrdfc_trigger & MMP3_FREQCH_DRAM_CH1) == 0)
				&& (index == 0))
			|| (((pmu->ddrdfc_trigger & MMP3_FREQCH_DRAM_CH2) == 0)
				&& (index == 1))) {
			/* no change to current channel, bypass configuration*/
			continue;
		}

		param.oldkhz = pmu->pl_curr.khz[MMP3_CLK_DDR_1 + index];
		param.newkhz = pl->khz[MMP3_CLK_DDR_1 + index];
		if (param.oldkhz < param.newkhz)
			l2h = 1;
		else
			l2h = 0;

		type = __raw_readl(FIXADDR(base, DMCU_SDRAM_CTRL4))
						& DMCU_SDRAM_TYPE_MASK;
		tmpval = __raw_readl(pmu->mc_slp_req) & ~(3u << bit);
		param.newcas = pl->dram.timing->cas;
		param.newtiming = pl->dram.timing->table[DMCRT_TM].entry;
		param.newtiming_cnt = pl->dram.timing->table[DMCRT_TM].count;
#if USE_PHYTUNING
		param.phytuning = pl->dram.timing->table[DMCRT_PH].entry;
		param.phytuning_cnt = pl->dram.timing->table[DMCRT_PH].count;
#endif
#if USE_WL
		param.wl = pl->dram.timing->table[DMCRT_WL].entry;
		param.wl_cnt = pl->dram.timing->table[DMCRT_WL].count;
#endif
#if USE_RL
		param.rl = pl->dram.timing->table[DMCRT_RL].entry;
		param.rl_cnt = pl->dram.timing->table[DMCRT_RL].count;
#endif
		switch (type) {
		case DMCU_SDRAM_TYPE_DDR3:
			if (l2h) {
				tmpval |=  (3u << bit);
				UPDATE_REG(tmpval, pmu->mc_slp_req);
				mmp3_ddrhwt_ddr3_l2h(base, &param, tabidx);
			} else {
				tmpval |=  (2u << bit);
				UPDATE_REG(tmpval, pmu->mc_slp_req);
				mmp3_ddrhwt_ddr3_h2l(base, &param, tabidx);
			}
			break;
		case DMCU_SDRAM_TYPE_LPDDR2:
			if (l2h) {
				tmpval |= (1u << bit);
				UPDATE_REG(tmpval, pmu->mc_slp_req);
				mmp3_ddrhwt_lpddr2_l2h(base, &param, tabidx);
			} else {
				tmpval |= (0u << bit);
				UPDATE_REG(tmpval, pmu->mc_slp_req);
				mmp3_ddrhwt_lpddr2_h2l(base, &param, tabidx);
			}
			break;
		default:
			pr_err("<PM> unsupported DDR type in DFC\n");
			break;
		}
	}
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
	pmu->source_status[MMP3_CLK_MP1] = 0;
	pmu->source_status[MMP3_CLK_AXI_1] = 0;
	pmu->source_status[MMP3_CLK_DDR_1] = 0;
	if (curpl.core.val != pl->core.val) {
		if (pl->core.op == MMP3_FREQ_OP_UPDATE) {
			pmu->pl_curr.core.op = MMP3_FREQ_OP_SHOW;
			change |= MMP3_FREQCH_CORE;
			logentry->marker |= TRACE_DFC_MARKER_CORE;
			logentry->pp_core = (unsigned char) pmu->pp_curr;
			pmu->trigge_stat[MMP3_CLK_MP1]++;
			if (pmu->pl_curr.core.psrc != pl->core.psrc) {
				pmu->source_status[MMP3_CLK_MP1] =
					0x80000000 | pmu->pl_curr.core.psrc;
				/* make sure target source enabled*/
				mmp3_fccs_enable(pl->core.psrc);
			}
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
			if (pmu->pl_curr.axi.asrc != pl->axi.asrc) {
				pmu->source_status[MMP3_CLK_AXI_1] =
					0x80000000 | pmu->pl_curr.axi.asrc;
				/* make sure target source enabled*/
				mmp3_fccs_enable(pl->axi.asrc);
			}
		} else
			pl->axi.val = curpl.axi.val;
	}
	if (curpl.dram.val != pl->dram.val) {
		if (pl->dram.op == MMP3_FREQ_OP_UPDATE) {
			pmu->pl_curr.dram.op = MMP3_FREQ_OP_SHOW;

			change |= pmu->ddrdfc_trigger;
			logentry->marker |= TRACE_DFC_MARKER_DRAM;
			logentry->pp_dram = (unsigned char) pmu->pp_curr;
			pmu->trigge_stat[MMP3_CLK_DDR_1]++;
			if (pmu->pl_curr.dram.dsrc != pl->dram.dsrc) {
				pmu->source_status[MMP3_CLK_DDR_1] =
					0x80000000 | pmu->pl_curr.dram.dsrc;
				/* make sure target source enabled*/
				mmp3_fccs_enable(pl->dram.dsrc);
			}
			if (pmu->pl_curr.dram.mode4x != pl->dram.mode4x) {
				u32 speed = mmp3_fccs_speed(pl->dram.dsrc);
				speed = speed / FIELD2DIV(pl->dram.pre_d);
				if (pl->dram.mode4x) {
					/* need to program doubler */
					mmp3_dpll_on(pmu, 0, 1, speed);
					if (pmu->ddrdfc_trigger
						& MMP3_FREQCH_DRAM_CH2) {
						mmp3_dpll_on(pmu, 1, 1, speed);
					}
				} else {
					/* doubler support program in place */
					mmp3_dpll_off(pmu, 0);
					if (pmu->ddrdfc_trigger
						& MMP3_FREQCH_DRAM_CH2) {
						mmp3_dpll_off(pmu, 1);
					}
				}
			}
		} else
			pl->dram.val = curpl.dram.val;
	}

	mmp3_freq_plan_print_dbg(pmu, &pmu->pl_curr, 0);

	if (change != 0) {
		/* change required */
		mmp3_update_freq_plan(pmu, pl);

		/* Update DDR register table*/
		if (change & MMP3_FREQCH_DRAM) {
			if (pl->dram.timing != NULL) {
				/* timing parameter provided, apply it */
				mmp3_udpate_ddr_parameter_table(pmu, pl);
			} else {
				pr_err("<PM> DRAM TIMING update not valid, "
					"DFC abort!!\n");
				change &= ~MMP3_FREQCH_DRAM;
			}
		}

	} else {
		/* no change, need to clear PJ_RD_STATUS */
		mmp3_clear_PJ_RD_STATUS(pmu);
	}

	return change;
}

static void mmp3_dfc_trigger(struct mmp3_pmu *pmu, struct mmp3_freq_plan *pl,
	union trace_dfc_log *logentry, u32 change, u32 *time, u32*same)
{
	u32 samex, timex;
	u32 dfc_val;
	int done = 0, wait = 0, ret = 0;
	int try;
	int old, new;

	old = pmu->pl_curr.khz[MMP3_CLK_MP1];
	new = pl->khz[MMP3_CLK_MP1];

	if (change == 0)
		return;

	timex = read_timestamp();

	BUG_ON(irqs_disabled());

	/* compose trigger val */
	dfc_val = get_sync_buf(CC_OFFSET);
	dfc_val = MMP3_PROTECT_CC(dfc_val); /* set reserved */
	dfc_val = dfc_val | change | (MMP3_FREQCH_VOLTING);

#define DDR_FREQ_CHG_DONE2	(1 << 12)
#define DDR_FREQ_CHG_DONE1	(1 << 4)
#define AXI_FREQ_CHG_DONE	(1 << 5)
#define PJ_FREQ_CHG_DONE	(1 << 3)

	if (change & MMP3_FREQCH_CORE)
		wait |= PJ_FREQ_CHG_DONE;
	if (change & MMP3_FREQCH_AXI)
		wait |= AXI_FREQ_CHG_DONE;
	if (change & MMP3_FREQCH_DRAM_CH1)
		wait |= DDR_FREQ_CHG_DONE1;
	if (change & MMP3_FREQCH_DRAM_CH2)
		wait |= DDR_FREQ_CHG_DONE2;

	if (change & MMP3_FREQCH_CORE) {
		if ((old < new) && (new > 400000) && (old <= 400000))
			cm_vote_mp1();
	}

	writel(wait, APMU_IMR);		/* unmask according irqs */
	writel(0x0, APMU_ISR);		/* clear status */

	/* if (change & (MMP3_FREQCH_CORE | MMP3_FREQCH_DRAM)) { */
	if(1){
#if 0
		if (cpu_is_mmp3_b0p())
			ret = mmp3_trigger_dfc_ll_b0p(dfc_val, (u32)sync_buf);
		else
#endif
			ret = mmp3_trigger_dfc_ll(dfc_val, (u32)sync_buf);
	} else {
		local_irq_disable();
		writel(get_sync_buf(CC_OFFSET), pmu->cc);
		writel(get_sync_buf(CC2_OFFSET), pmu->cc2);
		writel(get_sync_buf(CC3_OFFSET), pmu->cc3);
		writel(get_sync_buf(BUS_OFFSET), pmu->bus);
		writel(get_sync_buf(FCCR_OFFSET), pmu->fccr);
		writel(dfc_val, pmu->cc);

		for (try = 0; try < 1000; try++) {
			done = readl(APMU_ISR);
			done &= wait;
			if (done == wait)
				break;
		}

		if (try >= 1000)
			printk(KERN_ERR "%s: done %x wait %x\n", __func__, done, wait);
		local_irq_enable();
	}

	/* clear dfc irq status */
	writel(0x1fff, APMU_ISR);

	timex = read_timestamp() - timex;

	if (ret) {
		printk(KERN_WARNING "%s: failed acquire lock\n", __func__);
		return;
	}

	if (change & MMP3_FREQCH_CORE) {
		if ((old > new) && (new <= 400000) && (old > 400000))
			cm_cancel_vote_mp1();
	}

	/* now read back current frequency settings, for query*/
	mmp3_get_freq_plan(pmu, &pmu->pl_curr, true);

	samex = mmp3_compare_freq_plan(&pmu->pl_curr, pl);

	if (samex != 0)
		logentry->marker |= TRACE_DFC_MARKER_ERR;
#if MMP3_PP_TABLE_DIFF_BYIDX
	else {
		int c;
		/* patch the calculation */
		if ((change & MMP3_FREQCH_CORE) != 0) {
			for (c = MMP3_CLK_MP1; c <= MMP3_CLK_ACLK; c++) {
				pmu->pl_curr.khz[c]
					= pmu->pps[pmu->pp_curr].khz[c];
			}
		}
		if ((change & MMP3_FREQCH_AXI) != 0) {
			for (c = MMP3_CLK_AXI_1; c <= MMP3_CLK_AXI_2; c++) {
				pmu->pl_curr.khz[c]
					= pmu->pps[pmu->pp_curr].khz[c];
			}
		}
		if ((change & MMP3_FREQCH_DRAM) != 0) {
			for (c = MMP3_CLK_DDR_1; c <= MMP3_CLK_DDR_2; c++) {
				pmu->pl_curr.khz[c]
					= pmu->pps[pmu->pp_curr].khz[c];
			}
		}
	}
#endif

	trace_dfc(logentry);

	if (time != NULL)
		*time = timex;
	if (same != NULL)
		*same = samex;

	/* done, PJ_RD_STATUS should have been cleared by HW*/
}

#define GETSRC(x) ((x) & 0x7)
static void mmp3_dfc_postchange(struct mmp3_pmu *pmu, struct mmp3_freq_plan *pl,
	union trace_dfc_log *logentry, u32 time, u32 same)
{
	int oldsrc;

	if (same != 0) {
		/* usually should not be here, log a message here */
		printk(KERN_WARNING "<PM> DFC result is not the requested one\n");
		printk(KERN_WARNING "<PM> CORE STATUS%08x\n", readl(APMU_CORE_STATUS));
		printk(KERN_WARNING "<PM> CC %08x CC2 %08x CC3 %08x BUS %08x\n",
			readl(APMU_CC_PJ),  readl(APMU_CC2_PJ),
			readl(APMU_CC3_PJ), readl(APMU_BUS));
		printk(KERN_WARNING "<PM> DM_CC %08x DM_CC2 %08x FCCR %08x\n",
			readl(APMU_DM_CC_PJ), readl(APMU_DM2_CC_PJ), readl(MPMU_FCCR));

		mmp3_freq_plan_print_info(pmu, &pmu->pl_curr, time);
	} else {
		mmp3_freq_plan_print_dbg(pmu, &pmu->pl_curr, time);
	}

	/* post clock source settings */
	if (pmu->source_status[MMP3_CLK_MP1] != 0) {
		oldsrc = GETSRC(pmu->source_status[MMP3_CLK_MP1]);
		if (oldsrc == pmu->pl_curr.core.psrc) {
			/*
			 * source not updated, fc fail, disable previously
			 * enabled source
			 */
			mmp3_fccs_disable(pl->core.psrc);
		} else {
			/* updated, old source can be disabled */
			mmp3_fccs_disable(oldsrc);
		}
	}
	if (pmu->source_status[MMP3_CLK_DDR_1] != 0) {
		oldsrc = GETSRC(pmu->source_status[MMP3_CLK_DDR_1]);
		if (oldsrc == pmu->pl_curr.dram.dsrc) {
			/*
			 * source not updated, fc fail, disable previously
			 * enabled source
			 */
			mmp3_fccs_disable(pl->dram.dsrc);
		} else {
			/* updated, old source can be disabled */
			mmp3_fccs_disable(oldsrc);
		}
	}
	if (pmu->source_status[MMP3_CLK_AXI_1] != 0) {
		oldsrc = GETSRC(pmu->source_status[MMP3_CLK_AXI_1]);
		if (oldsrc == pmu->pl_curr.axi.asrc) {
			/*
			 * source not updated, fc fail, disable previously
			 * enabled source
			 */
			mmp3_fccs_disable(pl->axi.asrc);
		} else {
			/* updated, old source can be disabled */
			mmp3_fccs_disable(oldsrc);
		}
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
	union trace_dfc_log logentry;
	u32 change, time, same;

	spin_lock(&dfcsch_lock);

	if (atomic_read(&glb_hp_lock)) {
		spin_unlock(&dfcsch_lock);
		return;
	}

	change = mmp3_prepare_freqch(pmu, pl, &logentry);
	if (change == 0) {
		spin_unlock(&dfcsch_lock);
		return;
	}

	time = 0;
	same = 0;
	__raw_writel(1, pmu->swstat);

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
		spin_unlock(&dfcsch_lock);
		ret = wait_for_completion_timeout(&sch.comp,
						msecs_to_jiffies(500));
		time = sch.time;
		same = sch.same;
		if (ret <= 0) {
			/* timeout, which indicates that the trigger
			 * is not triggering any more
			 */
			spin_lock(&dfcsch_lock);
			if (dfcsch != NULL) {
				dfcsch_active = 0;
				dfcsch = NULL;
				mmp3_dfc_trigger(pmu, pl, &logentry,
							change, &time, &same);
			}
			spin_unlock(&dfcsch_lock);
		}
	} else {
		mmp3_dfc_trigger(pmu, pl, &logentry, change, &time, &same);
		spin_unlock(&dfcsch_lock);
	}

	mmp3_dfc_postchange(pmu, pl, &logentry, time, same);

	__raw_writel(0, pmu->swstat);
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

void mmp3_pm_update_dram_timing_table(int count, struct dmc_timing_entry *tab)
{
	int i, j = 0;
	struct mmp3_pmu *pmu = &mmp3_pmu_config;
	printk(KERN_INFO"<PM> Fixup DDR Frequency Table parameters:\n");
	for (i = 0; i < pmu->ppscnt; i++) {
		if (i < count)
			j = i;
		pmu->pps[i].dram.op = MMP3_FREQ_OP_GET;
		pmu->pps[i].dram.dsrc = tab[j].dsrc;
		pmu->pps[i].dram.mode4x = tab[j].mode4x;
		pmu->pps[i].dram.pre_d = tab[j].pre_d;
		pmu->pps[i].dram.timing = &tab[j];
		printk(KERN_INFO"<PM> [%d]: src:%d-mode:%d-pre_d:%d\n"
				, i
				, pmu->pps[i].dram.dsrc
				, pmu->pps[i].dram.mode4x
				, pmu->pps[i].dram.pre_d
				);
		mmp3_freq_plan_cal(pmu, &(pmu->pps[i]));
	}

#if MMP3_PP_TABLE_DIFF_BYIDX
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
#endif

}
EXPORT_SYMBOL_GPL(mmp3_pm_update_dram_timing_table);

/* Low Power IDLE related function */
static inline void mmp3_mod_idle_config(struct mmp3_cpu_idle_config *cic,
			u32 target_value) {
	u32 the_value;
	the_value = readl(cic->idle_config);
	the_value &= cic->idle_config_keep_mask;
	the_value |= (target_value & cic->idle_config_valid_mask);
	writel(the_value, cic->idle_config);
}

/*#define MMP3_WAKE_ON_DVM*/
#define MMP3_PM_C1_INCG 0x0
#define MMP3_PM_C1_EXCG 0x2
#define MMP3_PM_C2_LPPD 0x62
#define MMP3_PM_C2_L1_L2_PWD 0x820004e2
#define MMP3_PM_C2_L1_PWD 0x82000462
#define MMP3_PM_C2_L1_RETENT_L2_PWD 0x800004a2
#define MMP3_PM_C2_MPPD 0x72
#define MMP3_PM_D2_L2_PWD 0x00000462

void mmp3_pm_enter_idle(int cpu)
{
	register u32 reg;
	u32 state = MMP3_PM_C1_INCG;
	struct mmp3_cpu_idle_config *cic;

	cic = &(mmp3_percpu[mmp3_smpid()].cic);

	mmp3_mod_idle_config(cic, state);
	/* no DFC in process*/
	trace_idle_entry(state);

	/* need to make sure this configuration is on every core */
	__asm__ volatile ("mrc p15, 1, %0, c15, c1, 0" : "=r" (reg));
#if defined(MMP3_WAKE_ON_DVM)
	/* enable DVM wake up for software check */
	reg &= ~(1 << 22);
#else
	/* disable DVM wake up for software check */
	reg |= (1 << 22);
#endif
	__asm__ volatile ("mcr p15, 1, %0, c15, c1, 0" : : "r" (reg));

#if defined(CONFIG_SMP) && defined(MMP3_WAKE_ON_DVM) && defined(CONFIG_ARM_GIC)
	while (1) {
		__asm__ __volatile__ ("wfi");
		if ((__raw_readl(cic->intr_status) & 0x3ff) != 1023)
			break; /* real wake up, break loop to handle*/
	}
#else
	arch_idle();
#endif

	trace_idle_exit(__raw_readl(cic->wake_status));
}

void mmp3_pm_enter_c2(int cpu, int hpg)
{
	u32 state = MMP3_PM_C2_L1_PWD;
	struct mmp3_cpu_idle_config *cic;
	int core_id = mmp3_smpid();
	u32 c1_cfg, c2_cfg;

	if (atomic_read(&glb_hp_lock) && !hpg)
		return;

	trace_idle_entry(state);

	cic = &(mmp3_percpu[core_id].cic);

	c2_cfg  = readl(cic->idle_config);
	c2_cfg &= cic->idle_config_keep_mask;
	c2_cfg |= (MMP3_PM_C2_L1_PWD & cic->idle_config_valid_mask);

	c1_cfg  = readl(cic->idle_config);
	c1_cfg &= cic->idle_config_keep_mask;
	c1_cfg |= (MMP3_PM_C1_INCG & cic->idle_config_valid_mask);

	if (core_id == 0 || core_id == 2)
		set_sync_buf(CM_CID_OFFSET, core_id);

	if (core_id == 0) {
		set_sync_buf(MP1_C1_CFG_OFFSET, c1_cfg);
		set_sync_buf(MP1_C2_CFG_OFFSET, c2_cfg);
	} else if (core_id == 1) {
		set_sync_buf(MP2_C1_CFG_OFFSET, c1_cfg);
		set_sync_buf(MP2_C2_CFG_OFFSET, c2_cfg);
	} else if (core_id == 2) {
		set_sync_buf(MM_C1_CFG_OFFSET, c1_cfg);
		set_sync_buf(MM_C2_CFG_OFFSET, c2_cfg);
	}

	mmp3_enter_c2((u32)sync_buf);

	/* disable global irq of ICU for MP1, MP2, MM*/
	__raw_writel(0x1, MMP3_ICU_GBL_IRQ1_MSK);
	__raw_writel(0x1, MMP3_ICU_GBL_IRQ2_MSK);
	__raw_writel(0x1, MMP3_ICU_GBL_IRQ3_MSK);
	__raw_writel(0x1, MMP3_ICU_GBL_IRQ4_MSK);
	__raw_writel(0x1, MMP3_ICU_GBL_IRQ5_MSK);
	__raw_writel(0x1, MMP3_ICU_GBL_IRQ6_MSK);

	trace_idle_exit(__raw_readl(cic->wake_status));
}


static void mmp3_do_idle(void)
{
	if (!need_resched())
		mmp3_pm_enter_idle(smp_processor_id());
	local_irq_enable();
}

static void program_ddr3_calibration_cmd_b0(unsigned int dmcu, unsigned int tabidx)
{
	COMMON_DEF(dmcu);
	BEGIN_TABLE(tabidx);

	/* make sure that Mck4 is halt for any read\write commands */
	INSERT_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x2);
	/* PHY DLL reset */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x20000000);
	/* PHY DLL Update */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x40000000);
	/* Sync 2x clock (EOP) */
	PAUSE_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x80000000);

	/* Reset DLL */
	INSERT_ENTRY(DMCU_SDRAM_CTRL1, 0, 0x40);
	/* Send LMR0 DLL */
	INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, 0x03000100);
	/* Send LMR2 DLL */
	INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, 0x03000400);
	/* ZQ Calibration */
	INSERT_ENTRY(DMCU_USER_COMMAND0, ALLBITS, 0x03001000);
	/* resume scheduler */
	LAST_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x0);
}

static void program_lpddr2_calibration_cmd_b0(unsigned int dmcu, unsigned int tabidx)
{
	COMMON_DEF(dmcu);
	BEGIN_TABLE(tabidx);

	/* make sure that Mck4 is halt for any read\write commands */
	INSERT_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x2);
	/* PHY DLL reset */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x20000000);
	/* PHY DLL Update */
	INSERT_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x40000000);
	/* Sync 2x clock (EOP) */
	PAUSE_ENTRY(DMCU_PHY_CTRL14, ALLBITS, 0x80000000);

	/* MR 1 */
	INSERT_ENTRY(DMCU_USER_COMMAND1, ALLBITS, 0x03020001);
	/* MR 2 */
	INSERT_ENTRY(DMCU_USER_COMMAND1, ALLBITS, 0x03020002);
	/* resume scheduler */
	LAST_ENTRY(DMCU_SDRAM_CTRL14, ALLBITS, 0x0);
}

static void program_dll_table_b0(unsigned int dmcu, unsigned int tabidx)
{
	u32 mc_parctr;
	unsigned int ddr_type;

	ddr_type = (__raw_readl(dmcu + 0x58) & 0x1C) >> 2;
	if (ddr_type == 0x2)
		program_ddr3_calibration_cmd_b0(dmcu, tabidx);
	else
		program_lpddr2_calibration_cmd_b0(dmcu, tabidx);

	mc_parctr = __raw_readl(APMU_MC_PAR_CTRL) & ~(0x4);
	if (dmcu == (u32)mmp3_pmu_config.dmcu[0]) {
		mc_parctr &= ~(0x7 << 5);
		mc_parctr |= (tabidx << 5);
	} else {
		mc_parctr &= ~(0x7 << 8);
		mc_parctr |= (tabidx << 8);
	}
	__raw_writel(mc_parctr, APMU_MC_PAR_CTRL);
}

static u32 ccic, gc, vmeta, audio_clk, apcr;
void mmp3_set_wakeup_src(void)
{
	int val = 0;

	/* enable wakeup 7 as wakeup input source */
	val |= (1 << 7);
	/* enable rtc as wakeup input source */
	val |= ((1 << 17) | (1 << 4));
	/* enable gpio as wakeup input source */
	val |= (1 << 2);
	__raw_writel(val, MPMU_AWUCRM);
}

static void d2(void)
{
	u32 reg;
	u32 tmp_gc;
	/* walk around for B0: program dll table before entering D2 */
	program_dll_table_b0((u32)mmp3_pmu_config.dmcu[0], 0);

	reg = __raw_readl(APMU_BUS);
	if (reg & (1 << 1))
		/* prgram dll table for MC2 only when it is released from reset */
		program_dll_table_b0((u32)mmp3_pmu_config.dmcu[1], 0);
	else
		/* make the wake up state machine do not wait for ack from MC2 idle */
		__raw_writel(__raw_readl(APMU_DEBUG2) | (1 << 1), APMU_DEBUG2);

	ccic = __raw_readl(APMU_CCIC_RST);
	gc = __raw_readl(APMU_GC);
	tmp_gc = gc & 0x600 ;
	vmeta = __raw_readl(APMU_VMETA);
	/* Disable power to other power islands: ISP, GC and VMeta */
	__raw_writel(0x00, APMU_CCIC_RST);
	/*__raw_writel(0x00, APMU_GC);*/
	__raw_writel(tmp_gc, APMU_GC);
	__raw_writel(0x00, APMU_VMETA);

	/* Disable power to Audio island */
	audio_clk = __raw_readl(APMU_AUDIO_CLK_RES_CTRL);
	__raw_writel(0x00, APMU_AUDIO_CLK_RES_CTRL);

	reg = __raw_readl(APMU_AUDIO_SRAM_PWR);
	reg |= (0x3 << 6);
	reg |= (0x3 << 8);
	reg |= (0x3 << 10);
	__raw_writel(reg, APMU_AUDIO_SRAM_PWR);

	/* Dragonite power control */
	__raw_writel(0xbe086000, MPMU_CPCR);

	apcr = __raw_readl(MPMU_APCR);
	/* PJ power control, wake up port 2(gpio),4(rtc),7(pmic) enabled*/
	__raw_writel(__raw_readl(MPMU_APCR) | (1 << 31) | (1 << 27) |
			(1 << 26) | (1 << 20) | (1 << 22) | (1 << 23) |
			(1 << 19) | (1 << 16) | (1 << 17) | (1 << 25),
			MPMU_APCR);
}

extern void l2x0_flush_all(void);
extern void l2x0_disable(void);
extern void l2x0_enable(void);
extern void l2x0_inv_all(void);
void mmp3_pm_enter_d2(void)
{
	struct mmp3_cpu_idle_config *cic;
	int core_id = mmp3_smpid();
	u32 the_value;
	/*register u32 reg;*/

	cic = &(mmp3_percpu[core_id].cic);

	d2();
	mmp3_mod_idle_config(cic, MMP3_PM_D2_L2_PWD);

	/* d2 workaround: shut down AT clock by setting bit [31]
	 * of PMUA_PJ_IDLE_CONFIG */
	the_value = readl(cic->idle_config);
	the_value &= ~(1 << 31);
	writel(the_value, cic->idle_config);

	mmp3_set_wakeup_src();

	/* workaround: keep SL2 power off */
	__raw_writel(__raw_readl(0xfe282a48) & ~(1 << 15), 0xfe282a48);
	__raw_writel(__raw_readl(0xfe282a4c) & ~(1 << 15), 0xfe282a4c);
	__raw_writel(__raw_readl(0xfe282a50) & ~(1 << 15), 0xfe282a50);

	printk("before suspend\n");

	/* d2 workaround: enable RTC & PMIC & GPIO ICU wake up */
	__raw_writel(0x2f, ICU1_REG(0x10));
	__raw_writel(0x2f, ICU1_REG(0x14));
	__raw_writel(0x2f, ICU1_REG(0xc4));

	/* disable automatic pad calibration */
	__raw_writel(__raw_readl(0xfe50023c) & ~(1 << 16), 0xfe50023c);

	flush_cache_all();
	dsb();
	/* flush outer cache */
	outer_flush_all();
	dsb();

	/* L2 cache operation before D2 */
	l2x0_flush_all();
	dsb();
	l2x0_disable();

	__asm__ __volatile__ ("wfi");

	/* L2 cache operation after D2 */
	l2x0_inv_all();
	dsb();
	l2x0_enable();

	/* enable automatic pad calibration */
	__raw_writel(__raw_readl(0xfe50023c) | (1 << 16), 0xfe50023c);

	__raw_writel(0x0, ICU1_REG(0x10));
	__raw_writel(0x0, ICU1_REG(0x14));
	__raw_writel(0x0, ICU1_REG(0xc4));

	printk("after resume\n");

	/* disable global irq of ICU for MP1, MP2, MM*/
	__raw_writel(0x1, MMP3_ICU_GBL_IRQ1_MSK);
	__raw_writel(0x1, MMP3_ICU_GBL_IRQ2_MSK);
	__raw_writel(0x1, MMP3_ICU_GBL_IRQ3_MSK);
	__raw_writel(0x1, MMP3_ICU_GBL_IRQ4_MSK);
	__raw_writel(0x1, MMP3_ICU_GBL_IRQ5_MSK);
	__raw_writel(0x1, MMP3_ICU_GBL_IRQ6_MSK);

	__raw_writel(apcr, MPMU_APCR);

	/* resotre audio clocks otherwise will hang later */
	__raw_writel(audio_clk, APMU_AUDIO_CLK_RES_CTRL);

	__raw_writel(ccic, APMU_CCIC_RST);
	__raw_writel(gc, APMU_GC);
	__raw_writel(vmeta, APMU_VMETA);

	mmp3_mod_idle_config(cic, MMP3_PM_C1_INCG);
}

#ifdef CONFIG_SMP
extern unsigned long c2_reserve_pa;
#define C2_RESERVE_SIZE	(1024 * 1024)
#endif

static int __init mmp3_pm_init(void)
{
	int i, j;
	u32 ddrt;
	struct mmp3_pmu *pmu = &mmp3_pmu_config;

#ifdef CONFIG_SMP
	/* FIXME: Here we assume all cores will be on before here! */
	num_cpus = num_online_cpus();
#endif

	pmu->swstat_store = 0;
	pmu->swstat = &(pmu->swstat_store);
	spin_lock_init(&(pmu->mmp3_fc_spin));
	mutex_init(&(pmu->mmp3_fc_lock));

#ifdef CONFIG_SMP
	sync_buf = __arm_ioremap(c2_reserve_pa, PAGE_SIZE, MT_UNCACHED);
	if (sync_buf == NULL) {
		pr_err("%s: failed to remap memory for C2\n", __func__);
		BUG();
	}

	memset(sync_buf, 0x0, PAGE_SIZE);

	set_sync_buf(INT_STATUS_ADDR_OFFSET, (u32)INTR_STAT);
	set_sync_buf(WAKE_STATUS_ADDR_OFFSET, (u32)WAKE_STAT);
	set_sync_buf(SGIR_ADDR_OFFSET, (u32)(GIC_DIST_VIRT_BASE + GIC_DIST_SOFTINT));
	set_sync_buf(APMU_BASE_ADDR_OFFSET, APMU_VIRT_BASE);

	coherent_state = (unsigned int *)sync_buf;
	/* we are at coherent state */
	for (i = 0; i < num_cpus; i++)
		coherent_state[i] = 1;
#endif

	for (j = 0; j < ARRAY_SIZE(mmp3_fccs); j++) {
		mmp3_fccs[j].source =
			clk_get(NULL, mmp3_fccs[j].name);
		if (mmp3_fccs[j].source != NULL) {
			mmp3_fccs[j].frequency =
				clk_get_rate(mmp3_fccs[j].source) / 1000;
		} else {
			panic("mmp3_pm_init: invalid source %s\n",
				mmp3_fccs[j].name);
			return -1;
		}
	}

	/* 1. set bit[0] to ignore SP idle status for DFC
	 * 2. set bit[1] to mask the Moltres halt ack to DFC state machine
	 */
	__raw_writel(0x00000001, APMU_DEBUG);
	__raw_writel(0x0, APMU_DEBUG2);

	/* unmask and clear all dfc irqs */
	writel(0x0, APMU_IRWC);
	writel(0x0, APMU_IMR);

	pm_idle = mmp3_do_idle;

	/* vote SPSD*/
	__raw_writel(__raw_readl(MPMU_APCR) | (1u << 28), MPMU_APCR);

	/* fix DDR settings according to DDR module */
	ddrt = (__raw_readl(pmu->mc_interleave) & (0x7f << 0));
	if (ddrt == 0) {
		pmu->ddrdfc_trigger = MMP3_FREQCH_DRAM_CH1;
		printk(KERN_INFO "Dual MC disabled\n");
	} else {
		pmu->ddrdfc_trigger = MMP3_FREQCH_DRAM;
		printk(KERN_INFO "Dual MC enabled, config 0x%x\n", ddrt);
	}

	for (i = 0; i < pmu->ppscnt; i++)
		mmp3_freq_plan_cal(pmu, &(pmu->pps[i]));

#if MMP3_PP_TABLE_DIFF_BYIDX
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
#endif

	if (0)
		pmu->setrate = mmp3_clk_setrate_lowbar;
	else
		pmu->setrate = mmp3_clk_setrate_localized;

	mmp3_get_freq_plan(pmu, &pmu->pl_curr, true);
	pmu->pl_curr.core.op = MMP3_FREQ_OP_SHOW;
	pmu->pl_curr.dram.op = MMP3_FREQ_OP_SHOW;
	pmu->pl_curr.axi.op = MMP3_FREQ_OP_SHOW;
	mmp3_freq_plan_print_info(pmu, &pmu->pl_curr, 0);

	/* set default ddr table */
	for (j = 0; j < pmu->ppscnt; j++) {
		pmu->pps[j].dram.op = MMP3_FREQ_OP_GET;
		pmu->pps[j].dram.dsrc = pmu->pl_curr.dram.dsrc;
		pmu->pps[j].dram.pre_d = pmu->pl_curr.dram.pre_d;
		pmu->pps[j].dram.mode4x = pmu->pl_curr.dram.mode4x;
	}

	/* now let's hold source reference */
	mmp3_fccs_enable(pmu->pl_curr.core.psrc);
	mmp3_fccs_enable(pmu->pl_curr.dram.dsrc);
	mmp3_fccs_enable(pmu->pl_curr.axi.asrc);

	dfcsch = NULL;
	dfcsch_active = 0;

	register_cpu_notifier(&mmp3_pm_cpu_notifier);

	mmp3_setfreq(MMP3_CLK_AXI_1, 250000); /* lower default fabric speed*/

	/* register fb notifier */
	fb_register_client(&mmp3_fb_notif);

	return 0;
}

postcore_initcall(mmp3_pm_init);
