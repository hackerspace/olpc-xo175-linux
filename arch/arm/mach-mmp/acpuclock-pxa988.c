/*
 *  linux/arch/arm/mach-mmp/acpuclock-pxa988.c
 *
 *  Author:	Zhoujie Wu <zjwu@marvell.com>
 *  Copyright:	(C) 2012 Marvell International Ltd.
 *
 *  based on arch/arm/mach-mmp/acpuclock-pxa910.c
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2012 Marvell International Ltd.
 * All Rights Reserved
 */
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/delay.h>
#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/pm_qos_params.h>
#include <asm/io.h>
#include <mach/cputype.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-apmu.h>
#include <mach/regs-ciu.h>
#include <mach/regs-mcu.h>
#include <mach/clock-pxa988.h>
#include <mach/pxa988_lowpower.h>
#include <mach/pxa988_ddr.h>
#include <plat/debugfs.h>

/* core,ddr,axi clk src sel set register desciption */
union pmum_fccr {
	struct {
		unsigned int pll1fbd:9;
		unsigned int pll1refd:5;
		unsigned int pll1cen:1;
		unsigned int mfc:1;
		unsigned int reserved0:3;
		unsigned int axiclksel0:1;
		unsigned int reserved1:3;
		unsigned int ddrclksel:2;
		unsigned int axiclksel1:1;
		unsigned int seaclksel:2;
		unsigned int i2sclksel:1;
		unsigned int mohclksel:3;
	} b;
	unsigned int v;
};

/* core,ddr,axi clk src sel status register description */
union pmua_pllsel {
	struct {
		unsigned int cpclksel:2;
		unsigned int apclksel:2;
		unsigned int ddrclksel:2;
		unsigned int axiclksel:2;
		unsigned int reserved0:24;
	} b;
	unsigned int v;
};

/* core,ddr,axi clk div and fc trigger register description */
union pmua_cc {
	struct {
		unsigned int core_clk_div:3;
		unsigned int bus_mc_clk_div:3;
		unsigned int biu_clk_div:3;
		unsigned int l2_clk_div:3;
		unsigned int ddr_clk_div:3;
		unsigned int bus_clk_div:3;
		unsigned int async1:1;
		unsigned int async2:1;
		unsigned int async3:1;
		unsigned int async3_1:1;
		unsigned int async4:1;
		unsigned int async5:1;
		unsigned int core_freq_chg_req:1;
		unsigned int ddr_freq_chg_req:1;
		unsigned int bus_freq_chg_req:1;
		unsigned int core_allow_spd_chg:1;
		unsigned int core_dyn_fc:1;
		unsigned int dclk_dyn_fc:1;
		unsigned int aclk_dyn_fc:1;
		unsigned int core_rd_st_clear:1;
	} b;
	unsigned int v;
};

/* peri clk div set register description */
union pmua_cc2 {
	struct {
		unsigned int peri_clk_div:3;
		unsigned int peri_clk_dis:1;
		unsigned int reserved0:12;
		unsigned int cpu0_core_rst:1;
		unsigned int reserved1:1;
		unsigned int cpu0_dbg_rst:1;
		unsigned int cpu0_wdt_rst:1;
		unsigned int cpu1_core_rst:1;
		unsigned int reserved2:1;
		unsigned int cpu1_dbg_rst:1;
		unsigned int cpu1_wdt_rst:1;
		unsigned int reserved3:8;
	} b;
	unsigned int v;
};

/* core,ddr,axi div status register description */
union pmua_dm_cc {
	struct {
		unsigned int core_clk_div:3;
		unsigned int bus_mc_clk_div:3;
		unsigned int biu_clk_div:3;
		unsigned int l2_clk_div:3;
		unsigned int ddr_clk_div:3;
		unsigned int bus_clk_div:3;
		unsigned int async1:1;
		unsigned int async2:1;
		unsigned int async3:1;
		unsigned int async3_1:1;
		unsigned int async4:1;
		unsigned int async5:1;
		unsigned int cp_rd_status:1;
		unsigned int ap_rd_status:1;
		unsigned int cp_fc_done:1;
		unsigned int ap_fc_done:1;
		unsigned int dclk_fc_done:1;
		unsigned int aclk_fc_done:1;
		unsigned int reserved:2;
	} b;
	unsigned int v;
};

/* peri clk src sel status register description */
union pmua_dm_cc2 {
	struct {
		unsigned int peri_clk_div:3;
		unsigned int reserved:29;
	} b;
	unsigned int v;
};


#define AP_SRC_SEL_MASK		0x7
#define UNDEF_OP		-1
#define MHZ			(1000 * 1000)
#define MHZ_TO_KHZ		(1000)


/*
 * AP clock source:
 * 0x0 = PLL1 624 MHz
 * 0x1 = PLL1 1248 MHz  or PLL3_CLKOUT
 * (depending on PLL3_CR[18])
 * 0x2 = PLL2_CLKOUT
 * 0x3 = PLL2_CLKOUTP
 */
enum ap_clk_sel {
	AP_CLK_SRC_PLL1_624 = 0x0,
	AP_CLK_SRC_PLL1_1248 = 0x1,
	AP_CLK_SRC_PLL2 = 0x2,
	AP_CLK_SRC_PLL2P = 0x3,
	AP_CLK_SRC_PLL3P = 0x11,
};

/*
 * DDR/AXI clock source:
 * 0x0 = PLL1 416 MHz
 * 0x1 = PLL1 624 MHz
 * 0x2 = PLL2_CLKOUT
 * 0x3 = PLL2_CLKOUTP
 */
enum ddr_axi_clk_sel {
	DDR_AXI_CLK_SRC_PLL1_416 = 0x0,
	DDR_AXI_CLK_SRC_PLL1_624 = 0x1,
	DDR_AXI_CLK_SRC_PLL2 = 0x2,
	DDR_AXI_CLK_SRC_PLL2P = 0x3,
};

enum ddr_type {
	LPDDR2_400M = 0,
	LPDDR2_533M,
	DDR3,
};

struct pxa988_cpu_opt {
	unsigned int pclk;		/* core clock */
	unsigned int l2clk;		/* L2 cache interface clock */
	unsigned int pdclk;		/* DDR interface clock */
	unsigned int baclk;		/* bus interface clock */
	unsigned int periphclk;		/* PERIPHCLK */
	enum ap_clk_sel ap_clk_sel;	/* core src sel val */
	struct clk *parent;		/* core clk parent node */
	unsigned int ap_clk_src;	/* core src rate */
	unsigned int pclk_div;		/* core clk divider*/
	unsigned int l2clk_div;		/* L2 clock divider */
	unsigned int pdclk_div;		/* DDR interface clock divider */
	unsigned int baclk_div;		/* bus interface clock divider */
	unsigned int periphclk_div;	/* PERIPHCLK divider */
};

struct pxa988_ddr_axi_opt {
	unsigned int dclk;		/* ddr clock */
	unsigned int ddr_tbl_index;	/* ddr FC table index */
	unsigned int aclk;		/* axi clock */
	enum ddr_axi_clk_sel ddr_clk_sel;/* ddr src sel val */
	enum ddr_axi_clk_sel axi_clk_sel;/* axi src sel val */
	unsigned int ddr_clk_src;	/* ddr src rate */
	unsigned int axi_clk_src;	/* axi src rate */
	struct clk *ddr_parent;		/* ddr clk parent node */
	struct clk *axi_parent;		/* axi clk parent node */
	unsigned int dclk_div;		/* ddr clk divider */
	unsigned int aclk_div;		/* axi clk divider */
};

/*
 * Below struct is used to describe cpu, ddr type and the corresponding
 * OPs used for the platform. chipid and ddrtype actually is SW flag.
 * 1. As HW chipid maybe the same in different chips, fuseid and chipid
 * is used to together to identify the differnet chip.
 * SW could hack below chipid to any value, as long as we could match
 * the chip and corresponding ops.
 * 2. DDRtype is neccessary, as we may use the same chip with different
 * DDR in different platform, such as LPDDR400 and LPDDR533. It also
 * possible that we could NOT run up to 533M even we use LPDDR533
 * due to silicon limitation. We could not only depend on the information
 * read from HW. It is better that platform tell us the ddrtype. Then FC
 * could know which DDR OPs could be used.
 */
struct platform_opt {
	unsigned int cpuid;
	unsigned int chipid;
	enum ddr_type ddrtype;
	char *cpu_name;
	struct pxa988_cpu_opt *cpu_opt;
	unsigned int cpu_opt_size;
	struct pxa988_ddr_axi_opt *ddr_axi_opt;
	unsigned int ddr_axi_opt_size;
};

/* DDR fc table: 0 - non flag; 1 - pause flag; 2 - end flag */
enum ddr_fc_table_flag {
	DDR_FC_TABLE_NONE = 0,
	DDR_FC_TABLE_PAUSE = 1,
	DDR_FC_TABLE_END = 2,
};

struct ddr_fc_table_cmd {
	unsigned int reg;
	unsigned int val;
	enum ddr_fc_table_flag flag;
};

/* mutex lock protecting frequency change */
static DEFINE_MUTEX(core_freqs_mutex);
static DEFINE_MUTEX(ddr_freqs_mutex);
static DEFINE_SPINLOCK(fc_seq_lock);

/* current platform OP struct */
static struct platform_opt *cur_platform_opt;

/* current core OP */
static struct pxa988_cpu_opt *cur_cpu_op;

/* current DDR/AXI OP */
static struct pxa988_ddr_axi_opt *cur_ddraxi_op;

/* record DDR request from CP, only for debugfs show function */
static bool cp_reset_block_ddr_fc;

static void get_cur_cpu_op(struct pxa988_cpu_opt *cop);
static void get_cur_ddr_axi_op(struct pxa988_ddr_axi_opt *cop);


/*
 * PHY setting need to be tuned on real silicon by SV/DE.
 * Currently, we use the default values provided by DE.
 * table_idx is set to 0xf as default, which means invalid table index
 * It will be set to the correct value in pxa988_ddr_fc_table()
 */
static struct platform_ddr_setting lpddr2_setting[] = {
	{
		.ddr_freq = 156,
		.cas_latency = 0x1,	/* RL3/WL1 */
		.table_idx = 0xf,
		.timing = {
			.entry[0] = {DMCU_SDRAM_TIMING1, 0x488a0065},
			.entry[1] = {DMCU_SDRAM_TIMING2, 0x42330155},
			.entry[2] = {DMCU_SDRAM_TIMING3, 0x20161612},
			.entry[3] = {DMCU_SDRAM_TIMING4, 0x3022643d},
			.entry[4] = {DMCU_SDRAM_TIMING5, 0x04070082},
			.entry[5] = {DMCU_SDRAM_TIMING6, 0x00f0e49c},
			.entry[6] = {DMCU_SDRAM_TIMING7, 0x00008801},
			.entry[7] = {DMCU_SDRAM_TIMING8, 0x00000000},
		},
		.phy = {
			.entry[0] = {DMCU_PHY_CTRL3, 0x00004444},
			.entry[1] = {DMCU_PHY_CTRL7, 0x13300aa9},
			.entry[2] = {DMCU_PHY_CTRL8, 0x03300aa0},
			.entry[3] = {DMCU_PHY_CTRL9, 0x000000aa},
		},

	},
	{
		.ddr_freq = 208,
		.cas_latency = 0x2,	/* RL4/WL2 */
		.table_idx = 0xf,
		.timing = {
			.entry[0] = {DMCU_SDRAM_TIMING1, 0x488e0065},
			.entry[1] = {DMCU_SDRAM_TIMING2, 0x534401c5},
			.entry[2] = {DMCU_SDRAM_TIMING3, 0x201e1e12},
			.entry[3] = {DMCU_SDRAM_TIMING4, 0x30228452},
			.entry[4] = {DMCU_SDRAM_TIMING5, 0x058900b2},
			.entry[5] = {DMCU_SDRAM_TIMING6, 0x01312cd0},
			.entry[6] = {DMCU_SDRAM_TIMING7, 0x00008801},
			.entry[7] = {DMCU_SDRAM_TIMING8, 0x00000000},
		},
		.phy = {
			.entry[0] = {DMCU_PHY_CTRL3, 0x00004444},
			.entry[1] = {DMCU_PHY_CTRL7, 0x13300aa9},
			.entry[2] = {DMCU_PHY_CTRL8, 0x03300aa0},
			.entry[3] = {DMCU_PHY_CTRL9, 0x000000aa},
		},

	},
	{
		.ddr_freq = 312,
		.cas_latency = 0x3,	/* RL5/WL2 */
		.table_idx = 0xf,
		.timing = {
			.entry[0] = {DMCU_SDRAM_TIMING1, 0x4cd40065},
			.entry[1] = {DMCU_SDRAM_TIMING2, 0x74650295},
			.entry[2] = {DMCU_SDRAM_TIMING3, 0x202c2c1b},
			.entry[3] = {DMCU_SDRAM_TIMING4, 0x3022c47a},
			.entry[4] = {DMCU_SDRAM_TIMING5, 0x080e0102},
			.entry[5] = {DMCU_SDRAM_TIMING6, 0x01d1c539},
			.entry[6] = {DMCU_SDRAM_TIMING7, 0x00008801},
			.entry[7] = {DMCU_SDRAM_TIMING8, 0x00000000},
		},
		.phy = {
			.entry[0] = {DMCU_PHY_CTRL3, 0x00004444},
			.entry[1] = {DMCU_PHY_CTRL7, 0x13300aa9},
			.entry[2] = {DMCU_PHY_CTRL8, 0x03300aa0},
			.entry[3] = {DMCU_PHY_CTRL9, 0x000000aa},
		},

	},
	{
		.ddr_freq = 400,
		.cas_latency = 0x4,	/* RL6/WL3 */
		.table_idx = 0xf,
		.timing = {
			.entry[0] = {DMCU_SDRAM_TIMING1, 0x4cda0065},
			.entry[1] = {DMCU_SDRAM_TIMING2, 0x94860345},
			.entry[2] = {DMCU_SDRAM_TIMING3, 0x2038381b},
			.entry[3] = {DMCU_SDRAM_TIMING4, 0x3022f89b},
			.entry[4] = {DMCU_SDRAM_TIMING5, 0x0a110144},
			.entry[5] = {DMCU_SDRAM_TIMING6, 0x0242418f},
			.entry[6] = {DMCU_SDRAM_TIMING7, 0x00008801},
			.entry[7] = {DMCU_SDRAM_TIMING8, 0x00000000},
		},
		.phy = {
			.entry[0] = {DMCU_PHY_CTRL3, 0x00004444},
			.entry[1] = {DMCU_PHY_CTRL7, 0x13300aa9},
			.entry[2] = {DMCU_PHY_CTRL8, 0x03300aa0},
			.entry[3] = {DMCU_PHY_CTRL9, 0x000000aa},
		},

	},
	{
		.ddr_freq = 533,
		.cas_latency = 0x6,	/* RL8/WL4 */
		.table_idx = 0xf,
		.timing = {
			.entry[0] = {DMCU_SDRAM_TIMING1, 0x51220065},
			.entry[1] = {DMCU_SDRAM_TIMING2, 0xc6a80465},
			.entry[2] = {DMCU_SDRAM_TIMING3, 0x204b4b24},
			.entry[3] = {DMCU_SDRAM_TIMING4, 0x301350d1},
			.entry[4] = {DMCU_SDRAM_TIMING5, 0x0d9701b4},
			.entry[5] = {DMCU_SDRAM_TIMING6, 0x03030215},
			.entry[6] = {DMCU_SDRAM_TIMING7, 0x00008801},
			.entry[7] = {DMCU_SDRAM_TIMING8, 0x00000000},
		},
		.phy = {
			.entry[0] = {DMCU_PHY_CTRL3, 0x00004444},
			.entry[1] = {DMCU_PHY_CTRL7, 0x13300aa9},
			.entry[2] = {DMCU_PHY_CTRL8, 0x03300aa0},
			.entry[3] = {DMCU_PHY_CTRL9, 0x000000aa},
		},
	},

};

static inline void insert_entry_ex(struct ddr_fc_table_cmd *cmd,
		unsigned int table, unsigned int entry)
{
	unsigned int regval;

	if (cmd == NULL) {
		pr_err("<PM> DDR fc table entry setup error!\n");
		return;
	}

	regval = cmd->reg;

	__raw_writel(cmd->val, DMCU_VIRT_REG(DMCU_HWTDAT0));

	if (cmd->flag == DDR_FC_TABLE_PAUSE)
		regval |= DMCU_HWTPAUSE;
	else if (cmd->flag == DDR_FC_TABLE_END)
		regval |= DMCU_HWTEND;
	__raw_writel(regval, DMCU_VIRT_REG(DMCU_HWTDAT1));

	regval = (((table << 5) | entry) & 0xff);
	__raw_writel(regval, DMCU_VIRT_REG(DMCU_HWTCTRL));
}

#define INSERT_ENTRY(regid, value, table)			\
	do {							\
		cmd.reg = DMCU_PHYS_REG(regid);			\
		cmd.val = value;				\
		cmd.flag = DDR_FC_TABLE_NONE;			\
		insert_entry_ex(&cmd, table, entry);		\
		entry++;					\
	} while (0)

#define PAUSE_ENTRY(regid, value, table)			\
	do {							\
		cmd.reg = DMCU_PHYS_REG(regid);			\
		cmd.val = value;				\
		cmd.flag = DDR_FC_TABLE_PAUSE;			\
		insert_entry_ex(&cmd, table, entry);		\
		entry++;					\
	} while (0)

#define LAST_ENTRY(regid, value, table)				\
	do {							\
		cmd.reg = DMCU_PHYS_REG(regid);			\
		cmd.val = value;				\
		cmd.flag = DDR_FC_TABLE_END;			\
		insert_entry_ex(&cmd, table, entry);		\
		entry++;					\
	} while (0)

/* Table 0 is used for PHY DLL reset/update when DDR resume from shutdown */
static void pxa988_ddr_lpm_table(void)
{
	struct ddr_fc_table_cmd cmd;
	unsigned int entry = 0;

	/* reset master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, 0x20000000, 0);
	/* udpate master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, 0x40000000, 0);
	/* synchronize 2x clock */
	LAST_ENTRY(DMCU_PHY_CTRL14, 0x80000000, 0);
}

/* #define DDR_FC_PHY_TUNING 1 */
static void pxa988_ddr_fc_table_lpddr2(struct platform_ddr_setting *setting)
{
	struct ddr_timing *timing = &setting->timing;
#ifdef DDR_FC_PHY_TUNING
	struct ddr_phy *phy = &setting->phy;
#endif
	struct ddr_fc_table_cmd cmd;
	unsigned int entry = 0, regval = 0, map = 0, table = 0;

	table = setting->table_idx;

	if (__raw_readl(DMCU_VIRT_REG(DMCU_MAP_CS0)) & DMCU_MAP_VALID)
		map |= DMCU_CMD_CSSEL_CS0;
	if (__raw_readl(DMCU_VIRT_REG(DMCU_MAP_CS1)) & DMCU_MAP_VALID)
		map |= DMCU_CMD_CSSEL_CS1;

	/*
	 * We use PAUSE, so for each frequency point, 2 tables are required.
	 * For the 1st table, update DDR timing parameters and reset DLL.
	 * And in the 2nd table, update DDR mode registers.
	 */

	/* 1. update all timing parameters */

	/* a) update CAS */
	regval = __raw_readl(DMCU_VIRT_REG(DMCU_SDRAM_CTRL4))
		& (~DMCU_SDRAM_CTRL4_CL_MASK);
	regval |= setting->cas_latency << DMCU_SDRAM_CTRL4_CL_SHIFT;
	INSERT_ENTRY(DMCU_SDRAM_CTRL4, regval, table); /* CAS latency */

	/* b) update all timing registers, and then PAUSE */
	/*
	 * accoring to DE, we only need to update timing register 1~3, 5, 6
	 * since timing register 4, 7, 8 are same for all frequency
	 */
	INSERT_ENTRY(timing->entry[0].reg, timing->entry[0].val, table);
	INSERT_ENTRY(timing->entry[1].reg, timing->entry[1].val, table);
	INSERT_ENTRY(timing->entry[2].reg, timing->entry[2].val, table);
	INSERT_ENTRY(timing->entry[4].reg, timing->entry[4].val, table);

#ifndef DDR_FC_PHY_TUNING
	PAUSE_ENTRY(timing->entry[5].reg, timing->entry[5].val, table);
#else
	INSERT_ENTRY(timing->entry[5].reg, timing->entry[5].val, table);

	/* 2. update PHY registers */

	/*
	 * TODO:
	 * PHY setting update may not mandatory, it depends on real silicon
	 * if possible, we will remove this part later
	 */
	INSERT_ENTRY(phy->entry[0].reg, phy->entry[0].val, table);
	INSERT_ENTRY(phy->entry[1].reg, phy->entry[1].val, table);
	INSERT_ENTRY(phy->entry[2].reg, phy->entry[2].val, table);
	PAUSE_ENTRY(phy->entry[3].reg, phy->entry[3].val, table);
#endif

	/* 3. reset DLL */
	/* reset master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, 0x20000000, table);
	/* update master DLL */
	INSERT_ENTRY(DMCU_PHY_CTRL14, 0x40000000, table);
	/* synchronize 2x clock */
	INSERT_ENTRY(DMCU_PHY_CTRL14, 0x80000000, table);
	/* halt scheduler */
	LAST_ENTRY(DMCU_SDRAM_CTRL14, 0x2, table);

	/* 4. update DDR mode registers, programmed in 2nd table */
	entry = 0;
	INSERT_ENTRY(DMCU_USER_COMMAND1, (map | 0x20001), (table + 1));
	INSERT_ENTRY(DMCU_USER_COMMAND1, (map | 0x20002), (table + 1));
	/* resume scheduler */
	LAST_ENTRY(DMCU_SDRAM_CTRL14, 0, (table + 1));
}

/* FIXME: DDR3 is not supported on emei, so currently this function is empty */
static void __maybe_unused pxa988_ddr_fc_table_ddr3(
		struct platform_ddr_setting *setting, unsigned int table)
{
}

/* Current design only support 3 DDR ops at max */
#define MAX_DDR_OP_NUM	3
static void pxa988_ddr_fc_table(unsigned int *ddr_freq,
				unsigned int ddr_freq_count)
{
	unsigned int type;
	int i, j;

	if (ddr_freq_count > MAX_DDR_OP_NUM)
		pr_warn("<PM> Cannot support more than 3 DDR ops!\n");

	type = __raw_readl(DMCU_VIRT_REG(DMCU_SDRAM_CTRL4)) &
					DMCU_SDRAM_TYPE_MASK;
	switch (type) {
	case DMCU_SDRAM_TYPE_DDR3:
		/* FIXME: DDR3 is not supported now */
		break;
	case DMCU_SDRAM_TYPE_LPDDR2:
		for (i = 0; i < ddr_freq_count; i++) {
			for (j = 0; j < ARRAY_SIZE(lpddr2_setting); j++) {
				if (lpddr2_setting[j].ddr_freq == ddr_freq[i]) {
					/* init the table_idx first */
					lpddr2_setting[j].table_idx = i * 2 + 1;
					pxa988_ddr_fc_table_lpddr2(
							&lpddr2_setting[j]);
					break;
				}

				/*
				 * If DDR freq is not in lpddr2_setting,
				 * we should stop here, or it will using
				 * incorrect setting for DDR freq-change,
				 * which may cause system unstable, so BUG_ON.
				 */
				if (j == ARRAY_SIZE(lpddr2_setting)) {
					pr_err("<PM> DDR freq %d is not supported!\n"
						"Continue may cause system untable!",
						ddr_freq[i]);
					BUG();
				}
			}
		}
		break;
	default:
		pr_err("<PM> unsupported DDR type\n");
		break;
	}
}

/*
 * For 988:
 * L2CLK = PCLK / (L2_CLK_DIV +1)
 * BIU_CLK = L2_CLK / (BIU_CLK_DIV +1)
 * MC_CLK = L2_CLK / (MC_CLK_DIV +1)
 * Emei Z0:
 * PERIPHCLK = PCLK /2 * (PERI_CLK_DIV+1)
 * Emei A0:
 * PERIPHCLK = PCLK /4 * (PERI_CLK_DIV+1)
 *
 * FIXME:
 * 1. pdclk/paclk can use 1:1 with l2clk when in low speed,
 * and 1:2 when pclk is in high speed
 * 2. For Emei Z0, PERIPHCLK is divided from pclk, the divider
 * is even and ranges from 2~16. It is better to select a lower
 * frequency for power saving since it does NOT have very higher
 * frequency requirement. Current DE suggests to use pclk/8 as
 * PERIPHCLK.
 * 3. For Emei A0, PERIPHCLK divider is from 4~32.
 */
static struct pxa988_cpu_opt pxa988_op_array[] = {
	{
		.pclk = 156,
		.l2clk = 156,
		.pdclk = 78,
		.baclk = 78,
		.periphclk = 19,
		.ap_clk_sel = AP_CLK_SRC_PLL1_624,
	},
	{
		.pclk = 312,
		.l2clk = 312,
		.pdclk = 156,
		.baclk = 156,
		.periphclk = 39,
		.ap_clk_sel = AP_CLK_SRC_PLL1_624,
	},
	{
		.pclk = 624,
		.l2clk = 312,
		.pdclk = 312,
		.baclk = 156,
		.periphclk = 78,
		.ap_clk_sel = AP_CLK_SRC_PLL1_624,
	},
	{
		.pclk = 800,
		.l2clk = 400,
		.pdclk = 400,
		.baclk = 200,
		.periphclk = 100,
		.ap_clk_sel = AP_CLK_SRC_PLL2,
	},
#if 0
	/*
	 * pll3 has duty cycle issue on Z1 if its rate is higher than 800M,
	 * disable 1G PP at first on Z1
	 */
	{
		.pclk = 1000,
		.l2clk = 500,
		.pdclk = 250,
		.baclk = 250,
		.periphclk = 125,
		.ap_clk_sel = AP_CLK_SRC_PLL3P,
	},
#endif
	{
		.pclk = 1248,
		.l2clk = 624,
		.pdclk = 312,
		.baclk = 312,
		.periphclk = 156,
		.ap_clk_sel = AP_CLK_SRC_PLL1_1248,
	},
};

/*
 * 1) On Emei Z0, only support three ddr rates, be careful
 * when changing the PP table. The table should only have
 * three PPs and the PPs are ordered ascending.
 * 2) Table base DDR FC is implemented. The corresponding
 * ddr_tbl_index should be 1,3,5. If the PP tbl size is larger
 * than 3, it will only fill the first three rates' timing to tbl 1,3,5
 * 3) Make sure the ddr and axi rate's src sel is correct
 * 4) FIXME: high ddr request means high axi is NOT
 * very reasonable
 */
static struct pxa988_ddr_axi_opt lpddr400_axi_oparray[] = {
	{
		.dclk = 156,
		.ddr_tbl_index = 1,
		.aclk = 78,
		.ddr_clk_sel = DDR_AXI_CLK_SRC_PLL1_624,
		.axi_clk_sel = DDR_AXI_CLK_SRC_PLL1_624,
	},
#if 0
	{
		.dclk = 208,
		.ddr_tbl_index = 3,
		.aclk = 156,
		.ddr_clk_sel = DDR_AXI_CLK_SRC_PLL1_416,
		.axi_clk_sel = DDR_AXI_CLK_SRC_PLL1_624,
	},
#endif
	{
		.dclk = 312,
		.ddr_tbl_index = 3,
		.aclk = 156,
		.ddr_clk_sel = DDR_AXI_CLK_SRC_PLL1_624,
		.axi_clk_sel = DDR_AXI_CLK_SRC_PLL1_624,
	},
	/*
	 * Enable 400Mhz as it is stable after updating the DDR timing.
	 * Also follow DE's suggestion to use PLL2OUTP as DDR 400MHz
	 * clock source.
	 */
#if 1
	{
		.dclk = 400,
		.ddr_tbl_index = 5,
		.aclk = 208,
		.ddr_clk_sel = DDR_AXI_CLK_SRC_PLL2P,
		.axi_clk_sel = DDR_AXI_CLK_SRC_PLL1_416,
	},
#endif
};

static struct platform_opt platform_op_arrays[] = {
	{
		.cpuid = 0x8000,
		.chipid = 0xc928,
		.ddrtype = LPDDR2_400M,
		.cpu_name = "PXA988",
		.cpu_opt = pxa988_op_array,
		.cpu_opt_size = ARRAY_SIZE(pxa988_op_array),
		.ddr_axi_opt = lpddr400_axi_oparray,
		.ddr_axi_opt_size = ARRAY_SIZE(lpddr400_axi_oparray),
	},
};

static struct clk *cpu_sel2parent(enum ap_clk_sel ap_sel)
{
	if (ap_sel == AP_CLK_SRC_PLL1_624)
		return clk_get_sys(NULL, "pll1_624");
	else if (ap_sel == AP_CLK_SRC_PLL1_1248)
		return clk_get_sys(NULL, "pll1_1248");
	else if (ap_sel == AP_CLK_SRC_PLL2)
		return clk_get_sys(NULL, "pll2");
	else if (ap_sel == AP_CLK_SRC_PLL2P)
		return clk_get_sys(NULL, "pll2p");
	else if (ap_sel == AP_CLK_SRC_PLL3P)
		return clk_get_sys(NULL, "pll3p");
	else
		return ERR_PTR(-ENOENT);
}

static struct clk *ddr_axi_sel2parent(enum ddr_axi_clk_sel ddr_axi_sel)
{
	if (ddr_axi_sel == DDR_AXI_CLK_SRC_PLL1_416)
		return clk_get_sys(NULL, "pll1_416");
	else if (ddr_axi_sel == DDR_AXI_CLK_SRC_PLL1_624)
		return clk_get_sys(NULL, "pll1_624");
	else if (ddr_axi_sel == DDR_AXI_CLK_SRC_PLL2)
		return clk_get_sys(NULL, "pll2");
	else if (ddr_axi_sel == DDR_AXI_CLK_SRC_PLL2P)
		return clk_get_sys(NULL, "pll2p");
	else
		return ERR_PTR(-ENOENT);
}

static void __init __init_platform_opt(void)
{
	unsigned int i, chipid;
	enum ddr_type ddrtype;
	struct platform_opt *proc;

	if (cpu_is_pxa988())
		chipid = 0xc928;
	/*
	 * FIXME: ddr type is hacked here as it can not be read from
	 * HW, but FC code needs this info to identify DDR OPs.
	 */
	ddrtype = LPDDR2_400M;

	for (i = 0; i < ARRAY_SIZE(platform_op_arrays); i++) {
		proc = platform_op_arrays + i;
		if ((proc->chipid == chipid) &&
			(proc->ddrtype == ddrtype))
			break;
	}
	BUG_ON(i == ARRAY_SIZE(platform_op_arrays));
	cur_platform_opt = proc;
}

static void __init __init_ddr_table(void)
{
	unsigned int ddr_freq[MAX_DDR_OP_NUM], ddr_freq_count = 0;
	int i;

	if (cur_platform_opt == NULL) {
		pr_err("<PM> cur_platform_opt is NULL!"
			"It must be init before DDR table update!\n");
		return;
	}

	/*
	 * Pick the dclk info from cur_platform_opt,
	 * in order to update DDR FC table
	 */
	for (i = 0; i < MAX_DDR_OP_NUM; i++)
		ddr_freq[i] = 0;

	/*
	 * Available DDR freq are stored in cur_platform_opt->ddr_axi_opt
	 * They are sorted ascending and will never have repeated value
	 */
	for (i = 0; i < cur_platform_opt->ddr_axi_opt_size; i++) {
		if (i == MAX_DDR_OP_NUM) {
			/*
			 * If more than 3 DDR ops are found,
			 * only store the first 3 ones.
			 */
			pr_warn("<PM> too many DDR ops!\n");
			goto ddr_table_update;

		}
		ddr_freq[i] = cur_platform_opt->ddr_axi_opt[i].dclk;
		ddr_freq_count++;
	}

ddr_table_update:
	/* update the DDR table for exit of DDR shutdown */
	pxa988_ddr_lpm_table();

	/* update the DDR table for FC */
	pxa988_ddr_fc_table(ddr_freq, ddr_freq_count);
}

static void __init __init_cpu_opt(void)
{
	struct clk *parent = NULL;
	struct pxa988_cpu_opt *cpu_opt, *cop;
	unsigned int cpu_opt_size = 0, i;

	cpu_opt = cur_platform_opt->cpu_opt;
	cpu_opt_size = cur_platform_opt->cpu_opt_size;

	pr_info("pclk(src:sel,div) l2clk(src,div)\t"
		"pdclk(src,div)\tbaclk(src,div)\t"
		"periphclk(src,div)\n");

	for (i = 0; i < cpu_opt_size; i++) {
		cop = &cpu_opt[i];
		parent = cpu_sel2parent(cop->ap_clk_sel);
		BUG_ON(IS_ERR(parent));
		cop->parent = parent;
		cop->ap_clk_src = clk_get_rate(parent) / MHZ;
		cop->pclk_div =
			cop->ap_clk_src / cop->pclk - 1;
		if (cop->l2clk) {
			cop->l2clk_div =
				cop->pclk / cop->l2clk - 1;
			cop->pdclk_div =
				cop->l2clk / cop->pdclk - 1;
			cop->baclk_div =
				cop->l2clk / cop->baclk - 1;
		} else {
			cop->pdclk_div =
				cop->pclk / cop->pdclk - 1;
			cop->baclk_div =
				cop->pclk / cop->baclk - 1;
		}
		if (cop->periphclk)
			cop->periphclk_div =
				cop->pclk / (2 * cop->periphclk) - 1;

		pr_info("%d(%d:%d,%d)\t%d([%s],%d)\t"
			"%d([%s],%d)\t%d([%s],%d)\t%d([%s],%d)\n",
			cop->pclk, cop->ap_clk_src,
			cop->ap_clk_sel & AP_SRC_SEL_MASK,
			cop->pclk_div,
			cop->l2clk, cop->l2clk ? "pclk" : "NULL",
			cop->l2clk_div,
			cop->pdclk, cop->l2clk ? "l2clk" : "pclk",
			cop->pdclk_div,
			cop->baclk, cop->l2clk ? "l2clk" : "pclk",
			cop->baclk_div,
			cop->periphclk,
			cop->periphclk ? "pclk" : "NULL",
			cop->periphclk_div);
	}
}

static void __init __init_ddr_axi_opt(void)
{
	struct clk *parent = NULL;
	struct pxa988_ddr_axi_opt *ddr_axi_opt, *cop;
	unsigned int ddr_axi_opt_size = 0, i;

	ddr_axi_opt = cur_platform_opt->ddr_axi_opt;
	ddr_axi_opt_size = cur_platform_opt->ddr_axi_opt_size;

	pr_info("dclk(src:sel,div) aclk(src:sel,div)\n");
	for (i = 0; i < ddr_axi_opt_size; i++) {
		cop = &ddr_axi_opt[i];
		parent = ddr_axi_sel2parent(cop->ddr_clk_sel);
		BUG_ON(IS_ERR(parent));
		cop->ddr_parent = parent;
		cop->ddr_clk_src =
			clk_get_rate(parent) / MHZ;
		cop->dclk_div =
			cop->ddr_clk_src / (2 * cop->dclk) - 1;

		parent = ddr_axi_sel2parent(cop->axi_clk_sel);
		BUG_ON(IS_ERR(parent));
		cop->axi_parent = parent;
		cop->axi_clk_src =
			clk_get_rate(parent) / MHZ;
		cop->aclk_div =
			cop->axi_clk_src / cop->aclk - 1;

		pr_info("%d(%d:%d,%d)\t%d(%d:%d,%d)\n",
			cop->dclk, cop->ddr_clk_src,
			cop->ddr_clk_sel, cop->dclk_div,
			cop->aclk, cop->axi_clk_src,
			cop->axi_clk_sel, cop->aclk_div);
	}
}

static void __init __init_fc_setting(void)
{
	unsigned int regval;
	union pmua_cc cc_ap, cc_cp;
	/*
	 * enable AP FC done interrupt for one step,
	 * while not use three interrupts by three steps
	 */
	__raw_writel((1 << 1), APMU_IMR);

	/* always vote for CP allow AP FC */
	cc_cp.v = __raw_readl(APMU_CP_CCR);
	cc_cp.b.core_allow_spd_chg = 1;
	__raw_writel(cc_cp.v, APMU_CP_CCR);

	regval = __raw_readl(APMU_DEBUG);
	/* CA9 doesn't support halt acknowledge, mask it */
	regval |= (1 << 1);
	/*
	 * Always set AP_WFI_FC and CP_WFI_FC, then PMU will
	 * automaticlly send out clk-off ack when core is WFI
	 */
	regval |= (1 << 21) | (1 << 22);
	/*
	 * mask CP clk-off ack and cp halt ack for DDR/AXI FC
	 * this bits should be unmasked after cp is released
	 */
	regval |= (1 << 0) | (1 << 3);
	__raw_writel(regval, APMU_DEBUG);

	/* always use async for DDR, AXI interface */
	cc_ap.v = __raw_readl(APMU_CCR);
	cc_ap.b.async5 = 1;
	cc_ap.b.async4 = 1;
	cc_ap.b.async3_1 = 1;
	cc_ap.b.async3 = 1;
	cc_ap.b.async2 = 1;
	cc_ap.b.async1 = 1;
	__raw_writel(cc_ap.v, APMU_CCR);
}

static unsigned int cpu_rate2_op_index(unsigned int rate)
{
	unsigned int index;
	struct pxa988_cpu_opt *op_array =
		cur_platform_opt->cpu_opt;
	unsigned int op_array_size =
		cur_platform_opt->cpu_opt_size;

	if (unlikely(rate > op_array[op_array_size - 1].pclk))
		return op_array_size - 1;

	for (index = 0; index < op_array_size; index++)
		if (op_array[index].pclk >= rate)
			break;

	return index;
}

static unsigned int ddr_rate2_op_index(unsigned int rate)
{
	unsigned int index;
	struct pxa988_ddr_axi_opt *op_array =
		cur_platform_opt->ddr_axi_opt;
	unsigned int op_array_size =
		cur_platform_opt->ddr_axi_opt_size;

	if (unlikely(rate > op_array[op_array_size - 1].dclk))
		return op_array_size - 1;

	for (index = 0; index < op_array_size; index++)
		if (op_array[index].dclk >= rate)
			break;

	return index;
}

static int fc_lock_ref_cnt;
static void get_fc_lock(void)
{
	union pmua_dm_cc dm_cc_ap;

	fc_lock_ref_cnt++;

	if (fc_lock_ref_cnt == 1) {
		int timeout = 100000;

		/* AP-CP FC mutual exclusion */
		dm_cc_ap.v = __raw_readl(APMU_CCSR);
		while (dm_cc_ap.b.cp_rd_status && timeout) {
			dm_cc_ap.v = __raw_readl(APMU_CCSR);
			timeout--;
		}
		if (timeout <= 0) {
			pr_err("cp does not release its fc lock\n");
			BUG();
		}
	}
}

static void put_fc_lock(void)
{
	union pmua_cc cc_ap;

	fc_lock_ref_cnt--;

	if (fc_lock_ref_cnt < 0)
		pr_err("unmatched put_fc_lock\n");

	if (fc_lock_ref_cnt == 0) {
		/* write 1 to MOH_RD_ST_CLEAR to clear MOH_RD_STATUS */
		cc_ap.v = __raw_readl(APMU_CCR);
		cc_ap.b.core_rd_st_clear = 1;
		__raw_writel(cc_ap.v, APMU_CCR);
		cc_ap.b.core_rd_st_clear = 0;
		__raw_writel(cc_ap.v, APMU_CCR);
	}
}

static void get_cur_cpu_op(struct pxa988_cpu_opt *cop)
{
	union pmua_pllsel pllsel;
	union pmua_dm_cc dm_cc_ap;
	union pmua_cc cc_cp;
	union pmua_dm_cc2 dm_cc2_ap;
	unsigned int pll1_pll3_sel;
	struct clk *parent;

	get_fc_lock();

	dm_cc_ap.v = __raw_readl(APMU_CCSR);
	dm_cc2_ap.v = __raw_readl(APMU_CC2SR);
	cc_cp.v = __raw_readl(APMU_CP_CCR);
	cc_cp.b.core_rd_st_clear = 1;
	__raw_writel(cc_cp.v, APMU_CP_CCR);
	cc_cp.b.core_rd_st_clear = 0;
	__raw_writel(cc_cp.v, APMU_CP_CCR);
	pllsel.v = __raw_readl(APMU_PLL_SEL_STATUS);
	pll1_pll3_sel = __raw_readl(MPMU_PLL3CR);

	pr_debug("div%x sel%x\n", dm_cc_ap.v, pllsel.v);
	BUG_ON(!cop->parent);

	if (pllsel.b.apclksel == (cop->ap_clk_sel & AP_SRC_SEL_MASK))
		cop->ap_clk_src = clk_get_rate(cop->parent) / MHZ;
	else {
		if ((pllsel.b.apclksel == 0x1) && \
			(pll1_pll3_sel & (1 << 18))) {
			parent = cpu_sel2parent(AP_CLK_SRC_PLL3P);
			cop->ap_clk_src = clk_get_rate(parent) / MHZ;
		} else {
			/* err case : current src is NOT our target */
			parent = cpu_sel2parent(pllsel.b.apclksel);
			cop->parent = parent;
			cop->ap_clk_src = clk_get_rate(parent) / MHZ;
			pr_err("%s cpu clk tsrc:%d csel:%d\n",
				__func__, cop->ap_clk_src, pllsel.b.apclksel);
		}
	}
	cop->pclk = cop->ap_clk_src / (dm_cc_ap.b.core_clk_div + 1);
	if (cop->l2clk) {
		cop->l2clk = cop->pclk / (dm_cc_ap.b.l2_clk_div + 1);
		cop->pdclk = cop->l2clk / (dm_cc_ap.b.bus_mc_clk_div + 1);
		cop->baclk = cop->l2clk / (dm_cc_ap.b.biu_clk_div + 1);
	} else {
		cop->pdclk = cop->pclk / (dm_cc_ap.b.bus_mc_clk_div + 1);
		cop->baclk = cop->pclk / (dm_cc_ap.b.biu_clk_div + 1);
	}
	if (cop->periphclk) {
		cop->periphclk =
			cop->pclk / (dm_cc2_ap.b.peri_clk_div + 1) / 2;
	}

	put_fc_lock();
}

static void get_cur_ddr_axi_op(struct pxa988_ddr_axi_opt *cop)
{
	union pmua_pllsel pllsel;
	union pmua_dm_cc dm_cc_ap;
	union pmua_cc cc_cp;
	struct clk *parent;

	get_fc_lock();

	dm_cc_ap.v = __raw_readl(APMU_CCSR);
	cc_cp.v = __raw_readl(APMU_CP_CCR);
	cc_cp.b.core_rd_st_clear = 1;
	__raw_writel(cc_cp.v, APMU_CP_CCR);
	cc_cp.b.core_rd_st_clear = 0;
	__raw_writel(cc_cp.v, APMU_CP_CCR);
	pllsel.v = __raw_readl(APMU_PLL_SEL_STATUS);

	pr_debug("div%x sel%x\n", dm_cc_ap.v, pllsel.v);
	BUG_ON((!cop->ddr_parent) || (!cop->axi_parent));

	if (likely(pllsel.b.ddrclksel == cop->ddr_clk_sel))
		cop->ddr_clk_src = clk_get_rate(cop->ddr_parent) / MHZ;
	else {
		parent = ddr_axi_sel2parent(pllsel.b.ddrclksel);
		cop->ddr_parent = parent;
		cop->ddr_clk_src = clk_get_rate(parent) / MHZ;
		pr_err("%s ddr clk tsrc:%d csel:%d parent:%s\n",
			__func__, cop->ddr_clk_src,
			pllsel.b.ddrclksel, cop->ddr_parent->name);
	}
	if (likely(pllsel.b.axiclksel == cop->axi_clk_sel))
		cop->axi_clk_src = clk_get_rate(cop->axi_parent) / MHZ;
	else {
		parent = ddr_axi_sel2parent(pllsel.b.axiclksel);
		cop->axi_parent = parent;
		cop->axi_clk_src = clk_get_rate(parent) / MHZ;
		pr_err("%s axi clk tsrc:%d csel:%d parent:%s\n",
			__func__, cop->axi_clk_src,
			pllsel.b.axiclksel, cop->axi_parent->name);
	}
	cop->dclk = cop->ddr_clk_src / (dm_cc_ap.b.ddr_clk_div + 1) / 2;
	cop->aclk = cop->axi_clk_src / (dm_cc_ap.b.bus_clk_div + 1);

	put_fc_lock();
}

static void wait_for_fc_done(void)
{
	int timeout = 1000000;
	while (!((1 << 1) & __raw_readl(APMU_ISR)) && timeout)
		timeout--;
	if (timeout <= 0)
		panic("AP frequency change timeout!\n");
	__raw_writel(0x0, APMU_ISR);
}

static void pll1_pll3_switch(enum ap_clk_sel sel)
{
	unsigned int regval;

	if ((sel != AP_CLK_SRC_PLL3P) ||
		(sel != AP_CLK_SRC_PLL1_1248))
		return;

	regval = __raw_readl(MPMU_PLL3CR);
	if (sel == AP_CLK_SRC_PLL1_1248)
		regval &= ~(1 << 18);
	else
		regval |= (1 << 18);
	__raw_writel(regval, MPMU_PLL3CR);
}

static void set_ap_clk_sel(struct pxa988_cpu_opt *top)
{
	union pmum_fccr fccr;

	pll1_pll3_switch(top->ap_clk_sel);

	fccr.v = __raw_readl(MPMU_FCCR);
	fccr.b.mohclksel =
		top->ap_clk_sel & AP_SRC_SEL_MASK;
	__raw_writel(fccr.v, MPMU_FCCR);
}

static void set_periph_clk_div(struct pxa988_cpu_opt *top)
{
	union pmua_cc2 cc_ap2;

	cc_ap2.v = __raw_readl(APMU_CC2R);
	cc_ap2.b.peri_clk_div = top->periphclk_div;
	__raw_writel(cc_ap2.v, APMU_CC2R);
}

static void set_ddr_clk_sel(struct pxa988_ddr_axi_opt *top)
{
	union pmum_fccr fccr;

	fccr.v = __raw_readl(MPMU_FCCR);
	fccr.b.ddrclksel = top->ddr_clk_sel;
	__raw_writel(fccr.v, MPMU_FCCR);
}

static void set_axi_clk_sel(struct pxa988_ddr_axi_opt *top)
{
	union pmum_fccr fccr;

	fccr.v = __raw_readl(MPMU_FCCR);
	fccr.b.axiclksel0 = top->axi_clk_sel & 0x1;
	fccr.b.axiclksel1 = (top->axi_clk_sel & 0x2) >> 1;
	__raw_writel(fccr.v, MPMU_FCCR);
}

static void set_ddr_tbl_index(unsigned int index)
{
	unsigned int regval;

	pmu_register_lock();
	index = (index > 0x7) ? 0x7 : index;
	regval = __raw_readl(APMU_MC_HW_SLP_TYPE);
	regval &= ~(0x1 << 6);		/* enable tbl based FC */
	regval &= ~(0x7 << 3);		/* clear ddr tbl index */
	regval |= (index << 3);
	__raw_writel(regval, APMU_MC_HW_SLP_TYPE);
	pmu_register_unlock();
}

static void core_fc_seq(struct pxa988_cpu_opt *cop,
			    struct pxa988_cpu_opt *top)
{
	union pmua_cc cc_ap, cc_cp;

	/* 0) Pre FC : check CP allow AP FC voting */
	cc_cp.v = __raw_readl(APMU_CP_CCR);
	if (unlikely(!cc_cp.b.core_allow_spd_chg)) {
		pr_warning("%s CP doesn't allow AP FC!\n",
			__func__);
		cc_cp.b.core_allow_spd_chg = 1;
		__raw_writel(cc_cp.v, APMU_CP_CCR);
	}

	/* 1) Pre FC : AP votes allow FC */
	cc_ap.v = __raw_readl(APMU_CCR);
	cc_ap.b.core_allow_spd_chg = 1;

	/* 2) issue core FC */
	/* 2.1) set pclk src */
	set_ap_clk_sel(top);
	/* 2.2) select div for pclk, l2clk, pdclk, baclk */
	cc_ap.b.core_clk_div = top->pclk_div;
	if (top->l2clk)
		cc_ap.b.l2_clk_div = top->l2clk_div;
	cc_ap.b.bus_mc_clk_div = top->pdclk_div;
	cc_ap.b.biu_clk_div = top->baclk_div;
	/* 2.3) set periphclk div */
	if (top->periphclk)
		set_periph_clk_div(top);

	cc_ap.b.core_freq_chg_req = 1;
	/* used only for core FC, will NOT trigger fc_sm */
	/* cc_ap.b.core_dyn_fc = 1; */

	/* 2.4) set div and FC req trigger core FC */
	pr_debug("CORE FC APMU_CCR[%x]\n", cc_ap.v);
	__raw_writel(cc_ap.v, APMU_CCR);
	wait_for_fc_done();

	/* 3) Post FC : AP clear allow FC voting */
	cc_ap.v = __raw_readl(APMU_CCR);
	cc_ap.b.core_allow_spd_chg = 0;
	__raw_writel(cc_ap.v, APMU_CCR);
}

static int set_core_freq(struct pxa988_cpu_opt *old, struct pxa988_cpu_opt *new)
{
	struct pxa988_cpu_opt cop;
	struct clk *old_parent;
	int ret = 0;

	pr_debug("CORE set_freq start: old %u, new %u\n",
		old->pclk, new->pclk);
	get_fc_lock();

	memcpy(&cop, old, sizeof(struct pxa988_cpu_opt));
	get_cur_cpu_op(&cop);
	if (unlikely((cop.ap_clk_src != old->ap_clk_src) ||
		(cop.pclk != old->pclk) ||
		(cop.l2clk != old->l2clk) ||
		(cop.pdclk != old->pdclk) ||
		(cop.baclk != old->baclk) ||
		(cop.periphclk != old->periphclk))) {
		pr_err("psrc pclk l2clk pdclk baclk periphclk\n");
		pr_err("OLD %d %d %d %d %d %d\n", old->ap_clk_src,
		       old->pclk, old->l2clk, old->pdclk, old->baclk,
		       old->periphclk);
		pr_err("CUR %d %d %d %d %d %d\n", cop.ap_clk_src,
		       cop.pclk, cop.l2clk, cop.pdclk, cop.baclk,
		       cop.periphclk);
		pr_err("NEW %d %d %d %d %d %d\n", new->ap_clk_src,
		       new->pclk, new->l2clk, new->pdclk, new->baclk,
		       new->periphclk);
		dump_stack();
	}

	old_parent = cop.parent;
	clk_enable(new->parent);
	core_fc_seq(&cop, new);

	memcpy(&cop, new, sizeof(struct pxa988_cpu_opt));
	get_cur_cpu_op(&cop);
	if (unlikely((cop.ap_clk_src != new->ap_clk_src) ||
		(cop.pclk != new->pclk) ||
		(cop.l2clk != new->l2clk) ||
		(cop.pdclk != new->pdclk) ||
		(cop.baclk != new->baclk) ||
		(cop.periphclk != new->periphclk))) {
		pr_err("unsuccessful frequency change!\n");
		pr_err("psrc pclk l2clk pdclk baclk periphclk\n");
		pr_err("CUR %d %d %d %d %d %d\n", cop.ap_clk_src,
		       cop.pclk, cop.l2clk, cop.pdclk, cop.baclk,
		       cop.periphclk);
		pr_err("NEW %d %d %d %d %d %d\n", new->ap_clk_src,
			new->pclk, new->l2clk, new->pdclk, new->baclk,
			new->periphclk);
		ret = -EAGAIN;
		if (cop.ap_clk_src != new->ap_clk_src)
			clk_disable(new->parent);
		goto out;
	}

	clk_disable(old_parent);
out:
	put_fc_lock();
	pr_debug("CORE set_freq end: old %u, new %u\n",
		old->pclk, new->pclk);
	return ret;
}

static void pxa988_cpu_init(struct clk *clk)
{
	unsigned int op_index;
	struct pxa988_cpu_opt cur_op;
	struct pxa988_cpu_opt *op_array;

	BUG_ON(!cur_platform_opt);

	__init_cpu_opt();

	/* get cur core rate */
	op_array = cur_platform_opt->cpu_opt;
	memcpy(&cur_op, &op_array[0], sizeof(struct pxa988_cpu_opt));
	get_cur_cpu_op(&cur_op);
	op_index = cpu_rate2_op_index(cur_op.pclk);
	cur_cpu_op = &op_array[op_index];

	clk->rate = op_array[op_index].pclk;
	clk->parent = op_array[op_index].parent;
	clk->dynamic_change = 1;

	/*
	 * hard code loops_per_jiffy to highest according to highest
	 * core frequency 1248M
	 */
	loops_per_jiffy = 9707520;

	pr_info(" CPU boot up @%lu\n", clk->rate);
}

static long pxa988_cpu_round_rate(struct clk *clk, unsigned long rate)
{
	struct pxa988_cpu_opt *op_array =
		cur_platform_opt->cpu_opt;
	unsigned int op_array_size =
		cur_platform_opt->cpu_opt_size, index;

	rate /= MHZ;

	if (unlikely(rate > op_array[op_array_size - 1].pclk))
		return op_array[op_array_size - 1].pclk;

	for (index = 0; index < op_array_size; index++)
		if (op_array[index].pclk >= rate)
			break;

	return op_array[index].pclk * MHZ;
}

static int pxa988_cpu_setrate(struct clk *clk, unsigned long rate)
{
	struct pxa988_cpu_opt *md_new, *md_old;
	unsigned int index;
	int ret = 0;
	unsigned long flags;
	struct pxa988_cpu_opt *op_array =
		cur_platform_opt->cpu_opt;

	rate /= MHZ;
	index = cpu_rate2_op_index(rate);

	md_new = &op_array[index];
	if (md_new == cur_cpu_op)
		return 0;

	mutex_lock(&core_freqs_mutex);
	md_old = cur_cpu_op;

	/*
	 * FIXME: we do NOT enable clk here because pll3
	 * clk_enable and pll1_pll3_switch will do the
	 * same thing, we should handle it carefully.
	 * For example, pll1_1248 -> pll3, clk_enable(&pll3)
	 * will switch src to pll3, which will cause issue.
	 * clk_enable and disable will be handled in set_core_freq.
	 */
	/* clk_enable(md_new->parent); */

	spin_lock_irqsave(&fc_seq_lock, flags);

	/*
	 * Switching pll1_1248 and pll3p may generate glitch
	 * step 1),2),3) is neccessary
	 */
	if (((md_old->ap_clk_sel == AP_CLK_SRC_PLL3P) && \
		(md_new->ap_clk_sel == AP_CLK_SRC_PLL1_1248)) || \
		((md_old->ap_clk_sel == AP_CLK_SRC_PLL1_1248) && \
		(md_new->ap_clk_sel == AP_CLK_SRC_PLL3P))) {
		/* 1) use startup op(op0) as a bridge */
		ret = set_core_freq(md_old, &op_array[0]);
		if (ret)
			goto tmpout;
		/* 2) change PLL3_CR[18] to select pll1_1248 or pll3p */
		pll1_pll3_switch(md_new->ap_clk_sel);
		/* 3) switch to op which uses pll1_1248/pll3p */
		ret = set_core_freq(&op_array[0], md_new);
	} else {
		ret = set_core_freq(md_old, md_new);
	}

tmpout:
	spin_unlock_irqrestore(&fc_seq_lock, flags);
	if (ret)
		goto out;
	cur_cpu_op = md_new;

	clk_reparent(clk, md_new->parent);
	/*clk_disable(md_old->parent);*/
out:
	mutex_unlock(&core_freqs_mutex);
	return ret;
}

static unsigned long pxa988_cpu_getrate(struct clk *clk)
{
	if (cur_cpu_op)
		return cur_cpu_op->pclk * MHZ;
	else
		pr_err("%s: cur_cpu_op NULL\n", __func__);

	return 0;
}

/* do nothing only used to adjust proper clk->refcnt */
static int clk_dummy_enable(struct clk *clk)
{
	return 0;
}

static void clk_dummy_disable(struct clk *clk)
{
}

struct clkops cpu_clk_ops = {
	.init = pxa988_cpu_init,
	.enable = clk_dummy_enable,
	.disable = clk_dummy_disable,
	.round_rate = pxa988_cpu_round_rate,
	.setrate = pxa988_cpu_setrate,
	.getrate = pxa988_cpu_getrate,
};

static struct clk pxa988_cpu_clk = {
	.name = "cpu",
	.lookup = {
		.con_id = "cpu",
	},
	.ops = &cpu_clk_ops,
};

static void ddr_axi_fc_seq(struct pxa988_ddr_axi_opt *cop,
			    struct pxa988_ddr_axi_opt *top)
{
	union pmua_cc cc_ap, cc_cp;

	/* 0) Pre FC : check CP allow AP FC voting */
	cc_cp.v = __raw_readl(APMU_CP_CCR);
	if (unlikely(!cc_cp.b.core_allow_spd_chg)) {
		pr_warning("%s CP doesn't allow AP FC!\n",
			__func__);
		cc_cp.b.core_allow_spd_chg = 1;
		__raw_writel(cc_cp.v, APMU_CP_CCR);
	}

	/* 1) Pre FC : AP votes allow FC */
	cc_ap.v = __raw_readl(APMU_CCR);
	cc_ap.b.core_allow_spd_chg = 1;

	/* 2) issue DDR FC */
	if ((cop->ddr_clk_src != top->ddr_clk_src) || \
	    (cop->dclk != top->dclk)) {
		/* 2.1) set dclk src */
		set_ddr_clk_sel(top);
		/* 2.2) enable tbl based FC and set DDR tbl num */
		set_ddr_tbl_index(top->ddr_tbl_index);
		/* 2.3) select div for dclk */
		cc_ap.b.ddr_clk_div = top->dclk_div;
		/* 2.4) select ddr FC req bit */
		cc_ap.b.ddr_freq_chg_req = 1;
	}

	/* 3) issue AXI FC */
	if ((cop->axi_clk_src != top->axi_clk_src) || \
	    (cop->aclk != top->aclk)) {
		/* 3.1) set aclk src */
		set_axi_clk_sel(top);
		/* 3.2) select div for aclk */
		cc_ap.b.bus_clk_div = top->aclk_div;
		/* 3.3) select axi FC req bit */
		cc_ap.b.bus_freq_chg_req = 1;
	}

	/* 4) set div and FC req bit trigger DDR/AXI FC */
	pr_debug("DDR FC APMU_CCR[%x]\n", cc_ap.v);
	__raw_writel(cc_ap.v, APMU_CCR);
	wait_for_fc_done();

	/* 5) Post FC : AP clear allow FC voting */
	cc_ap.v = __raw_readl(APMU_CCR);
	cc_ap.b.core_allow_spd_chg = 0;
	__raw_writel(cc_ap.v, APMU_CCR);
}

static int set_ddr_axi_freq(struct pxa988_ddr_axi_opt *old,
	struct pxa988_ddr_axi_opt *new)
{
	struct pxa988_ddr_axi_opt cop;
	struct clk *ddr_old_parent, *axi_old_parent;
	int ret = 0, errflag = 0;

	pr_debug("DDR set_freq start: old %u, new %u\n",
		old->dclk, new->dclk);
	get_fc_lock();

	memcpy(&cop, old, sizeof(struct pxa988_ddr_axi_opt));
	get_cur_ddr_axi_op(&cop);
	if (unlikely((cop.ddr_clk_src != old->ddr_clk_src) ||
		(cop.axi_clk_src != old->axi_clk_src) ||
		(cop.dclk != old->dclk) ||
		(cop.aclk != old->aclk))) {
		pr_err(" dsrc dclk asrc aclk");
		pr_err("OLD %d %d %d %d\n", old->ddr_clk_src,
		       old->dclk, old->axi_clk_src, old->aclk);
		pr_err("CUR %d %d %d %d\n", cop.ddr_clk_src,
		       cop.dclk, cop.axi_clk_src, cop.aclk);
		pr_err("NEW %d %d %d %d\n", new->ddr_clk_src,
		       new->dclk, new->axi_clk_src, new->aclk);
		dump_stack();
	}

	ddr_old_parent = cop.ddr_parent;
	axi_old_parent = cop.axi_parent;
	clk_enable(new->ddr_parent);
	clk_enable(new->axi_parent);
	ddr_axi_fc_seq(&cop, new);

	memcpy(&cop, new, sizeof(struct pxa988_ddr_axi_opt));
	get_cur_ddr_axi_op(&cop);
	if (unlikely((cop.ddr_clk_src != new->ddr_clk_src) ||
		(cop.dclk != new->dclk))) {
		clk_disable(new->ddr_parent);
		errflag = 1;
	}
	if (unlikely((cop.axi_clk_src != new->axi_clk_src) ||
		(cop.aclk != new->aclk))) {
		clk_disable(new->axi_parent);
		errflag = 1;
	}
	if (unlikely(errflag)) {
		pr_err("DDR_AXI:unsuccessful frequency change!\n");
		pr_err(" dsrc dclk asrc aclk");
		pr_err("CUR %d %d %d %d\n", cop.ddr_clk_src,
		       cop.dclk, cop.axi_clk_src, cop.aclk);
		pr_err("NEW %d %d %d %d\n", new->ddr_clk_src,
		       new->dclk, new->axi_clk_src, new->aclk);
		ret = -EAGAIN;
		goto out;
	}

	clk_disable(ddr_old_parent);
	clk_disable(axi_old_parent);
out:
	put_fc_lock();
	pr_debug("DDR set_freq end: old %u, new %u\n",
		old->dclk, new->dclk);
	return ret;
}

static void pxa988_ddraxi_init(struct clk *clk)
{
	struct pxa988_ddr_axi_opt cur_op;
	struct pxa988_ddr_axi_opt *ddr_axi_opt;
	unsigned int op_index;
	unsigned long axi_rate;

	BUG_ON(!cur_platform_opt);
	__init_ddr_axi_opt();

	/* get core cur frequency */
	ddr_axi_opt = cur_platform_opt->ddr_axi_opt;
	memcpy(&cur_op, &ddr_axi_opt[0],
		sizeof(struct pxa988_ddr_axi_opt));
	get_cur_ddr_axi_op(&cur_op);
	op_index = ddr_rate2_op_index(cur_op.dclk);
	cur_ddraxi_op = &ddr_axi_opt[op_index];

	clk->rate = ddr_axi_opt[op_index].dclk;
	clk->parent = ddr_axi_opt[op_index].ddr_parent;
	clk->dynamic_change = 1;
	axi_rate = ddr_axi_opt[op_index].aclk;

	pr_info(" DDR boot up @%lu\n", clk->rate);
	pr_info(" AXI boot up @%lu\n", axi_rate);
}

static long pxa988_ddraxi_round_rate(struct clk *clk, unsigned long rate)
{
	struct pxa988_ddr_axi_opt *op_array =
		cur_platform_opt->ddr_axi_opt;
	unsigned int op_array_size =
		cur_platform_opt->ddr_axi_opt_size;
	unsigned int index;

	rate /= MHZ;

	if (unlikely(rate > op_array[op_array_size - 1].dclk))
		return op_array[op_array_size - 1].dclk;

	for (index = 0; index < op_array_size; index++)
		if (op_array[index].dclk >= rate)
			break;

	return op_array[index].dclk * MHZ;
}

static int pxa988_ddraxi_setrate(struct clk *clk, unsigned long rate)
{
	struct pxa988_ddr_axi_opt *md_new, *md_old;
	unsigned int index;
	int ret = 0;
	unsigned long flags;
	struct pxa988_ddr_axi_opt *op_array =
		cur_platform_opt->ddr_axi_opt;

	rate /= MHZ;
	index = ddr_rate2_op_index(rate);

	md_new = &op_array[index];
	if (md_new == cur_ddraxi_op)
		return 0;

	mutex_lock(&ddr_freqs_mutex);
	md_old = cur_ddraxi_op;

	/* clk_enable(md_new->ddr_parent); */

	spin_lock_irqsave(&fc_seq_lock, flags);
	ret = set_ddr_axi_freq(md_old, md_new);
	spin_unlock_irqrestore(&fc_seq_lock, flags);
	if (ret)
		goto out;
	cur_ddraxi_op = md_new;

	clk_reparent(clk, md_new->ddr_parent);
	/* clk_disable(md_old->ddr_parent); */
out:
	mutex_unlock(&ddr_freqs_mutex);
	return ret;
}

static unsigned long pxa988_ddraxi_getrate(struct clk *clk)
{
	if (cur_ddraxi_op)
		return cur_ddraxi_op->dclk * MHZ;
	else
		pr_err("%s: cur_ddraxi_op NULL\n", __func__);

	return 0;
}

struct clkops ddr_clk_ops = {
	.init = pxa988_ddraxi_init,
	.enable = clk_dummy_enable,
	.disable = clk_dummy_disable,
	.round_rate = pxa988_ddraxi_round_rate,
	.setrate = pxa988_ddraxi_setrate,
	.getrate = pxa988_ddraxi_getrate,
};

static struct clk pxa988_ddr_clk = {
	.name = "ddr",
	.lookup = {
		.con_id = "ddr",
	},
	.ops = &ddr_clk_ops,
};

/*
 * Every ddr/axi FC, fc_sm will halt AP,CP and
 * wait for the halt_ack from AP and CP.
 * If CP is in reset state, CP can not send this ack.
 * and system may hang. SW need to set a debug
 * register to ignore the CP ack if CP is in reset.
 */
/*
 * Interface used by telephony
 * cp_holdcp:
 * 1) acquire_fc_mutex
 * 2) hold CP (write APRR)
 * 3) mask the cp halt and clk-off of debug register
 * 4) release_fc_mutex
 * cp_releasecp:
 * 1) acquire_fc_mutex
 * 2) clear the cp halt and clk-off of debug register
 * 3) Write APRR to release CP from reset
 * 4) wait 10ms
 * 5) release_fc_mutex
 */
void acquire_fc_mutex(void)
{
	mutex_lock(&ddr_freqs_mutex);
	cp_reset_block_ddr_fc = true;
}
EXPORT_SYMBOL(acquire_fc_mutex);

/* called after release cp */
void release_fc_mutex(void)
{
	cp_reset_block_ddr_fc = false;
	mutex_unlock(&ddr_freqs_mutex);
}
EXPORT_SYMBOL(release_fc_mutex);

/* Interface used to get ddr op num */
unsigned int pxa988_get_ddr_op_num(void)
{
	return cur_platform_opt->ddr_axi_opt_size;
}

/* Interface used to get ddr avaliable rate, unit khz */
unsigned int pxa988_get_ddr_op_rate(unsigned int index)
{
	struct pxa988_ddr_axi_opt *ddr_opt;

	if (index >= cur_platform_opt->ddr_axi_opt_size) {
		pr_err("%s index out of range!\n", __func__);
		return -EINVAL;
	}

	ddr_opt = cur_platform_opt->ddr_axi_opt;
	return ddr_opt[index].dclk * MHZ_TO_KHZ;
}

#ifdef CONFIG_CPU_FREQ_TABLE
static struct cpufreq_frequency_table *cpufreq_tbl;

static void __init_cpufreq_table(void)
{
	struct pxa988_cpu_opt *cpu_opt;
	unsigned int cpu_opt_size = 0, i;

	cpu_opt = cur_platform_opt->cpu_opt;
	cpu_opt_size = cur_platform_opt->cpu_opt_size;

	cpufreq_tbl =
		kmalloc(sizeof(struct cpufreq_frequency_table) * \
					(cpu_opt_size + 1), GFP_KERNEL);
	if (!cpufreq_tbl)
		return;

	for (i = 0; i < cpu_opt_size; i++) {
		cpufreq_tbl[i].index = i;
		cpufreq_tbl[i].frequency = cpu_opt[i].pclk * MHZ_TO_KHZ;
	}

	cpufreq_tbl[i].index = i;
	cpufreq_tbl[i].frequency = CPUFREQ_TABLE_END;

	for_each_possible_cpu(i)
		cpufreq_frequency_table_get_attr(cpufreq_tbl, i);
}
#else
#define __init_cpufreq_table() do {} while (0);
#endif


#ifdef CONFIG_DEBUG_FS
/* #####################Debug Function######################## */
static int dump_cpu_op(char *buf, size_t size,
		struct pxa988_cpu_opt *q)
{
	return snprintf(buf, size, "pclk:%d pdclk:%d baclk:%d l2clk:%d "
			"periphclk:%d ap_clk_src:%d\n",
			q->pclk, q->pdclk, q->baclk, q->l2clk,
			q->periphclk, q->ap_clk_src);
}

static int dump_ddr_axi_op(char *buf, size_t size,
		struct pxa988_ddr_axi_opt *q)
{
	return snprintf(buf, size, "dclk:%d aclk:%d ddr_clk_src:%d "
			"axi_clk_src:%d\n",
			q->dclk, q->aclk, q->ddr_clk_src,
			q->axi_clk_src);
}

/* Display current operating point */
static ssize_t cur_cpu_op_show(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	char buf[256];
	int len = 0;
	size_t size = sizeof(buf) - 1;

	len = dump_cpu_op(buf, size - len, cur_cpu_op);
	return simple_read_from_buffer(buffer, count, ppos, buf, len);
}

const struct file_operations dp_cur_cpu_op_fops = {
	.read = cur_cpu_op_show,
};

static ssize_t cur_ddr_axi_op_show(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	char buf[256];
	int len = 0;
	size_t size = sizeof(buf) - 1;

	len = dump_ddr_axi_op(buf, size - len, cur_ddraxi_op);
	return simple_read_from_buffer(buffer, count, ppos, buf, len);
}

const struct file_operations dp_cur_ddr_axi_op_fops = {
	.read = cur_ddr_axi_op_show,
};

/* Dump all operating point */
static ssize_t ops_show(struct file *filp, char __user *buffer,
	size_t count, loff_t *ppos)
{
	char *p;
	int len = 0;
	size_t ret;
	unsigned int i;
	struct pxa988_cpu_opt *cpu_ops =
		cur_platform_opt->cpu_opt;
	unsigned int cpu_ops_size =
		cur_platform_opt->cpu_opt_size;
	struct pxa988_ddr_axi_opt *ddr_ops =
		cur_platform_opt->ddr_axi_opt;
	unsigned int ddr_ops_size =
		cur_platform_opt->ddr_axi_opt_size;
	size_t size = PAGE_SIZE - 1;

	p = (char *)__get_free_pages(GFP_NOIO, 0);
	if (!p)
		return -ENOMEM;

	len += snprintf(p + len, size - len, "CPU OP:\n");
	for (i = 0; i < cpu_ops_size; i++)
		len += dump_cpu_op(p + len, size - len, cpu_ops + i);
	len += snprintf(p + len, size - len, "\n");

	len += snprintf(p + len, size - len, "DDR_AXI OP:\n");
	for (i = 0; i < ddr_ops_size; i++)
		len += dump_ddr_axi_op(p + len, size - len, ddr_ops + i);
	len += snprintf(p + len, size - len, "\n");

	if (len == size)
		pr_warn("%s The dump buf is not large enough!\n", __func__);

	ret = simple_read_from_buffer(buffer, count, ppos, p, len);
	free_pages((unsigned long)p, 0);

	return ret;
}

const struct file_operations dp_ops_fops = {
	.read = ops_show,
};

/* show CP block AP DDR FC status */
static ssize_t cp_block_ddr_fc_show(struct file *filp,
	char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[32];
	int len = 0;
	size_t size = sizeof(buf) - 1;

	len = snprintf(buf, size, "%d\n", cp_reset_block_ddr_fc);
	return simple_read_from_buffer(buffer, count, ppos, buf, len);
}

const struct file_operations cp_block_ddr_fc_fops = {
	.read = cp_block_ddr_fc_show,
};

static int __init __init_create_fc_debugfs_node(void)
{
	struct dentry *fc;
	struct dentry *dp_cur_cpu_op, *dp_cur_ddraxi_op, *dp_ops;
	struct dentry *dp_cp_block_ddr_fc;

	fc = debugfs_create_dir("fc", pxa);
	if (!fc)
		return -ENOENT;

	dp_cur_cpu_op = debugfs_create_file("cur_cpu_op", 0444,
		fc, NULL, &dp_cur_cpu_op_fops);
	if (!dp_cur_cpu_op)
		goto err_cur_cpu_op;

	dp_cur_ddraxi_op = debugfs_create_file("cur_ddr_axi_op", 0444,
		fc, NULL, &dp_cur_ddr_axi_op_fops);
	if (!dp_cur_ddraxi_op)
		goto err_cur_ddraxi_op;

	dp_ops = debugfs_create_file("ops", 0444,
		fc, NULL, &dp_ops_fops);
	if (!dp_ops)
		goto err_dp_ops;

	dp_cp_block_ddr_fc = debugfs_create_file("cp_block_ddr_fc", 0444,
		fc, NULL, &cp_block_ddr_fc_fops);
	if (!dp_cp_block_ddr_fc)
		goto err_dp_cp_block_ddr_fc;

	return 0;

err_dp_cp_block_ddr_fc:
	debugfs_remove(dp_ops);
	dp_ops = NULL;
err_dp_ops:
	debugfs_remove(dp_cur_ddraxi_op);
	dp_cur_ddraxi_op = NULL;
err_cur_ddraxi_op:
	debugfs_remove(dp_cur_cpu_op);
	dp_cur_cpu_op = NULL;
err_cur_cpu_op:
	debugfs_remove(fc);
	fc = NULL;
	return -ENOENT;
}
#endif

static int __init pxa988_freq_init(void)
{
	int ret = 0;

	__init_platform_opt();

	__init_ddr_table();

	__init_fc_setting();

	pxa988_init_one_clock(&pxa988_cpu_clk);
	pxa988_init_one_clock(&pxa988_ddr_clk);
	clk_enable(&pxa988_cpu_clk);
	clk_enable(&pxa988_ddr_clk);

	__init_cpufreq_table();

#ifdef CONFIG_DEBUG_FS
	ret = __init_create_fc_debugfs_node();
#endif
	return ret;
}
postcore_initcall_sync(pxa988_freq_init);
