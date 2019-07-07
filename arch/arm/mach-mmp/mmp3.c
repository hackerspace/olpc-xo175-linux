/*
 * linux/arch/arm/mach-mmp/mmp3.c
 *
 * code name MMP3
 *
 * Copyright (C) 2009 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/usb/mv_usb.h>
#include <linux/notifier.h>

#include <asm/smp_twd.h>
#include <asm/mach/time.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>

#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-pmu.h>
#include <mach/regs-ciu.h>
#include <mach/cputype.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/dma.h>
#include <mach/devices.h>
#include <mach/mmp3.h>
#include <mach/regs-usb.h>
#include <mach/soc_vmeta.h>
#include <mach/mmp_dma.h>
#include <linux/memblock.h>
#include <linux/regdump_ops.h>

#include <linux/platform_device.h>

#include <plat/mfp.h>
#include <plat/pmem.h>

#include "common.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)

#define APMASK(i)	(GPIO_REGS_VIRT + BANK_OFF(i) + 0x9c)
#define ISP_POLL_COUNT (10)

static struct mfp_addr_map mmp3_addr_map[] __initdata = {
	MFP_ADDR_X(GPIO0, GPIO58, 0x54),
	MFP_ADDR_X(GPIO59, GPIO73, 0x280),
	MFP_ADDR_X(GPIO74, GPIO101, 0x170),
	MFP_ADDR_X(GPIO102, GPIO103, 0x0),
	MFP_ADDR_X(GPIO115, GPIO122, 0x260),
	MFP_ADDR_X(GPIO124, GPIO141, 0xc),
	MFP_ADDR_X(GPIO143, GPIO151, 0x220),
	MFP_ADDR_X(GPIO152, GPIO153, 0x248),
	MFP_ADDR_X(GPIO154, GPIO155, 0x254),

	MFP_ADDR(GPIO142, 0x8),
	MFP_ADDR(GPIO114, 0x164),
	MFP_ADDR(GPIO123, 0x148),

	MFP_ADDR(GPIO168, 0x1e0),
	MFP_ADDR(GPIO167, 0x1e4),
	MFP_ADDR(GPIO166, 0x1e8),
	MFP_ADDR(GPIO165, 0x1ec),
	MFP_ADDR(GPIO107, 0x1f0),
	MFP_ADDR(GPIO106, 0x1f4),
	MFP_ADDR(GPIO105, 0x1f8),
	MFP_ADDR(GPIO104, 0x1fc),
	MFP_ADDR(GPIO111, 0x200),
	MFP_ADDR(GPIO164, 0x204),
	MFP_ADDR(GPIO163, 0x208),
	MFP_ADDR(GPIO162, 0x20c),
	MFP_ADDR(GPIO161, 0x210),
	MFP_ADDR(GPIO110, 0x214),
	MFP_ADDR(GPIO109, 0x218),
	MFP_ADDR(GPIO108, 0x21c),
	MFP_ADDR(GPIO110, 0x214),
	MFP_ADDR(GPIO112, 0x244),
	MFP_ADDR(GPIO160, 0x250),
	MFP_ADDR(GPIO113, 0x25c),
	/* FIXME: Zx does not have this pin, define here will not impact */
	MFP_ADDR(GPIO171, 0x2c8),

	MFP_ADDR_X(TWSI1_SCL, TWSI1_SDA, 0x140),
	MFP_ADDR_X(TWSI4_SCL, TWSI4_SDA, 0x2bc),
	MFP_ADDR(PMIC_INT, 0x2c4),
	MFP_ADDR(CLK_REQ, 0x160),

	MFP_ADDR_END,
};

#ifdef CONFIG_SMP
unsigned long c2_reserve_pa;
#define C2_RESERVE_SIZE	(1024 * 1024)
#endif

void __init mmp3_reserve(void)
{
	/*
	 * reserve first page for uboot bootstrap, otherwise
	 * unused core may run unpredictable instruction.
	 *
	 * NOTE: if enabled hypervisor code, then need reserve
	 * the first 4KB from the address PLAT_PHYS_OFFSET.
	 */
	BUG_ON(memblock_reserve(PLAT_PHYS_OFFSET, 0x1000));

	/*reserve memory for pmem*/
	pxa_reserve_pmem_memblock();
#ifdef CONFIG_SMP
	c2_reserve_pa = memblock_alloc(C2_RESERVE_SIZE, PAGE_SIZE);
	if (!c2_reserve_pa) {
		pr_err("%s: failed to reserve memory for C2\n", __func__);
		BUG();
	}
	BUG_ON(memblock_free(c2_reserve_pa, C2_RESERVE_SIZE));
	BUG_ON(0 != memblock_remove(c2_reserve_pa, C2_RESERVE_SIZE));
#endif
}

static void __init mmp3_init_gpio(void)
{
	int i;

	/* enable GPIO clock */
	__raw_writel(APBC_APBCLK | APBC_FNCLK, APBC_MMP2_GPIO);

	/* unmask GPIO edge detection for all 6 banks -- APMASKx */
	for (i = 0; i < 6; i++)
		__raw_writel(0xffffffff, APMASK(i));

	pxa_init_gpio(IRQ_MMP3_GPIO, 0, 167, NULL);
}

#ifdef CONFIG_REGDUMP
static struct regdump_ops pmua_regdump_ops = {
	.dev_name = "MMP3-PMUA",
};

static struct regdump_region pmua_dump_region[] = {
	{"PMUA_CC_SP",			0x000, 4, regdump_cond_true},
	{"PMUA_CC_PJ",			0x004, 4, regdump_cond_true},
	{"PMUA_DM_CC_SP",		0x008, 4, regdump_cond_true},
	{"PMUA_DM_CC_PJ",		0x00c, 4, regdump_cond_true},
	{"PMUA_FC_TIMER",		0x010, 4, regdump_cond_true},
	{"PMUA_SP_IDLE_CFG",		0x014, 4, regdump_cond_true},
	{"PMUA_PJ_IDLE_CFG",		0x018, 4, regdump_cond_true},
	{"PMUA_CCIC_CLK_GATE_CTRL",	0x028, 4, regdump_cond_true},
	{"PMUA_FBRC1_CLK_GATE_CTRL",	0x02c, 4, regdump_cond_true},
	{"PMUA_FBRC2_CLK_GATE_CTRL",	0x030, 4, regdump_cond_true},
	{"PMUA_PMU_CLK_GATE_CTRL",	0x040, 4, regdump_cond_true},
	{"PMUA_IRE_CLK_RES_CTRL",	0x048, 4, regdump_cond_true},
	{"PMUA_DISPLAY1_CLK_RES_CTRL",	0x04c, 4, regdump_cond_true},
	{"PMUA_CCIC_CLK_RES_CTRL",	0x050, 4, regdump_cond_true},
	{"PMUA_SDH1_CLK_RES_CTRL",	0x054, 4, regdump_cond_true},
	{"PMUA_SDH2_CLK_RES_CTRL",	0x058, 4, regdump_cond_true},
	{"PMUA_USB_CLK_RES_CTRL",	0x05c, 4, regdump_cond_true},
	{"PMUA_NF_CLK_RES_CTRL",	0x060, 4, regdump_cond_true},
	{"PMUA_DMA_CLK_RES_CTRL",	0x064, 4, regdump_cond_true},
	{"PMUA_WTM_CLK_RES_CTRL",	0x068, 4, regdump_cond_true},
	{"PMUA_BUS_CLK_RES_CTRL",	0x06c, 4, regdump_cond_true},
	{"PMUA_WAKE_CLR_MASK",		0x07c, 4, regdump_cond_true},
	{"PMUA_PWR_STBL_TIMER",		0x084, 4, regdump_cond_true},
	{"PMUA_SRAM_PWR_DWN",		0x08c, 4, regdump_cond_true},
	{"PMUA_CORE_STATUS",		0x090, 4, regdump_cond_true},
	{"PMUA_RES_FRM_SLP_CLR",	0x094, 4, regdump_cond_true},
	{"PMUA_AP_IRWC",		0x09c, 4, regdump_cond_true},
	{"PMUA_AP_ISR",			0x0a0, 4, regdump_cond_true},
	{"PMUA_VMETA_CLK_RES_CTRL",	0x0a4, 4, regdump_cond_true},
	{"PMUA_MC_HW_SLP_TYPE",		0x0b0, 4, regdump_cond_true},
	{"PMUA_MC_SLP_REQ_PJ",		0x0b4, 4, regdump_cond_true},
	{"PMUA_MC_SW_SLP_TYPE",		0x0c0, 4, regdump_cond_true},
	{"PMUA_PLL_SEL_STATUS",		0x0c4, 4, regdump_cond_true},
	{"PMUA_GC_CLK_RES_CTRL",	0x0cc, 4, regdump_cond_true},
	{"PMUA_SMC_CLK_RES_CTRL",	0x0d4, 4, regdump_cond_true},
	{"PMUA_GLB_CLK_CTRL",		0x0dc, 4, regdump_cond_true},
	{"PMUA_PWR_ONOFF_CTRL",		0x0e0, 4, regdump_cond_true},
	{"PMUA_PWR_ISL_TIMER",		0x0e4, 4, regdump_cond_true},
	{"PMUA_SDH3_CLK_RES_CTRL",	0x0e8, 4, regdump_cond_true},
	{"PMUA_SDH4_CLK_RES_CTRL",	0x0ec, 4, regdump_cond_true},
	{"PMUA_CCIC2_CLK_RES_CTRL",	0x0f4, 4, regdump_cond_true},
	{"PMUA_HSIC1_CLK_RES_CTRL",	0x0f8, 4, regdump_cond_true},
	{"PMUA_HSIC2_CLK_RES_CTRL",	0x0fc, 4, regdump_cond_true},
	{"PMUA_FSIC3_CLK_RES_CTRL",	0x100, 4, regdump_cond_true},
	{"PMUA_SLIM_CLK_RES_CTRL",	0x104, 4, regdump_cond_true},
	{"PMUA_AUDIO_CLK_RES_CTRL",	0x10c, 4, regdump_cond_true},
	{"PMUA_DISPLAY2_CLK_RES_CTRL",	0x110, 4, regdump_cond_true},
	{"PMUA_CCIC2_CLK_GATE_CTRL",	0x118, 4, regdump_cond_true},
	{"PMUA_MC_PAR_CTRL",		0x11c, 4, regdump_cond_true},
	{"PMUA_SPMI_CLK_RES_CTRL",	0x140, 4, regdump_cond_true},
	{"PMUA_EPD_CLK_RES_CTRL",	0x144, 4, regdump_cond_true},
	{"PMUA_USB3SS_CLK_RES_CTRL",	0x148, 4, regdump_cond_true},
	{"PMUA_CC2_SP",			0x14c, 4, regdump_cond_true},
	{"PMUA_CC2_PJ",			0x150, 4, regdump_cond_true},
	{"PMUA_DM2_CC_SP",		0x154, 4, regdump_cond_true},
	{"PMUA_DM2_CC_PJ",		0x158, 4, regdump_cond_true},
	{"PMUA_SDH5_CLK_RES_CTRL",	0x160, 4, regdump_cond_true},
	{"PMUA_DSA_CLK_RES_CTRL",	0x164, 4, regdump_cond_true},
	{"IOC_CTRL",			0x184, 4, regdump_cond_true},
	{"PMUA_CC3_PJ",			0x188, 4, regdump_cond_true},
	{"PMUA_TPIU_CLK_RES_CTRL",	0x18c, 4, regdump_cond_true},
	{"PMUA_DEBUG2",			0x190, 4, regdump_cond_true},
	{"PMUA_RF1P_CTRL",		0x1c0, 4, regdump_cond_true},
	{"PMUA_RF2P_CTRL",		0x1c4, 4, regdump_cond_true},
	{"PMUA_SR1P_CTRL",		0x1c8, 4, regdump_cond_true},
	{"PMUA_SR2P_CTRL",		0x1cc, 4, regdump_cond_true},
	{"PMUA_SR1P_CTRL",		0x1c8, 4, regdump_cond_true},
	{"PMUA_BROM_CTRL",		0x1d0, 4, regdump_cond_true},
	{"PMUA_ISP_PWR_CTRL",		0x1fc, 4, regdump_cond_true},
	{"PMUA_PJ_IDLE_CFG2",		0x200, 4, regdump_cond_true},
	{"PMUA_PJ_IDLE_CFG3",		0x204, 4, regdump_cond_true},
	{"PMUA_FASTENET_CLK_RES_CTRL",	0x210, 4, regdump_cond_true},
	{"PMUA_ISLAND_PWR_STATUS",	0x220, 4, regdump_cond_true},
	{"PMUA_ISP_CLK_RES_CTRL",	0x224, 4, regdump_cond_true},
	{"PMUA_AUDIO_ISLAND_SRAM_PWR_DWN_CTRL",	0x240, 4, regdump_cond_true},
	{"PMUA_GENERIC_CTRL",		0x244, 4, regdump_cond_true},
	{"PMUA_PJ_C0_CC4",		0x248, 4, regdump_cond_true},
	{"PMUA_PJ_C1_CC4",		0x24c, 4, regdump_cond_true},
	{"PMUA_PJ_C2_CC4",		0x250, 4, regdump_cond_true},
	{"PMUA_PJ_MP_SUBSYS_CC",	0x254, 4, regdump_cond_true},
	{"PMUA_MC1_DOUBLER_CTRL",	0x258, 4, regdump_cond_true},
	{"PMUA_MC2_DOUBLER_CTRL",	0x25c, 4, regdump_cond_true},
	{"PMUA_DOUBLER_GENERIC_CTRL",	0x260, 4, regdump_cond_true},
	{"PMUA_MC1_DOUBLER_STAT",	0x264, 4, regdump_cond_true},
	{"PMUA_MC2_DOUBLER_STAT",	0x268, 4, regdump_cond_true},
	{"PMUA_SDHM_DLL_CTRL0",		0x26c, 4, regdump_cond_true},
	{"PMUA_SDHM_DLL_CTRL1",		0x270, 4, regdump_cond_true},
	{"PMUA_SDHM_DLL_STAT0",		0x274, 4, regdump_cond_true},
	{"PMUA_SDHM_DLL_STAT1",		0x278, 4, regdump_cond_true}
};

static void __init mmp3_init_pmua_regdump(void)
{
	pmua_regdump_ops.base = PMUA_VIRT_BASE;
	pmua_regdump_ops.regions = pmua_dump_region;
	pmua_regdump_ops.reg_nums = ARRAY_SIZE(pmua_dump_region);
	register_regdump_ops(&pmua_regdump_ops);
}
#else
static void inline  __init mmp3_init_pmua_regdump() {}
#endif

void __init mmp3_init_irq(void)
{
	gic_init(0, 29, (void __iomem *) GIC_DIST_VIRT_BASE, (void __iomem *) GIC_CPU_VIRT_BASE);

	mmp3_init_gic();
}

#ifdef CONFIG_CACHE_L2X0
static void mmp3_init_l2x0(void)
{
	void __iomem *l2x0_base = ioremap(SL2C_PHYS_BASE, SZ_4K);
	if (IS_ERR(l2x0_base)) {
		printk(KERN_ERR "L2 map failed %ld\n", PTR_ERR(l2x0_base));
	} else {
		/* Auxiliary Control:
		   TODO: According to the manual, this register should be
			written in secure access, we may need to move the
			configuration in early stage of boot if TZ enabled

		   [ 0.. 2]	cycles of latency of data RAM read
		   [ 3.. 5]	cycles of latency of data RAM write
		   [ 6.. 8]	cycles of latency of tag RAM
		   [ 9..11]	cycles of latency of dirty RAM
		   [12]		exclusive cache op, 0:disable,1:enable
		   [13..16]	associativity
		   [17..19]	way-size
		   [20]		event monitor bus enable
		   [21]		parity enable
		   [22]		shared attribute override enable
		   [23..24]	force write allocate
				0: use AWCACHE
				1: force no WA
				2: force WA on
				3: internal mapped
		   [25]		reserved, SBO/RAO
		   [26]		Non-secure lockdown enable
		   [27]		Non-secure interrupt access enable
		   [28]		data prefetch enable
		   [29]		instruction prefetch enable
		   [30..31]	reserved, SBZ
		*/
		/*
		   forece NO WA, for A0 memory performance, bug in WA
		   64KB way-size
		   clear bit[16] to make sure l2x0_init call take it as 8-way
		   Overall enabling of L2 prefetching, when enabled, the
		   L2X0 prefetching kernel config can take effect. The feature
		   requires both here and the config code in L2X0 driver
		   to work.
		*/
		l2x0_init(l2x0_base, 0x30860000, 0xC200FFFF);
	}
}
#else
static void mmp3_init_l2x0(void) {}
#endif

#define DMCU_REG(mcu, x) (DMCU_VIRT_BASE + ((mcu) * 0x10000) + (x))
#define DMCU0_PORT_PRIORITY_VA DMCU_REG(0, 0x140)
#define DMCU1_PORT_PRIORITY_VA DMCU_REG(1, 0x140)
/* raise display related DDR ports to critical with low latency requirement*/
#define DEFAULT_DDR_PORT_PRI (0xff003030)

static void mmp3_fabric_ddr_config(void)
{
	u32  regval;

	/* fabric dynamic clock gating */
	regval = __raw_readl(CIU_FABRIC_CKGT_CTRL0);
	regval &= ~(0xFE00AAAA);	/* clear reserved bits */
					/* [31:25] reserved*/
	regval &= ~(0x1 << 24);		/*Fab11 aclk, VMETA AXI*/
	regval &= ~(0x1 << 23);		/*Fab9 aclk, MDMA/VDMA*/
	regval &= ~(0x7f << 16);	/*MC1x2 fabrics*/
					/* 15, reserved*/
	regval &= ~(0x1 << 14);		/*fabric2 aclk, LCD/FE/IRE/Fabric5*/
					/* 13, reserved*/
	/* disable DCG on fabric 3, JIRA:MMP3-1586 */
	regval |= (0x1 << 12);		/*fabric3 aclk3, ISP/CCIC/Fabric4*/
					/* 11, reserved*/
	regval &= ~(0x1 << 10);		/*fabric1 aclk2, PJ/SP/APB/AHB*/
					/* 9, reserved*/
	regval &= ~(0x1 << 8);		/*fabric4 aclk, USB/SD/NFC/SMC*/
					/* 7, reserved*/
	regval &= ~(0x1 << 6);		/*fabric4 aclk2, USB/SD/NFC/SMC*/
					/* 5, reserved*/
	regval &= ~(0x1 << 4);		/*fabric10 aclk, USB/SD/NFC/SMC*/
					/* 3, reserved*/
	regval &= ~(0x1 << 2);		/*fabric10 aclk2, USB/SD/NFC/SMC*/
					/* 1, reserved*/
	regval &= ~(0x1 << 0);		/*fabric5 aclk5, HSI/SPMI/SLIMBUS0*/
	__raw_writel(regval, CIU_FABRIC_CKGT_CTRL0);

	regval = __raw_readl(CIU_FABRIC_CKGT_CTRL1);
	regval &= ~(0xFFFFFFFC);	/* clear reserved bits */
					/* [31:2] reserved*/
	regval &= ~(0x1 << 1);		/*audio fabric*/
	regval &= ~(0x1 << 0);		/*DDR P5*/
	__raw_writel(regval, CIU_FABRIC_CKGT_CTRL1);

	regval = __raw_readl(CIU_FABRIC_CKGT_CTRL2);
	regval &= ~(0xFFFFFFFE);	/* clear reserved bits */
					/* [31:1] reserved*/
	regval &= ~(0x1 << 0);		/*GC Fabric*/
	__raw_writel(regval, CIU_FABRIC_CKGT_CTRL2);

	dsb();
	pr_info("%s: 0x%08x, 0x%08x, 0x%08x\n", __func__
		, __raw_readl(CIU_FABRIC_CKGT_CTRL0)
		, __raw_readl(CIU_FABRIC_CKGT_CTRL1)
		, __raw_readl(CIU_FABRIC_CKGT_CTRL2)
		);

	/* update priority*/
	pr_info("%s: DDR interleave size %d KB\n", __func__
		, ciu_ddr_ilv_size()/1024);
	if (ciu_ddr_ilv_on()) {
		__raw_writel(DEFAULT_DDR_PORT_PRI, DMCU0_PORT_PRIORITY_VA);
		__raw_writel(DEFAULT_DDR_PORT_PRI, DMCU1_PORT_PRIORITY_VA);
		pr_info("%s: DDR Port Priority %x, %x\n", __func__
			, __raw_readl(DMCU0_PORT_PRIORITY_VA)
			, __raw_readl(DMCU1_PORT_PRIORITY_VA)
			);
	} else {
		__raw_writel(DEFAULT_DDR_PORT_PRI, DMCU0_PORT_PRIORITY_VA);
		pr_info("%s: DDR Port Priority %x\n", __func__
			, __raw_readl(DMCU0_PORT_PRIORITY_VA));

		/* make sure 2nd MC is off */
		regval = __raw_readl(APMU_BUS);
		regval &= ~(1u << 1);
		__raw_writel(regval, APMU_BUS);

		/* force 2nd MC clock generating block to be clock gated*/
		regval = __raw_readl(APMU_REG(0x40));
		regval &= ~(3u << 18);
		regval |= (1u << 18);
		__raw_writel(regval, APMU_REG(0x40));
	}

	dsb();

}

static void __init mmp3_timer_init(void)
{
	uint32_t clk_rst;

#ifdef CONFIG_LOCAL_TIMERS
	twd_base = (void __iomem *)TWD_VIRT_BASE;
#endif

	/* this is early, we have to initialize the CCU registers by
	 * ourselves instead of using clk_* API. Clock rate is defined
	 * by APBC_TIMERS_FNCLKSEL and enabled free-running
	 */
	__raw_writel(APBC_APBCLK | APBC_RST, APBC_MMP2_TIMERS);

	/* 6.5MHz, bus/functional clock enabled, release reset */
	clk_rst = APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(1);
	__raw_writel(clk_rst, APBC_MMP2_TIMERS);

#ifdef CONFIG_SOC_LOCAL_TIMERS
	timer_init(IRQ_MMP3_TIMER1, IRQ_MMP3_TIMER2, IRQ_MMP3_TIMER3);
#else
	//timer_init(IRQ_MMP3_TIMER1, IRQ_MMP3_TIMER2);
#endif
}

struct sys_timer mmp3_timer = {
	.init   = mmp3_timer_init,
};

#define PJ4B_WCB_MIN_MSK	(0x3f)
#define PJ4B_WCB_MIN_SHFT	(1)
#define PJ4B_WCB_MAX_MSK	(0x3f)
#define PJ4B_WCB_MAX_SHFT	(7)
#define PJ4B_WCB_EVCT_MSK	(0x7fff)
#define PJ4B_WCB_EVCT_SHFT	(13)
#define OMITFLD			((unsigned long)-1)
#define UPDATE_ON_VALID(lval, rval, msk, shft)		\
	do if (rval != OMITFLD) {			\
		lval &= ~((msk) << (shft));		\
		lval |= (((rval) & (msk)) << (shft));	\
	} while (0)

static unsigned long pj4b_wcb_config(unsigned long min, unsigned long max,
			unsigned long evct)
{
	register unsigned long regval;
	__asm__("mrc p15, 1, %0, c15, c2, 1" : "=r" (regval));
	UPDATE_ON_VALID(regval, min, PJ4B_WCB_MIN_MSK, PJ4B_WCB_MIN_SHFT);
	UPDATE_ON_VALID(regval, max, PJ4B_WCB_MAX_MSK, PJ4B_WCB_MAX_SHFT);
	UPDATE_ON_VALID(regval, evct, PJ4B_WCB_EVCT_MSK, PJ4B_WCB_EVCT_SHFT);
	__asm__("mcr p15, 1, %0, c15, c2, 1" : : "r" (regval));
	return regval;
}

struct platform_device mmp3_device_asoc_sspa1 = {
	.name		= "mmp3-sspa-dai",
	.id		= 0,
};

struct platform_device mmp3_device_asoc_sspa2 = {
	.name		= "mmp3-sspa-dai",
	.id		= 1,
};

struct platform_device mmp3_device_asoc_platform = {
	.name		= "mmp3-pcm-audio",
	.id		= -1,
};

struct platform_device mmp3_device_asoc_hdmi = {
	.name		= "dummy-codec",
	.id		= -1,
};

static int __init mmp3_init(void)
{
	/*
	  let's make minimum WCB open entries to 2 to boost memory access
	*/
	pj4b_wcb_config(2, OMITFLD, OMITFLD);

	mmp3_init_l2x0();

	mmp3_fabric_ddr_config();

	mfp_init_base(MFPR_VIRT_BASE);
	mfp_init_addr(mmp3_addr_map);

	mmp3_init_gpio();
	mmp3_init_pmua_regdump();

	pxa_init_dma(IRQ_MMP3_DMA_RIQ, 16);
	mmp_init_dma(IRQ_MMP3_DMA_RIQ);

	platform_device_register(&mmp3_device_asoc_sspa1);
	platform_device_register(&mmp3_device_asoc_platform);
	platform_device_register(&mmp3_device_asoc_hdmi);

	return 0;
}

postcore_initcall(mmp3_init);

/* on-chip devices */
MMP3_DEVICE(uart1, "pxa2xx-uart", 0, UART1, 0xd4030000, 0x30, 4, 5);
MMP3_DEVICE(uart2, "pxa2xx-uart", 1, UART2, 0xd4017000, 0x30, 20, 21);
MMP3_DEVICE(uart3, "pxa2xx-uart", 2, UART3, 0xd4018000, 0x30, 22, 23);
MMP3_DEVICE(uart4, "pxa2xx-uart", 3, UART4, 0xd4016000, 0x30, 18, 19);
MMP3_DEVICE(nand, "pxa3xx-nand", -1, NAND, 0xd4283000, 0x100, 28, 29);
MMP3_DEVICE(sdh0, "sdhci-pxa", 0, MMC, 0xd4280000, 0x120);
MMP3_DEVICE(sdh1, "sdhci-pxa", 1, MMC2, 0xd4280800, 0x120);
MMP3_DEVICE(sdh2, "sdhci-pxa", 2, MMC3, 0xd4281000, 0x120);
MMP3_DEVICE(sdh3, "sdhci-pxa", 3, MMC4, 0xd4281800, 0x120);
MMP3_DEVICE(twsi1, "pxa2xx-i2c", 0, TWSI1, 0xd4011000, 0x70);
MMP3_DEVICE(twsi2, "pxa2xx-i2c", 1, TWSI2, 0xd4031000, 0x70);
MMP3_DEVICE(twsi3, "pxa2xx-i2c", 2, TWSI3, 0xd4032000, 0x70);
MMP3_DEVICE(twsi4, "pxa2xx-i2c", 3, TWSI4, 0xd4033000, 0x70);
MMP3_DEVICE(twsi5, "pxa2xx-i2c", 4, TWSI5, 0xd4033800, 0x70);
MMP3_DEVICE(twsi6, "pxa2xx-i2c", 5, TWSI6, 0xd4034000, 0x70);
MMP3_DEVICE(pwm1, "mmp2-pwm", 0, NONE, 0xd401a000, 0x10);
MMP3_DEVICE(pwm2, "mmp2-pwm", 1, NONE, 0xd401a400, 0x10);
MMP3_DEVICE(pwm3, "mmp2-pwm", 2, NONE, 0xd401a800, 0x10);
MMP3_DEVICE(pwm4, "mmp2-pwm", 3, NONE, 0xd401ac00, 0x10);
MMP3_DEVICE(keypad, "pxa27x-keypad", -1, KEYPAD, 0xd4012000, 0x4c);
MMP3_DEVICE(fb, "pxa168-fb", 0, LCD, 0xd420b000, 0x500);
MMP3_DEVICE(fb_ovly, "pxa168fb_ovly", 0, LCD, 0xd420b000, 0x500);
MMP3_DEVICE(v4l2_ovly, "pxa168-v4l2_ovly", 0, LCD, 0xd420b000, 0x500);
MMP3_DEVICE(fb_tv, "pxa168-fb", 1, LCD, 0xd420b000, 0x500);
MMP3_DEVICE(fb_tv_ovly, "pxa168fb_ovly", 1, LCD, 0xd420b000, 0x500);
MMP3_DEVICE(v4l2_tv_ovly, "pxa168-v4l2_ovly", 1, LCD, 0xd420b000, 0x500);
MMP3_DEVICE(hdmi, "uio-hdmi", -1, HDMI, 0xd420b000, 0x1fff);
MMP3_DEVICE(ddr_devfreq, "devfreq-ddr", -1, NONE, 0xd0000000, 0x1000);
MMP3_DEVICE(sspa1, "mmp2-sspa", 0, SSPA1, 0xc0ffdc00, 0xb0, ADMA1_CH1,
	    ADMA1_CH0);
MMP3_DEVICE(sspa2, "mmp2-sspa", 1, SSPA2, 0xc0ffdd00, 0xb0, ADMA2_CH1,
	    ADMA2_CH0);
MMP3_DEVICE(ssp1, "mmp-ssp", 1, SSP1, 0xd4035000, 0x40, 6, 7);
MMP3_DEVICE(ssp2, "mmp-ssp", 2, SSP2, 0xd4036000, 0x40, 10, 11);
MMP3_DEVICE(ssp3, "mmp-ssp", 3, SSP3, 0xd4037000, 0x40, 12, 13);
MMP3_DEVICE(ssp4, "mmp-ssp", 4, SSP4, 0xd4039000, 0x40, 14, 15);
MMP3_DEVICE(thermal, "mmp-thermal", -1, THERMAL_SENSOR, 0xd403b000, 0x10);

MMP3_DEVICE(audiosram, "mmp-sram", 0, NONE, 0xd1030000, 0x20000);
MMP3_DEVICE(camera0, "mv-camera", 0, CCIC1, 0xd420a000, 0x2ff);
MMP3_DEVICE(camera1, "mv-camera", 3, CCIC2, 0xd420a800, 0x2ff);
MMP3_DEVICE(videosram, "mmp-sram", 1, NONE, 0xd1020000, 0x10000);

void mmp3_clear_keypad_wakeup(void)
{
	uint32_t val;
	uint32_t mask = (1 << 5);

	/* wake event clear is needed in order to clear keypad interrupt */
	val = __raw_readl(APMU_WAKE_CLR);
	__raw_writel(val | mask, APMU_WAKE_CLR);
}

static struct resource mmp3_resource_rtc[] = {
	{ 0xd4010000, 0xd40100ff, NULL, IORESOURCE_MEM, },
	{ IRQ_MMP3_RTC, IRQ_MMP3_RTC, NULL, IORESOURCE_IRQ, },
	{ IRQ_MMP3_RTC_ALARM, IRQ_MMP3_RTC_ALARM, NULL, IORESOURCE_IRQ, },
};

struct platform_device mmp3_device_rtc = {
	.name           = "mmp-rtc",
	.id             = -1,
	.resource       = mmp3_resource_rtc,
	.num_resources  = ARRAY_SIZE(mmp3_resource_rtc),
};

struct platform_device mmp3_device_vnc_touch = {
	.name   = "vnc-ts",
	.id     = -1,
};

#ifdef CONFIG_USB_SUPPORT
static u64 usb_dma_mask = ~(u32)0;

#ifdef CONFIG_USB_PXA_U2O
struct resource mmp3_u2o_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA168_U2O_REGBASE + U2x_CAPREGS_OFFSET,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2O_PHYBASE,
		.end	= PXA168_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_MMP3_USB_OTG,
		.end	= IRQ_MMP3_USB_OTG,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device mmp3_device_u2o = {
	.name		= "pxa-u2o",
	.id		= -1,
	.resource	= mmp3_u2o_resources,
	.num_resources	= ARRAY_SIZE(mmp3_u2o_resources),
	.dev		=  {
		.dma_mask		= &usb_dma_mask,
		.coherent_dma_mask	= 0xffffffff,
	}
};
#endif

#ifdef CONFIG_USB_EHCI_PXA
#ifdef CONFIG_USB_EHCI_PXA_U2O
struct resource mmp3_u2oehci_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA168_U2O_REGBASE + U2x_CAPREGS_OFFSET,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2O_PHYBASE,
		.end	= PXA168_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_MMP3_USB_OTG,
		.end	= IRQ_MMP3_USB_OTG,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device mmp3_device_u2oehci = {
	.name		= "pxa-u2oehci",
	.id		= -1,
	.dev		= {
		.dma_mask		= &usb_dma_mask,
		.coherent_dma_mask	= 0xffffffff,
	},

	.num_resources	= ARRAY_SIZE(mmp3_u2oehci_resources),
	.resource	= mmp3_u2oehci_resources,
};

#ifdef CONFIG_USB_PXA_U2O_OTG
struct resource mmp3_u2ootg_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA168_U2O_REGBASE + U2x_CAPREGS_OFFSET,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2O_PHYBASE,
		.end	= PXA168_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_MMP3_USB_OTG,
		.end	= IRQ_MMP3_USB_OTG,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device mmp3_device_u2ootg = {
	.name		= "pxa-otg",
	.id		= -1,
	.dev		= {
		.dma_mask		= &usb_dma_mask,
		.coherent_dma_mask	= 0xffffffff,
	},

	.num_resources	= ARRAY_SIZE(mmp3_u2ootg_resources),
	.resource	= mmp3_u2ootg_resources,
};

#endif	/* CONFIG_USB_PXA_U2O_OTG */
#endif  /* CONFIG_USB_EHCI_PXA_U2O */

#ifdef CONFIG_USB_EHCI_PXA_U2H_HSIC
static struct resource mmp3_hsic1_resources[] = {
	/* reg base */
	[0] = {
		.start	= MMP3_HSIC1_REGBASE + U2x_CAPREGS_OFFSET,
		.end	= MMP3_HSIC1_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* phybase */
	[1] = {
		.start	= MMP3_HSIC1_PHYBASE,
		.end	= MMP3_HSIC1_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_MMP3_USB_HS1,
		.end	= IRQ_MMP3_USB_HS1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device mmp3_hsic1_device = {
	.name		= "mmp3-hsic",
	.id		= -1,
	.dev		= {
		.dma_mask		= &usb_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},

	.num_resources	= ARRAY_SIZE(mmp3_hsic1_resources),
	.resource	= mmp3_hsic1_resources,
};

static struct resource mmp3_hsic2_resources[] = {
	/* reg base */
	[0] = {
		.start  = MMP3_HSIC2_REGBASE + U2x_CAPREGS_OFFSET,
		.end    = MMP3_HSIC2_REGBASE + USB_REG_RANGE,
		.flags  = IORESOURCE_MEM,
		.name   = "capregs",
	},
	/* phybase */
	[1] = {
		.start  = MMP3_HSIC2_PHYBASE,
		.end    = MMP3_HSIC2_PHYBASE + USB_PHY_RANGE,
		.flags  = IORESOURCE_MEM,
		.name   = "phyregs",
	},
	[2] = {
		.start  = IRQ_MMP3_USB_HS2,
		.end    = IRQ_MMP3_USB_HS2,
		.flags  = IORESOURCE_IRQ,
	},
};

struct platform_device mmp3_hsic2_device = {
	.name           = "mmp3-hsic",
	.id             = -1,
	.dev            = {
		.dma_mask               = &usb_dma_mask,
		.coherent_dma_mask      = DMA_BIT_MASK(32),
	},

	.num_resources  = ARRAY_SIZE(mmp3_hsic2_resources),
	.resource       = mmp3_hsic2_resources,
};

int mmp3_hsic_private_init(struct mv_op_regs *opregs, unsigned int phyregs)
{
	u32 hsic_int;
	u32 status;
	int count;

	/*disable connect irq*/
	hsic_int = __raw_readl(phyregs + HSIC_INT);
	hsic_int &= ~HSIC_INT_CONNECT_INT_EN;
	__raw_writel(hsic_int, phyregs + HSIC_INT);

	/* enable port power and reserved bit 25 */
	status = __raw_readl(&opregs->portsc[0]);
	status |= (PORTSCX_PORT_POWER) | (1 << 25);
	/* Clear bits 30:31 for HSIC to be enabled */
	status &= ~(0x3 << 30);
	__raw_writel(status, &opregs->portsc[0]);

	/* test mode: force enable hs */
	status = __raw_readl(&opregs->portsc[0]);
	status &= ~(0xf << 16);
	status |= (0x5 << 16);
	__raw_writel(status, &opregs->portsc[0]);

	/* disable test mode */
	status = __raw_readl(&opregs->portsc[0]);
	status &= ~(0xf << 16);
	__raw_writel(status, &opregs->portsc[0]);

	/* check HS ready */
	count = 0x10000;
	do {
		hsic_int = __raw_readl(phyregs + HSIC_INT);
		count--;
	} while ((count >= 0)
		&& !(hsic_int & (HSIC_INT_HS_READY | HSIC_INT_CONNECT)));
	if (count <= 0) {
		pr_err("HS_READY not set: hsic_int 0x%x\n", hsic_int);
		return -EAGAIN;
	}

	/* issue port reset */
	status = __raw_readl(&opregs->portsc[0]);
	status |= PORTSCX_PORT_RESET;
	__raw_writel(status, &opregs->portsc[0]);

	/* check reset done */
	count = 0x10000;
	do {
		status = __raw_readl(&opregs->portsc[0]);
		count--;
	} while ((count >= 0) && !(status & PORTSCX_PORT_RESET));
	if (count <= 0) {
		pr_err("port reset not done: portsc 0x%x\n", status);
		return -EAGAIN;
	}

	return 0;
}

#endif  /* CONFIG_USB_EHCI_PXA_U2H_HSIC */

#endif  /* CONFIG_USB_EHCI_PXA */
#endif  /* CONFIG_USB_SUPPORT */
#ifdef CONFIG_UIO_VMETA
/* vmeta soc specific functions */
void vmeta_power_switch(unsigned int enable)
{
	unsigned int reg_vmpwr = 0;
	unsigned int timeout;

	reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
	if (VMETA_PWR_ENABLE == enable) {
		if (reg_vmpwr & (APMU_VMETA_PWRUP_ON|APMU_VMETA_ISB))
			return; /*Pwr is already on*/

		/* 0. reset to boot default value, source to PLL1 */
		writel(APMU_VMETA_CLK_RES_CTRL_DFT, APMU_VMETA_CLK_RES_CTRL);

		/* 1. Turn on power switches */
		reg_vmpwr |= APMU_VMETA_PWRUP_SLOW_RAMP;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);
		udelay(10);
		reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
		reg_vmpwr |= APMU_VMETA_PWRUP_ON;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);
		udelay(10);

		/* 2. disable isolation*/
		reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
		reg_vmpwr |= APMU_VMETA_ISB;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);

		/* 3 Assert the redundancy repair signal */
		reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
		reg_vmpwr |= APMU_VMETA_REDUN_START;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);
		timeout = 1000;
		do {
			if (--timeout == 0) {
				WARN(1, "vmeta: REDUN_START timeout!\n");
				return;
			}
			udelay(10);
			reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
		} while (reg_vmpwr & APMU_VMETA_REDUN_START);

		/* 4 enable dummy clock to the SRAM*/
		reg_vmpwr = readl(APMU_ISLD_VMETA_PDWN_CTRL);
		reg_vmpwr |= APMU_ISLD_CMEM_DMMYCLK_EN;
		writel(reg_vmpwr, APMU_ISLD_VMETA_PDWN_CTRL);
		udelay(250);
		reg_vmpwr = readl(APMU_ISLD_VMETA_PDWN_CTRL);
		reg_vmpwr &= ~APMU_ISLD_CMEM_DMMYCLK_EN;
		writel(reg_vmpwr, APMU_ISLD_VMETA_PDWN_CTRL);

		/* 5. enable vMeta AXI clock */
		reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
		reg_vmpwr |= APMU_VMETA_AXICLK_EN;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);

		/* 6. set up vMeta if required*/

		/* 7. enable vMeta clock */
		reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
		reg_vmpwr |= APMU_VMETA_CLK_EN;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);

		/* 8. deassert AXI reset*/
		reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
		reg_vmpwr |= APMU_VMETA_AXI_RST;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);

		/* 9. deassert vMeta reset*/
		reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
		reg_vmpwr |= APMU_VMETA_RST;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);
		udelay(10);

		/* 10. gate clock */
		reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
		reg_vmpwr &= ~APMU_VMETA_CLK_EN;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);
		reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
		reg_vmpwr &= ~APMU_VMETA_AXICLK_EN;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);

	} else if (VMETA_PWR_DISABLE == enable) {
		if ((reg_vmpwr & (APMU_VMETA_PWRUP_ON|APMU_VMETA_ISB)) == 0)
			return; /*Pwr is already off*/

		/* 1. isolation */
		reg_vmpwr &= ~APMU_VMETA_ISB;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);

		/* 2. reset*/
		reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
		reg_vmpwr &= ~(APMU_VMETA_AXI_RST | APMU_VMETA_RST);
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);

		/* 3. make sure clock disabled*/
		reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
		reg_vmpwr &= ~(APMU_VMETA_CLK_EN | APMU_VMETA_AXICLK_EN);
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);

		/* 4. turn off power */
		reg_vmpwr &= ~APMU_VMETA_PWRUP_ON;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);
	}
}

static void mmp_vmeta_unset_op_constraint_work(struct work_struct *work)
{
	struct vmeta_instance *vi = container_of(work, struct vmeta_instance, unset_op_work.work);

	vi->vop_real = VMETA_OP_INVALID;
	pm_qos_update_request(&vi->qos_cpufreq_min, PM_QOS_DEFAULT_VALUE);
	pm_qos_update_request(&vi->qos_ddrfreq_min, PM_QOS_DEFAULT_VALUE);
}


int vmeta_init_constraint(struct vmeta_instance *vi)
{
	mutex_init(&vi->op_mutex);
	INIT_DELAYED_WORK(&vi->unset_op_work, mmp_vmeta_unset_op_constraint_work);
	pm_qos_add_request(&vi->qos_cpufreq_min, PM_QOS_CPUFREQ_MIN,
			PM_QOS_DEFAULT_VALUE);
	pm_qos_add_request(&vi->qos_ddrfreq_min, PM_QOS_DDR_DEVFREQ_MIN,
            PM_QOS_DEFAULT_VALUE);
	return 0;
}

int vmeta_clean_constraint(struct vmeta_instance *vi)
{
	cancel_delayed_work_sync(&vi->unset_op_work);
	pm_qos_remove_request(&vi->qos_cpufreq_min);
	pm_qos_remove_request(&vi->qos_ddrfreq_min);
	printk(KERN_INFO "vmeta op clean up\n");

	return 0;
}

int vmeta_freq_change(struct vmeta_instance *vi, int step)
{
	int vop = vi->vop + step;

	if (vop > VMETA_OP_MIN && vop < VMETA_OP_MAX)
		return vop;
	else
		return -1;
}

int vmeta_runtime_constraint(struct vmeta_instance *vi, int on)
{
	int vop = vi->vop;

	mutex_lock(&vi->op_mutex);
	if (on) {
		cancel_delayed_work_sync(&vi->unset_op_work);
		if (vop < VMETA_OP_MIN || vop > VMETA_OP_MAX) {
			printk(KERN_DEBUG "unsupport vmeta vop=%d\n", vi->vop);
			vop = VMETA_OP_1080P_MAX;
		}
		if (vop == vi->vop_real)
			goto out;

		if (vop >= VMETA_OP_VGA && vop <= VMETA_OP_VGA_MAX) {
			printk(KERN_DEBUG "VGA!!!\n");
			pm_qos_update_request(&vi->qos_cpufreq_min, 200);
			pm_qos_update_request(&vi->qos_ddrfreq_min, PM_QOS_DEFAULT_VALUE);
		} else if (vop >= VMETA_OP_720P && vop <= VMETA_OP_720P_MAX) {
			printk(KERN_DEBUG "720P!!!\n");
			pm_qos_update_request(&vi->qos_cpufreq_min, 200);
			pm_qos_update_request(&vi->qos_ddrfreq_min, PM_QOS_DEFAULT_VALUE);
		} else { /* 1080p and default ops */
			printk(KERN_DEBUG "1080P!!!\n");
			pm_qos_update_request(&vi->qos_cpufreq_min, 400);
			pm_qos_update_request(&vi->qos_ddrfreq_min, 533333);
		}
		vi->vop_real = vop;
		printk(KERN_DEBUG "set dvfm vop_real=%d\n", vi->vop_real);
	} else {
		if (!vi->plat_data->power_down_ms)
			vi->plat_data->power_down_ms = 100;
		schedule_delayed_work(&vi->unset_op_work, msecs_to_jiffies(vi->plat_data->power_down_ms));
	}

out:
	mutex_unlock(&vi->op_mutex);
	return 0;
}

#endif


#define GC2D_CLK_DIV(n)		((n & 0xF) << 28)
#define GC2D_CLK_DIV_MSK	GC2D_CLK_DIV(0xF)
#define GC2D_CLK_SRC_SEL(n)	((n & 3) << 12)
#define GC2D_CLK_SRC_SEL_MSK	GC2D_CLK_SRC_SEL(3)
#define GC2D_AXICLK_EN		(1u << 19)
#define GC2D_AXI_RST_N		(1u << 18)

#define GC3D_CLK_DIV(n)		((n & 0xF) << 24)
#define GC3D_CLK_DIV_MSK	GC3D_CLK_DIV(0xF)
#define GC3D_CLK_SRC_SEL(n)	((n & 3) << 6)
#define GC3D_CLK_SRC_SEL_MSK	GC3D_CLK_SRC_SEL(3)
#define GC3D_ACLK_SEL(n)	((n & 3) << 4)
#define GC3D_ACLK_SEL_MSK	GC3D_ACLK_SEL(3)
#define GC3D_AXICLK_EN		(1u << 2)
#define GC3D_AXI_RST_N		(1u << 0)

#define GC2D3D_CLK_EN		(1u << 3)
#define GC2D3D_RST_N		(1u << 1)

#define GC_PWRUP(n)		((n & 3) << 9)
#define GC_PWRUP_MSK		GC_PWRUP(3)
#define GC_ISB			(1u << 8)

#define GC_CLK_RATE(div, src, aclk)				\
	(GC2D_CLK_DIV(div) | GC2D_CLK_SRC_SEL(src)		\
	| GC3D_CLK_DIV(div) | GC3D_CLK_SRC_SEL(src)		\
	| GC3D_ACLK_SEL(aclk))

#define GC_CLK_RATE_MSK						\
	(GC2D_CLK_DIV_MSK | GC2D_CLK_SRC_SEL_MSK		\
	| GC3D_CLK_DIV_MSK | GC3D_CLK_SRC_SEL_MSK		\
	| GC3D_ACLK_SEL_MSK)

#define GC_CLKRST_BOOT_DEFAULT GC_CLK_RATE(3, 0, 0)
/*
	1. Since CCIC is moved to ISPDMA power island on B0 board,
	need additional power enabling to access CCIC
	2.DXO and smart sensor will conflict that share this
	PMUA_ISP_PWR_CTRL.So if one of any sensor is poweron,
	we all can't poweroff ISP.
*/

static atomic_t isppwr_count;
int isppwr_power_control(int on)
{
	int reg;
	unsigned char count = ISP_POLL_COUNT;

	if (on) {
		if (0 == atomic_read(&isppwr_count)) {
			/*set ISP regs to the default value*/
			writel(0, APMU_ISPCLK);
			writel(0, APMU_ISPPWR);

			/*1. turn on the power switch*/
			reg = readl(APMU_ISPPWR);
			reg |= 0x1 << 9;
			writel(reg, APMU_ISPPWR);
			udelay(10);

			reg |= 0x3 << 9;
			writel(reg, APMU_ISPPWR);
			udelay(10);

			/*2. disable isp isolation*/
			reg |= 0x1 << 8;
			writel(reg, APMU_ISPPWR);

			/*3. start memory redundacy repair*/
			reg = readl(APMU_ISPCLK);
			reg |= 0x1 << 2;
			writel(reg, APMU_ISPCLK);
			udelay(10);
			while ((readl(APMU_ISPCLK) & (0x1 << 2)) && count--)
				udelay(5);
			if (readl(APMU_ISPCLK) & (0x1 << 2))
				printk(KERN_ERR "dxoisp err in memory redundacy repair\n");

			/*4. enable dummy clocks to the SRAMS*/
			reg = readl(APMU_ISLD_CI_CTRL);
			reg |= 0x1 << 4;
			writel(reg, APMU_ISLD_CI_CTRL);
			udelay(250);
			reg &= ~(0x1 << 4);
			writel(reg, APMU_ISLD_CI_CTRL);

			/*5. enable clks*/
			/*enable ISP AXI clock*/
			reg = readl(APMU_ISPCLK);
			reg |= 0x1 << 3;
			writel(reg, APMU_ISPCLK);
			/*clock source and clock divider */
			reg &= ~0xF00;
			reg |= 0x2 << 8;
			writel(reg, APMU_ISPCLK);
			reg &= ~0xC0;
			reg |= 0x1 << 6;
			writel(reg, APMU_ISPCLK);
			/*enable ISP clk*/
			reg |= 0x1 << 4;
			writel(reg, APMU_ISPCLK);
			/*enable CCIC1 AXI Arbiter clock*/
			reg = readl(APMU_CCIC_RST);
			reg |= 0x1 << 15;
			writel(reg, APMU_CCIC_RST);

			/*6. de-asset*/
			/*De-Assert ISP AXI reset*/
			reg = readl(APMU_ISPCLK);
			reg |= 0x1 << 0;
			writel(reg, APMU_ISPCLK);
			/*De-Assert ISP software reset*/
			reg |= 0x1 << 1;
			writel(reg, APMU_ISPCLK);
			/*De-Assert CCIC1 AXI Arbiter reset*/
			reg = readl(APMU_CCIC_RST);
			reg |= 0x1 << 16;
			writel(reg, APMU_CCIC_RST);

			/*7. gate clk*/
			/*disable ccic1 AXI Arbiter Clock*/
			reg = readl(APMU_CCIC_RST);
			reg &= ~(0x1 << 15);
			writel(reg, APMU_CCIC_RST);
			/*Disable ISP AXI clock*/
			reg = readl(APMU_ISPCLK);
			reg &= ~(0x1 << 4);
			writel(reg, APMU_ISPCLK);
			/*Disable ISP clock*/
			reg &= ~(0x1 << 3);
			writel(reg, APMU_ISPCLK);
		}

		if (atomic_inc_return(&isppwr_count) < 1) {
			printk(KERN_ERR "isp power on err\n");
			return -EINVAL;
		}
	} else {
		if( 1 == atomic_read(&isppwr_count)) {
			/*enable clk for reset*/
			reg = readl(APMU_ISPCLK);
			reg |= 0x1 << 3;
			writel(reg, APMU_ISPCLK);
			reg |= 0x1 << 4;
			writel(reg, APMU_ISPCLK);
			reg = readl(APMU_CCIC_RST);
			reg |= 0x1 << 15;
			writel(reg, APMU_CCIC_RST);

			/*start to power down ISP*/
			/*1. enable isp isolation*/
			reg = readl(APMU_ISPPWR);
			reg &= ~(0x1 << 8);
			writel(reg, APMU_ISPPWR);

			/*2. assert*/
			reg = readl(APMU_CCIC_RST);
			reg &= ~(0x1 << 16);
			writel(reg, APMU_CCIC_RST);
			reg = readl(APMU_ISPCLK);
			reg &= ~(0x1 << 0);
			writel(reg, APMU_ISPCLK);
			reg &= ~(0x1 << 1);
			writel(reg, APMU_ISPCLK);

			/*3. gate clk*/
			reg = readl(APMU_CCIC_RST);
			reg &= ~(0x1 << 15);
			writel(reg, APMU_CCIC_RST);
			reg = readl(APMU_ISPCLK);
			reg &= ~(0x1 << 4);
			writel(reg, APMU_ISPCLK);
			reg &= ~(0x1 << 3);
			writel(reg, APMU_ISPCLK);

			/*4. turn off the power switch*/
			reg = readl(APMU_ISPPWR);
			reg &= ~(0x3 << 9);
			writel(reg, APMU_ISPPWR);

			atomic_set(&isppwr_count, 0);
		} else if(atomic_dec_return(&isppwr_count) < 0) {
			printk(KERN_ERR "isp power off err\n");
			return -EINVAL;
		}
	}
	return 0;
}

void gc_pwr(int power_on)
{
	unsigned long regval;

	regval = __raw_readl(APMU_GC_CLK_RES_CTRL);
	if (power_on) {
		if (regval & (GC_PWRUP_MSK | GC_ISB))
			return; /*Pwr is already on*/

		/* 0, set to boot default value, source on PLL1*/
		writel(GC_CLKRST_BOOT_DEFAULT, APMU_GC_CLK_RES_CTRL);

		/* 1. Turn on power switches */
		regval = readl(APMU_GC_CLK_RES_CTRL);
		regval &= ~GC_PWRUP_MSK;
		regval |= GC_PWRUP(1);
		writel(regval, APMU_GC_CLK_RES_CTRL);
		udelay(10);
		regval |= GC_PWRUP(3);
		writel(regval, APMU_GC_CLK_RES_CTRL);
		udelay(10);

		/* 2. disable isolation*/
		regval = readl(APMU_GC_CLK_RES_CTRL);
		regval |= GC_ISB;
		writel(regval, APMU_GC_CLK_RES_CTRL);

		/* 3. enable SRAM dummy clock*/

		regval = readl(APMU_ISLD_GC2000_PDWN_CTRL);
		regval |= APMU_ISLD_CMEM_DMMYCLK_EN;
		writel(regval, APMU_ISLD_GC2000_PDWN_CTRL);
		udelay(250);
		regval = readl(APMU_ISLD_GC2000_PDWN_CTRL);
		regval &= ~APMU_ISLD_CMEM_DMMYCLK_EN;
		writel(regval, APMU_ISLD_GC2000_PDWN_CTRL);

		/* 4. enable AXI clocks*/
		regval = readl(APMU_GC_CLK_RES_CTRL);
		regval |= (GC3D_AXICLK_EN | GC2D_AXICLK_EN);
		writel(regval, APMU_GC_CLK_RES_CTRL);

		/* 5. change source if necessary*/

		/* 6. enable GC clock */
		regval = readl(APMU_GC_CLK_RES_CTRL);
		regval |= GC2D3D_CLK_EN;
		writel(regval, APMU_GC_CLK_RES_CTRL);

		/* 7. deassert resets*/
		/* GC300 requires AXI reset first, not align with B0 datasheet*/
		regval = readl(APMU_GC_CLK_RES_CTRL);
		regval |= GC2D3D_RST_N;
		writel(regval, APMU_GC_CLK_RES_CTRL);
		udelay(100);
		regval = readl(APMU_GC_CLK_RES_CTRL);
		regval |= (GC2D_AXI_RST_N | GC3D_AXI_RST_N);
		writel(regval, APMU_GC_CLK_RES_CTRL);

		/* 8 gate clock */
		regval = readl(APMU_GC_CLK_RES_CTRL);
		regval &= ~(GC2D_AXICLK_EN | GC3D_AXICLK_EN | GC2D3D_CLK_EN);
		writel(regval, APMU_GC_CLK_RES_CTRL);

	} else {
		if ((regval & (GC_PWRUP_MSK | GC_ISB)) == 0)
			return; /*Pwr is already off*/

		/* 1. isolation */
		regval = readl(APMU_GC_CLK_RES_CTRL);
		regval &= ~GC_ISB;
		writel(regval, APMU_GC_CLK_RES_CTRL);

		/* 2. reset*/
		regval = readl(APMU_GC_CLK_RES_CTRL);
		regval &= ~(GC2D_AXI_RST_N | GC3D_AXI_RST_N | GC2D3D_RST_N);
		writel(regval, APMU_GC_CLK_RES_CTRL);

		/* 3. make sure clock disabled*/
		regval = readl(APMU_GC_CLK_RES_CTRL);
		regval &= ~(GC2D_AXICLK_EN | GC3D_AXICLK_EN | GC2D3D_CLK_EN);
		writel(regval, APMU_GC_CLK_RES_CTRL);

		/* 4. turn off power */
		regval = readl(APMU_GC_CLK_RES_CTRL);
		regval &= ~GC_PWRUP_MSK;
		writel(regval, APMU_GC_CLK_RES_CTRL);
	}
}
EXPORT_SYMBOL_GPL(gc_pwr);

#ifdef CONFIG_MMP3_THERMAL
extern unsigned long read_temperature_sensor(int index);
static int
thermal_check(struct notifier_block *this, unsigned long event, void *ptr)
{
	printk(KERN_ERR "Current Temp0: %ld Temp1: %ld Temp2 %ld\n\n",
			read_temperature_sensor(0),
			read_temperature_sensor(1),
			read_temperature_sensor(2));
	return NOTIFY_DONE;
}

static struct notifier_block panic_thermal_check = {
	.notifier_call = thermal_check,
};

static int __init panic_notifier(void)
{
	atomic_notifier_chain_register(&panic_notifier_list, &panic_thermal_check);
	return 0;
}

core_initcall(panic_notifier);
#endif
