/*
 * linux/arch/arm/mach-mmp/fc-mmp2.c
 *
 * Copyright:	(C) 2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <mach/cputype.h>
#include <mach/sram.h>
#include <mach/mmp2_pm.h>
#include <mach/board_fc_ops.h>
#include <asm/io.h>
#include <generated/mach-types.h>

#undef FC_DEBUG
#ifdef FC_DEBUG
#define mmp2_fc_debug(s, args...)	printk(KERN_ERR s, ## args);
#else
#define mmp2_fc_debug(s, args...)
#endif

/*
 * frequence change based on mmp2
 */
extern struct mmp2_pm_info *mmp2_pm_info_p;
static DEFINE_MUTEX(pwr_i2c_conflict_mutex);
static void *pm_fc_vaddr, *pm_fc_vstack;
static struct mmp2_fc_param *mmp2_fc_param_p;
static struct tasklet_struct fc_seq_tasklet;
static int fc_seq_tasklet_en;
static atomic_t fb_is_suspended = ATOMIC_INIT(0);
DECLARE_COMPLETION(freq_complete);

struct mmp2_op mmp2_ops[] = {
	{
		.name = "Ultra Low MIPS",
		.vcc_core = MMP2_PPT_ULTRA_LOW_MIPS,
		.pclk_mhz = 100,
		.pdclk_mhz = 100,
		.baclk_mhz = 100,
		.xpclk_mhz = 100,
		.dclk_mhz = 400,
		.aclk_mhz = 100
	},
	{
		.name = "Low MIPS",
		.vcc_core = MMP2_PPT_LOW_MIPS,
		.pclk_mhz = 200,
		.pdclk_mhz = 200,
		.baclk_mhz = 200,
		.xpclk_mhz = 200,
		.dclk_mhz = 400,
		.aclk_mhz = 200
	},
	{
		.name = "Video/Graphics M MIPS",
		.vcc_core = MMP2_PPT_VG_MIPS,
		.pclk_mhz = 400,
		.pdclk_mhz = 400,
		.baclk_mhz = 400,
		.xpclk_mhz = 400,
		.dclk_mhz = 400,
		.aclk_mhz = 266
	},
	{
		.name = "Video/Graphics H MIPS",
		.vcc_core = MMP2_PPT_VG_HIGH_MIPS,
		.pclk_mhz = 800,
		.pdclk_mhz = 400,
		.baclk_mhz = 400,
		.xpclk_mhz = 400,
		.dclk_mhz = 400,
		.aclk_mhz = 266
	},
	{
		.name = "Ultra High MIPS",
		.vcc_core = MMP2_PPT_ULTRA_HIGH_MIPS,
		.pclk_mhz = 988,
		.pdclk_mhz = 494,
		.baclk_mhz = 494,
		.xpclk_mhz = 494,
		.dclk_mhz = 400,
		.aclk_mhz = 266
	},
};

inline void pwr_i2c_conflict_mutex_lock(void)
{
	if (machine_is_brownstone())
		mutex_lock(&pwr_i2c_conflict_mutex);
}
EXPORT_SYMBOL(pwr_i2c_conflict_mutex_lock);

inline void pwr_i2c_conflict_mutex_unlock(void)
{
	if (machine_is_brownstone())
		mutex_unlock(&pwr_i2c_conflict_mutex);
}
EXPORT_SYMBOL(pwr_i2c_conflict_mutex_unlock);

int mmp2_get_op_number(void)
{
	return ARRAY_SIZE(mmp2_ops);
}
EXPORT_SYMBOL(mmp2_get_op_number);

int check_cur_op(void)
{
	int pj4_clk_mhz, i;

	pj4_clk_mhz = mmp2_get_pj4_clk() / 1000000;
	for (i = 0; i < mmp2_get_op_number(); i++) {
		if (pj4_clk_mhz > mmp2_ops[i].pclk_mhz - 50 && pj4_clk_mhz < mmp2_ops[i].pclk_mhz + 50)				break;
	}
	if (i < mmp2_get_op_number())
		return i;
	else
		return MMP2_DEFAULT_OP_IDX;
}
EXPORT_SYMBOL(check_cur_op);

int mmp2_get_op_freq(int idx)
{
	if (idx < 0 || idx >= mmp2_get_op_number())
		return 0;
	return mmp2_ops[idx].pclk_mhz * MHZ_TO_KHZ;
}
EXPORT_SYMBOL(mmp2_get_op_freq);

typedef enum {
	PLL1_div_2 = 0,
	PLL1,
	PLL2,
	VCXO,
	INVALID_CLOCK
} clk_sel;

static clk_sel clk_source_select(unsigned int freq)
{
	switch (freq) {
	case 26:
		return VCXO;
	case 400:
		return PLL1_div_2;
	case 800:
		return PLL1;
	default:
		if (((797 + 3) % freq) < 3) {
			if (freq < 400)
				return PLL1_div_2;
			else if (freq < 800)
				return PLL1;
		} else if (freq <= 988)
			return PLL2;
		else
			return INVALID_CLOCK;
	};
	return INVALID_CLOCK;
}

/* get pll freq: MHZ */
static unsigned int get_pll_freq(clk_sel pll_mode)
{
	unsigned int freq;
	unsigned int mpmu_pll2cr;
	unsigned int refd, fbd;

	if (pll_mode == PLL1_div_2)
		freq = 800 / 2;
	else if (pll_mode == PLL1)
		freq = 800;
	else if (pll_mode == PLL2) {
		mpmu_pll2cr = readl(mmp2_pm_info_p->pmum_base + PLL2CR_OFF);
		fbd = (mpmu_pll2cr & PMUM_PLL2CR_FBDIV_MSK) >> 10;
		refd = (mpmu_pll2cr & PMUM_PLL2CR_FBDIV_MSK) >> 19;
		if (refd == 4)
			freq = 26 * (fbd + 2) / 6;
		else
			freq = 0;
	} else if (pll_mode == VCXO)
		freq = 26;
	else
		freq = 0;
	return freq;
}

static unsigned int pj_pm_setup_pll2(unsigned int freq)
{
	/* Fix the pll2refdiv to 1(+2), to get 8.66MHz ref clk
	 * Stable val recomended between 8-12MHz. To get the reqd
	 * freq val, just program the fbdiv
	 * freq takes effect during a fc req
	 */
	volatile unsigned int temp;
	unsigned int refdiv = 4;		// fixed to generate 8.66Mhz refclk
	unsigned int fbdiv;

	fbdiv = (freq / 26 * (refdiv + 2) - 2) + ((freq * (refdiv + 2)) % 26 > 13);  //round off

	/* make sure pll2 is in reset */
	temp = readl(mmp2_pm_info_p->pmum_base + PLL2_CTRL1);
	temp &= ~(0x1 << 29);
	writel(temp, mmp2_pm_info_p->pmum_base + PLL2_CTRL1);

	temp = readl(mmp2_pm_info_p->pmum_base + PLL2CR_OFF);
	/* Enable the pll2 and program the divider values */
	temp &= ~PMUM_PLL2CR_PLL2_SW_EN;
	writel(temp, mmp2_pm_info_p->pmum_base + PLL2CR_OFF);

	temp &= ~(PMUM_PLL2CR_FBDIV_MSK | PMUM_PLL2CR_REFDIV_MSK);
	temp |= ((unsigned int)refdiv << PMUM_PLL2CR_REFDIV_BASE);
	temp |= (fbdiv << PMUM_PLL2CR_FBDIV_BASE);
	temp |= PMUM_PLL2CR_CTRL;
	writel(temp, mmp2_pm_info_p->pmum_base + PLL2CR_OFF);

	temp |= PMUM_PLL2CR_PLL2_SW_EN;
	writel(temp, mmp2_pm_info_p->pmum_base + PLL2CR_OFF);

	/* pll2 out of reset */
	temp = readl(mmp2_pm_info_p->pmum_base + PLL2_CTRL1);
	temp |= (0x1 << 29);
	writel(temp, mmp2_pm_info_p->pmum_base + PLL2_CTRL1);

	temp = 0x200000;
	while( temp-- > 0 );

	return freq;
}

static void fc_seq_bh(unsigned long data)
{
	unsigned long flags, tmp;

	local_fiq_disable();
	local_irq_save(flags);
	run_fc_seq(mmp2_fc_param_p, mmp2_pm_info_p);

	tmp = readl(mmp2_pm_info_p->pmua_base + CC_MOH_OFF);
	tmp |= PMUA_CC_MOH_MOH_RD_ST_CLEAR;
	writel(tmp, mmp2_pm_info_p->pmua_base + CC_MOH_OFF);

	local_irq_restore(flags);
	local_fiq_enable();
	complete(&freq_complete);
}

int wakeup_freq_seq(void)
{
	if (!atomic_read(&fc_seq_tasklet.count) && fc_seq_tasklet_en) {
		fc_seq_tasklet_en = 0;
		tasklet_schedule(&fc_seq_tasklet);
	}
	return 0;
}
EXPORT_SYMBOL(wakeup_freq_seq);

static int PMUcore2_hw_fc_seq(struct mmp2_pm_info *info, int old_idx, int new_idx)
{
	uint32_t tmp, cc_reg, cc_sp, fccr, ret;
	uint32_t pll2_cur_freq, pll2_freq, pll_pj4_freq = 0, pll_ad_freq = 0;
	uint32_t pclk, pdclk, baclk, xpclk, dclk, aclk;
	uint32_t new_vcc_vol, old_vcc_vol;
	clk_sel pll_new_core, pll_new_ddr_axi;

	pclk = mmp2_ops[new_idx].pclk_mhz;
	pdclk = mmp2_ops[new_idx].pdclk_mhz;
	baclk = mmp2_ops[new_idx].baclk_mhz;
	xpclk = mmp2_ops[new_idx].xpclk_mhz;
	dclk = mmp2_ops[new_idx].dclk_mhz * 2;
	aclk = mmp2_ops[new_idx].aclk_mhz;

	pll2_cur_freq = get_pll_freq(PLL2);
	if (pll2_cur_freq == 0)
		pll2_freq = 988;
	else
		pll2_freq = pll2_cur_freq;
	if (pclk < pll2_freq && pclk != 0)
		pll2_freq = pclk;

	tmp = readl(info->pmua_base + CC_MOH_OFF);
	tmp &= ~PMUA_CC_MOH_MOH_RD_ST_CLEAR;
	writel(tmp, info->pmua_base + CC_MOH_OFF);

	if (cpu_is_mmp2()) {
		tmp = readl(info->pmua_base + DEBUG_REG);
		tmp |= (1 << 21);
		writel(tmp, info->pmua_base + DEBUG_REG);
	}
	tmp = readl(info->pmua_base + MOH_IMR_OFF);
	tmp |= (PMUA_MOH_IMR_MOH_FC_INTR_MASK);
	writel(tmp, info->pmua_base + MOH_IMR_OFF);

	tmp = readl(info->pmua_base + DM_CC_MOH_OFF);
	while (tmp & PMUA_DM_CC_MOH_SEA_RD_STATUS)
		tmp = readl(info->pmua_base + DM_CC_MOH_OFF);

	pll_new_core = clk_source_select(mmp2_get_op_freq(new_idx) / MHZ_TO_KHZ);
	pll_new_ddr_axi = clk_source_select(dclk);

	/* clock gating register */
	cc_reg = readl(info->pmum_base + ACGR_OFF);
	switch (pll_new_core) {
	case PLL1_div_2:
		pll_pj4_freq = 800 >> 1;
		cc_reg |= PMUM_ACGR_PLL1_2;
		writel(cc_reg, info->pmum_base + ACGR_OFF);
		break;
	case PLL1:
		pll_pj4_freq = 800;
		cc_reg |= PMUM_ACGR_PLL1;
		writel(cc_reg, info->pmum_base + ACGR_OFF);
		break;
	case PLL2:
		if (pll2_cur_freq != pll2_freq)
			pll_pj4_freq = pj_pm_setup_pll2(pll2_freq);
		else
			pll_pj4_freq = pll2_cur_freq;
		cc_reg |= PMUM_ACGR_PLL2EN;
		/* enable the new pll here */
		writel(cc_reg, info->pmum_base + ACGR_OFF);
		break;
	case VCXO:
		pll_pj4_freq = 26;
		break;
	default:
		break;
	}

	if (pclk > pll_pj4_freq)
		pclk = pll_pj4_freq;
	if (pdclk > pll_pj4_freq)
		pdclk = pll_pj4_freq;
	if (baclk > pll_pj4_freq)
		baclk = pll_pj4_freq;
	if (xpclk > pll_pj4_freq)
		xpclk = pll_pj4_freq;

	cc_reg = readl(info->pmum_base + ACGR_OFF);
	switch (pll_new_ddr_axi) {
	case PLL1_div_2:
		pll_ad_freq = 800 >> 1;
		cc_reg |= PMUM_ACGR_PLL1_2;
		writel(cc_reg, info->pmum_base + ACGR_OFF);
		break;
	case PLL1:
		pll_ad_freq = 800;
		cc_reg |= PMUM_ACGR_PLL1;
		writel(cc_reg, info->pmum_base + ACGR_OFF);
		break;
	case PLL2:
		pll_ad_freq = pll_pj4_freq;
		break;
	case VCXO:
		pll_ad_freq = 26;
		break;
	default:
		break;
	}
	if (aclk > pll_ad_freq)
		aclk = pll_ad_freq;
	if (dclk > pll_ad_freq)
		dclk = pll_ad_freq;

	/* frequency change register */
	fccr = readl(info->pmum_base + FCCR_OFF);
	fccr &= ~(PMUM_FCCR_MOHCLKSEL_MSK | PMUM_FCCR_AXICLKSEL_MSK);
	fccr |= ((pll_new_core << PMUM_FCCR_MOHCLKSEL_BASE) | (pll_new_ddr_axi << PMUM_FCCR_AXICLKSEL_BASE));

	cc_reg = 0;
	/* pclk divider */
	if (pclk != 0)
		cc_reg |= ((pll_pj4_freq / pclk - 1) << PMUA_CC_MOH_CORE_CLK_DIV_BASE);
	/* xp clock divider */
	if (xpclk != 0)
		cc_reg |= ((pll_pj4_freq / xpclk - 1) << PMUA_DM_CC_MOH_XP_CLK_DIV_BASE);
	/* bus clock divider */
	if (baclk != 0) {
		cc_reg |= ((pll_pj4_freq / baclk - 1) << PMUA_DM_CC_MOH_BIU_CLK_DIV_BASE);
		cc_reg |= ((pll_pj4_freq / baclk - 1) << PMUA_DM_CC_MOH_BUS_MC_CLK_DIV_BASE);
	}
	/* D clock divider */
	if (dclk != 0)
		cc_reg |= ((pll_ad_freq / dclk - 1) << PMUA_DM_CC_MOH_DDR_CLK_DIV_BASE);
	/* fabric clock */
	if (aclk != 0)
		cc_reg |= ((pll_ad_freq / aclk - 1) << PMUA_DM_CC_MOH_BUS_CLK_DIV_BASE);

	if (pdclk != dclk)
		cc_reg |= PMUA_CC_MOH_ASYNC2;
	else
		cc_reg &= ~PMUA_CC_MOH_ASYNC2;
	if (baclk != aclk)
		cc_reg |= PMUA_CC_MOH_ASYNC5;
	else
		cc_reg &= ~PMUA_CC_MOH_ASYNC5;
	cc_reg |= PMUA_CC_MOH_ASYNC2;
	cc_reg |= PMUA_CC_MOH_ASYNC5;

	cc_sp = cc_reg;

	if (mmp2_ops[old_idx].pclk_mhz != mmp2_ops[new_idx].pclk_mhz)
		cc_reg |= PMUA_CC_MOH_MOH_FREQ_CHG_REQ;
	if (mmp2_ops[old_idx].dclk_mhz != mmp2_ops[new_idx].dclk_mhz)
		cc_reg |= PMUA_CC_MOH_DDR_FREQ_CHG_REQ;
	if (mmp2_ops[old_idx].aclk_mhz != mmp2_ops[new_idx].aclk_mhz)
		cc_reg |= PMUA_CC_MOH_BUS_FREQ_CHG_REQ;

	/* set parameters to assemble code */
	mmp2_fc_param_p->va_pmum = info->pmum_base;
	mmp2_fc_param_p->va_pmua = info->pmua_base;
	mmp2_fc_param_p->va_dmcu = info->dmcu_base;
	mmp2_fc_param_p->cc_ap = cc_reg;
	mmp2_fc_param_p->fccr = fccr;
	/* FIXME  dclk not changed by default */
	mmp2_fc_param_p->mem_config = 255;
	new_vcc_vol = board_get_voltage(mmp2_ops[new_idx].vcc_core);
	old_vcc_vol = board_get_voltage(mmp2_ops[old_idx].vcc_core);
	if (new_vcc_vol == old_vcc_vol)
		mmp2_fc_param_p->v_flag = 0;
	else if (new_vcc_vol > old_vcc_vol)
		mmp2_fc_param_p->v_flag = 1;
	else if (new_vcc_vol < old_vcc_vol)
		mmp2_fc_param_p->v_flag = 2;
	mmp2_fc_param_p->voltage = new_vcc_vol * 1000;

	tmp = readl(info->pmua_base + CC_SEA_OFF);
	tmp = (cc_sp | PMUA_CC_SEA_SEA_ALLOW_SPD_CHG);
	writel(tmp, info->pmua_base + CC_SEA_OFF);

	tmp = readl(info->pmua_base + DEBUG_REG);
	tmp |= (1 << 2);
	writel(tmp, info->pmua_base + DEBUG_REG);

	mmp2_fc_debug("param: va_pmua=0x%X\n", (unsigned int)mmp2_fc_param_p->va_pmua);
	mmp2_fc_debug("param: va_pmum=0x%X\n", (unsigned int)mmp2_fc_param_p->va_pmum);
	mmp2_fc_debug("param: va_dmcu=0x%X\n", (unsigned int)mmp2_fc_param_p->va_dmcu);
	mmp2_fc_debug("param: cc_ap=0x%X\n", mmp2_fc_param_p->cc_ap);
	mmp2_fc_debug("param: fccr=0x%X\n", mmp2_fc_param_p->fccr);
	mmp2_fc_debug("param: voltage=%d\n", mmp2_fc_param_p->voltage);

	/* wakup dvfm seq */
	if (machine_is_brownstone())
		mutex_lock(&pwr_i2c_conflict_mutex);
	fc_seq_tasklet_en = 1;
	tasklet_enable(&fc_seq_tasklet);
	if (atomic_read(&fb_is_suspended))
		wakeup_freq_seq();
	ret = wait_for_completion_timeout(&freq_complete, msecs_to_jiffies(50));
	if (!ret) {
		ret = -ETIMEDOUT;
		goto out;
	} else
		ret = 0;

out:
	tasklet_disable(&fc_seq_tasklet);
	tasklet_kill(&fc_seq_tasklet);
	if (machine_is_brownstone())
		mutex_unlock(&pwr_i2c_conflict_mutex);
	return ret;
}

void mmp2_fc_seq(int old_idx, int new_idx)
{
	mmp2_fc_debug("enter fc seq: old_idx=%d new_idx=%d\n", old_idx, new_idx);
	mmp2_pm_info_p->fc_vaddr = pm_fc_vaddr;
	mmp2_pm_info_p->fc_vstack = pm_fc_vstack;
	PMUcore2_hw_fc_seq(mmp2_pm_info_p, old_idx, new_idx);
}
EXPORT_SYMBOL(mmp2_fc_seq);

static int dvfm_fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *evdata = data;

	if ((event == FB_EVENT_SUSPEND) || ((event == FB_EVENT_BLANK) &&
			(*(int *)evdata->data != FB_BLANK_UNBLANK)))
		atomic_set(&fb_is_suspended, 1);
	else if ((event == FB_EVENT_RESUME) || ((event == FB_EVENT_BLANK) &&
			(*(int *)evdata->data == FB_BLANK_UNBLANK)))
		atomic_set(&fb_is_suspended, 0);

	return 0;
}

static struct notifier_block dvfm_fb_notif = {
	.notifier_call = dvfm_fb_notifier_callback,
};

static int __init mmp2_fc_init(void)
{
	if (!cpu_is_mmp2())
		return -EIO;

	pm_fc_vaddr = (void *)FC_VIRT_BASE;
	pm_fc_vstack = pm_fc_vaddr + 0x800;
	mmp2_fc_debug("%s: vaddr=%X, vstack=%X\n", __func__, (unsigned int)pm_fc_vaddr, (unsigned int)pm_fc_vstack);
	setup_fc_seq(pm_fc_vaddr);

	if (!(mmp2_fc_param_p = kzalloc(sizeof(struct mmp2_fc_param), GFP_KERNEL))) {
		pr_err("failed to request mem for mmp2_fc_param\n");
		goto err;
	}
	tasklet_init(&fc_seq_tasklet, fc_seq_bh, 0);
	tasklet_disable(&fc_seq_tasklet);

	fb_register_client(&dvfm_fb_notif);

	return 0;
err:
	return -ENOMEM;
}
module_init(mmp2_fc_init);







