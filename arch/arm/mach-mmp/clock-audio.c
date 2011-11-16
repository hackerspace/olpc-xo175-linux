/*
 * linux/arch/arm/mach-mmp/clock-mmp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/syscore_ops.h>
#include <mach/regs-audio.h>
#include <mach/regs-apmu.h>
#include <mach/regs-sspa.h>

#include "common.h"
#include "clock.h"

#define MMP_VCXO_VAL	26000000

static unsigned long aclk_rate = AUDIO_CLOCK_RATE_PRE_INIT;

/* audio pll -> audio clk */
static struct aclk_set ac_set_135mhz[] = {
/*   aclk    pdiv    o_m    o_p  *
 * --------- -----  -----  ----- */
{ 135475200,   1,     0,     0 },
{  33868800,   4,     0,     0 },
{  22579200,   6,     1,     1 },
{  16934400,   8,     1,     0 },
{  11289600,  12,     2,     1 },
{   8467200,  16,     2,     0 },
{   5644800,  24,     4,     1 },
};

static struct aclk_set ac_set_147mhz[] = {
/*   aclk    pdiv    o_m    o_p  *
 * --------- -----  -----  ----- */
{ 147456000,   1,     0,     0 },
{  36864000,   4,     0,     0 },
{  24576000,   6,     1,     1 },
{  18432000,   8,     1,     0 },
{  16384000,   9,     1,     2 },
{  12288000,  12,     2,     1 },
{   9216000,  16,     2,     0 },
{   8192000,  18,     2,     2 },
{   6144000,  24,     4,     1 },
{   4096000,  36,     4,     2 },
{   3072000,  48,     6,     1 },
{   2048000,  72,     6,     2 },
};

/* vcxo -> audio pll */
static struct apll_set ap_set[] = {
/*   vcxo    mclk  fbc    fra       aclk_set                    *
 * --------  ----  ----  -------  ----------------------------- */
{ 26000000,   0,    0,   0x8a18,  ARRAY_AND_SIZE(ac_set_135mhz) },
{ 26000000,   0,    1,   0x0da1,  ARRAY_AND_SIZE(ac_set_147mhz) },
{ 38400000,   1,    2,   0x8208,  ARRAY_AND_SIZE(ac_set_135mhz) },
{ 38400000,   1,    3,   0xaaab,  ARRAY_AND_SIZE(ac_set_147mhz) },
};

static void audio_clk_enable(struct clk *clk)
{
	unsigned int val = 0;

	val = APMU_AUDIO_RST_DIS | APMU_AUDIO_ISO_DIS |
		APMU_AUDIO_CLK_ENA | APMU_AUDIO_PWR_UP;
	__raw_writel(val, APMU_AUDIO_CLK_RES_CTRL);
}

static void audio_clk_disable(struct clk *clk)
{
	unsigned int val;

	val = __raw_readl(APMU_AUDIO_CLK_RES_CTRL);
	val &= ~(APMU_AUDIO_RST_DIS | APMU_AUDIO_ISO_DIS |
		 APMU_AUDIO_CLK_ENA | APMU_AUDIO_PWR_UP);
	__raw_writel(val, APMU_AUDIO_CLK_RES_CTRL);
}

static int audio_clk_setrate(struct clk *clk, unsigned long rate)
{
	int i, j;
	struct apll_set *ap_s;
	struct aclk_set *ac_s;
	unsigned long val;

	pr_debug("%s: rate %ld\n", __func__, rate);

	for (i = 0; i < ARRAY_SIZE(ap_set); i++) {

		/* find audio pll setting */
		ap_s = &ap_set[i];
		if (ap_s->vcxo != MMP_VCXO_VAL)
			continue;

		/* find audio clk setting */
		for (j = 0; j < ap_s->ac_num; j++) {
			ac_s = &ap_s->ac_set[j];
			pr_debug("%s: aclk %d\n", __func__, ac_s->aclk);
			if (ac_s->aclk == rate)
				goto found;
		}
	}

	/* not found setting for the rate */
	return -EINVAL;

found:
	val = AUD_PLL_CTL1_CLK_SEL_AUDIO_PLL |
	      AUD_PLL_CTL1_PLL_LOCK |
	      AUD_PLL_CTL1_DIV_OCLK_PATTERN(ac_s->oclk_pattern);
	__raw_writel(val, AUD_PLL_CTL1);

	val = AUD_PLL_CTL0_DIV_OCLK_MODULO(ac_s->oclk_modulo) |
	      AUD_PLL_CTL0_FRACT(ap_s->fract) |
	      AUD_PLL_CTL0_ENA_DITHER |
	      AUD_PLL_CTL0_DIV_FBCCLK(ap_s->fbcclk) |
	      AUD_PLL_CTL0_DIV_MCLK(ap_s->mclk) |
	      AUD_PLL_CTL0_PU;
	__raw_writel(val, AUD_PLL_CTL0);

	clk->rate = rate;
	aclk_rate = rate;
	return 0;
}

/* Audio Bus Clock/Reset Controler */
struct clkops audio_clk_ops = {
	.enable		= audio_clk_enable,
	.disable	= audio_clk_disable,
	.setrate	= audio_clk_setrate,
};

static void sysclk_enable(struct clk *clk)
{
	unsigned int val;

	/* enable sysclk */
	val = __raw_readl(AUD_CTL);
	val |= AUD_CTL_SYSCLK_ENA;
	__raw_writel(val, AUD_CTL);
	pr_debug("%s: actrl %x\n", __func__, __raw_readl(AUD_CTL));
}

static void sysclk_disable(struct clk *clk)
{
	unsigned int val;

	/* disable sysclk */
	val = __raw_readl(AUD_CTL);
	val &= ~AUD_CTL_SYSCLK_ENA;
	__raw_writel(val, AUD_CTL);
	pr_debug("%s: actrl %x\n", __func__, __raw_readl(AUD_CTL));
}

static int sysclk_setrate(struct clk *clk, unsigned long rate)
{
	int val, div;

	/* aclk not init yet */
	if (aclk_rate < 0)
		return 0;

	div = aclk_rate;
	do_div(div, rate);
	pr_debug("%s: aclk_rate %ld rate %ld devider = %d\n",
		__func__, aclk_rate, rate, div);

	val = __raw_readl(AUD_CTL);
	val &= ~AUD_CTL_SYSCLK_DIV_MASK;
	val |= AUD_CTL_SYSCLK_DIV(div);
	__raw_writel(val, AUD_CTL);
	pr_debug("%s: actrl %x\n", __func__, __raw_readl(AUD_CTL));

	clk->rate = rate;
	return 0;
}

/* SSPA Audio Control Sysclk Divider */
struct clkops sysclk_ops = {
	.enable		= sysclk_enable,
	.disable	= sysclk_disable,
	.setrate	= sysclk_setrate,
};

static void sspa1_clk_enable(struct clk *clk)
{
	unsigned int val;

	val = __raw_readl(AUD_CTL);
	val |= AUD_CTL_S1_ENA;
	__raw_writel(val, AUD_CTL);
	pr_debug("%s: actrl %x\n", __func__, __raw_readl(AUD_CTL));
}

static void sspa1_clk_disable(struct clk *clk)
{
	unsigned int val;

	val = __raw_readl(AUD_CTL);
	val &= ~AUD_CTL_S1_ENA;
	__raw_writel(val, AUD_CTL);
	pr_debug("%s: actrl %x\n", __func__, __raw_readl(AUD_CTL));
}

static int sspa1_clk_setrate(struct clk *clk, unsigned long rate)
{
	int val, div;

	/* aclk not init yet */
	if (aclk_rate < 0)
		return 0;

	div = aclk_rate;
	do_div(div, rate);
	pr_debug("%s: aclk_rate %ld rate %ld devider %d\n",
		__func__, aclk_rate, rate, div);

	val = __raw_readl(AUD_CTL);
	val &= ~AUD_CTL_S1_CLK_DIV_MASK;
	val |= AUD_CTL_S1_CLK_DIV(div);
	__raw_writel(val, AUD_CTL);
	pr_debug("%s: actrl %x\n", __func__, __raw_readl(AUD_CTL));

	clk->rate = rate;
	return 0;
}

struct clkops sspa1_clk_ops = {
	.enable		= sspa1_clk_enable,
	.disable	= sspa1_clk_disable,
	.setrate	= sspa1_clk_setrate,
};

static void sspa2_clk_enable(struct clk *clk)
{
	unsigned int val;

	val = __raw_readl(AUD_CTL);
	val |= AUD_CTL_S2_ENA;
	__raw_writel(val, AUD_CTL);
	pr_debug("%s: actrl %x\n", __func__, __raw_readl(AUD_CTL));
}

static void sspa2_clk_disable(struct clk *clk)
{
	unsigned int val;

	val = __raw_readl(AUD_CTL);
	val &= ~AUD_CTL_S2_ENA;
	__raw_writel(val, AUD_CTL);
	pr_debug("%s: actrl %x\n", __func__, __raw_readl(AUD_CTL));
}

static int sspa2_clk_setrate(struct clk *clk, unsigned long rate)
{
	int val, div;

	/* aclk not init yet */
	if (aclk_rate < 0)
		return 0;

	div = aclk_rate;
	do_div(div, rate);
	pr_debug("%s: aclk_rate %ld rate %ld devider %d\n",
		__func__, aclk_rate, rate, div);

	val = __raw_readl(AUD_CTL);
	val &= ~AUD_CTL_S2_CLK_DIV_MASK;
	val |= AUD_CTL_S2_CLK_DIV(div);
	__raw_writel(val, AUD_CTL);
	pr_debug("%s: actrl %x\n", __func__, __raw_readl(AUD_CTL));

	clk->rate = rate;
	return 0;
}

struct clkops sspa2_clk_ops = {
	.enable		= sspa2_clk_enable,
	.disable	= sspa2_clk_disable,
	.setrate	= sspa2_clk_setrate,
};

static uint32_t aud_ctrl;
static uint32_t aud_pll_ctrl0;
static uint32_t aud_pll_ctrl1;

static struct clk *sysclk;
static struct clk *audioclk;

static int pxa_audio_clock_suspend(void)
{
	sysclk_disable(sysclk);

	aud_ctrl      = __raw_readl(AUD_CTL);
	aud_pll_ctrl0 = __raw_readl(AUD_PLL_CTL0);
	aud_pll_ctrl1 = __raw_readl(AUD_PLL_CTL1);

	return 0;
}

static void pxa_audio_clock_resume(void)
{
	__raw_writel(aud_ctrl,      AUD_CTL);
	__raw_writel(aud_pll_ctrl0, AUD_PLL_CTL0);
	__raw_writel(aud_pll_ctrl1, AUD_PLL_CTL1);

	sysclk_enable(sysclk);
}

struct syscore_ops pxa_audio_syscore_ops = {
	.suspend        = pxa_audio_clock_suspend,
	.resume         = pxa_audio_clock_resume,
};
