/*
 *  linux/arch/arm/mach-mmp/clock.h
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/clkdev.h>

struct clkops {
	void			(*enable)(struct clk *);
	void			(*disable)(struct clk *);
	unsigned long		(*getrate)(struct clk *);
	int			(*setrate)(struct clk *, unsigned long);
};

struct clk {
	const struct clkops	*ops;
	const char	*name;

	void __iomem	*clk_rst;	/* clock reset control register */
	int		fnclksel;	/* functional clock select (APBC) */
	uint32_t	enable_val;	/* value for clock enable (APMU) */
	unsigned long	rate;
	int		enabled;
};

extern struct clkops apbc_clk_ops;
extern struct clkops apmu_clk_ops;

#define APBC_CLK(_name, _reg, _fnclksel, _rate)			\
struct clk clk_##_name = {					\
		.clk_rst	= (void __iomem *)APBC_##_reg,	\
		.fnclksel	= _fnclksel,			\
		.rate		= _rate,			\
		.ops		= &apbc_clk_ops,		\
}

#define APBC_CLK_OPS(_name, _reg, _fnclksel, _rate, _ops)	\
struct clk clk_##_name = {					\
		.clk_rst	= (void __iomem *)APBC_##_reg,	\
		.fnclksel	= _fnclksel,			\
		.rate		= _rate,			\
		.ops		= _ops,				\
}

#define APMU_CLK(_name, _reg, _eval, _rate)			\
struct clk clk_##_name = {					\
		.clk_rst	= (void __iomem *)APMU_##_reg,	\
		.enable_val	= _eval,			\
		.rate		= _rate,			\
		.ops		= &apmu_clk_ops,		\
}

#define APMU_CLK_OPS(_name, _reg, _eval, _rate, _ops)		\
struct clk clk_##_name = {					\
		.clk_rst	= (void __iomem *)APMU_##_reg,	\
		.enable_val	= _eval,			\
		.rate		= _rate,			\
		.ops		= _ops,				\
}

#define MPMU_CLK_OPS(_name, _reg, _eval, _rate, _ops)		\
struct clk clk_##_name = {					\
		.clk_rst	= (void __iomem *)MPMU_##_reg,	\
		.enable_val	= _eval,			\
		.rate		= _rate,			\
		.ops		= _ops,				\
}

#define AUD_CLK_OPS(_name, _ops)				\
struct clk clk_##_name = {					\
		.ops		= _ops,				\
}

#define INIT_CLKREG(_clk, _devname, _conname)			\
	{							\
		.clk		= _clk,				\
		.dev_id		= _devname,			\
		.con_id		= _conname,			\
	}

extern struct clk clk_pxa168_gpio;
extern struct clk clk_pxa168_timers;


/*
 * MMP platform has audio island, the clock tree is:
 *
 *  ------------------
 *  | MPMU's i2s clk |---                          div1
 *  ------------------  |                         ------   ----------
 *                      |                      -->| /n |-->| sysclk |
 *                      |                      |  ------   ----------
 *  ------------------  |   ----------------   |
 *  |      vcxo      |--|-->| sspa clk mux |-->|
 *  ------------------  |   ----------------   |   div2
 *          |           |                      |  ------   ------------
 *         \ /          |                      -->| /n |-->| sspa clk |
 *  ------------------  |                         ------   ------------
 *  |   audio pll    |---
 *  ------------------
 *
 * For audio island has completely isolation by self,
 * it's better only use the vcxo or audio pll as the source;
 * and use audio pll we can get more flexible clock tree
 * for different sample rate.
 *
 * So now the flow for setting audio clock is:
 * vcxo -> pll -> audio clock
 *
 */
struct apll_set {
	/* in */
	uint32_t vcxo;

	/* pll setting */
	uint32_t mclk;
	uint32_t fbcclk;
	uint32_t fract;

	/* audio clk setting */
	struct aclk_set *ac_set;
	uint32_t ac_num;
};

struct aclk_set {
	/* audio clk out */
	uint32_t aclk;

	/* setting val */
	uint32_t postdiv;
	uint32_t oclk_modulo;
	uint32_t oclk_pattern;
};

#define AUDIO_CLOCK_RATE_PRE_INIT	(-1)

extern struct clkops audio_clk_ops;
extern struct clkops sysclk_ops;
extern struct clkops sspa1_clk_ops;
extern struct clkops sspa2_clk_ops;
