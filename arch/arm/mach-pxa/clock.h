#include <linux/clkdev.h>
#include <linux/syscore_ops.h>

#define INIT_CLKREG(_clk,_devname,_conname)		\
	{						\
		.clk		= _clk,			\
		.dev_id		= _devname,		\
		.con_id		= _conname,		\
	}

#define DEFINE_CK(_name, _cken, _ops)			\
struct clk clk_##_name = {				\
		.ops		= _ops,			\
		.enable_val	= CKEN_##_cken,		\
	}

#define DEFINE_CLK(_name, _ops, _rate)			\
struct clk clk_##_name = {				\
		.ops    = _ops,                         \
		.rate   = _rate,                        \
	}



#define DEFINE_PXA2_CKEN(_name, _cken, _rate)	\
struct clk clk_##_name = {				\
		.ops		= &clk_pxa2xx_cken_ops,	\
		.rate		= _rate,		\
		.enable_val	= CKEN_##_cken,		\
	}

extern const struct clkops clk_pxa2xx_cken_ops;

void clk_pxa2xx_cken_enable(struct clk *clk);
void clk_pxa2xx_cken_disable(struct clk *clk);

extern struct syscore_ops pxa2xx_clock_syscore_ops;

#if defined(CONFIG_PXA3xx) || defined(CONFIG_PXA95x)
#define DEFINE_PXA3_CKEN(_name, _cken, _rate)		\
struct clk clk_##_name = {				\
		.ops		= &clk_pxa3xx_cken_ops,	\
		.rate		= _rate,		\
		.enable_val	= CKEN_##_cken,		\
	}

extern const struct clkops clk_pxa3xx_cken_ops;
extern const struct clkops clk_pxa3xx_hsio_ops;
extern const struct clkops clk_pxa3xx_ac97_ops;
extern const struct clkops clk_pxa3xx_pout_ops;
extern const struct clkops clk_pxa3xx_smemc_ops;

extern void clk_pxa3xx_cken_enable(struct clk *);
extern void clk_pxa3xx_cken_disable(struct clk *);

extern struct syscore_ops pxa3xx_clock_syscore_ops;

#endif
