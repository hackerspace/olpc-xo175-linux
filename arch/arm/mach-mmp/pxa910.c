/*
 *  linux/arch/arm/mach-mmp/pxa910.c
 *
 *  Code specific to PXA910
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <asm/mach/time.h>
#include <asm/hardware/cache-tauros2.h>

#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/cputype.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/dma.h>
#include <mach/devices.h>
#include <mach/pxa910-squ.h>

#include <linux/platform_device.h>
#include <linux/mfd/ds1wm.h>
#include <linux/memblock.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <plat/mfp.h>
#include <plat/pmem.h>
#include "common.h"
#include "clock.h"
#include "acpuclock.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)
#define RIPC0_VIRT_BASE	(APB_VIRT_BASE + 0x3D000)
#define RIPC0_STATUS	(RIPC0_VIRT_BASE + 0x00)
#define FAB_CTRL	(AXI_VIRT_BASE + 0x260)

unsigned char __iomem *dmc_membase;
EXPORT_SYMBOL(dmc_membase);

static struct mfp_addr_map pxa910_mfp_addr_map[] __initdata =
{
	MFP_ADDR_X(GPIO0, GPIO54, 0xdc),
	MFP_ADDR_X(GPIO55, GPIO66, 0x2f0),
	MFP_ADDR_X(GPIO67, GPIO98, 0x1b8),
	MFP_ADDR_X(GPIO100, GPIO109, 0x238),
	MFP_ADDR_X(GPIO110, GPIO116, 0x298),

	MFP_ADDR(GPIO123, 0xcc),
	MFP_ADDR(GPIO124, 0xd0),

	MFP_ADDR(DF_IO0, 0x40),
	MFP_ADDR(DF_IO1, 0x3c),
	MFP_ADDR(DF_IO2, 0x38),
	MFP_ADDR(DF_IO3, 0x34),
	MFP_ADDR(DF_IO4, 0x30),
	MFP_ADDR(DF_IO5, 0x2c),
	MFP_ADDR(DF_IO6, 0x28),
	MFP_ADDR(DF_IO7, 0x24),
	MFP_ADDR(DF_IO8, 0x20),
	MFP_ADDR(DF_IO9, 0x1c),
	MFP_ADDR(DF_IO10, 0x18),
	MFP_ADDR(DF_IO11, 0x14),
	MFP_ADDR(DF_IO12, 0x10),
	MFP_ADDR(DF_IO13, 0xc),
	MFP_ADDR(DF_IO14, 0x8),
	MFP_ADDR(DF_IO15, 0x4),

	MFP_ADDR(DF_nCS0_SM_nCS2, 0x44),
	MFP_ADDR(DF_nCS1_SM_nCS3, 0x48),
	MFP_ADDR(SM_nCS0, 0x4c),
	MFP_ADDR(SM_nCS1, 0x50),
	MFP_ADDR(DF_WEn, 0x54),
	MFP_ADDR(DF_REn, 0x58),
	MFP_ADDR(DF_CLE_SM_OEn, 0x5c),
	MFP_ADDR(DF_ALE_SM_WEn, 0x60),
	MFP_ADDR(SM_SCLK, 0x64),
	MFP_ADDR(DF_RDY0, 0x68),
	MFP_ADDR(SM_BE0, 0x6c),
	MFP_ADDR(SM_BE1, 0x70),
	MFP_ADDR(SM_ADV, 0x74),
	MFP_ADDR(DF_RDY1, 0x78),
	MFP_ADDR(SM_ADVMUX, 0x7c),
	MFP_ADDR(SM_RDY, 0x80),

	MFP_ADDR_X(MMC1_DAT7, MMC1_WP, 0x84),
	MFP_ADDR(VCXOREQ, 0xd4),

	MFP_ADDR_END,
};

static u32 gc_current_clk_rate_flag;

#define MCB_CNTRL5_OFF 0x550
#define MCB_GC_SW_BYPASS (1<<2)

void gc_fc_ack_bypass(int bypass)
{
	unsigned int temp;

	temp = __raw_readl(dmc_membase + MCB_CNTRL5_OFF);
	if (bypass)
		__raw_writel(temp | MCB_GC_SW_BYPASS, dmc_membase + MCB_CNTRL5_OFF);
	else
		__raw_writel(temp & ~MCB_GC_SW_BYPASS, dmc_membase + MCB_CNTRL5_OFF);
}

static void gc500_clk_enable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);
	__raw_writel(tmp | 0x38, clk->clk_rst);
	gc_fc_ack_bypass(0);
}

static void gc500_clk_disable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);
	gc_fc_ack_bypass(1);
	__raw_writel(tmp & (~0x38), clk->clk_rst);
}

void gc_pwr(int power_on)
{
	if (power_on) {
		u32 tmp = __raw_readl(APMU_GC);

		tmp &= 0xc0;
		tmp |= gc_current_clk_rate_flag;
		__raw_writel(tmp | 0x1238, APMU_GC);	/* on1 */
		udelay(200); /* at least 200 us */
		__raw_writel(tmp | 0x1638, APMU_GC);	/* on2 */
		/* release function reset */
		__raw_writel(tmp | 0x163a, APMU_GC);
		udelay(100); /* at least 48 cycles */
		/* aReset hReset and disable isolation */
		__raw_writel(tmp | 0x173f, APMU_GC);

		gc_fc_ack_bypass(0);
	} else {
		gc_fc_ack_bypass(1);
		__raw_writel(0x738, APMU_GC);	/* reset AXI/AHB/function */
		udelay(100);
		__raw_writel(0x638, APMU_GC);	/* enable isolation */
		__raw_writel(0x238, APMU_GC);	/* off2 */
		__raw_writel(0x038, APMU_GC);	/* off1 */

		__raw_writel(0x0, APMU_GC);	/* all clear for power */
	}
}
EXPORT_SYMBOL(gc_pwr);

struct gc_rate_table {
	unsigned long	rate;
	unsigned int	flag;
};

static struct gc_rate_table gc500_rates [] = {
	/* put highest rate at the top of the table */
	{
		.rate	=	500500000,
		.flag	=	APMU_GC_PLL2_DIV2,
	},
	{
		.rate	=	403000000,
		.flag	=	APMU_GC_PLL2_DIV2,
	},
	{
		.rate	=	312000000,
		.flag	=	APMU_GC_312M,
	},
	{
		.rate	=	156000000,
		.flag	=	APMU_GC_156M,
	},
};

static int gc_lookaround_rate(unsigned long gc_clk2x, u32 *flag)
{
	int i;

	for (i=0; i<ARRAY_SIZE(gc500_rates); i++) {
		if (gc_clk2x >= gc500_rates[i].rate)
			break;
	}
	if (i==ARRAY_SIZE(gc500_rates)) i--;
	*flag = gc500_rates[i].flag;
	return gc500_rates[i].rate;
}

static int gc500_clk_setrate(struct clk *clk, unsigned long gc_clk2x)
{
#ifdef CONFIG_CPU_FREQ
	u32 tmp, flag;
	unsigned long pll2freq, rate;

	/* For pxa918, fix gc rate to 156MHZ */
	if (cpu_is_pxa918())
		rate = gc_lookaround_rate(312000000, &flag);
	/* For pxa921, fix gc rate to 250MHZ */
	else if (cpu_is_pxa921())
		rate = gc_lookaround_rate(500500000, &flag);
	/* For pxa920, gc clk can be selected freely */
	else
		rate = gc_lookaround_rate(gc_clk2x, &flag);

	if (rate > 312000000) {
		/* Note: get_pll2_freq use MHz instead of Hz */
		pll2freq = get_pll2_freq()*1000000;
		if (unlikely(pll2freq/2 != rate)) {
			printk(KERN_INFO "gc_clk2x will use %ldHz instead of %ldHz\n",
				pll2freq/2, rate);
			rate = pll2freq/2;
		}
		gc_aclk_fc();
	}

	printk("\nGC clk is %ldHz\n", rate);
	clk->rate = rate;
	__raw_writel(0xf, APMU_GC_PD);
	tmp = __raw_readl(clk->clk_rst);
	tmp &= ~0xc0;
	tmp |= flag;
	__raw_writel(tmp, clk->clk_rst);
	gc_current_clk_rate_flag = flag;
#endif
	return 0;
}

static unsigned long gc500_clk_getrate(struct clk *clk)
{
	return clk->rate;
}

struct clkops gc500_clk_ops = {
	.enable		= gc500_clk_enable,
	.disable	= gc500_clk_disable,
	.setrate	= gc500_clk_setrate,
	.getrate	= gc500_clk_getrate,
};


static void lcd_clk_enable(struct clk *clk)
{
	__raw_writel(clk->enable_val, clk->clk_rst);
}
static void lcd_clk_disable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);
	tmp &= ~0x38;	/* release from reset to keep register setting */
	__raw_writel(tmp, clk->clk_rst);
}

static int lcd_clk_setrate(struct clk *clk, unsigned long val)
{
	__raw_writel(val, clk->clk_rst);
	return 0;
}
static unsigned long lcd_clk_getrate(struct clk *clk)
{
	unsigned long rate = clk->rate;
	return rate;
}

struct clkops lcd_pn1_clk_ops = {
	.enable		= lcd_clk_enable,
	.disable	= lcd_clk_disable,
	.setrate	= lcd_clk_setrate,
	.getrate	= lcd_clk_getrate,
};

static void nand_clk_enable(struct clk *clk)
{
	__raw_writel(0x19b, clk->clk_rst);
}

static void nand_clk_disable(struct clk *clk)
{
	/* only disable peripheral clock */
	__raw_writel(0x18b, clk->clk_rst);
}

struct clkops nand_clk_ops = {
	.enable     = nand_clk_enable,
	.disable    = nand_clk_disable,
};

static void vctcxo_clk_enable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst = __raw_readl(clk->clk_rst);
	clk_rst |= clk->enable_val;
	__raw_writel(clk_rst, clk->clk_rst);
}

static void vctcxo_clk_disable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst = __raw_readl(clk->clk_rst);
	clk_rst &= ~clk->enable_val;
	__raw_writel(clk_rst, clk->clk_rst);
}

struct clkops vctcxo_clk_ops = {
	.enable		= vctcxo_clk_enable,
	.disable	= vctcxo_clk_disable,
};

#define APMASK(i)	(GPIO_REGS_VIRT + BANK_OFF(i) + 0x09c)

static void __init pxa910_init_gpio(void)
{
	int i;

	/* enable GPIO clock */
	__raw_writel(APBC_APBCLK | APBC_FNCLK, APBC_PXA910_GPIO);

	/* unmask GPIO edge detection for all 4 banks - APMASKx */
	for (i = 0; i < 4; i++)
		__raw_writel(0xffffffff, APMASK(i));

#ifdef CONFIG_SUSPEND
	pxa_init_gpio(IRQ_PXA910_AP_GPIO, 0, 127, pxa910_set_wake);
#endif
}

void __init pxa910_init_irq(void)
{
	icu_init_irq();
	pxa910_init_gpio();
}

void pxa910_ripc_lock(void)
{
	while (__raw_readl(RIPC0_STATUS))
		cpu_relax();
}

int pxa910_ripc_trylock(void)
{
	return !__raw_readl(RIPC0_STATUS);
}

void pxa910_ripc_unlock(void)
{
	__raw_writel(1, RIPC0_STATUS);
}

/* gssp clk ops: gssp is shared between AP and CP */
static void gssp_clk_enable(struct clk *clk)
{
	unsigned int gcer;
	/* GPB bus select: choose APB */
	__raw_writel(0x1, APBC_PXA910_GBS);
	/* GSSP clock control register: GCER */
	gcer = __raw_readl(clk->clk_rst) & ~(0x3 << 8);
	/* choose I2S clock */
	gcer |= APBC_FNCLK | (0x0 << 8);
	__raw_writel(gcer, clk->clk_rst);
	udelay(10);
	gcer |= APBC_APBCLK;
	__raw_writel(gcer, clk->clk_rst);
	udelay(10);
	gcer &= ~APBC_RST;
	__raw_writel(gcer, clk->clk_rst);
	pr_debug("gssp clk is open\n");
}

static void gssp_clk_disable(struct clk *clk)
{
	unsigned int gcer;
	gcer = __raw_readl(clk->clk_rst);
	gcer &= ~APBC_APBCLK;
	__raw_writel(gcer, clk->clk_rst);
	__raw_writel(0x0, APBC_PXA910_GBS);
	pr_debug("gssp clk is closed\n");
}

struct clkops gssp_clk_ops = {
	.enable = gssp_clk_enable,
	.disable = gssp_clk_disable,
	};

static void pwm_clk_enable(struct clk *clk)
{
	struct clk *clk_apb = NULL, *clk_share = NULL;
	unsigned long data;

	data = __raw_readl(clk->clk_rst) & ~(APBC_FNCLKSEL(7));
	data |= APBC_FNCLK | APBC_FNCLKSEL(clk->fnclksel);
	__raw_writel(data, clk->clk_rst);
	/*
	 * delay two cycles of the solwest clock between the APB bus clock
	 * and the functional module clock.
	 */
	udelay(10);

	if (!strcmp(clk->name, "pwm0")) {
		clk_share = clk_get_sys("pxa910-pwm.1", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk;
	} else if (!strcmp(clk->name, "pwm1")) {
		clk_share = clk_get_sys("pxa910-pwm.0", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk_share;
	} else if (!strcmp(clk->name, "pwm2")) {
		clk_share = clk_get_sys("pxa910-pwm.3", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk;
	} else if (!strcmp(clk->name, "pwm3")) {
		clk_share = clk_get_sys("pxa910-pwm.2", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk_share;
	}
	if ((clk->enabled + clk_share->enabled) == 1) {
		data = __raw_readl(clk_apb->clk_rst);
		data |= APBC_APBCLK;
		__raw_writel(data, clk_apb->clk_rst);
		udelay(10);
		if (!strcmp(clk->name, clk_apb->name)) {
			data = __raw_readl(clk->clk_rst);
			data &= ~APBC_RST;
			__raw_writel(data, clk->clk_rst);
		} else {
			data = __raw_readl(clk->clk_rst);
			data &= ~APBC_RST;
			__raw_writel(data, clk->clk_rst);
			data = __raw_readl(clk_apb->clk_rst);
			data &= ~APBC_RST;
			__raw_writel(data, clk_apb->clk_rst);
		}
	}
}

static void pwm_clk_disable(struct clk *clk)
{
	struct clk *clk_apb = NULL, *clk_share = NULL;
	unsigned long data;

	data = __raw_readl(clk->clk_rst) & ~(APBC_FNCLK | APBC_FNCLKSEL(7));
	__raw_writel(data, clk->clk_rst);
	udelay(10);

	if (!strcmp(clk->name, "pwm0")) {
		clk_share = clk_get_sys("pxa910-pwm.1", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk;
	} else if (!strcmp(clk->name, "pwm1")) {
		clk_share = clk_get_sys("pxa910-pwm.0", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk_share;
	} else if (!strcmp(clk->name, "pwm2")) {
		clk_share = clk_get_sys("pxa910-pwm.3", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk;
	} else if (!strcmp(clk->name, "pwm3")) {
		clk_share = clk_get_sys("pxa910-pwm.2", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk_share;
	}

	if ((clk->enabled + clk_share->enabled) == 0) {
		data = __raw_readl(clk_apb->clk_rst);
		data &= ~APBC_APBCLK;
		__raw_writel(data, clk_apb->clk_rst);
	}
}

struct clkops pwm_clk_ops = {
	.enable = pwm_clk_enable,
	.disable = pwm_clk_disable,
};

#define APBC_CLK_PWM_OPS(_name, _reg, _fnclksel, _rate, _ops)	\
	struct clk clk_##_name = {				\
		.name		= #_name,			\
		.clk_rst	= (void __iomem *)APBC_##_reg,	\
		.fnclksel	= _fnclksel,			\
		.rate		= _rate,			\
		.ops		= _ops,				\
	}

/* APB peripheral clocks */
static APBC_CLK(uart0, PXA910_UART0, 1, 14745600);
static APBC_CLK(uart1, PXA910_UART1, 1, 14745600);
static APBC_CLK(uart2, PXA910_UART2, 1, 14745600);
static APBC_CLK(twsi0, PXA910_TWSI0, 0, 33000000);
static APBC_CLK(twsi1, PXA910_TWSI1, 0, 33000000);
static APBC_CLK_PWM_OPS(pwm0, PXA910_PWM1, 0, 13000000, &pwm_clk_ops);
static APBC_CLK_PWM_OPS(pwm1, PXA910_PWM2, 0, 13000000, &pwm_clk_ops);
static APBC_CLK_PWM_OPS(pwm2, PXA910_PWM3, 0, 13000000, &pwm_clk_ops);
static APBC_CLK_PWM_OPS(pwm3, PXA910_PWM4, 0, 13000000, &pwm_clk_ops);
static APBC_CLK(ssp1, PXA910_SSP1, 4, 3250000);
static APBC_CLK(ssp2, PXA910_SSP2, 0, 0);
static APBC_CLK(rtc, PXA910_RTC, 0x8, 1);
static APBC_CLK(keypad, PXA910_KPC, 0, 32000);
static APBC_CLK(1wire,  PXA910_ONEWIRE,  0, 26000000);

static APMU_CLK_OPS(nand, NAND, 0x019b, 156000000, &nand_clk_ops);
static APMU_CLK(u2o, USB, 0x1b, 480000000);
static APMU_CLK(sdh0, SDH0, 0x001b, 44500000);
static APMU_CLK(sdh1, SDH1, 0x001b, 44500000);
/* Configure the clock as 52MHz since eMMC is used on SDH2 at pxa920
 * board. If SD 2.0 card is used on SDH2, according to SD 2.0 spec,
 * the max clock is limited to 50MHz, so this patch cannot be applied.
 */
static APMU_CLK(sdh2, SDH2, 0x005b, 52000000);
static APMU_CLK_OPS(lcd, LCD, 0x003f, 312000000, &lcd_pn1_clk_ops);
static APMU_CLK(ire, IRE, 0x9, 0);
static APMU_CLK(ccic_rst, CCIC_RST, 0x0, 312000000);
static APMU_CLK(ccic_gate, CCIC_GATE, 0xfff, 0);
static APMU_CLK_OPS(gc, GC, 0, 0, &gc500_clk_ops);
static APBC_CLK_OPS(gssp, PXA910_GCER, 0, 0, &gssp_clk_ops);

static MPMU_CLK_OPS(vctcxo, VRCR, 1, 26000000, &vctcxo_clk_ops);

/* device and clock bindings */
static struct clk_lookup pxa910_clkregs[] = {
	INIT_CLKREG(&clk_uart0, "pxa2xx-uart.0", NULL),
	INIT_CLKREG(&clk_uart1, "pxa2xx-uart.1", NULL),
	INIT_CLKREG(&clk_uart2, "pxa2xx-uart.2", NULL),
	INIT_CLKREG(&clk_twsi0, "pxa910-i2c.0", NULL),
	INIT_CLKREG(&clk_twsi1, "pxa910-i2c.1", NULL),
	INIT_CLKREG(&clk_pwm0, "pxa910-pwm.0", NULL),
	INIT_CLKREG(&clk_pwm1, "pxa910-pwm.1", NULL),
	INIT_CLKREG(&clk_pwm2, "pxa910-pwm.2", NULL),
	INIT_CLKREG(&clk_pwm3, "pxa910-pwm.3", NULL),
	INIT_CLKREG(&clk_ssp1, "pxa910-ssp.0", NULL),
	INIT_CLKREG(&clk_ssp2, "pxa910-ssp.1", NULL),
	INIT_CLKREG(&clk_gssp, "pxa910-ssp.4", NULL),
	INIT_CLKREG(&clk_nand, "pxa3xx-nand", NULL),
	INIT_CLKREG(&clk_u2o, NULL, "U2OCLK"),
	INIT_CLKREG(&clk_keypad, "pxa27x-keypad", NULL),
	INIT_CLKREG(&clk_rtc, NULL, "MMP-RTC"),
	INIT_CLKREG(&clk_1wire, NULL, "PXA-W1"),
	INIT_CLKREG(&clk_sdh0, "sdhci-pxa.0", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh1, "sdhci-pxa.1", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh2, "sdhci-pxa.2", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_lcd, NULL, "LCDCLK"),
	INIT_CLKREG(&clk_ire, "pxa910-ire.0", NULL),
	INIT_CLKREG(&clk_ccic_rst, "mv-camera.0", "CCICRSTCLK"),
	INIT_CLKREG(&clk_ccic_gate, "mv-camera.0", "CCICGATECLK"),
	INIT_CLKREG(&clk_gc, NULL, "GCCLK"),
	INIT_CLKREG(&clk_vctcxo, NULL, "VCTCXO"),
};

/*
 * ACIPC clock is initialized by CP, enable the clock by default
 * and this clock is always enabled.
 */
static void pxa910_init_acipc_clock(void)
{
	__raw_writel(0x3, APBC_PXA910_IPC);
}

static void pxa910_init_ripc_clock(void)
{
        __raw_writel(0x0, APBC_PXA910_RIPC);
}

struct platform_device pxa910_device_asoc_ssp1 = {
	.name		= "pxa-ssp-dai",
	.id		= 1,
};

struct platform_device pxa910_device_asoc_gssp = {
	.name		= "pxa-ssp-dai",
	.id		= 4,
};

struct platform_device pxa910_device_asoc_pcm = {
	.name		= "pxa-pcm-audio",
	.id		= -1,
};

struct platform_device pxa910_device_asoc_squ = {
	.name		= "pxa910-squ-audio",
	.id		= -1,
};

static int __init pxa910_init(void)
{
	if (cpu_is_pxa910()) {
#ifdef CONFIG_CACHE_TAUROS2
		tauros2_init();
#endif
		mfp_init_base(MFPR_VIRT_BASE);
		mfp_init_addr(pxa910_mfp_addr_map);
		pxa_init_dma(IRQ_PXA910_DMA_INT0, 32);
		pxa910_init_squ(2);

		platform_device_register(&pxa910_device_asoc_ssp1);
		platform_device_register(&pxa910_device_asoc_gssp);
		platform_device_register(&pxa910_device_asoc_pcm);
		platform_device_register(&pxa910_device_asoc_squ);

		/* Enable AXI write request for gc500 */
		__raw_writel(__raw_readl(FAB_CTRL) | 0x8, FAB_CTRL);
		clkdev_add_table(ARRAY_AND_SIZE(pxa910_clkregs));

		/* enable ac-ipc clock */
		pxa910_init_acipc_clock();
		pxa910_init_ripc_clock();
	}

	dmc_membase = ioremap(0xb0000000, 0x00001000);
	return 0;
}
postcore_initcall(pxa910_init);

/* system timer - clock enabled, 3.25MHz */
#define TIMER_CLK_RST	(APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(3))

static void __init pxa910_timer_init(void)
{
	/* reset and configure */
	__raw_writel(APBC_APBCLK | APBC_RST, APBC_PXA910_TIMERS);
	__raw_writel(TIMER_CLK_RST, APBC_PXA910_TIMERS);

	timer_init(IRQ_PXA910_AP1_TIMER1, IRQ_PXA910_AP1_TIMER2);
}

struct sys_timer pxa910_timer = {
	.init	= pxa910_timer_init,
};

/* on-chip devices */

/* NOTE: there are totally 3 UARTs on PXA910:
 *
 *   UART2   - Slow UART (can be used both by AP and CP)
 *   UART0/1 - Fast UART
 *
 * To be backward compatible with the legacy FFUART/BTUART/STUART sequence,
 * they are re-ordered as:
 *
 *   pxa910_device_uart0 - UART0 as FFUART
 *   pxa910_device_uart1 - UART1 as BTUART
 *   pxa910_device_uart2 - UART2 as GPS
 */
PXA910_DEVICE(uart0, "pxa2xx-uart", 0, UART0, 0xd4017000, 0x30, 21, 22);
PXA910_DEVICE(uart1, "pxa2xx-uart", 1, UART1, 0xd4018000, 0x30, 23, 24);
PXA910_DEVICE(uart2, "pxa2xx-uart", 2, UART2, 0xd4036000, 0x30, 4, 5);
PXA910_DEVICE(twsi0, "pxa910-i2c", 0, TWSI0, 0xd4011000, 0x40);
PXA910_DEVICE(twsi1, "pxa910-i2c", 1, TWSI1, 0xd4037000, 0x40);
PXA910_DEVICE(pwm1, "pxa910-pwm", 0, NONE, 0xd401a000, 0x10);
PXA910_DEVICE(pwm2, "pxa910-pwm", 1, NONE, 0xd401a400, 0x10);
PXA910_DEVICE(pwm3, "pxa910-pwm", 2, NONE, 0xd401a800, 0x10);
PXA910_DEVICE(pwm4, "pxa910-pwm", 3, NONE, 0xd401ac00, 0x10);
PXA910_DEVICE(ssp0, "pxa910-ssp", 0, SSP1, 0xd401b000, 0x90, 52, 53);
PXA910_DEVICE(ssp1, "pxa910-ssp", 1, SSP2, 0xd42a0c00, 0x90, 1, 2);
PXA910_DEVICE(ssp2, "pxa910-ssp", 2, SSP3, 0xd401C000, 0x90, 60, 61);
PXA910_DEVICE(gssp, "pxa910-ssp", 4, GSSP, 0xd4039000, 0x90, 6, 7);
PXA910_DEVICE(audiosram, "mmp-sram", 0, NONE, 0xd100a000, 0x15000);
PXA910_DEVICE(nand, "pxa3xx-nand", -1, NAND, 0xd4283000, 0x80, 97, 99);
PXA910_DEVICE(keypad, "pxa27x-keypad", -1, KEYPAD, 0xd4012000, 0x4c);
PXA910_DEVICE(sdh0, "sdhci-pxa", 0, MMC, 0xd4280000, 0x120);
PXA910_DEVICE(sdh1, "sdhci-pxa", 1, MMC, 0xd4280800, 0x120);
PXA910_DEVICE(sdh2, "sdhci-pxa", 2, MMC, 0xd4281000, 0x120);
PXA910_DEVICE(cnm, "pxa-cnm", -1, CNM, 0xd420d000, 0x1000);
PXA910_DEVICE(fb, "pxa168-fb", 0, LCD, 0xd420b000, 0x1ec);
PXA910_DEVICE(fb_ovly, "pxa168fb_ovly", 0, LCD, 0xd420b000, 0x1ec);
PXA910_DEVICE(ire, "pxa910-ire", 0, IRE, 0xd420C000, 0x90);
PXA910_DEVICE(camera, "mv-camera", 0, CCIC, 0xd420a000, 0xfff);

static struct resource pxa910_resource_rtc[] = {
	{ 0xd4010000, 0xd40100ff, NULL, IORESOURCE_MEM, },
	{ IRQ_PXA910_RTC_INT, IRQ_PXA910_RTC_INT, NULL, IORESOURCE_IRQ, },
	{ IRQ_PXA910_RTC_ALARM, IRQ_PXA910_RTC_ALARM, NULL, IORESOURCE_IRQ, },
};

struct platform_device pxa910_device_rtc = {
	.name		= "mmp-rtc",
	.id		= -1,
	.resource	= pxa910_resource_rtc,
	.num_resources	= ARRAY_SIZE(pxa910_resource_rtc),
};

static struct resource pxa910_resource_1wire[] = {
	{ 0xd4011800, 0xd4011814, NULL, IORESOURCE_MEM, },
	{ IRQ_PXA910_ONEWIRE, IRQ_PXA910_ONEWIRE, NULL,	\
	IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE, },
};

struct platform_device pxa910_device_1wire = {
	.name		= "pxa-w1",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(pxa910_resource_1wire),
	.resource	= pxa910_resource_1wire,
};

void pxa910_clear_keypad_wakeup(void)
{
	uint32_t val;
	uint32_t mask = APMU_PXA910_KP_WAKE_CLR;

	/* wake event clear is needed in order to clear keypad interrupt */
	val = __raw_readl(APMU_WAKE_CLR);
	__raw_writel(val | mask, APMU_WAKE_CLR);
}

static unsigned _cp_area_addr;
static unsigned _cp_area_size;
static int __init setup_cpmem(char *p)
{
	unsigned long size, start = 0x7000000;
	size = memparse(p, &p);
	if (*p == '@')
		start = memparse(p + 1, &p);

	_cp_area_addr = (unsigned)start;
	_cp_area_size = (unsigned)size;

	return 0;
}
early_param("cpmem", setup_cpmem);

static void __init pxa910_reserve_cpmem_memblock(void)
{
	if (!_cp_area_size)
		return;

	BUG_ON(memblock_reserve(_cp_area_addr, _cp_area_size) != 0);
	memblock_free(_cp_area_addr, _cp_area_size);
	memblock_remove(_cp_area_addr, _cp_area_size);
	printk(KERN_INFO "Reserving CP memory: %dM at %.8x\n",
				(unsigned)_cp_area_size/0x100000,
				(unsigned)_cp_area_addr);
}

#ifdef CONFIG_PXA910_1G_DDR_WORKAROUND
unsigned long _cnm_area_addr = 0x08000000;
unsigned long _cnm_area_size = 0x00200000;
static int __init setup_cnm_mem(char *p)
{
	unsigned long size, start = 0x08000000;
	size = memparse(p, &p);
	if (*p == '@')
		start = memparse(p + 1, &p);

	_cnm_area_addr = (unsigned)start;
	_cnm_area_size = (unsigned)size;

	return 0;
}
early_param("cnmmem", setup_cnm_mem);

static void __init pxa910_reserve_cnm_memblock(void)
{
	long ret;

	if (_cnm_area_size == 0)
		return;

	ret = memblock_reserve(_cnm_area_addr, _cnm_area_size);
	BUG_ON(ret != 0);

	memblock_free(_cnm_area_addr, _cnm_area_size);
	memblock_remove(_cnm_area_addr, _cnm_area_size);

	printk(KERN_INFO "Reserving cnm memory: %dM at %.8x\n",
				(unsigned)_cnm_area_size/0x100000,
				(unsigned)_cnm_area_addr);
}
#endif

void __init pxa910_reserve(void)
{
	/* Reserve memory for CP */
	pxa910_reserve_cpmem_memblock();

#ifdef CONFIG_PXA910_1G_DDR_WORKAROUND
	/* Reserve memory for cnm */
	pxa910_reserve_cnm_memblock();
#endif

#ifdef CONFIG_ANDROID_PMEM
	/* Reserve memory for pmem */
	pxa_reserve_pmem_memblock();
#endif
}
