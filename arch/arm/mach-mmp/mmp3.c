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

#include <asm/smp_twd.h>
#include <asm/mach/time.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>

#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/cputype.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/dma.h>
#include <mach/devices.h>
#include <mach/mmp3.h>
#include <mach/regs-usb.h>
#include <mach/soc_vmeta.h>
#include <mach/mmp_dma.h>

#include <linux/platform_device.h>

#include <plat/mfp.h>
#include <plat/pmem.h>

#include "common.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)

#define APMASK(i)	(GPIO_REGS_VIRT + BANK_OFF(i) + 0x9c)

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

void __init mmp3_reserve(void)
{
	/*reserve memory for pmem*/
	pxa_reserve_pmem_memblock();
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
		   [28..31]	reserved, SBZ
		*/
		/*
		   forece NO WA, for A0 memory performance, bug in WA
		   64KB way-size
		   clear bit[16] to make sure l2x0_init call take it as 8-way
		*/
		l2x0_init(l2x0_base, 0x00860000, 0xE200FFFF);
	}
}
#else
static void mmp3_init_l2x0(void) {}
#endif

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

	timer_init(IRQ_MMP3_TIMER1, IRQ_MMP3_TIMER2);
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

static int __init mmp3_init(void)
{
	/*
	  let's make minimum WCB open entries to 2 to boost memory access
	*/
	pj4b_wcb_config(2, OMITFLD, OMITFLD);

	mmp3_init_l2x0();

	mfp_init_base(MFPR_VIRT_BASE);
	mfp_init_addr(mmp3_addr_map);

	mmp3_init_gpio();

	pxa_init_dma(IRQ_MMP3_DMA_RIQ, 16);
	mmp_init_dma(IRQ_MMP3_DMA_RIQ);

	platform_device_register(&mmp3_device_asoc_sspa1);
	platform_device_register(&mmp3_device_asoc_platform);

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
MMP3_DEVICE(hdmi, "mmp3-hdmi", -1, HDMI, 0xd420b000, 0x1fff);
MMP3_DEVICE(sspa1, "mmp2-sspa", 0, SSPA1, 0xc0ffdc00, 0xb0, ADMA1_CH1,
	    ADMA1_CH0);
MMP3_DEVICE(sspa2, "mmp2-sspa", 1, SSPA2, 0xc0ffdd00, 0xb0, ADMA2_CH1,
	    ADMA2_CH0);
MMP3_DEVICE(audiosram, "mmp-sram", 0, NONE, 0xd1030000, 0x20000);
MMP3_DEVICE(camera0, "mv-camera", 0, CCIC1, 0xd420a000, 0x2ff);
MMP3_DEVICE(camera1, "mv-camera", 1, CCIC2, 0xd420a800, 0x2ff);
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

int mmp3_hsic_private_init(struct mv_op_regs *opregs, unsigned int phyregs)
{
	u32 hsic_int;
	u32 status;
	int count;

	hsic_int = __raw_readl(phyregs + HSIC_INT);
	status = __raw_readl(&opregs->portsc[0]);
	/*disable connect irq*/
	hsic_int &= ~HSIC_INT_CONNECT_INT_EN;
	__raw_writel(hsic_int, phyregs + HSIC_INT);

	/* enable port power and reserved bit 25 */
	status = __raw_readl(&opregs->portsc[0]);
	status |= (0x00001000) | (1 << 25);
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
		status = __raw_readl(&opregs->portsc[0]);
		count--;
	} while ((count >= 0) && !(hsic_int & HSIC_INT_HS_READY)
		&& !(hsic_int & HSIC_INT_CONNECT));
	if (count <= 0) {
		printk(KERN_INFO "HSIC_INT_HS_READY not set: hsic_int 0x%x\n",
			hsic_int);
		return -EAGAIN;
	}

	/* issue port reset */
	status = __raw_readl(&opregs->portsc[0]);
	status |= (1<<8);
	__raw_writel(status, &opregs->portsc[0]);

	/* check reset done */
	count = 0x10000;
	do {
		status = __raw_readl(&opregs->portsc[0]);
		count--;
	} while ((count >= 0) && !(status & (1<<8)));
	if (count <= 0) {
		printk(KERN_INFO "port reset not done: portsc 0x%x\n", status);
		return -EAGAIN;
	}
	return 0;
}

#endif  /* CONFIG_USB_EHCI_PXA_U2H_HSIC */

#endif  /* CONFIG_USB_EHCI_PXA */
#endif  /* CONFIG_USB_SUPPORT */
#ifdef CONFIG_UIO_VMETA
/* vmeta soc specific functions */
int mmp__vmeta_set_dvfm_constraint(int idx)
{
	return 0;
	/*dvfm_disable(idx);*/
}

int mmp__vmeta_unset_dvfm_constraint(int idx)
{
	return 0;
	/*dvfm_enable(idx);*/
}
void vmeta_pwr(unsigned int enableDisable)
{
	unsigned int reg_vmpwr = 0;
	reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
	if (VMETA_PWR_ENABLE == enableDisable) {
		if (reg_vmpwr & (APMU_VMETA_PWRUP_ON|APMU_VMETA_ISB))
			return; /*Pwr is already on*/

		reg_vmpwr |= APMU_VMETA_PWRUP_SLOW_RAMP;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);
		reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
		mdelay(1);

		reg_vmpwr |= APMU_VMETA_PWRUP_ON;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);
		reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);
		mdelay(1);

		reg_vmpwr = readl(APMU_VMETA_CLK_RES_CTRL);

		reg_vmpwr |= APMU_VMETA_ISB;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);
	} else if (VMETA_PWR_DISABLE == enableDisable) {
		if ((reg_vmpwr & (APMU_VMETA_PWRUP_ON|APMU_VMETA_ISB)) == 0)
			return; /*Pwr is already off*/

		reg_vmpwr &= ~APMU_VMETA_ISB;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);

		reg_vmpwr &= ~APMU_VMETA_PWRUP_ON;
		writel(reg_vmpwr, APMU_VMETA_CLK_RES_CTRL);
	}
}

#endif
