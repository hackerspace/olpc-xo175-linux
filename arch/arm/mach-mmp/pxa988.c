/*
 * linux/arch/arm/mach-mmp/pxa988.c
 *
 * code name PXA988
 *
 * Copyright (C) 2012 Marvell International Ltd.
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
#include <linux/notifier.h>
#include <linux/memblock.h>
#include <linux/platform_device.h>

#include <asm/mach/time.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>

#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-pmu.h>
#include <mach/cputype.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/dma.h>
#include <mach/devices.h>
#include <mach/pxa988.h>
#include <mach/regs-timers.h>
#include <mach/pxa910-squ.h>

#include <plat/mfp.h>
#include <plat/pmem.h>

#include "common.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)

static struct mfp_addr_map pxa988_addr_map[] __initdata = {

	MFP_ADDR_X(GPIO0, GPIO54, 0xdc),
	MFP_ADDR_X(GPIO67, GPIO98, 0x1b8),
	MFP_ADDR_X(GPIO100, GPIO109, 0x238),
	MFP_ADDR_X(GPIO110, GPIO116, 0x298),

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
	MFP_ADDR(ANT_SW4, 0x26c),

	MFP_ADDR_X(MMC1_DAT7, MMC1_WP, 0x84),

	MFP_ADDR(GPIO124, 0xd0),

	MFP_ADDR(CLK_REQ, 0xcc),

	MFP_ADDR_END,
};

#ifdef CONFIG_SMP
u32 pm_reserve_pa;
#define PM_RESERVE_SIZE	(1024 * 1024)
#endif

void __init pxa988_reserve(void)
{
#ifdef CONFIG_ANDROID_PMEM
	/*reserve memory for pmem*/
	pxa_reserve_pmem_memblock();
#endif
#ifdef CONFIG_SMP
	pm_reserve_pa = memblock_alloc(PM_RESERVE_SIZE, PAGE_SIZE);
	if (!pm_reserve_pa) {
		pr_err("%s: failed to reserve memory for PM\n", __func__);
		BUG();
	}
	BUG_ON(memblock_free(pm_reserve_pa, PM_RESERVE_SIZE));
	BUG_ON(0 != memblock_remove(pm_reserve_pa, PM_RESERVE_SIZE));
#endif
}

void __init pxa988_init_irq(void)
{
	void __iomem *dist_base = (void __iomem *)GIC_DIST_VIRT_BASE;
	void __iomem *cpu_base = (void __iomem *)GIC_CPU_VIRT_BASE;

	gic_init(0, 29, dist_base, cpu_base);

	pxa988_init_gic();
}

#ifdef CONFIG_CACHE_L2X0
static void pxa988_l2_cache_init(void)
{
	void __iomem *l2x0_base;

	l2x0_base = ioremap(SL2C_PHYS_BASE, SZ_4K);
	BUG_ON(!l2x0_base);

	l2x0_init(l2x0_base, 0x30860000, 0xC200FFFF);
}
#else
#define pxa988_l2_cache_init()
#endif

static void __init pxa988_timer_init(void)
{
	uint32_t clk_rst;

	__raw_writel(APBC_APBCLK | APBC_RST, APBC_PXA988_TIMERS);
	clk_rst = APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(1);
	__raw_writel(clk_rst, APBC_PXA988_TIMERS);

	stimer_source_select(2, TIMER_RATE_32K);
	stimer_event_config(0, 0, IRQ_PXA988_AP_TIMER1, 6500000);
	stimer_event_config(1, 1, IRQ_PXA988_AP_TIMER2_3, 6500000);
	stimer_device_init(TIMERS1_VIRT_BASE);
}

struct sys_timer pxa988_timer = {
	.init   = pxa988_timer_init,
};

void pxa988_clear_keypad_wakeup(void)
{
	uint32_t val;
	uint32_t mask = APMU_PXA988_KP_WAKE_CLR;

	/* wake event clear is needed in order to clear keypad interrupt */
	val = __raw_readl(APMU_WAKE_CLR);
	__raw_writel(val | mask, APMU_WAKE_CLR);
}

struct platform_device pxa988_device_asoc_ssp1 = {
	.name		= "pxa-ssp-dai",
	.id		= 1,
};

struct platform_device pxa988_device_asoc_squ = {
	.name		= "pxa910-squ-audio",
	.id		= -1,
};

static int __init pxa988_init(void)
{
	pxa988_l2_cache_init();

	mfp_init_base(MFPR_VIRT_BASE);
	mfp_init_addr(pxa988_addr_map);

	/* would remove such pxa910 interface when kernel upgrade */
	pxa910_init_squ(2);
	/* add ssp and squ device for hifi audio */
	platform_device_register(&pxa988_device_asoc_ssp1);
	platform_device_register(&pxa988_device_asoc_squ);

	return 0;
}

postcore_initcall(pxa988_init);

/* on-chip devices */
PXA988_DEVICE(uart1, "pxa2xx-uart", 0, UART1_CP, 0xd4036000, 0x30, 4, 5);
PXA988_DEVICE(uart2, "pxa2xx-uart", 1, UART2, 0xd4017000, 0x30, 21, 22);
PXA988_DEVICE(uart3, "pxa2xx-uart", 2, UART3, 0xd4018000, 0x30, 23, 24);
PXA988_DEVICE(keypad, "pxa27x-keypad", -1, KEYPAD, 0xd4012000, 0x4c);
PXA988_DEVICE(twsi0, "pxa910-i2c", 0, I2C_AP,   0xd4011000, 0x40);
PXA988_DEVICE(twsi1, "pxa910-i2c", 1, I2C_INT,  0xd4037000, 0x40);
PXA988_DEVICE(twsi2, "pxa910-i2c", 2, I2C_CP,   0xd4010800, 0x40);
PXA988_DEVICE(ssp0, "pxa988-ssp", 0, SSP0, 0xd401b000, 0x90, 52, 53);
PXA988_DEVICE(ssp1, "pxa988-ssp", 1, SSP1, 0xd42a0c00, 0x90, 1, 2);
PXA988_DEVICE(ssp2, "pxa988-ssp", 2, SSP2, 0xd401C000, 0x90, 60, 61);
PXA988_DEVICE(asram, "mmp-sram", 0, NONE, SRAM_AUDIO_BASE, SRAM_AUDIO_SIZE);
PXA988_DEVICE(vsram, "mmp-sram", 1, NONE, SRAM_VIDEO_BASE, SRAM_VIDEO_SIZE);
PXA988_DEVICE(fb, "pxa168-fb", 0, LCD, 0xd420b000, 0x1ec);
PXA988_DEVICE(fb_ovly, "pxa168fb_ovly", 0, LCD, 0xd420b000, 0x1ec);

/* TODO Fake implementation for bring up */
void handle_coherency_maint_req(void *p) {};

