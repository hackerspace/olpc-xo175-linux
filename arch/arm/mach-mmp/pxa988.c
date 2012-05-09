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

#include <plat/mfp.h>
#include <plat/pmem.h>

#include "common.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)

static struct mfp_addr_map pxa988_addr_map[] __initdata = {

	MFP_ADDR_X(GPIO0, GPIO54, 0xdc),
	MFP_ADDR_X(GPIO67, GPIO98, 0x1b8),
	MFP_ADDR_X(GPIO100, GPIO109, 0x238),
	MFP_ADDR_X(GPIO110, GPIO116, 0x298),

	MFP_ADDR_X(MMC1_DAT7, MMC1_WP, 0x84),

	MFP_ADDR(GPIO124, 0xd0),

	MFP_ADDR(CLK_REQ, 0xcc),

	MFP_ADDR_END,
};

void __init pxa988_reserve(void)
{
#ifdef CONFIG_ANDROID_PMEM
	/*reserve memory for pmem*/
	pxa_reserve_pmem_memblock();
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


static int __init pxa988_init(void)
{
	pxa988_l2_cache_init();

	mfp_init_base(MFPR_VIRT_BASE);
	mfp_init_addr(pxa988_addr_map);

	return 0;
}

postcore_initcall(pxa988_init);

/* on-chip devices */
PXA988_DEVICE(uart1, "pxa2xx-uart", 0, UART1_CP, 0xd4036000, 0x30, 4, 5);
PXA988_DEVICE(uart2, "pxa2xx-uart", 1, UART2, 0xd4017000, 0x30, 21, 22);
PXA988_DEVICE(uart3, "pxa2xx-uart", 2, UART3, 0xd4018000, 0x30, 23, 24);


/* TODO Fake implementation for bring up */
void handle_coherency_maint_req(void *p) {};

