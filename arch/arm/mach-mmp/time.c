/*
 * linux/arch/arm/mach-mmp/time.c
 *
 *   Support for clocksource and clockevents
 *
 * Copyright (C) 2008 Marvell International Ltd.
 * All rights reserved.
 *
 *   2008-04-11: Jason Chagas <Jason.chagas@marvell.com>
 *   2008-10-08: Bin Yang <bin.yang@marvell.com>
 *
 * The timers module actually includes three timers, each timer with up to
 * three match comparators. Timer #0 is used here in free-running mode as
 * the clock source, and match comparator #1 used as clock event device.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/delay.h>

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/sched.h>

#include <asm/sched_clock.h>
#include <mach/addr-map.h>
#include <mach/regs-timers.h>
#include <mach/regs-apbc.h>
#include <mach/regs-mpmu.h>
#include <mach/irqs.h>
#include <mach/cputype.h>
#include <asm/mach/time.h>

#ifdef CONFIG_LOCAL_TIMERS
#include <asm/localtimer.h>
#endif

#include "clock.h"

#define GEN_TMR_REG_BASE0 (APB_VIRT_BASE + 0x80000)
#define GEN_TMR_CFG (GEN_TMR_REG_BASE0 + 0xB0)

#define TIMERS_VIRT_BASE	TIMERS1_VIRT_BASE

#define MAX_DELTA		(0xfffffffe)
#define MIN_DELTA		(32)

#define US2CYC_SCALE_FACTOR	10

static unsigned long us2cyc_scale;

#ifdef CONFIG_CPU_MMP3
static void __init set_us2cyc_scale(unsigned long cyc_rate)
{
	unsigned long long v = (unsigned long long)cyc_rate <<
				US2CYC_SCALE_FACTOR;
	do_div(v, USEC_PER_SEC);
	us2cyc_scale = v;
}
#endif

unsigned long us2cyc(unsigned long usecs)
{
	return ((unsigned long long)usecs * us2cyc_scale) >>
				US2CYC_SCALE_FACTOR;
}

static DEFINE_CLOCK_DATA(cd);

/*
 * FIXME: the timer needs some delay to stablize the counter capture
 */
static inline uint32_t timer_read(int counter)
{
	volatile int delay __maybe_unused = 2;
	volatile uint32_t val = 0, val2 = 0;

	if (counter) {
		/* 32KHz timer */
		do {
			val = __raw_readl(TIMERS_VIRT_BASE + TMR_CR(2));
			val2 = __raw_readl(TIMERS_VIRT_BASE + TMR_CR(2));
		} while (val2 != val);
	} else {
		__raw_writel(1, TIMERS_VIRT_BASE + TMR_CVWR(2));
		/* We should delay 3 times of ticks to get the timer update */
		udelay(DELAY_US);
		val =  __raw_readl(TIMERS_VIRT_BASE + TMR_CVWR(2));
	}

	return val;
}

unsigned long long notrace sched_clock(void)
{
#ifdef CONFIG_PXA_32KTIMER
	u32 cyc = timer_read(1);
#else
	u32 cyc = timer_read(0);
#endif
	return cyc_to_sched_clock(&cd, cyc, (u32)~0);
}

static void notrace mmp_update_sched_clock(void)
{
#ifdef CONFIG_PXA_32KTIMER
	u32 cyc = timer_read(1);
#else
	u32 cyc = timer_read(0);
#endif
	update_sched_clock(&cd, cyc, (u32)~0);
}

static irqreturn_t timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;

	/* disable and clear pending interrupt status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(0));
	__raw_writel(0x1, TIMERS_VIRT_BASE + TMR_ICR(0));
	c->event_handler(c);
	return IRQ_HANDLED;
}

static irqreturn_t timer_32k_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;

	/* disable and clear pending interrupt status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(1));
	__raw_writel(0x1, TIMERS_VIRT_BASE + TMR_ICR(1));
	c->event_handler(c);
	return IRQ_HANDLED;
}

static int timer_set_next_event(unsigned long delta,
				struct clock_event_device *dev)
{
	unsigned long flags, next;
	uint32_t cer = __raw_readl(TIMERS_VIRT_BASE + TMR_CER);
	uint32_t reg;
	unsigned int i;

	if (delta < MIN_DELTA)
		delta = MIN_DELTA;

	local_irq_save(flags);

#ifdef CONFIG_PXA_32KTIMER
	/* set preloader reg value */
	__raw_writel(0, TIMERS_VIRT_BASE + TMR_PLVR(1));
	/* disable clk counter */
	__raw_writel(cer & ~0x2, TIMERS_VIRT_BASE + TMR_CER);
	/* clear pending interrupt status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(1));
	__raw_writel(0x1, TIMERS_VIRT_BASE + TMR_ICR(1));
	/* change clk source to 13M to fasten 2 wait cycle */
	reg = __raw_readl(APBC_PXA168_TIMERS);
	__raw_writel(reg & 0xf, APBC_PXA168_TIMERS);
	reg = __raw_readl(TIMERS_VIRT_BASE + TMR_CCR);
	__raw_writel(reg & 0x63, TIMERS_VIRT_BASE + TMR_CCR);
	/*
	 * If camera source is 13M, then 2 cycle is 155ns. For
	 * 806M cpu, a nop operation takes 4/806 * 10 ^ 9 = 5ns.
	 * So the circulate count is 155/5 = 31. Use 100 for safe.
	 * */
	for (i = 0; i < 100; i++)
		__asm__ __volatile__("nop");
	/* set delta value */
	__raw_writel(delta - 2, TIMERS_VIRT_BASE + TMR_TN_MM(1,0));
	/* change clk source back to 32.768K */
	reg = __raw_readl(APBC_PXA168_TIMERS);
	__raw_writel(reg | 0x30, APBC_PXA168_TIMERS);
	reg = __raw_readl(TIMERS_VIRT_BASE + TMR_CCR);
	__raw_writel(reg | 0x4, TIMERS_VIRT_BASE + TMR_CCR);
	/* enable interrupt, timer counter */
	__raw_writel(0x1, TIMERS_VIRT_BASE + TMR_IER(1));
	__raw_writel(cer | 0x2, TIMERS_VIRT_BASE + TMR_CER);
#else
	/* set preloader reg value */
	__raw_writel(0, TIMERS_VIRT_BASE + TMR_PLVR(0));
	/* disable clk counter */
	__raw_writel(cer & ~0x1, TIMERS_VIRT_BASE + TMR_CER);
	/* clear pending interrupt status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(0));
	__raw_writel(0x1, TIMERS_VIRT_BASE + TMR_ICR(0));
	/* Use 200 to ensure 3.25M timer is also safe */
	for (i = 0; i < 200; i++)
		__asm__ __volatile__("nop");
	/* set delta value */
	__raw_writel(delta - 2, TIMERS_VIRT_BASE + TMR_TN_MM(0,0));
	/* enable interrupt, timer counter */
	__raw_writel(0x1, TIMERS_VIRT_BASE + TMR_IER(0));
	__raw_writel(cer | 0x1, TIMERS_VIRT_BASE + TMR_CER);
#endif

	local_irq_restore(flags);
	return 0;
}

static void timer_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *dev)
{
	unsigned long flags;

	local_irq_save(flags);
	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* disable the matching interrupt */
#ifdef CONFIG_PXA_32KTIMER
		__raw_writel(0x00, TIMERS_VIRT_BASE + TMR_IER(1));
#else
		__raw_writel(0x00, TIMERS_VIRT_BASE + TMR_IER(0));
#endif
		break;
	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	}
	local_irq_restore(flags);
}

static struct clock_event_device ckevt = {
	.name		= "clockevent",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.rating		= 200,
	.set_next_event	= timer_set_next_event,
	.set_mode	= timer_set_mode,
};

static cycle_t clksrc_read_fast(struct clocksource *cs)
{
	return timer_read(0);
}

static struct clocksource cksrc_fast = {
	.name		= "fast",
	.rating		= 200,
	.read		= clksrc_read_fast,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static cycle_t clksrc_read_32k(struct clocksource *cs)
{
	return timer_read(1);
}

static struct clocksource cksrc_32k = {
	.name		= "32k",
	.rating		= 150,
	.read		= clksrc_read_32k,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init timer_config(void)
{
	uint32_t ccr = __raw_readl(TIMERS_VIRT_BASE + TMR_CCR);
	uint32_t cer = __raw_readl(TIMERS_VIRT_BASE + TMR_CER);
	uint32_t cmr = __raw_readl(TIMERS_VIRT_BASE + TMR_CMR);

	/* disable Timer 0 & 1 & 2*/
	__raw_writel(cer & ~0x7, TIMERS_VIRT_BASE + TMR_CER);

	/* clock frequency from clock/reset control register for Timer 0 */
	ccr &= ~0x7f;			/* Timer 0 (2-bit), Timer 1 (3-bit), Timer 2 (2-bit)) */
	ccr |= TMR_CCR_CS_1(1);		/* Timer 1 -- 32KHz */
#ifdef CONFIG_PXA_32KTIMER
	ccr |= TMR_CCR_CS_2(2);		/* Timer 2 -- 32KHz */
#endif
	__raw_writel(ccr, TIMERS_VIRT_BASE + TMR_CCR);

	/* free-running mode */
	__raw_writel(cmr | 0x07, TIMERS_VIRT_BASE + TMR_CMR);

	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_PLCR(0)); /* free-running */
	__raw_writel(0x7, TIMERS_VIRT_BASE + TMR_ICR(0));  /* clear status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(0));
	/* Timer 1 */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_PLCR(1)); /* free-running */
	__raw_writel(0x7, TIMERS_VIRT_BASE + TMR_ICR(1));  /* clear status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(1));  /* disable int */
	/* Timer 2 */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_PLCR(2)); /* free-running */
	__raw_writel(0x7, TIMERS_VIRT_BASE + TMR_ICR(2));  /* clear status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(2));  /* disable int */

	/* enable timer counter */
	__raw_writel(cer | 0x07, TIMERS_VIRT_BASE + TMR_CER);
}

static struct irqaction timer_fast_irq = {
	.name		= "fast timer",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= timer_interrupt,
	.dev_id		= &ckevt,
};

static struct irqaction timer_32k_irq = {
	.name		= "32k timer",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= timer_32k_interrupt,
	.dev_id		= &ckevt,
};

#ifdef CONFIG_CPU_MMP3
static void generic_timer_access(void)
{
	__raw_writel(0xbaba, CP_TIMERS2_VIRT_BASE + TMR_WFAR);
	__raw_writel(0xeb10, CP_TIMERS2_VIRT_BASE + TMR_WSAR);
}

static void generic_timer_config(void)
{
	unsigned int value;

	__raw_writel(0x10, MPMU_CPRR);
	/* set the clock select as 3.25M */
	generic_timer_access();
	value = __raw_readl(GEN_TMR_CFG);
	value &= ~(0x3 << 4);
	generic_timer_access();
	__raw_writel(value, GEN_TMR_CFG);

	/* set the count enable */
	generic_timer_access();
	value = __raw_readl(GEN_TMR_CFG);
	value |= 0x3;
	generic_timer_access();
	__raw_writel(value, GEN_TMR_CFG);
}
#endif

void __init timer_init(int irq0, int irq1)
{
	timer_config();

#ifdef CONFIG_CPU_MMP3
	generic_timer_config();
	set_us2cyc_scale(CLOCK_TICK_RATE / 2);
#endif

#ifdef CONFIG_PXA_32KTIMER
	init_sched_clock(&cd, mmp_update_sched_clock, 32, 32768);
	clockevents_calc_mult_shift(&ckevt, 32768, 4);
#else
	init_sched_clock(&cd, mmp_update_sched_clock, 32, CLOCK_TICK_RATE);
	clockevents_calc_mult_shift(&ckevt, CLOCK_TICK_RATE, 4);
#endif

	clocksource_calc_mult_shift(&cksrc_fast, CLOCK_TICK_RATE, 4);
	clocksource_calc_mult_shift(&cksrc_32k, 32768, 4);

	ckevt.max_delta_ns = clockevent_delta2ns(MAX_DELTA, &ckevt);
	ckevt.min_delta_ns = clockevent_delta2ns(MIN_DELTA, &ckevt);
	ckevt.cpumask = cpumask_of(0);

	setup_irq(irq0, &timer_fast_irq);
	setup_irq(irq1, &timer_32k_irq);

#ifdef CONFIG_PXA_32KTIMER
	clocksource_register_hz(&cksrc_32k, 32768);
#else
	clocksource_register_hz(&cksrc_fast, CLOCK_TICK_RATE);
#endif

	clockevents_register_device(&ckevt);
}
