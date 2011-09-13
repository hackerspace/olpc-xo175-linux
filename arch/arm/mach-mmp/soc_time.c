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
 * The timers module actually includes three timers, each timer with upto
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

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/cnt32_to_63.h>

#include <mach/addr-map.h>
#include <mach/regs-timers.h>
#include <mach/regs-apbc.h>
#include <mach/regs-mpmu.h>
#include <mach/irqs.h>
#include <mach/cputype.h>
#include <asm/sched_clock.h>
#include <asm/mach/time.h>

#include <asm/localtimer.h>

#include "clock.h"

#define GEN_TMR_REG_BASE0 (APB_VIRT_BASE + 0x80000)
#define GEN_TMR_CFG (GEN_TMR_REG_BASE0 + 0xB0)

#define TIMERS_VIRT_BASE	TIMERS1_VIRT_BASE

#define MAX_DELTA		(0xfffffffe)
#define MIN_DELTA		(32)

#define TCR2NS_SCALE_FACTOR	10
#define US2CYC_SCALE_FACTOR	10
static inline cycle_t timer_read(int counter);
static unsigned long tcr2ns_scale;
static unsigned long us2cyc_scale;
static DEFINE_CLOCK_DATA(cd);

static struct clock_event_device * local_clock_dev;

static void __init set_tcr2ns_scale(unsigned long tcr_rate)
{
	unsigned long long v = 1000000000ULL << TCR2NS_SCALE_FACTOR;
	do_div(v, tcr_rate);
	tcr2ns_scale = v;
	/*
	 * We want an even value to automatically clear the top bit
	 * returned by cnt32_to_63() without an additional run time
	 * instruction. So if the LSB is 1 then round it up.
	 */
	if (tcr2ns_scale & 1)
		tcr2ns_scale++;
}

unsigned long long sched_clock(void)
{
	u32 cyc = timer_read(1);
	return cyc_to_sched_clock(&cd, cyc, (u32)~0);
}

static void notrace mmp_update_sched_clock(void)
{
	u32 cyc = timer_read(1);
	update_sched_clock(&cd, cyc, (u32)~0);
}

static void __init set_us2cyc_scale(unsigned long cyc_rate)
{
	unsigned long long v = (unsigned long long)cyc_rate <<
				US2CYC_SCALE_FACTOR;
	do_div(v, USEC_PER_SEC);
	us2cyc_scale = v;
}

unsigned long us2cyc(unsigned long usecs)
{
	return ((unsigned long long)usecs * us2cyc_scale) >>
				US2CYC_SCALE_FACTOR;
}

static int timer0_set_next_event(unsigned long delta,
				struct clock_event_device *dev)
{
	unsigned long next;

	if (delta < MIN_DELTA)
		delta = MIN_DELTA;

	/* clear pending interrupt status and enable */
	__raw_writel(0x01, TIMERS_VIRT_BASE + TMR_ICR(0));
	__raw_writel(0x01, TIMERS_VIRT_BASE + TMR_IER(0));

	next = timer_read(0) + delta;
	__raw_writel(next, TIMERS_VIRT_BASE + TMR_TN_MM(0, 0));

	return 0;
}


static int timer1_set_next_event(unsigned long delta,
				struct clock_event_device *dev)
{
	unsigned long next;

	if (delta < MIN_DELTA)
		delta = MIN_DELTA;

	/* clear pending interrupt status and enable */
	__raw_writel(0x01, TIMERS_VIRT_BASE + TMR_ICR(1));
	__raw_writel(0x01, TIMERS_VIRT_BASE + TMR_IER(1));

	next = timer_read(1) + delta;
	__raw_writel(next, TIMERS_VIRT_BASE + TMR_TN_MM(1, 0));

	return 0;
}

static void timer0_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *dev)
{
	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* disable the matching interrupt */
		__raw_writel(0x00, TIMERS_VIRT_BASE + TMR_IER(0));
		break;
	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	case CLOCK_EVT_MODE_UNUSED:
	{
		unsigned long cer = __raw_readl(TIMERS_VIRT_BASE + TMR_CER);
		__raw_writel(cer | (1 << 0), TIMERS_VIRT_BASE + TMR_CER);
	}
	}
}
static void timer1_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *dev)
{
	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* disable the matching interrupt */
		__raw_writel(0x00, TIMERS_VIRT_BASE + TMR_IER(1));
		break;
	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	}
}

static struct clock_event_device ckevt0 = {
	.name		= "clockevent0",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.rating		= 200,
	.set_next_event	= timer0_set_next_event,
	.set_mode	= timer0_set_mode,
};

static struct clock_event_device ckevt1 = {
	.name		= "clockevent1",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.rating		= 200,
	.set_next_event	= timer1_set_next_event,
	.set_mode	= timer1_set_mode,
};

static irqreturn_t timer0_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = local_clock_dev;

	/* disable and clear pending interrupt status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(0));
	__raw_writel(0x1, TIMERS_VIRT_BASE + TMR_ICR(0));
	c->event_handler(c);
	return IRQ_HANDLED;
}

static struct irqaction timer0_irq = {
	.name		= "timer0",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= timer0_interrupt,
	.dev_id		= &ckevt0,
};

static irqreturn_t timer1_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;

	/* disable and clear pending interrupt status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(1));
	__raw_writel(0x1, TIMERS_VIRT_BASE + TMR_ICR(1));
	c->event_handler(c);
	return IRQ_HANDLED;
}

static struct irqaction timer1_irq = {
	.name		= "timer1",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= timer1_interrupt,
	.dev_id		= &ckevt1,
};

/*
 * FIXME: the timer needs some delay to stablize the counter capture
 */
static uint32_t timer_hw_inited;

cycle_t timer_read(int counter)
{
	volatile int delay __maybe_unused = 2;
	unsigned long flags;
	volatile uint32_t val = 0;
#ifdef CONFIG_PXA_32KTIMER
	volatile uint32_t val2 = 0;
#endif

	if (timer_hw_inited == 0)
		return 0;

	local_irq_save(flags);

#ifdef CONFIG_PXA_32KTIMER
	/* 32KHz timer */
	do {
		val = __raw_readl(TIMERS_VIRT_BASE + TMR_CR(counter));
		val2 = __raw_readl(TIMERS_VIRT_BASE + TMR_CR(counter));
	} while (val2 != val);
#else
	__raw_writel(1, TIMERS_VIRT_BASE + TMR_CVWR(counter));
	while (delay--) {
		val = __raw_readl(TIMERS_VIRT_BASE + TMR_CVWR(counter));
	}
	val = __raw_readl(TIMERS_VIRT_BASE + TMR_CVWR(counter));
#endif
	local_irq_restore(flags);

	return val;
}

cycle_t read_timer(struct clocksource *cs)
{
	return timer_read(1);
}

cycle_t read_fast_timer(void)
{
	volatile int delay __maybe_unused = 2;
	unsigned long flags;
	volatile uint32_t val = 0;

	if (timer_hw_inited == 0)
		return 0;

	local_irq_save(flags);

	__raw_writel(1, TIMERS_VIRT_BASE + TMR_CVWR(2));

	while (delay--) {
		val = __raw_readl(TIMERS_VIRT_BASE + TMR_CVWR(2));
	}
	val = __raw_readl(TIMERS_VIRT_BASE + TMR_CVWR(2));

	local_irq_restore(flags);

	return val;
}

static struct clocksource cksrc = {
	.name		= "soc_clock",
	.rating		= 200,
	.read		= read_timer,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init timer_config(void)
{
	unsigned long ccr = __raw_readl(TIMERS_VIRT_BASE + TMR_CCR);
	unsigned long cer = __raw_readl(TIMERS_VIRT_BASE + TMR_CER);
	unsigned long cmr = __raw_readl(TIMERS_VIRT_BASE + TMR_CMR);

	/* disable Timer 0 & 1 */
	__raw_writel(cer & ~0x7, TIMERS_VIRT_BASE + TMR_CER);

	/* clock frequency from clock/reset control register for Timer 0 */
	ccr &= ~0x7f;
#ifdef CONFIG_PXA_32KTIMER
	ccr |= TMR_CCR_CS_0(1) | TMR_CCR_CS_1(1);
#else
	ccr |= TMR_CCR_CS_0(0) | TMR_CCR_CS_1(0);
#endif
	ccr |= TMR_CCR_CS_2(0);
	__raw_writel(ccr, TIMERS_VIRT_BASE + TMR_CCR);

	/* free-running mode for Timer 0 & 1 */
	__raw_writel(cmr | 0x07, TIMERS_VIRT_BASE + TMR_CMR);

	/* Timer 0 */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_PLCR(0)); /* free-running */
	__raw_writel(0x7, TIMERS_VIRT_BASE + TMR_ICR(0));  /* clear status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(0));  /* disable int */
	/* Timer 1 */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_PLCR(1)); /* free-running */
	__raw_writel(0x7, TIMERS_VIRT_BASE + TMR_ICR(1));  /* clear status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(1));  /* disable int */

	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_PLCR(2)); /* free-running */
	__raw_writel(0x7, TIMERS_VIRT_BASE + TMR_ICR(2));  /* clear status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(2));  /* disable int */

	/* enable Timer 0 & 1 & 2*/
	__raw_writel(cer | 0x07, TIMERS_VIRT_BASE + TMR_CER);
	timer_hw_inited = 1;
}

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

void __init timer_init(int irq0, int irq1)
{
	timer_config();

#ifdef CONFIG_PXA_32KTIMER
	init_sched_clock(&cd, mmp_update_sched_clock, 32, 32768);
	set_tcr2ns_scale(32768);
	clocksource_calc_mult_shift(&cksrc, 32768, 4);
	clockevents_calc_mult_shift(&ckevt0, 32768, 4);
	clockevents_calc_mult_shift(&ckevt1, 32768, 4);
#else
	init_sched_clock(&cd, mmp_update_sched_clock, 32, CLOCK_TICK_RATE);
	set_tcr2ns_scale(CLOCK_TICK_RATE);
	clocksource_calc_mult_shift(&cksrc, CLOCK_TICK_RATE, 4);
	clockevents_calc_mult_shift(&ckevt0, CLOCK_TICK_RATE, 4);
	clockevents_calc_mult_shift(&ckevt1, CLOCK_TICK_RATE, 4);
#endif

	generic_timer_config();
	set_us2cyc_scale(CLOCK_TICK_RATE / 2);
	ckevt0.irq = irq0;
	ckevt1.irq = irq1;
	setup_irq(irq0, &timer0_irq);
	setup_irq(irq1, &timer1_irq);
	irq_set_affinity(irq0, cpumask_of(1));
	irq_set_affinity(irq1, cpumask_of(0));

	clocksource_register(&cksrc);

	ckevt0.max_delta_ns = clockevent_delta2ns(MAX_DELTA, &ckevt0);
	ckevt0.min_delta_ns = clockevent_delta2ns(MIN_DELTA, &ckevt0);
	ckevt0.cpumask = cpumask_of(1);

	ckevt1.max_delta_ns = clockevent_delta2ns(MAX_DELTA, &ckevt1);
	ckevt1.min_delta_ns = clockevent_delta2ns(MIN_DELTA, &ckevt1);
	ckevt1.cpumask = cpumask_of(0);

	clockevents_register_device(&ckevt1);
}

int local_timer_setup(struct clock_event_device *evt)
{
	unsigned long cer = __raw_readl(TIMERS_VIRT_BASE + TMR_CER);

	/* Use existing clock_event for cpu 0 */
        if (!smp_processor_id())
		return 0;

	memcpy(evt, &ckevt0, sizeof(struct clock_event_device));
	irq_set_affinity(evt->irq, cpumask_of(1));
	__raw_writel(cer | (1 << 0), TIMERS_VIRT_BASE + TMR_CER);

	local_clock_dev = evt;
	clockevents_register_device(evt);
	return 0;
}
