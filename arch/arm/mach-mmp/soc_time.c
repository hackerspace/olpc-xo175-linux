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

#define US2CYC_SCALE_FACTOR	10
static inline cycle_t timer_read(int counter);
static unsigned long us2cyc_scale;
static DEFINE_CLOCK_DATA(cd);
static DEFINE_SPINLOCK(timer_lock);

static int irq_timer0, irq_timer1;

unsigned long long sched_clock(void)
{
	u32 cyc = timer_read(2);
	return cyc_to_sched_clock(&cd, cyc, (u32)~0);
}

static void notrace mmp_update_sched_clock(void)
{
	u32 cyc = timer_read(2);
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

static int timer_set_next_event(unsigned long delta,
				struct clock_event_device *dev)
{
	unsigned long flags;
	uint32_t cer, reg;
	unsigned int timer, i;

	spin_lock_irqsave(&timer_lock, flags);

	if (delta < MIN_DELTA)
		delta = MIN_DELTA;

	if (dev->irq == irq_timer0)
		timer = 0;
	else if (dev->irq == irq_timer1)
		timer = 1;
	else
		timer = 2;

	cer = __raw_readl(TIMERS_VIRT_BASE + TMR_CER);

	/* set preloader reg value */
	__raw_writel(0, TIMERS_VIRT_BASE + TMR_PLVR(timer));
	/* disable clk counter */
	__raw_writel(cer & ~(1<<timer), TIMERS_VIRT_BASE + TMR_CER);
	/* clear pending interrupt status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(timer));
	__raw_writel(0x1, TIMERS_VIRT_BASE + TMR_ICR(timer));
	reg = __raw_readl(TIMERS_VIRT_BASE + TMR_CCR);
	reg = reg & 0x6f;
	reg = reg & ~(0x3 << (timer << 1));
	__raw_writel(reg, TIMERS_VIRT_BASE + TMR_CCR);

	/*
	 * If camera source is 13M, then 2 cycle is 155ns. For
	 * 806M cpu, a nop operation takes 4/806 * 10 ^ 9 = 5ns.
	 * So the circulate count is 155/5 = 31. Use 100 for safe.
	 * */
	for (i = 0; i < 100; i++)
		__asm__ __volatile__("nop");
	/* set delta value */
	__raw_writel(delta - 2, TIMERS_VIRT_BASE + TMR_TN_MM(timer,0));

	reg = __raw_readl(TIMERS_VIRT_BASE + TMR_CCR);
	reg = reg & 0x6f;
	reg |= (0x1 << (timer << 1));
	__raw_writel(reg, TIMERS_VIRT_BASE + TMR_CCR);
	/* enable interrupt, timer counter */
	__raw_writel(0x1, TIMERS_VIRT_BASE + TMR_IER(timer));
	__raw_writel(cer | (1<<timer), TIMERS_VIRT_BASE + TMR_CER);

	spin_unlock_irqrestore(&timer_lock, flags);
	return 0;
}

static void timer_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *dev)
{
	unsigned long flags;
	unsigned long cer;
	int irq = dev->irq;
	int timer;

	if (irq == irq_timer0)
		timer = 0;
	else if (irq == irq_timer1)
		timer = 1;
	else
		timer = 2;

	spin_lock_irqsave(&timer_lock, flags);

	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
		cer = __raw_readl(TIMERS_VIRT_BASE + TMR_CER);
		__raw_writel(cer | (0x1 << timer), TIMERS_VIRT_BASE + TMR_CER);

		/* disable the matching interrupt */
		__raw_writel(0x00, TIMERS_VIRT_BASE + TMR_IER(timer));
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_UNUSED:
		cer = __raw_readl(TIMERS_VIRT_BASE + TMR_CER);
		__raw_writel(cer & ~(0x1 << timer), TIMERS_VIRT_BASE + TMR_CER);
		break;
	case CLOCK_EVT_MODE_RESUME:
		cer = __raw_readl(TIMERS_VIRT_BASE + TMR_CER);
		__raw_writel(cer | (0x1 << timer), TIMERS_VIRT_BASE + TMR_CER);
		break;
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	}

	spin_unlock_irqrestore(&timer_lock, flags);
	return;
}

static struct clock_event_device ckevt0 = {
	.name		= "clockevent0",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.rating		= 200,
	.set_next_event	= timer_set_next_event,
	.set_mode	= timer_set_mode,
};

static struct clock_event_device ckevt1 = {
	.name		= "clockevent1",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.rating		= 200,
	.set_next_event	= timer_set_next_event,
	.set_mode	= timer_set_mode,
};

static irqreturn_t timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;
	int timer;

	if (irq == irq_timer0)
		timer = 0;
	else if (irq == irq_timer1)
		timer = 1;
	else
		timer = 2;

	/* disable and clear pending interrupt status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(timer));
	__raw_writel(0x1, TIMERS_VIRT_BASE + TMR_ICR(timer));
	c->event_handler(c);

	return IRQ_HANDLED;
}

static struct irqaction timer0_irq = {
	.name		= "timer0",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= timer_interrupt,
};

static struct irqaction timer1_irq = {
	.name		= "timer1",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= timer_interrupt,
};

cycle_t timer_read(int counter)
{
	volatile int delay __maybe_unused = 2;
	unsigned long flags;
	volatile uint32_t val = 0;
#ifdef CONFIG_PXA_32KTIMER
	volatile uint32_t val2 = 0;
#endif

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
	return timer_read(2);
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
	ccr |= TMR_CCR_CS_0(1) | TMR_CCR_CS_1(1) | TMR_CCR_CS_2(2);
#else
	ccr |= TMR_CCR_CS_0(0) | TMR_CCR_CS_1(0) | TMR_CCR_CS_2(0);
#endif
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

	/* enable Timer 2 for clock source */
	__raw_writel(cer | 0x4, TIMERS_VIRT_BASE + TMR_CER);
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

void __init timer_init(int irq0, int irq1, int irq2)
{
	timer_config();

#ifdef CONFIG_PXA_32KTIMER
	init_sched_clock(&cd, mmp_update_sched_clock, 32, 32768);
	clocksource_calc_mult_shift(&cksrc, 32768, 4);
	clockevents_calc_mult_shift(&ckevt0, 32768, 4);
	clockevents_calc_mult_shift(&ckevt1, 32768, 4);
#else
	init_sched_clock(&cd, mmp_update_sched_clock, 32, CLOCK_TICK_RATE);
	clocksource_calc_mult_shift(&cksrc, CLOCK_TICK_RATE, 4);
	clockevents_calc_mult_shift(&ckevt0, CLOCK_TICK_RATE, 4);
	clockevents_calc_mult_shift(&ckevt1, CLOCK_TICK_RATE, 4);
#endif

	generic_timer_config();
	set_us2cyc_scale(CLOCK_TICK_RATE / 2);	/* use generic timer */

	clocksource_register(&cksrc);

	ckevt0.irq = irq_timer0 = irq0;
	ckevt1.irq = irq_timer1 = irq1;

	ckevt0.max_delta_ns = clockevent_delta2ns(MAX_DELTA, &ckevt0);
	ckevt0.min_delta_ns = clockevent_delta2ns(MIN_DELTA, &ckevt0);
	ckevt0.cpumask = cpumask_of(1);

	ckevt1.max_delta_ns = clockevent_delta2ns(MAX_DELTA, &ckevt1);
	ckevt1.min_delta_ns = clockevent_delta2ns(MIN_DELTA, &ckevt1);
	ckevt1.cpumask = cpumask_of(0);

	clockevents_register_device(&ckevt1);
	timer1_irq.dev_id = &ckevt1;
	setup_irq(irq1, &timer1_irq);
	irq_set_affinity(irq1, cpumask_of(0));
	return;
}

int local_timer_setup(struct clock_event_device *evt)
{
	unsigned int cpu = smp_processor_id();

	if (cpu == 1) {
		memcpy(evt, &ckevt0, sizeof(struct clock_event_device));

		clockevents_register_device(evt);
		timer0_irq.dev_id = evt;
		irq_set_affinity(evt->irq, cpumask_of(cpu));
		setup_irq(evt->irq, &timer0_irq);
	}

	return 0;
}
