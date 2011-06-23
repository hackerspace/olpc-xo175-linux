/*
 * arch/arm/mach-pxa/time.c
 *
 * PXA clocksource, clockevents, and OST interrupt handlers.
 * Copyright (c) 2007 by Bill Gatliff <bgat@billgatliff.com>.
 *
 * Derived from Nicolas Pitre's PXA timer handler Copyright (c) 2001
 * by MontaVista Software, Inc.  (Nico, your code rocks!)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/sched.h>

#include <asm/div64.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/sched_clock.h>
#include <mach/regs-ost.h>

/*
 * This is PXA's sched_clock implementation. This has a resolution
 * of at least 308 ns and a maximum value of 208 days.
 *
 * The return value is guaranteed to be monotonic in that range as
 * long as there is always less than 582 seconds between successive
 * calls to sched_clock() which should always be the case in practice.
 */
static DEFINE_CLOCK_DATA(cd);

static inline u32 pxa_timer_read(void)
{
#ifdef CONFIG_PXA_32KTIMER
	return OSCR4;
#else
	return OSCR;
#endif
}

unsigned long long notrace sched_clock(void)
{
	u32 cyc = pxa_timer_read();
	return cyc_to_sched_clock(&cd, cyc, (u32)~0);
}

static void notrace pxa_update_sched_clock(void)
{
	u32 cyc = pxa_timer_read();
	update_sched_clock(&cd, cyc, (u32)~0);
}


#define MIN_OSCR_DELTA 16
static irqreturn_t
pxa_ost4_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;

	/* Disarm the compare/match, signal the event. */
	OIER &= ~OIER_E4;
	OSSR = OSSR_M4;
	c->event_handler(c);

	return IRQ_HANDLED;
}


static irqreturn_t
pxa_ost0_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;

	/* Disarm the compare/match, signal the event. */
	OIER &= ~OIER_E0;
	OSSR = OSSR_M0;
	c->event_handler(c);

	return IRQ_HANDLED;
}

static int
pxa_osmr_set_next_event(unsigned long delta, struct clock_event_device *dev)
{
	unsigned long next, oscr;

#ifdef CONFIG_PXA_32KTIMER
	OIER |= OIER_E4;
	next = OSCR4 + delta;
	OSMR4 = next;
	oscr = OSCR4;
#else
	OIER |= OIER_E0;
	next = OSCR + delta;
	OSMR0 = next;
	oscr = OSCR;
#endif

	return (signed)(next - oscr) <= MIN_OSCR_DELTA ? -ETIME : 0;
}

static void
pxa_osmr_set_mode(enum clock_event_mode mode, struct clock_event_device *dev)
{
	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* initializing, released, or preparing for suspend */
#ifdef CONFIG_PXA_32KTIMER
		OIER &= ~OIER_E4;
		OSSR = OSSR_M4;
#else
		OIER &= ~OIER_E0;
		OSSR = OSSR_M0;
#endif
		break;

	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	}
}

static struct clock_event_device ckevt_pxa_osmr = {
#ifdef CONFIG_PXA_32KTIMER
	.name		= "osmr4",
#else
	.name		= "osmr0",
#endif
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.rating		= 200,
	.set_next_event	= pxa_osmr_set_next_event,
	.set_mode	= pxa_osmr_set_mode,
};

static struct irqaction pxa_ost0_irq = {
	.name		= "ost0",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= pxa_ost0_interrupt,
	.dev_id		= &ckevt_pxa_osmr,
};

static struct irqaction pxa_ost4_irq = {
	.name		= "ost4",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= pxa_ost4_interrupt,
	.dev_id		= &ckevt_pxa_osmr,
};

static void __init pxa_timer_init(void)
{
	unsigned long clock_tick_rate = get_clock_tick_rate();

	OIER = 0;
	OSSR = OSSR_M0 | OSSR_M1 | OSSR_M2 | OSSR_M3 | OSSR_M4;

	OMCR4 = 0xc1;
	OSCR4 = 1;

#ifdef CONFIG_PXA_32KTIMER
	init_sched_clock(&cd, pxa_update_sched_clock, 32, 32768);
	clockevents_calc_mult_shift(&ckevt_pxa_osmr, 32768, 4);
#else
	init_sched_clock(&cd, pxa_update_sched_clock, 32, clock_tick_rate);
	clockevents_calc_mult_shift(&ckevt_pxa_osmr, clock_tick_rate, 4);
#endif
	ckevt_pxa_osmr.max_delta_ns =
		clockevent_delta2ns(0x7fffffff, &ckevt_pxa_osmr);
	ckevt_pxa_osmr.min_delta_ns =
		clockevent_delta2ns(MIN_OSCR_DELTA * 2, &ckevt_pxa_osmr) + 1;
	ckevt_pxa_osmr.cpumask = cpumask_of(0);

	setup_irq(IRQ_OST0, &pxa_ost0_irq);
	setup_irq(IRQ_OST_4_11, &pxa_ost4_irq);

	clocksource_mmio_init((void __iomem *)&OSCR, "oscr0", clock_tick_rate, 150, 32,
		clocksource_mmio_readl_up);
	clocksource_mmio_init((void __iomem *)&OSCR4, "oscr4", 32768, 200, 32,
		clocksource_mmio_readl_up);
	clockevents_register_device(&ckevt_pxa_osmr);
}

#ifdef CONFIG_PM
static unsigned long osmr[5], oier, oscr;

static void pxa_timer_suspend(void)
{
	osmr[0] = OSMR0;
	osmr[1] = OSMR1;
	osmr[2] = OSMR2;
	osmr[3] = OSMR3;
	osmr[4] = OSMR4;
	oier = OIER;
#ifdef CONFIG_PXA_32KTIMER
	oscr = OSCR4;
#else
	oscr = OSCR;
#endif
}

static void pxa_timer_resume(void)
{
	/*
	 * Ensure that we have at least MIN_OSCR_DELTA between match
	 * register 0 and the OSCR, to guarantee that we will receive
	 * the one-shot timer interrupt.  We adjust OSMR0 in preference
	 * to OSCR to guarantee that OSCR is monotonically incrementing.
	 */
#ifdef CONFIG_PXA_32KTIMER
	if (osmr[4] - oscr < MIN_OSCR_DELTA)
		osmr[4] += MIN_OSCR_DELTA;
#else
	if (osmr[0] - oscr < MIN_OSCR_DELTA)
		osmr[0] += MIN_OSCR_DELTA;
#endif
	OSMR0 = osmr[0];
	OSMR1 = osmr[1];
	OSMR2 = osmr[2];
	OSMR3 = osmr[3];
	OIER = oier;
#ifdef CONFIG_PXA_32KTIMER
	OSCR4 = oscr;
#else
	OSCR = oscr;
#endif
}
#else
#define pxa_timer_suspend NULL
#define pxa_timer_resume NULL
#endif

struct sys_timer pxa_timer = {
	.init		= pxa_timer_init,
	.suspend	= pxa_timer_suspend,
	.resume		= pxa_timer_resume,
};
