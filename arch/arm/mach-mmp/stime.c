/*
 * linux/arch/arm/mach-mmp/stime.c
 *
 * Timer source and event devices support
 *
 * Copyright (C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/irq.h>
#include <asm/sched_clock.h>
#include <mach/regs-timers.h>

#include "common.h"

#define NUM_EVTDEV_MAX	3

#define MAX_DELTA	(0xfffffffe)
#define MIN_DELTA	(32)

#define TIMER_ROLE_NUL	0
#define TIMER_ROLE_SRC	1
#define TIMER_ROLE_EVT	2

/*
 * struct stimer_counter_device - soc timer counter device descriptor
 * @clkevt:	the local copy of the clock_event_device structure
 * @irqact:	the irq structure for this clock event device
 * @name:	ptr to clock event and irq name
 * @ctoidx:	a mapping from cpu index to clock event device index
 * @role:	the timer plays as source or event or null
 * @rate:	clock rate of the counter
 */
struct stimer_counter_device {
	struct clock_event_device	clkevt;
	struct irqaction		irqact;
	const char			*name;
	int				ctoidx;
	int				role;
	u32				rate;
};

struct stimer_counter_device st_cntdev_table[NUM_EVTDEV_MAX] = {
	{.name = "stimer0"},
	{.name = "stimer1"},
	{.name = "stimer2"}
};

static DEFINE_CLOCK_DATA(cd);

/* the virtual base address of the timer module */
static u32 st_base_virt;
/* the clock rate select for the clock source */
static u32 st_src_rate;
/* the index of the timer from which's counter we read for the source tick */
static u32 st_src_index;
/* flag to indicate whether the clock source is initialized */
static u32 st_src_initialized;

/*
 * get the index of the event device, from the name, who's 6th character
 * increace from '0', like "stimer0".
 */
static inline int stimer_get_index(struct clock_event_device *c)
{
	return c->name[6] - '0';
}

static inline u32 stimer_reg_get(int reg)
{
	return __raw_readl(st_base_virt + reg);
}

static inline void stimer_reg_set(u32 val, int reg)
{
	__raw_writel(val, st_base_virt + reg);
}

static inline void stimer_reg_set_bits(u32 set, int reg)
{
	u32 tmp = __raw_readl(st_base_virt + reg) | set;
	__raw_writel(tmp, st_base_virt + reg);
}

static inline void stimer_reg_clear_bits(u32 clear, int reg)
{
	u32 tmp = __raw_readl(st_base_virt + reg) & ~clear;
	__raw_writel(tmp, st_base_virt + reg);
}

static cycle_t stimer_read_tick_count(int n)
{
	u32 val1, val2;

	/* sched_clock would call in before the timers are initialized,
	* return 0 in this case
	*/
	if (!st_src_initialized)
		return 0;

	if (st_cntdev_table[n].rate == TIMER_RATE_32K) {
		/* for 32KHz low speed timer, silicon requires double-read
		* of the counter register to ensure the read valure is valid.
		*/
		do {
			val1 = stimer_reg_get(TMR_CR(n));
			val2 = stimer_reg_get(TMR_CR(n));
		} while (val1 != val2);
	} else {
		/* for high speed timer, it's recomended to use the write-
		* for-read register to capture the counter value. the request
		* requires up to 3 timer clock cycles.
		*/
		int delay = 3;
		stimer_reg_set(1, TMR_CVWR(n));
		while (delay--)
			val1 = stimer_reg_get(TMR_CVWR(n));
	}

	return val1;
}

static
irqreturn_t stimer_interrupt_handler(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;
	int n = stimer_get_index(c);

	/* disable and clear pending interrupt status */
	stimer_reg_set(0, TMR_IER(n));
	stimer_reg_set(1, TMR_ICR(n));

	c->event_handler(c);
	return IRQ_HANDLED;
}

/*
 * select the "32K" clock source as the low speed source.
 */
static
void stimer_switch_lspeed(int n)
{
	u32 clear, set;

	switch (n) {
	case 0:
		clear = TMR_CCR_CS_0(3);
		set = TMR_CCR_CS_0(1);
		break;
	case 1:
		clear = TMR_CCR_CS_1(3);
		set = TMR_CCR_CS_1(1);
		break;
	case 2:
		clear = TMR_CCR_CS_2(3);
		set = TMR_CCR_CS_2(2);
		break;
	}

	stimer_reg_clear_bits(clear, TMR_CCR);
	stimer_reg_set_bits(set, TMR_CCR);
}

/*
 * select the "configurable clock source" as the high speed source.
 */
static
void stimer_switch_hspeed(int n)
{
	u32 clear, set;

	switch (n) {
	case 0:
		clear = TMR_CCR_CS_0(3);
		set = TMR_CCR_CS_0(0);
		break;
	case 1:
		clear = TMR_CCR_CS_1(3);
		set = TMR_CCR_CS_1(0);
		break;
	case 2:
		clear = TMR_CCR_CS_2(3);
		set = TMR_CCR_CS_2(0);
		break;
	}

	stimer_reg_clear_bits(clear, TMR_CCR);
	stimer_reg_set_bits(set, TMR_CCR);
}

static
int stimer_set_next_event(unsigned long delta, struct clock_event_device *c)
{
	int n = stimer_get_index(c);

	/* stop the timer counter first and then do the config */
	stimer_reg_clear_bits(BIT(n), TMR_CER);

	/* the request requires up to 3 timer clock cycles, here change to
	* high speed clock source to speed it up, if neccessary */
	if (st_cntdev_table[n].rate == TIMER_RATE_32K)
		stimer_switch_hspeed(n);

	/* the preload value if 0, here just need to set the delta to match */
	stimer_reg_set(delta, TMR_TN_MM(n, 0));

	/* enable the interrupt */
	stimer_reg_set(1, TMR_IER(n));

	/* change the source back if neccessary */
	if (st_cntdev_table[n].rate == TIMER_RATE_32K)
		stimer_switch_lspeed(n);

	/* kick the counter running */
	stimer_reg_set_bits(BIT(n), TMR_CER);

	return 0;
}

static
void stimer_set_mode(enum clock_event_mode mode, struct clock_event_device *c)
{
	int n = stimer_get_index(c);

	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* disable the matching interrupt */
		stimer_reg_set(0, TMR_IER(n));
		break;

	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;

	case CLOCK_EVT_MODE_UNUSED:
		/* disable the counting */
		stimer_reg_clear_bits(BIT(n), TMR_CER);
	}
}

static void notrace stimer_update_sched_clock(void)
{
	u32 cyc = stimer_read_tick_count(st_src_index);
	update_sched_clock(&cd, cyc, (u32)~0);
}

unsigned long long sched_clock(void)
{
	u32 cyc = stimer_read_tick_count(st_src_index);
	return cyc_to_sched_clock(&cd, cyc, (u32)~0);
}

/*
 * initialize the timer module according to the src select, all the timers
 * configured by the stimer_event_config would be released counting in
 * the free running mode.
 */
static
void __init stimer_controller_init(void)
{
	int n;

	/* disable all the timer counting */
	stimer_reg_set(0, TMR_CER);

	/* set all the timer in free running mode */
	stimer_reg_set(7, TMR_CMR);

	for (n = 0; n < NUM_EVTDEV_MAX; n++) {
		if (!st_cntdev_table[n].role)
			continue;

		/* select the clock source according to the clock rate */
		if (st_cntdev_table[n].rate == TIMER_RATE_32K)
			stimer_switch_lspeed(n);
		else
			stimer_switch_hspeed(n);

		/* set the preload in free running mode */
		stimer_reg_set(0, TMR_PLCR(n));
		/* set the preload value start always from 0 */
		stimer_reg_set(0, TMR_PLVR(n));
		/* clear all the interrupt status */
		stimer_reg_set(7, TMR_ICR(n));
		/* disable all the matching interrupt */
		stimer_reg_set(0, TMR_IER(n));
		/* enable the timer counting */
		stimer_reg_set_bits(BIT(n), TMR_CER);
	}

	st_src_initialized = 1;
}

int __init stimer_event_setup(struct clock_event_device *evt)
{
	struct clock_event_device *evtd;
	struct irqaction *irqa;
	int n, cpu;

	cpu = smp_processor_id();

	n = st_cntdev_table[cpu].ctoidx;

	evtd = &(st_cntdev_table[n].clkevt);
	irqa = &(st_cntdev_table[n].irqact);

	if (evt != evtd)
		memcpy(evt, evtd, sizeof(struct clock_event_device));

	/* setup and map the irq to the proper cpu */
	irqa->dev_id = evt;
	setup_irq(evt->irq, irqa);
	irq_set_affinity(evt->irq, evt->cpumask);

	clockevents_register_device(evt);
	return 0;
}

static cycle_t stimer_src_read_timer(struct clocksource *cs)
{
	return stimer_read_tick_count(st_src_index);
}

static struct clocksource cksrc = {
	.name		= "sclock",
	.rating		= 200,
	.read		= stimer_src_read_timer,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};


/*
 * init and save the timer/cpu/irq mapping
 * @n:		the index of the timer to use
 * @cpu:	the logical cpu index to use this timer
 * @irq:	the irq number for this timer event device
 * @r:		the clock rate of the event timer
 */
void __init stimer_event_config(int n, int cpu, int irq, int r)
{
	struct clock_event_device *evtd;
	struct irqaction *irqa;

	evtd = &(st_cntdev_table[n].clkevt);
	irqa = &(st_cntdev_table[n].irqact);

	evtd->name = st_cntdev_table[n].name;
	evtd->features = CLOCK_EVT_FEAT_ONESHOT;
	evtd->rating = 200;
	evtd->set_next_event = stimer_set_next_event;
	evtd->set_mode = stimer_set_mode;
	evtd->irq = irq;
	evtd->cpumask = cpumask_of(cpu);

	clockevents_calc_mult_shift(evtd, r, 4);
	evtd->max_delta_ns = clockevent_delta2ns(MAX_DELTA, evtd);
	evtd->min_delta_ns = clockevent_delta2ns(MIN_DELTA, evtd);

	irqa->name = st_cntdev_table[n].name;
	irqa->flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL;
	irqa->handler = stimer_interrupt_handler;

	st_cntdev_table[cpu].ctoidx = n;
	st_cntdev_table[n].role = TIMER_ROLE_EVT;
	st_cntdev_table[n].rate = r;
}

/*
 * select and save the timer index and clock rate for the clock source
 * @n:	the index of the timer used for clock source
 * @r:	the clock rate of the clock source
 */
void __init stimer_source_select(int n, int r)
{
	st_src_index = n;
	st_src_rate = r;
	st_cntdev_table[n].role = TIMER_ROLE_SRC;
	st_cntdev_table[n].rate = r;
}

/*
 * register the clock source and event devices, kick the timer running.
 * @base:	the virtual base address of the timer module
 */
void __init stimer_device_init(u32 base)
{
	struct clock_event_device *evtd;
	int n = st_cntdev_table[0].ctoidx;

	st_base_virt = base;

	stimer_controller_init();

	init_sched_clock(&cd, stimer_update_sched_clock, 32, st_src_rate);

	clocksource_register_hz(&cksrc, st_src_rate);

	evtd = &(st_cntdev_table[n].clkevt);
	stimer_event_setup(evtd);
}

int local_timer_setup(struct clock_event_device *evt)
{
	/* Use existing clock_event for cpu 0 */
	if (!smp_processor_id())
		return 0;

	return stimer_event_setup(evt);
}

