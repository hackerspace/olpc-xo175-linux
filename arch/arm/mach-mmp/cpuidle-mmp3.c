/*
 * linux/arch/arm/mach-mmp/cpuidle-mmp3.c
 *
 * Author:	Raul Xiong <xjian@marvell.com>
 * Copyright:	(C) 2011 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpuidle.h>
#include <mach/mmp3_pm.h>

static struct cpuidle_driver mmp3_idle_driver = {
	.name = "mmp3_idle",
	.owner = THIS_MODULE,
};

static DEFINE_PER_CPU(struct cpuidle_device, mmp3_cpuidle_device);

static int mmp3_enter_idle_c1(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{
	struct timespec ts_preidle, ts_postidle;
	int idletime_us;

	local_irq_disable();
	getnstimeofday(&ts_preidle);

	mmp3_pm_enter_idle(dev->cpu);

	getnstimeofday(&ts_postidle);
	idletime_us = (ts_postidle.tv_sec - ts_preidle.tv_sec) *
		USEC_PER_SEC + (ts_postidle.tv_nsec - ts_preidle.tv_nsec) /
		NSEC_PER_USEC;
	local_irq_enable();

	return idletime_us;
}

static int mmp3_cpuidle_register_device(unsigned int cpu)
{
	struct cpuidle_device *device;
	struct cpuidle_state *state;

	device = &per_cpu(mmp3_cpuidle_device, cpu);
	device->state_count = 0;
	device->cpu = cpu;

	state = &device->states[0];
	strcpy(state->name, "C1");
	strcpy(state->desc, "C1: core internal clock gate");
	state->exit_latency = 10; /* FIXME: what's the real latency? */
	state->target_residency = state->exit_latency * 2;
	state->flags = CPUIDLE_FLAG_TIME_VALID;
	state->enter = mmp3_enter_idle_c1;
	device->state_count++;

	state = &device->states[1];
	strcpy(state->name, "C2");
	strcpy(state->desc, "C2: core state is retained");
	state->exit_latency = 50; /* FIXME: what's the real latency? */
	state->target_residency = state->exit_latency * 2;
	state->flags = CPUIDLE_FLAG_TIME_VALID;
	/* FIXME: C2 doesn't work now, use c1 instead temporarily */
	state->enter = mmp3_enter_idle_c1;
	device->state_count++;

	if (cpuidle_register_device(device)) {
		pr_err("CPU%u: failed to register cpuidle device\n", cpu);
		return -EIO;
	}

	return 0;
}

static int __init mmp3_cpuidle_init(void)
{
	int ret, cpu;

	ret = cpuidle_register_driver(&mmp3_idle_driver);
	if (ret)
		return ret;

	for_each_possible_cpu(cpu) {
		if (mmp3_cpuidle_register_device(cpu))
			pr_err("CPU%u: error registering cpuidle\n", cpu);
	}

	return 0;
}

module_init(mmp3_cpuidle_init);
