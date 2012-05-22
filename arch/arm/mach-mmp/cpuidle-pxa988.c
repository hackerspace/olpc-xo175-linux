/*
 * linux/arch/arm/mach-mmp/cpuidle-pxa988.c
 *
 * Author:	Raul Xiong <xjian@marvell.com>
 * Copyright:	(C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpu_pm.h>
#include <linux/cpuidle.h>
#include <linux/pm_qos_params.h>

#include <asm/proc-fns.h>
#include <mach/pxa988_lowpower.h>

static enum pxa988_lowpower_mode pxa988_idle_mode[] = {
	PXA988_LPM_C1,
	PXA988_LPM_C2,
	PXA988_LPM_D1P,
	PXA988_LPM_D1,
	PXA988_LPM_D2,
};

static struct cpuidle_driver pxa988_idle_driver = {
	.name = "pxa988_idle",
	.owner = THIS_MODULE,
};

static DEFINE_PER_CPU(struct cpuidle_device, pxa988_cpuidle_device);

static DEFINE_MUTEX(cpuidle_mutex);

static int pxa988_enter_lpm(struct cpuidle_device *dev,
	struct cpuidle_state *state)
{
	enum pxa988_lowpower_mode *power_mode = cpuidle_get_statedata(state);
	struct timespec ts_preidle, ts_postidle;
	int idletime_us;

	local_irq_disable();
	local_fiq_disable();
	getnstimeofday(&ts_preidle);

	pxa988_enter_lowpower(dev->cpu, *power_mode);

	getnstimeofday(&ts_postidle);
	idletime_us = (ts_postidle.tv_sec - ts_preidle.tv_sec) *
		USEC_PER_SEC + (ts_postidle.tv_nsec - ts_preidle.tv_nsec) /
		NSEC_PER_USEC;
	local_fiq_enable();
	local_irq_enable();

	return idletime_us;
}

static int pxa988_cpuidle_register_device(unsigned int cpu)
{
	struct cpuidle_device *device;
	struct cpuidle_state *state;

	device = &per_cpu(pxa988_cpuidle_device, cpu);
	device->state_count = 0;
	device->cpu = cpu;

	state = &device->states[0];
	strcpy(state->name, "C1");
	strcpy(state->desc, "C1: Core external clock gated");
	state->exit_latency = 5; /* FIXME: what's the real latency? */
	state->target_residency = state->exit_latency * 2;
	state->flags = CPUIDLE_FLAG_TIME_VALID;
	state->enter = pxa988_enter_lpm;
	cpuidle_set_statedata(&device->states[0],
			&pxa988_idle_mode[0]);
	device->state_count++;

	state = &device->states[1];
	strcpy(state->name, "C2");
	strcpy(state->desc, "C2: Core power down");
	state->exit_latency = 50; /* FIXME: what's the real latency? */
	state->target_residency = state->exit_latency * 2;
	state->flags = CPUIDLE_FLAG_TIME_VALID;
	state->enter = pxa988_enter_lpm;
	cpuidle_set_statedata(&device->states[1],
			&pxa988_idle_mode[1]);
	device->state_count++;

	state = &device->states[2];
	strcpy(state->name, "D1P");
	strcpy(state->desc, "D1P: AP subsystem idle");
	state->exit_latency = 60; /* FIXME: what's the real latency? */
	state->target_residency = state->exit_latency * 2;
	state->flags = CPUIDLE_FLAG_TIME_VALID;
	state->enter = pxa988_enter_lpm;
	cpuidle_set_statedata(&device->states[2],
			&pxa988_idle_mode[2]);
	device->state_count++;

	state = &device->states[3];
	strcpy(state->name, "D1");
	strcpy(state->desc, "D1: AP sybsystem sleep");
	state->exit_latency = 70; /* FIXME: what's the real latency? */
	state->target_residency = state->exit_latency * 2;
	state->flags = CPUIDLE_FLAG_TIME_VALID;
	state->enter = pxa988_enter_lpm;
	cpuidle_set_statedata(&device->states[3],
			&pxa988_idle_mode[3]);
	device->state_count++;

	state = &device->states[4];
	strcpy(state->name, "D2");
	strcpy(state->desc, "D2: Full chip sleep");
	state->exit_latency = 100; /* FIXME: what's the real latency? */
	state->target_residency = state->exit_latency * 2;
	state->flags = CPUIDLE_FLAG_TIME_VALID;
	state->enter = pxa988_enter_lpm;
	cpuidle_set_statedata(&device->states[4],
			&pxa988_idle_mode[4]);
	device->state_count++;

	if (cpuidle_register_device(device)) {
		pr_err("CPU%u: failed to register cpuidle device\n", cpu);
		return -EIO;
	}

	return 0;
}

static int cpuidle_keep_axi_notify(struct notifier_block *b,
			      unsigned long min, void *v)
{
	int keep_ddr;
	int keep_vctcxo;

	mutex_lock(&cpuidle_mutex);

	keep_ddr = pm_qos_request(PM_QOS_CPUIDLE_KEEP_DDR);
	keep_vctcxo = pm_qos_request(PM_QOS_CPUIDLE_KEEP_VCTCXO);

	if (min) {
		pxa988_lpm_data[PXA988_LPM_D1P].valid = 0;
		pxa988_lpm_data[PXA988_LPM_D1].valid = 0;
		pxa988_lpm_data[PXA988_LPM_D2].valid = 0;
	} else {
		pxa988_lpm_data[PXA988_LPM_D1P].valid = 1;
		if (keep_ddr == 0)
			pxa988_lpm_data[PXA988_LPM_D1].valid = 1;
		if (keep_ddr == 0 && keep_vctcxo == 0)
			pxa988_lpm_data[PXA988_LPM_D2].valid = 1;
	}

	mutex_unlock(&cpuidle_mutex);

	return NOTIFY_OK;
}

static struct notifier_block cpuidle_keep_axi_notifier = {
	.notifier_call = cpuidle_keep_axi_notify,
};

static int cpuidle_keep_ddr_notify(struct notifier_block *b,
			      unsigned long min, void *v)
{
	int keep_axi;
	int keep_vctcxo;

	mutex_lock(&cpuidle_mutex);

	keep_axi = pm_qos_request(PM_QOS_CPUIDLE_KEEP_AXI);
	keep_vctcxo = pm_qos_request(PM_QOS_CPUIDLE_KEEP_VCTCXO);

	if (min) {
		pxa988_lpm_data[PXA988_LPM_D1].valid = 0;
		pxa988_lpm_data[PXA988_LPM_D2].valid = 0;
	} else {
		if (keep_axi == 0)
			pxa988_lpm_data[PXA988_LPM_D1].valid = 1;
		if (keep_axi == 0 && keep_vctcxo == 0)
			pxa988_lpm_data[PXA988_LPM_D2].valid = 1;
	}

	mutex_unlock(&cpuidle_mutex);

	return NOTIFY_OK;
}

static struct notifier_block cpuidle_keep_ddr_notifier = {
	.notifier_call = cpuidle_keep_ddr_notify,
};

static int cpuidle_keep_vctcxo_notify(struct notifier_block *b,
			      unsigned long min, void *v)
{
	int keep_axi;
	int keep_ddr;

	mutex_lock(&cpuidle_mutex);

	keep_axi = pm_qos_request(PM_QOS_CPUIDLE_KEEP_AXI);
	keep_ddr = pm_qos_request(PM_QOS_CPUIDLE_KEEP_DDR);

	if (min)
		pxa988_lpm_data[PXA988_LPM_D2].valid = 0;
	else
		if (keep_axi == 0 && keep_ddr == 0)
			pxa988_lpm_data[PXA988_LPM_D2].valid = 1;

	mutex_unlock(&cpuidle_mutex);

	return NOTIFY_OK;
}

static struct notifier_block cpuidle_keep_vctcxo_notifier = {
	.notifier_call = cpuidle_keep_vctcxo_notify,
};

static int __init pxa988_cpuidle_init(void)
{
	int ret, cpu;

	ret = cpuidle_register_driver(&pxa988_idle_driver);
	if (ret)
		return ret;

	for_each_possible_cpu(cpu) {
		if (pxa988_cpuidle_register_device(cpu))
			pr_err("CPU%u: error registering cpuidle\n", cpu);
	}

	pm_qos_add_notifier(PM_QOS_CPUIDLE_KEEP_AXI,
			&cpuidle_keep_axi_notifier);
	pm_qos_add_notifier(PM_QOS_CPUIDLE_KEEP_DDR,
			&cpuidle_keep_ddr_notifier);
	pm_qos_add_notifier(PM_QOS_CPUIDLE_KEEP_VCTCXO,
			&cpuidle_keep_vctcxo_notifier);

	return 0;
}

module_init(pxa988_cpuidle_init);
