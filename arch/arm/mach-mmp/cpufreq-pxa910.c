/*
 * linux/arch/arm/mach-mmp/cpufreq-pxa910.c
 *
 * Copyright (C) 2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/sysdev.h>
#include <linux/cpu.h>
#include <linux/slab.h>
#include <linux/pm_qos_params.h>
#include <linux/suspend.h>
#include "acpuclock.h"

static struct pm_qos_request_list cpufreq_qos_req_min;
static DEFINE_MUTEX(pxa910_cpufreq_lock);
static bool is_suspended;

static int pxa910_cpufreq_verify(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
			policy->cpuinfo.max_freq);
	return 0;
}

static unsigned int pxa910_cpufreq_get(unsigned int cpu)
{
	if (cpu != 0)
		return -EINVAL;

	return pxa910_get_freq() * MHZ_TO_KHZ;
}

static int pxa910_cpufreq_target(struct cpufreq_policy *policy,
			unsigned int target_freq, unsigned int relation)
{
	int index;
	struct cpufreq_frequency_table *table =
		cpufreq_frequency_get_table(smp_processor_id());

	mutex_lock(&pxa910_cpufreq_lock);
	if (is_suspended) {
		goto out;
	}

	if (cpufreq_frequency_table_target(policy, table, target_freq, relation,
			&index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		mutex_unlock(&pxa910_cpufreq_lock);
		return -EINVAL;
	}

	if (policy->cur == table[index].frequency)
		goto out;

#ifdef CONFIG_CPU_FREQ_DEBUG
	pr_info("target_freq is %d, index is %d\n", target_freq, index);
#endif

	pm_qos_update_request(&cpufreq_qos_req_min, table[index].frequency/1000);
out:
	mutex_unlock(&pxa910_cpufreq_lock);
	return 0;
}

static int pxa910_cpufreq_init(struct cpufreq_policy *policy)
{
	int current_freq_khz;

	struct cpufreq_frequency_table *table =
		cpufreq_frequency_get_table(smp_processor_id());

	BUG_ON(cpufreq_frequency_table_cpuinfo(policy, table));

	current_freq_khz = pxa910_get_freq() * MHZ_TO_KHZ;

	/* set default policy and cpuinfo */
	policy->cpuinfo.transition_latency = 1000; /* 1mSec latency */
	policy->cur = current_freq_khz;

	pm_qos_add_request(&cpufreq_qos_req_min, PM_QOS_CPUFREQ_MIN,
			current_freq_khz/MHZ_TO_KHZ);
	pr_info("CPUFREQ support for PXA910 initialized, cur %d\n",
			current_freq_khz);
	return 0;
}

static struct freq_attr* pxa_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver pxa910_cpufreq_driver = {
	.verify		= pxa910_cpufreq_verify,
	.target		= pxa910_cpufreq_target,
	.get		= pxa910_cpufreq_get,
	.init		= pxa910_cpufreq_init,
	.name		= "pxa910-cpufreq",
	.attr		= pxa_freq_attr,
};

static int pxa910_cpufreq_pm_notifier(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
	mutex_lock(&pxa910_cpufreq_lock);
	if (event == PM_SUSPEND_PREPARE) {
		is_suspended = true;
		pr_info("%s: disable cpu freq-chg before suspend\n",
				__func__);
	} else if (event == PM_POST_SUSPEND) {
		is_suspended = false;
		pr_info("%s: enable cpu freq-chg after resume\n",
				__func__);
	}
	mutex_unlock(&pxa910_cpufreq_lock);

	return NOTIFY_OK;
}

static struct notifier_block pxa910_cpu_pm_notifier = {
	.notifier_call = pxa910_cpufreq_pm_notifier,
};

static int __init cpufreq_init(void)
{
#ifdef CONFIG_SUSPEND
	register_pm_notifier(&pxa910_cpu_pm_notifier);
#endif
	return cpufreq_register_driver(&pxa910_cpufreq_driver);
}
module_init(cpufreq_init);

static void __exit cpufreq_exit(void)
{
	cpufreq_unregister_driver(&pxa910_cpufreq_driver);
}
module_exit(cpufreq_exit);

MODULE_DESCRIPTION("CPU frequency scaling driver for PXA910");
MODULE_LICENSE("GPL");
