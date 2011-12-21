/*
 * linux/arch/arm/mach-pxa/cpufreq-pxa95x.c
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
#if defined(CONFIG_PXA95x_DVFM)
#include <mach/dvfm.h>
#include <mach/pxa95x_dvfm.h>
#endif

#include "generic.h"

#define MAX_CORE_FREQS	50
#define MHZ_TO_KHZ	1000

/* Core frequencies table according to PP,
 in MHz */
static int core_freqs_table[MAX_CORE_FREQS];
static int num_pp;

static unsigned int pxa95x_freqs_num;
static struct cpufreq_frequency_table *pxa95x_freqs_table;

static unsigned int pxa95x_cpufreq_get(unsigned int cpu);

/* Interface under SYSFS */
/* Display duty cycles on all operating points */
static ssize_t all_supported_freqs_show(struct sys_device *sys_dev,
					struct sysdev_attribute *attr,
					char *buf)
{
	int len, i;

	len = sprintf(buf, "Supported CPU frequencies:\n");
	for (i = 0; i < num_pp; i++) {
		len += sprintf(&buf[len], "freq[%d] = %d MHz\n",
			       i, core_freqs_table[i]);
	}

	return len;
}

SYSDEV_ATTR(supported_freqs, 0444, all_supported_freqs_show, NULL);

/* Display duty cycles on all operating points */
static ssize_t current_freq_show(struct sys_device *sys_dev,
				 struct sysdev_attribute *attr, char *buf)
{
	int freq;

	freq = pxa95x_cpufreq_get(0);
	return sprintf(buf, "Current frequency = %d\n", freq);
}

SYSDEV_ATTR(current_freq, 0444, current_freq_show, NULL);

static struct attribute *cpufreq_stats_attr[] = {
	&attr_supported_freqs.attr,
	&attr_current_freq.attr,
};

static int stats_add(struct sys_device *sys_dev)
{
	int i, n, ret;
	n = ARRAY_SIZE(cpufreq_stats_attr);
	for (i = 0; i < n; i++) {
		ret = sysfs_create_file(&(sys_dev->kobj),
					cpufreq_stats_attr[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static int stats_rm(struct sys_device *sys_dev)
{
	int i, n;
	n = ARRAY_SIZE(cpufreq_stats_attr);
	for (i = 0; i < n; i++)
		sysfs_remove_file(&(sys_dev->kobj), cpufreq_stats_attr[i]);

	return 0;
}

static struct sysdev_driver cpufreq_stats_driver = {
	.add = stats_add,
	.remove = stats_rm,
};

static int freq_limits_get(int *freq_table, int num_pp, int *max, int *min)
{
	int max_freq;
	int min_freq;
	int i;

	max_freq = 0;
	min_freq = 0x7FFFFFFF;

	for (i = 0; i < num_pp; i++) {
		if (max_freq < freq_table[i])
			max_freq = freq_table[i];

		if (min_freq > freq_table[i])
			min_freq = freq_table[i];
	}

	*max = max_freq;
	*min = min_freq;

	return 0;
}

static int setup_freqs_table(struct cpufreq_policy *policy,
			     int *freqs_table, int num)
{
	struct cpufreq_frequency_table *table;
	int i, ret;

	table = kzalloc((num + 1) * sizeof(*table), GFP_KERNEL);
	if (table == NULL)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		table[i].index = i;
		table[i].frequency = freqs_table[i] * MHZ_TO_KHZ;
	}
	table[num].index = i;
	table[num].frequency = CPUFREQ_TABLE_END;

	pxa95x_freqs_num = num;
	pxa95x_freqs_table = table;

	ret = cpufreq_frequency_table_cpuinfo(policy, table);
	cpufreq_frequency_table_get_attr(table, policy->cpu);
	return ret;
}

static int pxa95x_cpufreq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, pxa95x_freqs_table);
}

static unsigned int pxa95x_cpufreq_get(unsigned int cpu)
{
	if (cpu != 0)
		return -EINVAL;

	return dvfm_current_core_freq_get() * MHZ_TO_KHZ;
}

static int pxa95x_cpufreq_target(struct cpufreq_policy *policy,
				 unsigned int target_freq,
				 unsigned int relation)
{
	int index;
	struct cpufreq_frequency_table *table =
		cpufreq_frequency_get_table(policy->cpu);

	if (cpufreq_frequency_table_target(policy, table, target_freq, relation,
				&index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		return -EINVAL;
	}

	if (policy->cur == table[index].frequency)
		return 0;

#ifdef CONFIG_CPU_FREQ_DEBUG
	pr_info("target_freq is %d, index is %d\n", target_freq, index);
#endif

	/* Set the required constraints as derived from the */
	/* target freq */
	dvfm_freq_set(table[index].frequency / MHZ_TO_KHZ,
			relation == CPUFREQ_RELATION_L ? RELATION_LOW : RELATION_HIGH);

	return 0;
}

static int pxa95x_cpufreq_init(struct cpufreq_policy *policy)
{
	int current_freq_khz;
	int ret = 0;
	int max_freq = 0;
	int min_freq = 0;

#ifdef CONFIG_PXA95x_DVFM
	ret = dvfm_core_freqs_table_get(core_freqs_table,
					&num_pp, MAX_CORE_FREQS);
	if (ret)
		pr_err("failed to get cpu frequency table");

	ret = freq_limits_get(core_freqs_table, num_pp, &max_freq, &min_freq);
	if (ret)
		pr_err("failed to get cpu frequency limits");

	/* set to 0 to indicate that no user configured */
	/* cpufreq until now */
	current_freq_khz = dvfm_current_core_freq_get() * MHZ_TO_KHZ;

	pr_debug("max_freq=%d, min_freq=%d", max_freq, min_freq);

	/* set default policy and cpuinfo */
	policy->cpuinfo.min_freq = min_freq * MHZ_TO_KHZ;
	policy->cpuinfo.max_freq = max_freq * MHZ_TO_KHZ;
	/* Set to 100us latency.
	 * This will cause the sampling_rate to 100ms.
	 * Need to tuning it later */
	policy->cpuinfo.transition_latency = 100 * 1000;

	policy->cur = policy->min = policy->max = current_freq_khz;

	/* setup cpuinfo frequency table */
	ret = setup_freqs_table(policy, core_freqs_table, num_pp);
	if (ret) {
		pr_err("failed to setup frequency table\n");
		return ret;
	}
#endif /* CONFIG_PXA95x_DVFM */

	pr_info("CPUFREQ support for PXA95x initialized\n");
	return 0;
}

static struct freq_attr* pxa_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver pxa95x_cpufreq_driver = {
	.verify = pxa95x_cpufreq_verify,
	.target = pxa95x_cpufreq_target,
	.init = pxa95x_cpufreq_init,
	.get = pxa95x_cpufreq_get,
	.name = "pxa95x-cpufreq",
	.attr = pxa_freq_attr,
};

static int __init cpufreq_init(void)
{
	int ret;

	ret = sysdev_driver_register(&cpu_sysdev_class, &cpufreq_stats_driver);
	if (ret)
		printk(KERN_ERR "Can't register cpufreq STATS in sysfs\n");

	return cpufreq_register_driver(&pxa95x_cpufreq_driver);
}

module_init(cpufreq_init);

static void __exit cpufreq_exit(void)
{
	sysdev_driver_unregister(&cpu_sysdev_class, &cpufreq_stats_driver);

	cpufreq_unregister_driver(&pxa95x_cpufreq_driver);
}

module_exit(cpufreq_exit);

MODULE_DESCRIPTION("CPU frequency scaling driver for PXA95x");
MODULE_LICENSE("GPL");
