/*
 * linux/arch/arm/mach-mmp/cpufreq-mmp2.c
 *
 * Copyright:	(C) 2010 Marvell International Ltd.
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
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <mach/mmp2_pm.h>

#define CONFIG_CPU_FREQ_DEBUG

static struct cpufreq_frequency_table *freq_table;
static int cur_op_idx = MMP2_DEFAULT_OP_IDX;
static DEFINE_MUTEX(mmp2_cpufreq_lock);

static unsigned int mmp2_cpufreq_get(unsigned int cpu)
{
	cur_op_idx = check_cur_op();
	return mmp2_get_op_freq(cur_op_idx);
}

static int mmp2_cpufreq_verify(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);
	return 0;
}

static int mmp2_cpufreq_target(struct cpufreq_policy *policy,
			       unsigned int target_freq, unsigned int relation)
{
	int new_idx, old_idx;
	struct cpufreq_freqs freqs;
	int ret = 0;

	mutex_lock(&mmp2_cpufreq_lock);
	if (cpufreq_frequency_table_target(policy, freq_table,
					   target_freq, relation, &new_idx)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		ret = -EINVAL;
		goto out;
	}

	if (policy->cur == freq_table[new_idx].frequency)
		goto out;

	freqs.cpu = 0;
	freqs.old = mmp2_cpufreq_get(0);
	freqs.new = target_freq;

#ifdef CONFIG_CPU_FREQ_DEBUG
	pr_info("cpufreq: target_freq %d, index %d -> %d\n",
		target_freq, cur_op_idx, new_idx);
#endif

	if (freqs.old == freqs.new)
		goto out;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	old_idx = cur_op_idx;
	mmp2_fc_seq(old_idx, new_idx);
	cur_op_idx = new_idx;
	freqs.new = mmp2_cpufreq_get(0);

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

out:
	mutex_unlock(&mmp2_cpufreq_lock);
	return ret;
}


static int mmp2_cpufreq_init(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_cpuinfo(policy, freq_table);
	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);

	/* 100us */
	policy->cpuinfo.transition_latency = 100000;
	policy->cur = mmp2_cpufreq_get(policy->cpu);

	pr_info("CPUFREQ support for mmp2 initialized, cur %d\n",
		policy->cur);
	return 0;
}

static struct freq_attr *pxa_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver mmp2_cpufreq_driver = {
	.verify =	mmp2_cpufreq_verify,
	.target =	mmp2_cpufreq_target,
	.get =		mmp2_cpufreq_get,
	.init =		mmp2_cpufreq_init,
	.name =		"mmp2-cpufreq",
	.attr =		pxa_freq_attr,
};

static int __init cpufreq_init(void)
{
	int freq_table_item_count, i;

	freq_table_item_count = mmp2_get_op_number();
	freq_table =
	    kzalloc((freq_table_item_count + 1) * sizeof(freq_table[0]),
		    GFP_KERNEL);
	if (IS_ERR(freq_table))
		return PTR_ERR(freq_table);

	for (i = 0; i < freq_table_item_count; i++) {
		freq_table[i].index = i;
		freq_table[i].frequency = mmp2_get_op_freq(i);
	}

	freq_table[freq_table_item_count].index = freq_table_item_count;
	freq_table[freq_table_item_count].frequency = CPUFREQ_TABLE_END;

	return cpufreq_register_driver(&mmp2_cpufreq_driver);
}
late_initcall(cpufreq_init);

static void __exit cpufreq_exit(void)
{
	if (!IS_ERR(freq_table))
		kfree(freq_table);

	cpufreq_unregister_driver(&mmp2_cpufreq_driver);
}

__exitcall(cpufreq_exit);

MODULE_DESCRIPTION("CPU frequency scaling driver for mmp2");
MODULE_LICENSE("GPL");

