/*
 * linux/arch/arm/mach-mmp/cpufreq-mmp3.c
 *
 * Author:	Raul Xiong <xjian@marvell.com>
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
#include <mach/mmp3_pm.h>
#include <plat/clock.h>

#define NUM_CPUS	3

/*
 * Frequency freq_table index must be sequential starting at 0
 * and frequencies must be ascending
 */
static struct cpufreq_frequency_table *freq_table;

static struct clk *cpu_clk;

static unsigned long target_cpu_speed[NUM_CPUS];
static DEFINE_MUTEX(mmp3_cpu_lock);

static int mmp3_cpufreq_verify(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);
	return 0;
}

static unsigned int mmp3_cpufreq_get(unsigned int cpu)
{
	if (cpu >= NUM_CPUS)
		return -EINVAL;

	return clk_get_rate(cpu_clk);
}

static unsigned long mmp3_cpu_highest_speed(void)
{
	unsigned long rate = 0;
	int i;

	for_each_online_cpu(i)
		rate = max(rate, target_cpu_speed[i]);
	return rate;
}

static int mmp3_cpufreq_target(struct cpufreq_policy *policy,
			       unsigned int target_freq, unsigned int relation)
{
	int index;
	struct cpufreq_freqs freqs;
	unsigned int highest_speed;
	int ret = 0;

	if (policy->cpu == NUM_CPUS - 1)
		return ret;

	mutex_lock(&mmp3_cpu_lock);

	if (cpufreq_frequency_table_target(policy, freq_table,
					   target_freq, relation, &index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		ret = -EINVAL;
		goto out;
	}

	if (policy->cur == freq_table[index].frequency)
		goto out;

	target_cpu_speed[policy->cpu] = freq_table[index].frequency;

	highest_speed = mmp3_cpu_highest_speed();

	freqs.old = mmp3_cpufreq_get(0);
	freqs.new = highest_speed;

#ifdef CONFIG_CPU_FREQ_DEBUG
	pr_info("cpu %d target_freq %d, index %d, highest_freq %d\n",
		policy->cpu, target_freq, index, highest_speed);
#endif

	if (freqs.old == freqs.new)
		goto out;

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	/*
	 * FIXME:
	 * clk_set_rate(cpu_clk, highest_speed);
	 * clk_set_rate will do spinlock with irq off, which is not
	 * valid for cpu frequency change case, one reason is the DFC
	 * routine will do mutex lock since it can be called from cpufreq
	 * and other agents like DDR and AXI thing. To do spinlock with
	 * irq off at top level do no good to complex and long clock change
	 * sequence, especially the sequency requires some system call that
	 * cannot be called from this kind of context.
	 * We will do a clock tree solution later that will solve the problem.
	 * Now we just call the raw clock change pointer to bypass the lock.
	 */
	cpu_clk->ops->setrate(cpu_clk, highest_speed);
	freqs.new = mmp3_cpufreq_get(0);

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

out:
	mutex_unlock(&mmp3_cpu_lock);
	return ret;
}

static int mmp3_cpufreq_init(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_cpuinfo(policy, freq_table);
	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);

	cpu_clk = clk_get(NULL, "cpu");
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	/* 100us, MMP3 FC takes 50 ~ 70 us*/
	policy->cpuinfo.transition_latency = 100000;
	policy->cur = mmp3_cpufreq_get(policy->cpu);
	target_cpu_speed[policy->cpu] = policy->cur;

	pr_info("CPUFREQ cpu %d support for mmp3 initialized, cur %d\n",
		policy->cpu, policy->cur);
	return 0;
}

static struct freq_attr *pxa_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver mmp3_cpufreq_driver = {
	.verify = mmp3_cpufreq_verify,
	.target = mmp3_cpufreq_target,
	.get = mmp3_cpufreq_get,
	.init = mmp3_cpufreq_init,
	.name = "mmp3-cpufreq",
	.attr = pxa_freq_attr,
};

static int __init cpufreq_init(void)
{
	/*
	 * FIXME:
	 * this need change when smp process id to real process id
	 * mapping is ready, then we can do frequency table for each core.
	 * currently the mapping depends on which core to boot.
	 * Also currently we assume only MP1/2 are active and they always runs
	 * at the same frequency. so we only use and trigger DFC target on MP1
	 */
	int freq_table_item_count, i;
	freq_table_item_count = mmp3_get_pp_number();
	freq_table =
	    kzalloc((freq_table_item_count + 1) * sizeof(freq_table[0]),
		    GFP_KERNEL);
	if (IS_ERR(freq_table))
		return PTR_ERR(freq_table);

	for (i = 0; i < freq_table_item_count; i++) {
		freq_table[i].index = i;
		freq_table[i].frequency = mmp3_get_pp_freq(i, MMP3_CLK_MP1);
	}

	freq_table[freq_table_item_count].index = freq_table_item_count;
	freq_table[freq_table_item_count].frequency = CPUFREQ_TABLE_END;

	return cpufreq_register_driver(&mmp3_cpufreq_driver);
}

module_init(cpufreq_init);

static void __exit cpufreq_exit(void)
{
	if (!IS_ERR(freq_table))
		kfree(freq_table);

	cpufreq_unregister_driver(&mmp3_cpufreq_driver);
}

module_exit(cpufreq_exit);

MODULE_DESCRIPTION("CPU frequency scaling driver for mmp3");
MODULE_LICENSE("GPL");
