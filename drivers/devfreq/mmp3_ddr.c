/*
 * linux/drivers/devfreq/mmp3_ddr.c -- DDR devfreq.
 *
 *  Copyright (C) 2012 Marvell International Ltd.
 *  All rights reserved.
 *
 *  2012-03-16  Yifan Zhang <zhangyf@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/devfreq.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <mach/mmp3_pm.h>
#include <mach/regs-ciu.h>
#include <linux/pm_qos_params.h>

#define DDR_FREQ_MAX 8
#define DDR_DEVFREQ_UPTHRESHOLD 70
#define DDR_DEVFREQ_DOWNDIFFERENTIAL 5

static struct devfreq *pdev_ddr;
static atomic_t dfc_trigger = ATOMIC_INIT(0);
extern atomic_t mmp3_fb_is_suspended;

static unsigned int ddr_freq_table[DDR_FREQ_MAX];
static unsigned int ddr_vir_base1;
static unsigned int ddr_vir_base0;
static unsigned int ddr_freq_len;
static unsigned int target_freq;
static unsigned int cur_freq;
static unsigned long last_freq;
int mmp3_ddr_devfreq_disable = 0;
static struct pm_qos_request_list min_ddr_req;

static DEFINE_MUTEX(mmp3_ddr_lock);
DECLARE_COMPLETION(vsync_complete);

static const struct devfreq_governor *default_gov = &devfreq_simple_ondemand;

static int interleave_is_on(void)
{
	return readl(CIU_DDR_ILV_CTRL) & 0x7f;
}

/*
 * calculate the ddr workload.
 * if interleave is on, get the average value of mc1 and mc0.
 * else get the primary mc1 only.
 */
static void get_ddr_cycles(unsigned long *total, unsigned long *busy)
{
	u32 total_cycle0, idle_cycle0;
	u32 total_cycle1, idle_cycle1;


	total_cycle1 = __raw_readl(ddr_vir_base1 + 0x450);
	idle_cycle1 = __raw_readl(ddr_vir_base1 + 0x458);


	if (interleave_is_on()) {

		pr_debug("interleave is on\n");

		total_cycle0 = __raw_readl(ddr_vir_base0 + 0x450);
		idle_cycle0 = __raw_readl(ddr_vir_base0 + 0x458);

		if (idle_cycle1 < idle_cycle0) {
			*total = total_cycle1;
			*busy = total_cycle1 - idle_cycle1;
		} else {
			*total = total_cycle0;
			*busy = total_cycle0 - idle_cycle0;
		}

		pr_debug("t1 %u, t0 %u, i1 %u, i0 %u\n", total_cycle1,
				total_cycle0, idle_cycle1, idle_cycle0);
	} else {
		pr_debug("interleave is off\n");
		*total = total_cycle1;
		*busy = total_cycle1 - idle_cycle1;

		pr_debug("t1 %u, i1 %u\n", total_cycle1, idle_cycle1);
	}
}

/* reset both mc1 and mc0 */
static void reset_ddr_counters(void)
{
	if (interleave_is_on()) {
		__raw_writel(0x0, ddr_vir_base0 + 0x440);
		__raw_writel(0x10, ddr_vir_base0 + 0x448);
		__raw_writel(0x819780, ddr_vir_base0 + 0x440);
		__raw_writel(0xffffffff, ddr_vir_base0 + 0x450);
		__raw_writel(0xffffffff, ddr_vir_base0 + 0x454);
		__raw_writel(0xffffffff, ddr_vir_base0 + 0x458);
	}

	__raw_writel(0x0, ddr_vir_base1 + 0x440);
	__raw_writel(0x10, ddr_vir_base1 + 0x448);
	__raw_writel(0x819780, ddr_vir_base1 + 0x440);
	__raw_writel(0xffffffff, ddr_vir_base1 + 0x450);
	__raw_writel(0xffffffff, ddr_vir_base1 + 0x454);
	__raw_writel(0xffffffff, ddr_vir_base1 + 0x458);

}

static int ddr_get_dev_status(struct device *dev,
			       struct devfreq_dev_status *stat)
{
	u32 workload;

	stat->current_frequency = mmp3_getfreq(MMP3_CLK_DDR_1);

	last_freq = stat->current_frequency;

	get_ddr_cycles(&stat->total_time, &stat->busy_time);

	workload = stat->busy_time * 100 / stat->total_time;

	pr_debug("workload is %d\n", workload);
	pr_debug("busy time is 0x%x, %u\n", (unsigned int)stat->busy_time,
		 (unsigned int)stat->busy_time);
	pr_debug("total time is 0x%x, %u\n\n", (unsigned int)stat->total_time,
		 (unsigned int)stat->total_time);

	reset_ddr_counters();

	return 0;
}

int wakeup_ddr_fc_seq(void)
{
	if (atomic_read(&dfc_trigger)) {
		if ((cur_freq != target_freq) && (mmp3_ddr_devfreq_disable == 0)) {
			mmp3_setfreq(MMP3_CLK_DDR_1, target_freq);
			if (interleave_is_on())
				mmp3_setfreq(MMP3_CLK_DDR_2, target_freq);
			cur_freq = mmp3_getfreq(MMP3_CLK_DDR_1);
		}
		complete(&vsync_complete);
	}

	return 0;
}

static int freq_notify_and_change(unsigned int idx)
{
	mutex_lock(&mmp3_ddr_lock);
	target_freq = ddr_freq_table[idx];
	atomic_set(&dfc_trigger, 1);
	if (atomic_read(&mmp3_fb_is_suspended))
		wakeup_ddr_fc_seq();
	wait_for_completion_timeout(&vsync_complete, msecs_to_jiffies(50));
	atomic_set(&dfc_trigger, 0);

	mutex_unlock(&mmp3_ddr_lock);

	return 0;
}

static int ddr_target(struct device *dev, unsigned long *freq, u32 flags)
{
	int i;
	unsigned int min_freq;

	pr_debug("%s: %u\n", __func__, (unsigned int)*freq);

	/*
	 * if freq is unsigned int max, change ddr to the max freq.
	 * because pm_qos_update_request take unsigned int parameter, so
	 * there would be truncation.
	 */
	if (*freq == UINT_MAX)
		*freq = ddr_freq_table[ddr_freq_len - 1];

	/* pm_qos_update_request(&min_ddr_req, *freq); */
	min_freq = min(*freq,
			(unsigned long)ddr_freq_table[ddr_freq_len - 1]);

	for (i = 0; i < ddr_freq_len; i++)
		if (ddr_freq_table[i] >= min_freq) break;


	freq_notify_and_change(i);


	*freq = mmp3_getfreq(MMP3_CLK_DDR_1);
	return 0;
}

static ssize_t ddr_freq_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int freq;

	sscanf(buf, "%d", &freq);
	mutex_lock(&mmp3_ddr_lock);
	if (freq >= pm_qos_request(PM_QOS_DDR_DEVFREQ_MIN) &&
	    (mmp3_ddr_devfreq_disable == 0)) {
		last_freq = freq;
		target_freq = freq;
		atomic_set(&dfc_trigger, 1);
		if (atomic_read(&mmp3_fb_is_suspended))
			wakeup_ddr_fc_seq();
		wait_for_completion_timeout(&vsync_complete,
					    msecs_to_jiffies(50));
		atomic_set(&dfc_trigger, 0);
	}
	mutex_unlock(&mmp3_ddr_lock);

	pr_info("ddr freq read back: %d\n",
			(unsigned int)mmp3_getfreq(MMP3_CLK_DDR_1));

	return size;
}

static ssize_t ddr_freq_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int i, loop_num, tmp;

	loop_num = mmp3_get_pp_number();

	for (i = 0; i < loop_num; i++) {
		tmp = mmp3_get_pp_freq(i, MMP3_CLK_DDR_1);
		pr_debug("%d is %d", i, tmp);
	}

	return sprintf(buf, "current ddr freq is: %d\n",
		 (unsigned int)mmp3_getfreq(MMP3_CLK_DDR_1));
}

/* one stage profiling */
static ssize_t dp_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	u32 total_cycle, idle_cycle;
	u32 total_cycle1, idle_cycle1;
	u32 total_cycle0, idle_cycle0;

	reset_ddr_counters();

	msleep(20);


	total_cycle1 = __raw_readl(ddr_vir_base1 + 0x450);
	idle_cycle1 = __raw_readl(ddr_vir_base1 + 0x458);

	if (interleave_is_on()) {
		total_cycle0 = __raw_readl(ddr_vir_base0 + 0x450);
		idle_cycle0 = __raw_readl(ddr_vir_base0 + 0x458);

		if (idle_cycle1 < idle_cycle0) {
			total_cycle = total_cycle1;
			idle_cycle = idle_cycle1;
		} else {
			total_cycle = total_cycle0;
			idle_cycle = idle_cycle0;
		}
	} else {
		total_cycle = total_cycle1;
		idle_cycle = idle_cycle1;
	}

	return sprintf(buf, "The total cycle is %u,"
		       "idle cycle is %u, workload is %d percent\n",
		       total_cycle, idle_cycle,
		       (total_cycle - idle_cycle) * 100 / total_cycle);
}

/* two stages profiling */
static ssize_t dp_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	char tmp[5];
	u32 total_cycle, idle_cycle;
	u32 total_cycle1, idle_cycle1;
	u32 total_cycle0, idle_cycle0;

	sscanf(buf, "%s", tmp);

	if (strcmp(tmp, "start") == 0) {

		reset_ddr_counters();

		pr_info("ddr profiler counting, echo \"stop\" to stop\n");

		goto dp_exit;
	} else if (strcmp(tmp, "stop") == 0) {
		total_cycle1 = __raw_readl(ddr_vir_base1 + 0x450);
		idle_cycle1 = __raw_readl(ddr_vir_base1 + 0x458);

		if (interleave_is_on()) {
			total_cycle0 = __raw_readl(ddr_vir_base0 + 0x450);
			idle_cycle0 = __raw_readl(ddr_vir_base0 + 0x458);

			if (idle_cycle1 < idle_cycle0) {
				total_cycle = total_cycle1;
				idle_cycle = idle_cycle1;
			} else {
				total_cycle = total_cycle0;
				idle_cycle = idle_cycle0;
			}
		} else {
			total_cycle = total_cycle1;
			idle_cycle = idle_cycle1;
		}
		pr_info("The total cycle is %u,"
				"idle cycle is %u, workload is %d percent\n",
				total_cycle, idle_cycle,
				(total_cycle - idle_cycle) * 100 / total_cycle);

		goto dp_exit;
	} else {
		pr_err("Only start and stop can be accepted\n");

		goto dp_exit;
	}

dp_exit:
	return size;
}

static ssize_t dmc_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int freq;

	sscanf(buf, "%d", &freq);

	pm_qos_update_request(&min_ddr_req, freq);

	return size;
}

static ssize_t dmc_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%u\n", (unsigned int)pdev_ddr->min_freq);
}


static DEVICE_ATTR(ddr_freq, S_IRUGO | S_IWUSR, ddr_freq_show, ddr_freq_store);
static DEVICE_ATTR(ddr_profiling, S_IRUGO | S_IWUSR, dp_show, dp_store);
static DEVICE_ATTR(ddr_min_constraint, S_IRUGO | S_IWUSR, dmc_show, dmc_store);

static struct devfreq_dev_profile ddr_devfreq_profile = {
	.initial_freq = 800000,
	/* FIXME turn off profiling until ddr devfreq tests are completed */
	.polling_ms = 50,
	.target = ddr_target,
	.get_dev_status = ddr_get_dev_status,
};

static void insert_item_into_array(u32 table[], u32 *len, u32 item)
{
	u32 i, j;

	pr_debug("+++++len is %d, items is %d\n", *len, item);

	if (*len == 0) {
		table[0] = item;
		(*len)++;

		pr_debug("len is %d, table[0] is %d\n", *len, table[0]);

		return;
	}

	for (i = 0; i < *len; i++) {
		if (item < table[i]) {
			for (j = *len; j > i; j--)
				table[j] = table[j - 1];
			table[i] = item;
			(*len)++;
			pr_debug("insert!, len is %d, i is %d\n", *len, i);
			return;
		} else if (item == table[i]) {
			pr_debug("equal\n");
			return;
		}
	}

	table[i] = item;
	(*len)++;

	pr_debug("in the last !, i is %d, len is %d", i, *len);

	return;
}

/*
 * This notifier is called when min constraint changes, assign the biggest
 * constraint to min_freq, change the freq to last_freq if possible.
 * min_freq = max(min_freq, last_freq) is to avoid this situation:
 * the user:
 * echo 0 > polling_interval
 * echo 800000 > tp.
 * the user is not aware of the kernel constraint so he thinks he has
 * fixed the freq to 800M, but when a kernel constraint release, the freq
 * will drop to other freq without user knowing it. so add last_freq to mark
 * the last freq manually set (profiling will clear it too). when constraint
 * notify, change to the last manually set or profiler value
 */

static int ddr_devfreq_constraint_notify(struct notifier_block *nb,
				  unsigned long min_freq, void *p)
{
	int i;
	unsigned long max;

	mutex_lock(&pdev_ddr->lock);
	max = pdev_ddr->max_freq;
	if (min_freq && max && min_freq > max)
		goto unlock;
	min_freq = min((unsigned long)pm_qos_request(PM_QOS_DDR_DEVFREQ_MIN),
			(unsigned long)ddr_freq_table[ddr_freq_len - 1]);

	pdev_ddr->min_freq = min_freq;

	min_freq = max(min_freq, last_freq);

	for (i = 0; i < ddr_freq_len; i++) {
		if (ddr_freq_table[i] >= min_freq) {
			freq_notify_and_change(i);
			break;
		}
	}

	pr_debug("%s: min_freq is %u, i is %d\n",
			__func__, (unsigned int)min_freq, i);
unlock:
	mutex_unlock(&pdev_ddr->lock);


	return NOTIFY_OK;
}

static struct notifier_block ddr_devfreq_constraint_notifier = {
	.notifier_call = ddr_devfreq_constraint_notify,
};

static struct devfreq_simple_ondemand_data ddr_ondemand_data = {
	.upthreshold = DDR_DEVFREQ_UPTHRESHOLD,
	.downdifferential = DDR_DEVFREQ_DOWNDIFFERENTIAL,
};


static int probe(struct platform_device *pdev)
{
	int i, ret;
	u32 tmp, loop_num;
	struct resource *res;

	pdev_ddr = devfreq_add_device(&pdev->dev,
				       &ddr_devfreq_profile,
				       default_gov, &ddr_ondemand_data);

	if (IS_ERR(pdev_ddr)) {
		pr_err("devfreq add error !\n");

		ret =  (unsigned long)pdev_ddr;
		goto exit_ret;
	}

	ddr_devfreq_profile.initial_freq = mmp3_getfreq(MMP3_CLK_DDR_1);

	i = device_create_file(&pdev->dev, &dev_attr_ddr_freq);
	if (i < 0) {
		pr_err("device attr tp create fail: %d\n", i);
		ret = -1;
		goto exit_ret;
	}

	i = device_create_file(&pdev->dev, &dev_attr_ddr_profiling);
	if (i < 0) {
		pr_err("device attr ddr_profiling create fail: %d\n", i);
		ret = -1;
		goto exit_ret;
	}

	i = device_create_file(&pdev->dev, &dev_attr_ddr_min_constraint);
	if (i < 0) {
		pr_err("device attr ddr_profiling create fail: %d\n", i);
		ret = -1;
		goto exit_ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no IO memory defined\n");
		return -ENOENT;
	}

	ddr_vir_base1 = (u32) ioremap_nocache(res->start,
					res->end - res->start);
	if (ddr_vir_base1 == 0) {
		ret = -ENOMEM;
		goto exit_ret;
	}

	ddr_vir_base0 = (u32) ioremap_nocache(res->start + 0x10000, 0x1000);
	if (ddr_vir_base0 == 0) {
		ret = -ENOMEM;
		goto failed_ioremap;
	}

	loop_num = mmp3_get_pp_number();

	for (i = 0; i < loop_num; i++) {
		tmp = mmp3_get_pp_freq(i, MMP3_CLK_DDR_1);
		insert_item_into_array(ddr_freq_table, &ddr_freq_len, tmp);
	}

/* DDR devfreq constraint */
	pm_qos_add_request(&min_ddr_req, PM_QOS_DDR_DEVFREQ_MIN,
			PM_QOS_DEFAULT_VALUE);

	if (pm_qos_add_notifier(PM_QOS_DDR_DEVFREQ_MIN,
				&ddr_devfreq_constraint_notifier))
		pr_err("%s: Failed to register min cpus PM QoS notifier\n",
			__func__);
			

/*
	if (kobject_init_and_add(&hotplug_kobj, &hotplug_dir_ktype,
				&cpu_sysdev_class.kset.kobj, "mmp_hotplug"))
		pr_err("%s: Failed to add kobject for hotplug\n", __func__);
*/

	pr_debug("now the freq table\n");
	for (i = 0; i < ddr_freq_len; i++)
		pr_debug("-----%d\n", ddr_freq_table[i]);

	reset_ddr_counters();

	return 0;

failed_ioremap:
	iounmap((void *)ddr_vir_base1);

exit_ret:
	return ret;
}

static int remove(struct platform_device *pdev)
{
	devfreq_remove_device(pdev_ddr);

	return 0;
}

static int devfreq_reboot_notifier_call(struct notifier_block *this,
                                       unsigned long code, void *_cmd)
{
       mutex_lock(&mmp3_ddr_lock);
       pr_err("%s: disabling devfreq\n", __func__);
       mmp3_ddr_devfreq_disable = 1;
       mutex_unlock(&mmp3_ddr_lock);

       return NOTIFY_DONE;
}


static struct notifier_block devfreq_reboot_notifier = {
       .notifier_call = devfreq_reboot_notifier_call,
};


static struct platform_driver ddr_devfreq_driver = {
	.probe = probe,
	.remove = remove,
	.driver = {
		   .name = "devfreq-ddr",
		   .owner = THIS_MODULE,
		   },
};

static void __init ddr_exit(void)
{
	unregister_reboot_notifier(&devfreq_reboot_notifier);

	platform_driver_unregister(&ddr_devfreq_driver);
}

static int __init ddr_init(void)
{
	register_reboot_notifier(&devfreq_reboot_notifier);

	return platform_driver_register(&ddr_devfreq_driver);
}

fs_initcall(ddr_init);
module_exit(ddr_exit);

MODULE_DESCRIPTION("ddr device driver");
MODULE_LICENSE("GPL");
