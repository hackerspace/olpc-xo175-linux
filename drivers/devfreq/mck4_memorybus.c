/*
 * linux/drivers/devfreq/mck4_memorybus.c
 *
 *  Copyright (C) 2012 Marvell International Ltd.
 *  All rights reserved.
 *
 *  2012-03-16  Yifan Zhang <zhangyf@marvell.com>
 *		Zhoujie Wu<zjwu@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/devfreq.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <plat/devfreq.h>

#define DDR_FREQ_MAX 8
#define DDR_DEVFREQ_UPTHRESHOLD 70
#define DDR_DEVFREQ_DOWNDIFFERENTIAL 5

#define KHZ_TO_HZ   1000

struct ddr_devfreq_data {
	struct devfreq *pdev_ddr;
	struct clk *ddr_clk;
	u32 mc_base_pri;/* primary addr */
	u32 mc_base_sec;/* secondray addr, only valid if interleave_is_on */
	/* used for platform have more than one DDR controller */
	u32 interleave_is_on;
	/* DDR frequency table used for platform */
	u32 ddr_freq_tbl[DDR_FREQ_MAX];	/* unit Khz */
	u32 ddr_freq_tbl_len;
	struct mutex mutex;
	spinlock_t lock;

	/* used for debug interface */
	atomic_t is_disabled;
	struct timespec last_ts;
};

/* default using ondemand governor */
static const struct devfreq_governor *default_gov =
	&devfreq_simple_ondemand;

/* default using 70% as upthreshold and 5% as downthreshold */
static struct devfreq_simple_ondemand_data ddr_ondemand_data = {
	.upthreshold = DDR_DEVFREQ_UPTHRESHOLD,
	.downdifferential = DDR_DEVFREQ_DOWNDIFFERENTIAL,
};

static inline void get_mc_cnt(u32 mc_base, u32 *total,
			u32 *idle, struct ddr_devfreq_data *data)
{
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	*total = readl(mc_base + 0x450);
	*idle = readl(mc_base + 0x458);
	spin_unlock_irqrestore(&data->lock, flags);
}

static inline void reset_mc_cnt(u32 mc_base,
			struct ddr_devfreq_data *data)
{
	unsigned long flags;
	/*
	 * cnt0 is used to collect Clock cycles, pc_clk_div = 1
	 * cnt2 is used to collect DPC idle cycles
	 */
	spin_lock_irqsave(&data->lock, flags);
	writel(0x0, mc_base + 0x440);
	writel(0x10, mc_base + 0x448);
	writel(0x819780, mc_base + 0x440);
	writel(0xffffffff, mc_base + 0x450);
	writel(0xffffffff, mc_base + 0x454);
	writel(0xffffffff, mc_base + 0x458);
	spin_unlock_irqrestore(&data->lock, flags);
}

/* calculate ddr workload according to busy and total time, unit percent */
static inline u32 cal_workload(unsigned long busy_time,
	unsigned long total_time)
{
	u64 tmp0, tmp1;

	if (!total_time || !busy_time)
		return 0;
	tmp0 = busy_time * 100;
	tmp1 = div_u64(tmp0, total_time);
	return (u32)tmp1;
}

/*
 * get the mck4 total and idle performance cnt
 * if interleave is on, get the busy one of primary and secondary mc.
 * else get the primary mc only.
 */
static void get_ddr_cycles(struct ddr_devfreq_data *data,
	unsigned long *total, unsigned long *busy)
{
	u32 total_pri, idle_pri;
	u32 total_sec, idle_sec;

	get_mc_cnt(data->mc_base_pri, &total_pri, &idle_pri, data);

	if (data->interleave_is_on) {
		dev_dbg(&data->pdev_ddr->dev, "interleave is on\n");
		get_mc_cnt(data->mc_base_sec, &total_sec, &idle_sec, data);
		if (total_sec < total_pri) {
			*total = total_sec;
			*busy = total_sec - idle_sec;
		} else {
			*total = total_pri;
			*busy = total_pri - idle_pri;
		}
		dev_dbg(&data->pdev_ddr->dev,
			"t_pri %u, t_sec %u, i_pri %u, i_sec %u\n",
			total_pri, total_sec,
			idle_pri, idle_sec);
	} else {
		dev_dbg(&data->pdev_ddr->dev, "interleave is off\n");
		*total = total_pri;
		*busy = total_pri - idle_pri;
		dev_dbg(&data->pdev_ddr->dev,
			"t_pri %u, i_pri %u\n",
			total_pri, idle_pri);
	}
}

/* reset both mc1 and mc0 */
static void reset_ddr_counters(struct ddr_devfreq_data *data)
{
	reset_mc_cnt(data->mc_base_pri, data);

	if (data->interleave_is_on)
		reset_mc_cnt(data->mc_base_sec, data);
}

static int ddr_get_dev_status(struct device *dev,
			       struct devfreq_dev_status *stat)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	u32 workload;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);

	stat->current_frequency = clk_get_rate(data->ddr_clk) / KHZ_TO_HZ;

	get_ddr_cycles(data, &stat->total_time, &stat->busy_time);
	workload = cal_workload(stat->busy_time, stat->total_time);

	dev_dbg(dev, "workload is %d precent\n", workload);
	dev_dbg(dev, "busy time is 0x%x, %u\n", (u32)stat->busy_time,
		 (u32)stat->busy_time);
	dev_dbg(dev, "total time is 0x%x, %u\n\n",
		(u32)stat->total_time,
		(u32)stat->total_time);

	reset_ddr_counters(data);
	return 0;
}

static int ddr_set_rate(struct ddr_devfreq_data *data, unsigned long tgt_rate)
{
	unsigned long cur_freq, tgt_freq;

	cur_freq = clk_get_rate(data->ddr_clk);
	tgt_freq = tgt_rate * KHZ_TO_HZ;

	if (cur_freq == tgt_freq)
		return 0;

	dev_dbg(&data->pdev_ddr->dev, "%s: curfreq %lu, tgtfreq %lu\n",
		__func__, cur_freq, tgt_freq);
	/* clk_set_rate will find a frequency larger or equal tgt_freq */
	clk_set_rate(data->ddr_clk, tgt_freq);
	return 0;
}

static int ddr_target(struct device *dev, unsigned long *freq, u32 flags)
{
	unsigned long tgt_freq;
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	struct devfreq *df;
	u32 *ddr_freq_table, ddr_freq_len;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);

	/* in normal case ddr fc will NOT be disabled */
	if (unlikely(atomic_read(&data->is_disabled))) {
		df = data->pdev_ddr;
		/*
		 * this function is called with df->locked, it is safe to
		 * read the polling_ms here
		 */
		if (df->profile->polling_ms)
			dev_err(dev, "[WARN] ddr ll fc is disabled from "\
				"debug interface, suggest to disable "\
				"the profiling at first!\n");
		*freq = clk_get_rate(data->ddr_clk) / KHZ_TO_HZ;
		return 0;
	}

	ddr_freq_table = &data->ddr_freq_tbl[0];
	ddr_freq_len = data->ddr_freq_tbl_len;
	dev_dbg(dev, "%s: %u\n", __func__, (u32)*freq);
	/*
	 * if freq is u32 max, change ddr to the max freq.
	 * because pm_qos_update_request take u32 parameter, so
	 * there would be truncation.
	 */
	if (*freq == UINT_MAX)
		*freq = ddr_freq_table[ddr_freq_len - 1];

	mutex_lock(&data->mutex);
	tgt_freq = min(*freq,
			(unsigned long)ddr_freq_table[ddr_freq_len - 1]);
	ddr_set_rate(data, tgt_freq);
	mutex_unlock(&data->mutex);

	*freq = clk_get_rate(data->ddr_clk) / KHZ_TO_HZ;
	return 0;
}

/* debug interface used to totally disable ddr fc */
static ssize_t disable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	int is_disabled;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);

	if (0x1 != sscanf(buf, "%d", &is_disabled)) {
		dev_err(dev, "<ERR> wrong parameter\n");
		return -E2BIG;
	}

	is_disabled = !!is_disabled;
	if (is_disabled == atomic_read(&data->is_disabled)) {
		dev_info(dev, "[WARNING] ddr fc is already %s\n",
			atomic_read(&data->is_disabled) ? \
			"disabled" : "enabled");
		return size;
	}

	if (is_disabled)
		atomic_inc(&data->is_disabled);
	else
		atomic_dec(&data->is_disabled);

	dev_info(dev, "[WARNING]ddr fc is %s from debug interface!\n",\
		atomic_read(&data->is_disabled) ? "disabled" : "enabled");
	return size;
}

static ssize_t disable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);
	return sprintf(buf, "ddr fc is_disabled = %d\n",
		 atomic_read(&data->is_disabled));
}

/*
 * Debug interface used to change ddr rate.
 * It will ignore all devfreq and Qos requests.
 * Use interface disable_ddr_fc prior to it.
 */
static ssize_t ddr_freq_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	int freq;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);

	if (!atomic_read(&data->is_disabled)) {
		dev_err(dev, "<ERR> It will change ddr rate,"\
			"disable ddr fc at first\n");
		return -EPERM;
	}

	if (0x1 != sscanf(buf, "%d", &freq)) {
		dev_err(dev, "<ERR> wrong parameter, "\
			"echo freq > ddr_freq to set ddr rate(unit Khz)\n");
		return -E2BIG;
	}
	clk_set_rate(data->ddr_clk, freq * KHZ_TO_HZ);

	dev_info(dev, "ddr freq read back: %lu\n", \
		clk_get_rate(data->ddr_clk) / KHZ_TO_HZ);

	return size;
}

static ssize_t ddr_freq_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);
	return sprintf(buf, "current ddr freq is: %lu\n",
		 clk_get_rate(data->ddr_clk) / KHZ_TO_HZ);
}

/* used to collect ddr cnt during 20ms */
static ssize_t dp_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	struct devfreq *df;
	unsigned long total_cycle, busy_cycle;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);
	df = data->pdev_ddr;

	mutex_lock(&df->lock);
	if (df->profile->polling_ms) {
		mutex_unlock(&df->lock);
		dev_err(dev, "<ERR> Disable profiling at first!\n");
		return -EPERM;
	}
	mutex_unlock(&df->lock);

	reset_ddr_counters(data);
	msleep(20);
	get_ddr_cycles(data, &total_cycle, &busy_cycle);

	return sprintf(buf, "Default timeSpan: 20ms, total cnt %lx(%lu), "
		       "idle cnt %lx(%lu), workload %d percent\n",
		       total_cycle, total_cycle,
		       total_cycle - busy_cycle,
		       total_cycle - busy_cycle,
		       cal_workload(busy_cycle, total_cycle));
}

/* used to collect ddr cnt during a time */
static ssize_t dp_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	char tmp[5];
	unsigned long total_cycle, busy_cycle;
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	struct devfreq *df;
	struct timespec old_ts;
	int cnttime_ms;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);
	df = data->pdev_ddr;

	mutex_lock(&df->lock);
	if (df->profile->polling_ms) {
		mutex_unlock(&df->lock);
		dev_err(dev, "<ERR> disable profiling at first\n");
		return -EPERM;
	}
	mutex_unlock(&df->lock);

	if (sscanf(buf, "%s", tmp) > sizeof(tmp)) {
		dev_err(dev, "<ERR> echo start\" or \"stop\n");
		return -EINVAL;
	}

	if (strcmp(tmp, "start") == 0) {
		reset_ddr_counters(data);
		getnstimeofday(&data->last_ts);
		dev_info(dev, "ddr profiler counting, echo \"stop\" to stop\n");
		goto dp_exit;
	} else if (strcmp(tmp, "stop") == 0) {
		get_ddr_cycles(data, &total_cycle, &busy_cycle);
		old_ts = data->last_ts;
		getnstimeofday(&data->last_ts);
		cnttime_ms = (data->last_ts.tv_sec - old_ts.tv_sec) *
			MSEC_PER_SEC +
			(data->last_ts.tv_nsec - old_ts.tv_nsec) /
			NSEC_PER_MSEC;
		dev_info(dev, "Timespan: %dms, total cycle %lx(%lu), "
			"idle cycle %lx(%lu), workload %d percent\n",
			cnttime_ms, total_cycle, total_cycle,
			total_cycle - busy_cycle,
			total_cycle - busy_cycle,
			cal_workload(busy_cycle, total_cycle));
		goto dp_exit;
	} else {
		dev_err(dev, "<ERR> Only start and stop can be accepted\n");
		return -EINVAL;
	}

dp_exit:
	return size;
}

static DEVICE_ATTR(disable_ddr_fc, S_IRUGO | S_IWUSR, \
	disable_show, disable_store);
static DEVICE_ATTR(ddr_freq, S_IRUGO | S_IWUSR, ddr_freq_show, ddr_freq_store);
static DEVICE_ATTR(ddr_profiling, S_IRUGO | S_IWUSR, dp_show, dp_store);

static struct devfreq_dev_profile ddr_devfreq_profile = {
	/* FIXME turn off profiling until ddr devfreq tests are completed */
	.polling_ms = 0,
	.target = ddr_target,
	.get_dev_status = ddr_get_dev_status,
};

static void insert_item_into_array(struct device *dev,
	u32 table[], u32 *len, u32 item)
{
	u32 i, j;

	dev_dbg(dev, "+++++len is %d, items is %d\n", *len, item);

	if (*len == 0) {
		table[0] = item;
		(*len)++;

		dev_dbg(dev, "len is %d, table[0] is %d\n", *len, table[0]);

		return;
	}

	for (i = 0; i < *len; i++) {
		if (item < table[i]) {
			for (j = *len; j > i; j--)
				table[j] = table[j - 1];
			table[i] = item;
			(*len)++;
			dev_dbg(dev, "insert!, len is %d, i is %d\n", *len, i);
			return;
		} else if (item == table[i]) {
			dev_dbg(dev, "equal\n");
			return;
		}
	}

	table[i] = item;
	(*len)++;

	dev_dbg(dev, "in the last !, i is %d, len is %d", i, *len);
	return ;
}

static int ddr_devfreq_probe(struct platform_device *pdev)
{
	int i = 0, res;
	int ret = 0;
	u32 tmp;
	struct device *dev = &pdev->dev;
	struct devfreq_platform_data *pdata = NULL;
	struct ddr_devfreq_data *data = NULL;
	const char *ddr_clk_name;

	pdata = (struct devfreq_platform_data *)dev->platform_data;
	if (!pdata) {
		dev_err(dev, "No platform data!\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct ddr_devfreq_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Cannot allocate memory.\n");
		return -ENOMEM;
	}

	ddr_clk_name = (pdata->clk_name) ? (pdata->clk_name) : "ddr";
	data->ddr_clk = clk_get_sys(NULL, ddr_clk_name);
	if (IS_ERR(data->ddr_clk)) {
		dev_err(dev, "Cannot get clk ptr.\n");
		ret = PTR_ERR(data->ddr_clk);
		goto err_clk_get;
	}

	data->interleave_is_on = pdata->interleave_is_on;
	data->mc_base_pri = pdata->hw_base[0];
	if (data->interleave_is_on)
		data->mc_base_sec = pdata->hw_base[1];

	/* save ddr frequency tbl */
	if (pdata->freq_table != NULL) {
		while (pdata->freq_table[i].frequency != DEVFREQ_TABLE_END) {
			tmp = pdata->freq_table[i].frequency;
			insert_item_into_array(dev, data->ddr_freq_tbl, \
				&data->ddr_freq_tbl_len, tmp);
			i++;
		}
	}
	mutex_init(&data->mutex);
	spin_lock_init(&data->lock);

	ddr_devfreq_profile.initial_freq =
		clk_get_rate(data->ddr_clk) / KHZ_TO_HZ;
	data->pdev_ddr = devfreq_add_device(&pdev->dev,
				       &ddr_devfreq_profile,
				       default_gov, &ddr_ondemand_data);
	if (IS_ERR(data->pdev_ddr)) {
		dev_err(dev, "devfreq add error !\n");
		ret =  (unsigned long)data->pdev_ddr;
		goto err_devfreq_add;
	}

	/* init default devfreq min_freq and max_freq */
	data->pdev_ddr->min_freq = data->ddr_freq_tbl[0];
	data->pdev_ddr->max_freq =
		data->ddr_freq_tbl[data->ddr_freq_tbl_len - 1];

	/* Pass the frequency table to devfreq framework */
	if (pdata->freq_table)
		devfreq_set_freq_table(data->pdev_ddr, pdata->freq_table);

	res = device_create_file(&pdev->dev, &dev_attr_disable_ddr_fc);
	if (res) {
		dev_err(dev,
			"device attr disable_ddr_fc create fail: %d\n", res);
		ret = -ENOENT;
		goto err_file_create0;
	}

	res = device_create_file(&pdev->dev, &dev_attr_ddr_freq);
	if (res) {
		dev_err(dev, "device attr ddr_freq create fail: %d\n", res);
		ret = -ENOENT;
		goto err_file_create1;
	}

	res = device_create_file(&pdev->dev, &dev_attr_ddr_profiling);
	if (res) {
		dev_err(dev, \
			"device attr ddr_profiling create fail: %d\n", res);
		ret = -ENOENT;
		goto err_file_create2;
	}
	platform_set_drvdata(pdev, data);
	reset_ddr_counters(data);
	return 0;

err_file_create2:
	device_remove_file(&pdev->dev, &dev_attr_ddr_freq);
err_file_create1:
	device_remove_file(&pdev->dev, &dev_attr_disable_ddr_fc);
err_file_create0:
	devfreq_remove_device(data->pdev_ddr);
err_devfreq_add:
err_clk_get:
	kfree(data);
	return ret;
}

static int ddr_devfreq_remove(struct platform_device *pdev)
{
	struct ddr_devfreq_data *data = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_disable_ddr_fc);
	device_remove_file(&pdev->dev, &dev_attr_ddr_freq);
	device_remove_file(&pdev->dev, &dev_attr_ddr_profiling);
	devfreq_remove_device(data->pdev_ddr);
	kfree(data);
	return 0;
}

static struct platform_driver ddr_devfreq_driver = {
	.probe = ddr_devfreq_probe,
	.remove = ddr_devfreq_remove,
	.driver = {
		.name = "devfreq-ddr",
		.owner = THIS_MODULE,
	},
};

static void __init ddr_devfreq_exit(void)
{
	platform_driver_unregister(&ddr_devfreq_driver);
}

static int __init ddr_devfreq_init(void)
{
	return platform_driver_register(&ddr_devfreq_driver);
}

fs_initcall(ddr_devfreq_init);
module_exit(ddr_devfreq_exit);

MODULE_DESCRIPTION("mck4 memorybus devfreq driver");
MODULE_LICENSE("GPL");
