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
#include <asm/io.h>
#include <linux/delay.h>
#include <mach/mmp3_pm.h>

#define DDR_FREQ_MAX 8

static struct devfreq *pdev_ddr;

static unsigned int ddr_freq_table[DDR_FREQ_MAX];
static unsigned int ddr_vir_base1;
static unsigned int ddr_vir_base0;
static unsigned int ddr_freq_len;
static unsigned int ddr_cur_freq;

static const struct devfreq_governor *default_gov = &devfreq_simple_ondemand;

static void get_ddr_cycles(unsigned long *total, unsigned long *busy)
{
	u32 total_cycle0, idle_cycle0;
	u32 total_cycle1, idle_cycle1;

	total_cycle0 = __raw_readl(ddr_vir_base0 + 0x450);
	idle_cycle0 = __raw_readl(ddr_vir_base0 + 0x458);

	total_cycle1 = __raw_readl(ddr_vir_base1 + 0x450);
	idle_cycle1 = __raw_readl(ddr_vir_base1 + 0x458);

	pr_debug("t1 %u, t0 %u, i1 %u, i0 %u\n", total_cycle1,
			total_cycle0, idle_cycle1, idle_cycle0);
	*total = (total_cycle0 >> 1) + (total_cycle1 >> 1);
	*busy = *total - (idle_cycle0 >> 1) - (idle_cycle1 >> 1);
}

static void reset_ddr_counters(void)
{
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

static int change_ddr_freq(unsigned int freq)
{
	mmp3_setfreq(MMP3_CLK_DDR_1, freq);

	return 0;
}

static int ddr_target(struct device *dev, unsigned long *freq)
{
	int i;

	for (i = 0; i < ddr_freq_len; i++)
		if (*freq <= ddr_freq_table[i])
			break;

	if (i == ddr_freq_len)
		i--;
	/* change ddr frequency */
	change_ddr_freq(ddr_freq_table[i]);
	pr_debug("ddr device changed to freq %u, o_freq is %u\n",
		 (unsigned int)ddr_freq_table[i], (unsigned int)*freq);

	return 0;
}

static ssize_t tp_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t size)
{
	int i;

	sscanf(buf, "%d", &i);
	pr_debug("Set ddr freq to %d\n", i);
	change_ddr_freq(i);
	pr_debug("ddr freq read back: %d\n",
		 (unsigned int)mmp3_getfreq(MMP3_CLK_DDR_1));

	return size;
}

static ssize_t tp_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int i, loop_num, tmp;

	loop_num = mmp3_get_pp_number();

	for (i = 0; i < loop_num; i++) {
		tmp = mmp3_get_pp_freq(i, MMP3_CLK_MP1);
		pr_debug("%d is %d", i, tmp);
	}

	return sprintf(buf, "loop_num is %d,\n", loop_num);
}

/* one stage profiling */
static ssize_t dp_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	unsigned int total_cycle, idle_cycle;

	reset_ddr_counters();

	msleep(20);

	total_cycle = __raw_readl(ddr_vir_base0 + 0x450);
	idle_cycle = __raw_readl(ddr_vir_base0 + 0x458);

	/*
	total_cycle1 = __raw_readl(ddr_vir_base1 + 0x450);
	idle_cycle1 = __raw_readl(ddr_vir_base1 + 0x458);
	*/

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
	unsigned int total_cycle, idle_cycle;

	sscanf(buf, "%s", tmp);

	if (strcmp(tmp, "start") == 0) {

		reset_ddr_counters();

		pr_info("ddr profiler counting, echo \"stop\" to stop\n");

		goto dp_exit;
	} else if (strcmp(tmp, "stop") == 0) {
		total_cycle = __raw_readl(ddr_vir_base1 + 0x450);
		idle_cycle = __raw_readl(ddr_vir_base1 + 0x458);

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

static DEVICE_ATTR(tp, S_IRUGO | S_IWUSR, tp_show, tp_store);
static DEVICE_ATTR(ddr_profiling, S_IRUGO | S_IWUSR, dp_show, dp_store);

static struct devfreq_dev_profile ddr_devfreq_profile = {
	.initial_freq = 800000,
	/* FIXME turn off profiling until ddr devfreq tests are completed */
	.polling_ms = 0,
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

static int probe(struct platform_device *pdev)
{
	int i, ret;
	u32 tmp, loop_num;
	struct resource *res;

	pdev_ddr = devfreq_add_device(&pdev->dev,
				       &ddr_devfreq_profile,
				       default_gov, NULL);

	if (IS_ERR(pdev_ddr)) {
		pr_err("devfreq add error !\n");

		ret =  (unsigned long)pdev_ddr;
		goto exit_ret;
	}

	ddr_devfreq_profile.initial_freq = mmp3_getfreq(MMP3_CLK_DDR_1);

	i = device_create_file(&pdev->dev, &dev_attr_tp);
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
	platform_driver_unregister(&ddr_devfreq_driver);
}

static int __init ddr_init(void)
{
	return platform_driver_register(&ddr_devfreq_driver);
}

module_init(ddr_init);
module_exit(ddr_exit);

MODULE_DESCRIPTION("ddr device driver");
MODULE_LICENSE("GPL");
