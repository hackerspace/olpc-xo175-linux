/*
 * Real Time Clock driver for Marvell 88PM80x PMIC
 *
 * Copyright (c) 2010 Marvell International Ltd.
 * Author:	Wenzeng Chen<wzch@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/mfd/88pm80x.h>
#include <linux/sched.h>

#ifdef CONFIG_RTC_MON
#include <mach/88pm8xxx-rtc.h>
#include <mach/regs-rtc.h>
#include <linux/miscdevice.h>
DECLARE_WAIT_QUEUE_HEAD(pm80x_rtc_update_head);
#endif

struct pm80x_rtc_info {
	struct pm80x_chip *chip;
	struct i2c_client *i2c;
	struct rtc_device *rtc_dev;
	struct device *dev;
	struct delayed_work calib_work;

	int irq;
	int vrtc;
	int (*sync) (unsigned int ticks);
};

static int pm80x_rtc_read_time(struct device *dev, struct rtc_time *tm);

#ifdef CONFIG_RTC_MON
static struct platform_device *g_pdev;
static long pm80x_rtcmon_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct rtc_time tm = { 0 };
	int ret = 0;

	switch (cmd) {
	case RTC_CHANGE_SUB:
		{
			DEFINE_WAIT(wait);
			prepare_to_wait(&pm80x_rtc_update_head,
					&wait, TASK_INTERRUPTIBLE);
			schedule();
			finish_wait(&pm80x_rtc_update_head, &wait);
		}
		ret = pm80x_rtc_read_time(&g_pdev->dev, &tm);
		if (ret < 0)
			pr_info
			    ("pxa_rtcmon_ioctl::pm80x_rtc_read_time fail]\n");
		else
			pr_info
			    ("pxa_rtcmon_ioctl::pm80x_rtc_read_time=%d:%d:%d\n",
			     tm.tm_hour, tm.tm_min, tm.tm_sec);
		ret = copy_to_user(uarg, &tm, sizeof(struct rtc_time));
		if (ret)
			ret = -EFAULT;
		break;
	default:
		pr_info("pxa_rtcmon_ioctl:default\n");
		ret = -ENOIOCTLCMD;
	}
	return ret;
}

static int pm80x_rtcmon_open(struct inode *inode, struct file *file)
{
	pr_info("pm80x_rtcmon_open::nothing done here\n");
	return 0;
}

static int pm80x_rtcmon_release(struct inode *inode, struct file *file)
{
	pr_info("pm80x_rtcmon_release::nothing done here\n");
	return 0;
}
#endif

static irqreturn_t rtc_update_handler(int irq, void *data)
{
	struct pm80x_rtc_info *info = (struct pm80x_rtc_info *)data;
	int mask;
	printk("rtc update handler\n");
	mask = PM800_ALARM | PM800_ALARM_WAKEUP;
	pm80x_set_bits(info->i2c, PM800_RTC_CONTROL, mask | PM800_ALARM1_EN, mask);
	rtc_update_irq(info->rtc_dev, 1, RTC_AF);
	return IRQ_HANDLED;
}

static int pm80x_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct pm80x_rtc_info *info = dev_get_drvdata(dev);

	if (enabled)
		pm80x_set_bits(info->i2c, PM800_RTC_CONTROL, PM800_ALARM1_EN,
			       PM800_ALARM1_EN);
	else
		pm80x_set_bits(info->i2c, PM800_RTC_CONTROL, PM800_ALARM1_EN, 0);
	return 0;
}

/*
 * Calculate the next alarm time given the requested alarm time mask
 * and the current time.
 */
static void rtc_next_alarm_time(struct rtc_time *next, struct rtc_time *now,
				struct rtc_time *alrm)
{
	unsigned long next_time;
	unsigned long now_time;

	next->tm_year = now->tm_year;
	next->tm_mon = now->tm_mon;
	next->tm_mday = now->tm_mday;
	next->tm_hour = alrm->tm_hour;
	next->tm_min = alrm->tm_min;
	next->tm_sec = alrm->tm_sec;

	rtc_tm_to_time(now, &now_time);
	rtc_tm_to_time(next, &next_time);

	if (next_time < now_time) {
		/* Advance one day */
		next_time += 60 * 60 * 24;
		rtc_time_to_tm(next_time, next);
	}
}

static int pm80x_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct pm80x_rtc_info *info = dev_get_drvdata(dev);
	unsigned char buf[4];
	unsigned long ticks, base, data;

	pm80x_bulk_read(info->i2c, PM800_USER_DATA3, 4, buf);
	base = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
	dev_dbg(info->dev, "%x-%x-%x-%x\n", buf[0], buf[1], buf[2], buf[3]);

	/* load 32-bit read-only counter */
	pm80x_bulk_read(info->i2c, PM800_RTC_COUNTER1, 4, buf);
	data = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
	ticks = base + data;
	dev_dbg(info->dev, "get base:0x%lx, RO count:0x%lx, ticks:0x%lx\n",
		base, data, ticks);
	rtc_time_to_tm(ticks, tm);
	return 0;
}

static int pm80x_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct pm80x_rtc_info *info = dev_get_drvdata(dev);
	unsigned char buf[4];
	unsigned long ticks, base, data;
	if ((tm->tm_year < 70) || (tm->tm_year > 138)) {
		dev_dbg(info->dev, "Set time %d out of range. "
			"Please set time between 1970 to 2038.\n",
			1900 + tm->tm_year);
		return -EINVAL;
	}
	rtc_tm_to_time(tm, &ticks);

	/* load 32-bit read-only counter */
	pm80x_bulk_read(info->i2c, PM800_RTC_COUNTER1, 4, buf);
	data = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
	base = ticks - data;
	dev_dbg(info->dev, "set base:0x%lx, RO count:0x%lx, ticks:0x%lx\n",
		base, data, ticks);
	buf[0] = base & 0xFF;
	buf[1] = (base >> 8) & 0xFF;
	buf[2] = (base >> 16) & 0xFF;
	buf[3] = (base >> 24) & 0xFF;
	pm80x_bulk_write(info->i2c, PM800_USER_DATA3, 4, buf);

	if (info->sync)
		info->sync(ticks);
#ifdef CONFIG_RTC_MON
	/* Update all subscribed about RTC set */
	wake_up_all(&pm80x_rtc_update_head);
	pr_info("pm80x_rtc_update_head\n");
#endif

	return 0;
}

static int pm80x_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct pm80x_rtc_info *info = dev_get_drvdata(dev);
	unsigned char buf[4];
	unsigned long ticks, base, data;
	int ret;

	pm80x_bulk_read(info->i2c, PM800_USER_DATA3, 4, buf);
	base = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
	dev_dbg(info->dev, "%x-%x-%x-%x\n", buf[0], buf[1], buf[2], buf[3]);

	pm80x_bulk_read(info->i2c, PM800_RTC_EXPIRE1_1, 4, buf);
	data = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
	ticks = base + data;
	dev_dbg(info->dev, "get base:0x%lx, RO count:0x%lx, ticks:0x%lx\n",
		base, data, ticks);

	rtc_time_to_tm(ticks, &alrm->time);
	ret = pm80x_reg_read(info->i2c, PM800_RTC_CONTROL);
	alrm->enabled = (ret & PM800_ALARM1_EN) ? 1 : 0;
	alrm->pending = (ret & (PM800_ALARM | PM800_ALARM_WAKEUP)) ? 1 : 0;
	return 0;
}

static int pm80x_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct pm80x_rtc_info *info = dev_get_drvdata(dev);
	struct rtc_time now_tm, alarm_tm;
	unsigned long ticks, base, data;
	unsigned char buf[4];
	int mask;

	pm80x_set_bits(info->i2c, PM800_RTC_CONTROL, PM800_ALARM1_EN, 0);

	pm80x_bulk_read(info->i2c, PM800_USER_DATA3, 4, buf);
	base = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
	dev_dbg(info->dev, "%x-%x-%x-%x\n", buf[0], buf[1], buf[2], buf[3]);

	/* load 32-bit read-only counter */
	pm80x_bulk_read(info->i2c, PM800_RTC_COUNTER1, 4, buf);
	data = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
	ticks = base + data;
	dev_dbg(info->dev, "get base:0x%lx, RO count:0x%lx, ticks:0x%lx\n",
		base, data, ticks);

	rtc_time_to_tm(ticks, &now_tm);
	dev_dbg(info->dev, "%s, now time : %lu\n", __func__, ticks);
	rtc_next_alarm_time(&alarm_tm, &now_tm, &alrm->time);
	/* get new ticks for alarm in 24 hours */
	rtc_tm_to_time(&alarm_tm, &ticks);
	dev_dbg(info->dev, "%s, alarm time: %lu\n", __func__, ticks);
	data = ticks - base;

	buf[0] = data & 0xff;
	buf[1] = (data >> 8) & 0xff;
	buf[2] = (data >> 16) & 0xff;
	buf[3] = (data >> 24) & 0xff;
	pm80x_bulk_write(info->i2c, PM800_RTC_EXPIRE1_1, 4, buf);
	if (alrm->enabled) {
		mask = PM800_ALARM | PM800_ALARM_WAKEUP | PM800_ALARM1_EN;
		pm80x_set_bits(info->i2c, PM800_RTC_CONTROL, mask, mask);
	} else {
		mask = PM800_ALARM | PM800_ALARM_WAKEUP | PM800_ALARM1_EN;
		pm80x_set_bits(info->i2c, PM800_RTC_CONTROL, mask,
			       PM800_ALARM | PM800_ALARM_WAKEUP);
	}
	return 0;
}

static const struct rtc_class_ops pm80x_rtc_ops = {
	.read_time = pm80x_rtc_read_time,
	.set_time = pm80x_rtc_set_time,
	.read_alarm = pm80x_rtc_read_alarm,
	.set_alarm = pm80x_rtc_set_alarm,
	.alarm_irq_enable = pm80x_rtc_alarm_irq_enable,
};

#ifdef CONFIG_RTC_MON
static const struct file_operations pm80x_rtcmon_fops = {
	.owner = THIS_MODULE,
	.open = pm80x_rtcmon_open,
	.release = pm80x_rtcmon_release,
	.unlocked_ioctl = pm80x_rtcmon_ioctl,
};

static struct miscdevice rtcmon_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rtcmon",
	.fops = &pm80x_rtcmon_fops,
};
#endif

#ifdef CONFIG_PM
static int pm80x_rtc_suspend(struct device *dev)
{
	struct pm80x_rtc_info *info = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		enable_irq_wake(info->chip->pm800_chip->irq);
		enable_irq_wake(info->irq);
	}
	return 0;
}

static int pm80x_rtc_resume(struct device *dev)
{
	struct pm80x_rtc_info *info = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		disable_irq_wake(info->chip->pm800_chip->irq);
		disable_irq_wake(info->irq);
	}
	return 0;
}

static struct dev_pm_ops pm80x_rtc_pm_ops = {
	.suspend = pm80x_rtc_suspend,
	.resume = pm80x_rtc_resume,
};
#endif
static int __devinit pm80x_rtc_probe(struct platform_device *pdev)
{
	struct pm80x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm80x_platform_data *pm80x_pdata;
	struct pm80x_rtc_pdata *pdata = NULL;
	struct pm80x_rtc_info *info;
	struct rtc_time tm;
	unsigned long ticks = 0;
	int ret;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL)
		dev_warn(&pdev->dev, "No platform data!\n");

	info = kzalloc(sizeof(struct pm80x_rtc_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->irq = platform_get_irq(pdev, 0);
	if (info->irq < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		ret = -EINVAL;
		goto out;
	}

	info->chip = chip;
	info->i2c = chip->base_page;
	info->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, info);

	ret = request_threaded_irq(info->irq, NULL, rtc_update_handler,
				   IRQF_ONESHOT, "rtc", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq, ret);
		goto out;
	}

	ret = pm80x_rtc_read_time(&pdev->dev, &tm);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to read initial time.\n");
		goto out_rtc;
	}
	if ((tm.tm_year < 70) || (tm.tm_year > 138)) {
		tm.tm_year = 70;
		tm.tm_mon = 0;
		tm.tm_mday = 1;
		tm.tm_hour = 0;
		tm.tm_min = 0;
		tm.tm_sec = 0;
		ret = pm80x_rtc_set_time(&pdev->dev, &tm);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to set initial time.\n");
			goto out_rtc;
		}
	}
	rtc_tm_to_time(&tm, &ticks);
	if (pdata && pdata->sync) {
		pdata->sync(ticks);
		info->sync = pdata->sync;
	}

	info->rtc_dev = rtc_device_register("88pm80x-rtc", &pdev->dev,
					    &pm80x_rtc_ops, THIS_MODULE);
	ret = PTR_ERR(info->rtc_dev);
	if (IS_ERR(info->rtc_dev)) {
		dev_err(&pdev->dev, "Failed to register RTC device: %d\n", ret);
		goto out_rtc;
	}
#ifdef CONFIG_RTC_MON
	g_pdev = pdev;
	ret = misc_register(&rtcmon_miscdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register rtcmon: %d\n", ret);
		goto out_rtc;
	} else
		pr_info("CONFIG_RTC_MON is on\n");
#endif
	/*
	 * enable internal XO instead of internal 3.25MHz clock since it can
	 * free running in PMIC power-down state.
	 */
	pm80x_set_bits(info->i2c, PM800_RTC_CONTROL, PM800_RTC1_USE_XO, PM800_RTC1_USE_XO);

	if (pdev->dev.parent->platform_data) {
		pm80x_pdata = pdev->dev.parent->platform_data;
		pdata = pm80x_pdata->rtc;
		if (pdata)
			info->rtc_dev->dev.platform_data = &pdata->rtc_wakeup;
	}

	device_init_wakeup(&pdev->dev, 1);

	return 0;
out_rtc:
	free_irq(info->irq, info);
out:
	kfree(info);
	return ret;
}

static int __devexit pm80x_rtc_remove(struct platform_device *pdev)
{
	struct pm80x_rtc_info *info = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);
	rtc_device_unregister(info->rtc_dev);
	free_irq(info->irq, info);
	kfree(info);
	return 0;
}

static struct platform_driver pm80x_rtc_driver = {
	.driver = {
		   .name = "88pm80x-rtc",
		   .owner = THIS_MODULE,
#ifdef CONFIG_PM
		   .pm = &pm80x_rtc_pm_ops,
#endif
		   },
	.probe = pm80x_rtc_probe,
	.remove = __devexit_p(pm80x_rtc_remove),
};

static int __init pm80x_rtc_init(void)
{
	return platform_driver_register(&pm80x_rtc_driver);
}

module_init(pm80x_rtc_init);

static void __exit pm80x_rtc_exit(void)
{
	platform_driver_unregister(&pm80x_rtc_driver);
#ifdef CONFIG_RTC_MON
	misc_deregister(&rtcmon_miscdev);
#endif

}

module_exit(pm80x_rtc_exit);

MODULE_DESCRIPTION("Marvell 88PM80x RTC driver");
MODULE_AUTHOR("Wenzeng Chen <wzch@marvell.com>");
MODULE_LICENSE("GPL");
