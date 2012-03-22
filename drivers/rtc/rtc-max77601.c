/*
 * Maxim max77601 RTC driver
 *
 * Copyright (c) 2012 Marvell Technology Ltd.
 * Yunfan Zhang <yfzhang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/mfd/max77601.h>

enum {
	RTC_SEC = 0,
	RTC_MIN,
	RTC_HOUR,
	RTC_WEEKDAY,
	RTC_MONTH,
	RTC_YEAR,
	RTC_DATE,
	RTC_BYTE_CNT
};

enum {
	MAX77601_RTCINT_RTC60S = 0,
	MAX77601_RTCINT_RTCA1,
	MAX77601_RTCINT_RTCA2,
	MAX77601_RTCINT_SMPL,
	MAX77601_RTCINT_RTC1S,
};

struct max77601_rtc_info {
	struct device *dev;
	struct i2c_client *i2c;
	struct rtc_device *rtc_dev;
	struct mutex io_lock;
	int irq_base;
	int irq;
	int core_irq;
};

static char *wday_s[7] = {
	"Sun", "Mon", "Tue", "Wed", "Thur", "Fri", "Sat"
};

#define MAX77601_RTC_RETRY_LIMIT	10
/* Year from 2000 to 2099 */
#define MAX77601_YEAR_BASE	100
#define MAX77601_YEAR_MAX	(MAX77601_YEAR_BASE + 99)

/* Return a negative else zero on success */
static int max77601_rtc_i2c_read(struct i2c_client *i2c,
				u8 reg, u8 *buf, u8 len)
{
	int ret;
	if (!i2c || !buf || len <= 0)
		return -EINVAL;
	/* Return the number of read bytes */
	ret = i2c_smbus_read_i2c_block_data(i2c, reg, len, buf);
	if (ret != len)
		return -EINVAL;
	return 0;
}

/* Return a negative errno code else zero on success */
static int max77601_rtc_i2c_write(struct i2c_client *i2c,
				u8 reg, u8 *buf, u8 len)
{
	if (!i2c || !buf || len <= 0)
		return -EINVAL;

	return i2c_smbus_write_i2c_block_data(i2c, reg, len, buf);
}

static int max77601_rtc_i2c_set_bits(struct i2c_client *i2c,
				u8 reg, u8 mask, u8 value)
{
	u8 tmp;
	int ret;

	ret = max77601_rtc_i2c_read(i2c, reg, &tmp, 1);
	if (ret)
		return ret;
	value = (tmp & ~mask) | (value & mask);
	return max77601_rtc_i2c_write(i2c, reg, &value, 1);
}

static int max77601_rtc_read(struct max77601_rtc_info *info, u8 reg,
				u8 *buf, u8 len)
{
	int ret;
	if (!info) {
		pr_err("%s:info is unavailable!\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&info->io_lock);
	ret = max77601_rtc_i2c_read(info->i2c, reg, buf, len);
	mutex_unlock(&info->io_lock);
	if (ret)
		pr_err("%s: failed to read reg!\n", __func__);
	return ret;
}

static int max77601_rtc_write(struct max77601_rtc_info *info, u8 reg,
				u8 *buf, u8 len)
{
	int ret;
	if (!info) {
		pr_err("%s:info is unavailable!\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&info->io_lock);
	ret = max77601_rtc_i2c_write(info->i2c, reg, buf, len);
	mutex_unlock(&info->io_lock);
	if (ret)
		pr_err("%s: failed to write reg!\n", __func__);
	return ret;
}

static int max77601_rtc_set_bits(struct max77601_rtc_info *info,
				u8 reg, u8 mask, u8 value)
{
	int ret;
	if (!info) {
		pr_err("%s:info is unavailable!\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&info->io_lock);
	ret = max77601_rtc_i2c_set_bits(info->i2c, reg, mask, value);
	mutex_unlock(&info->io_lock);
	if (ret)
		pr_err("%s: failed to set bits!\n", __func__);
	return ret;
}

/* Register value to rtc time */
static int max77601_rtc_reg_to_tm(u8 *reg, struct rtc_time *tm)
{
	u8 wkday = reg[RTC_WEEKDAY] & 0x7F;	/* 6:0 */
	if (unlikely(!wkday)) {
		pr_err("%s: RTC_WEEKDAY error\n", __func__);
		return -EINVAL;
	}
	tm->tm_year = reg[RTC_YEAR] + MAX77601_YEAR_BASE;	/* 7:0 */
	tm->tm_mon = (reg[RTC_MONTH] & 0x1F) - 1;	/* 4:0 */
	tm->tm_mday = reg[RTC_DATE] & 0x3F;	/* 5:0 */
	tm->tm_hour = reg[RTC_HOUR] & 0x3F;	/* 5:0 */
	tm->tm_min = reg[RTC_MIN] & 0x7F;	/* 6:0 */
	tm->tm_sec = reg[RTC_SEC] & 0x7F;	/* 6:0 */

	tm->tm_wday = 0;
	/* Bit[0..6]: Sun to Sat */
	while (!(wkday % 2)) {
		tm->tm_wday++;
		wkday >>= 1;
	}

	return 0;
}

/* RTC time to register value */
static int max77601_rtc_tm_to_reg(struct rtc_time *tm, u8 *reg, int alarm)
{
	u8 alarm_bit = alarm ? 0x80 : 0x00;

	reg[RTC_YEAR] = (tm->tm_year - MAX77601_YEAR_BASE) | alarm_bit;
	reg[RTC_MONTH] = (tm->tm_mon + 1) | alarm_bit;
	reg[RTC_DATE] = tm->tm_mday | alarm_bit;
	reg[RTC_HOUR] = tm->tm_hour | alarm_bit;
	reg[RTC_MIN] = tm->tm_min | alarm_bit;
	reg[RTC_SEC] = tm->tm_sec | alarm_bit;
	reg[RTC_WEEKDAY] = (1 << tm->tm_wday) | alarm_bit;

	return 0;
}

static int max77601_rtc_sync_read_buffer(struct max77601_rtc_info *info)
{
	int ret, retry = 0;
	u8 tmp;
	/* Issuing update read buffer */
	ret = max77601_rtc_set_bits(info, MAX77601_RTCUPDATE0,
				MAX77601_RBUDR, MAX77601_RBUDR);
	if (ret)
		return ret;
	/* Typical update time is 15ms */
	while (retry++ < MAX77601_RTC_RETRY_LIMIT) {
		msleep(20);
		ret = max77601_rtc_read(info, MAX77601_RTCUPDATE1, &tmp, 1);
		if (ret)
			return ret;
		if (tmp & MAX77601_RBUDF)
			break;
	}
	if (retry >= MAX77601_RTC_RETRY_LIMIT) {
		pr_err("%s: retry failed!\n", __func__);
		return -1;
	}
	/* Sync successful */
	return 0;
}

static int max77601_rtc_commit_write_buffer(struct max77601_rtc_info *info)
{
	int ret, retry = 0;
	u8 tmp;
	/* Issuing update register */
	ret = max77601_rtc_set_bits(info, MAX77601_RTCUPDATE0,
				MAX77601_UDR, MAX77601_UDR);
	if (ret)
		return ret;
	/* Typical update time is 15ms */
	while (retry++ < MAX77601_RTC_RETRY_LIMIT) {
		msleep(20);
		ret = max77601_rtc_read(info, MAX77601_RTCUPDATE1, &tmp, 1);
		if (ret)
			return ret;
		if (tmp & MAX77601_UDF)
			break;
	}
	if (retry >= MAX77601_RTC_RETRY_LIMIT) {
		pr_err("%s: retry failed!\n", __func__);
		return -1;
	}
	/* Commit successful */
	return 0;
}

static int max77601_rtc_read_sync(struct max77601_rtc_info *info,
				u8 reg, u8 *buf, u8 len)
{
	int ret = 0;

	ret = max77601_rtc_sync_read_buffer(info);
	if (ret)
		return ret;
	return max77601_rtc_read(info, reg, buf, len);
}

static int max77601_rtc_write_commit(struct max77601_rtc_info *info,
				u8 reg, u8 *buf, u8 len)
{
	int ret = 0;

	ret = max77601_rtc_write(info, reg, buf, len);
	if (ret)
		return ret;
	return max77601_rtc_commit_write_buffer(info);
}

static int max77601_rtc_set_bits_commit(struct max77601_rtc_info *info,
				u8 reg, u8 mask, u8 value)
{
	int ret;
	u8 tmp;
	if (!info) {
		pr_err("%s:info is unavailable!\n", __func__);
		return -EINVAL;
	}
	ret = max77601_rtc_read_sync(info, reg, &tmp, 1);
	if (ret)
		return ret;
	value = (tmp & ~mask) | (value & mask);
	return max77601_rtc_write_commit(info, reg, &value, 1);
}

static int max77601_rtc_valid_tm(struct rtc_time *tm)
{
	if (rtc_valid_tm(tm)) {
		pr_err("Time is invalid: %s, %d-%d-%d, %02d:%02d:%02d.\n",
			wday_s[tm->tm_wday], tm->tm_mday, tm->tm_mon + 1,
			tm->tm_year + 1900, tm->tm_hour,
			tm->tm_min, tm->tm_sec);
		return -EINVAL;
	}
	if (tm->tm_year < MAX77601_YEAR_BASE
	    || tm->tm_year > MAX77601_YEAR_MAX) {
		pr_err("%s: Year[%d] is out of range!"
			"Should be from %d to %d\n", __func__,
			1900 + tm->tm_year, 1900 + MAX77601_YEAR_BASE,
			1900 + MAX77601_YEAR_MAX);
		return -EINVAL;
	}
	return 0;
}

static irqreturn_t max77601_rtc_update_handler(int irq, void *data)
{
	struct max77601_rtc_info *info = (struct max77601_rtc_info *)data;
	u8 irq_status;
	int ret;

	ret = max77601_rtc_read(info, MAX77601_RTCINT, &irq_status, 1);
	if (ret)
		return ret;

	if (irq_status & MAX77601_RTCA1)
		rtc_update_irq(info->rtc_dev, 1, RTC_IRQF | RTC_AF);

	if (irq_status & MAX77601_RTC1S)
		rtc_update_irq(info->rtc_dev, 1, RTC_IRQF | RTC_UF);

	return IRQ_HANDLED;
}

static int max77601_rtc_alarm_irq_enable(struct device *dev,
				unsigned int enabled)
{
	struct max77601_rtc_info *info = dev_get_drvdata(dev);
	u8 mask, data;

	mask = MAX77601_RTCA1M;
	data = enabled ? 0 : MAX77601_RTCA1M;
	max77601_rtc_set_bits_commit(info, MAX77601_RTCINTM, mask, data);
	return 0;
}

static int max77601_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct max77601_rtc_info *info = dev_get_drvdata(dev);
	u8 buf[RTC_BYTE_CNT];
	int ret;

	ret = max77601_rtc_read_sync(info, MAX77601_RTCSEC, buf, RTC_BYTE_CNT);
	if (ret)
		return ret;
	return max77601_rtc_reg_to_tm(buf, tm);
}

static int max77601_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct max77601_rtc_info *info = dev_get_drvdata(dev);
	u8 buf[RTC_BYTE_CNT] = {0};
	int ret;

	if (max77601_rtc_valid_tm(tm))
		return -EINVAL;
	ret = max77601_rtc_tm_to_reg(tm, buf, 0);
	if (ret)
		return ret;
	return max77601_rtc_write_commit(info, MAX77601_RTCSEC,
					buf, RTC_BYTE_CNT);
}

static int max77601_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct max77601_rtc_info *info = dev_get_drvdata(dev);
	u8 buf[RTC_BYTE_CNT];
	int ret;

	ret = max77601_rtc_read_sync(info, MAX77601_RTCSECA1,
					buf, RTC_BYTE_CNT);
	if (ret)
		return ret;
	ret = max77601_rtc_reg_to_tm(buf, &alrm->time);
	if (ret)
		return ret;
	ret = max77601_rtc_read_sync(info, MAX77601_RTCINTM, buf, 1);
	if (ret)
		return ret;
	alrm->enabled = (buf[0] & MAX77601_RTCA1M) ? 0 : 1;

	return 0;
}

static int max77601_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct max77601_rtc_info *info = dev_get_drvdata(dev);
	u8 buf[RTC_BYTE_CNT] = {0};
	int ret;

	if (max77601_rtc_valid_tm(&alrm->time))
		return -EINVAL;
	ret = max77601_rtc_tm_to_reg(&alrm->time, buf, 1);
	if (ret)
		return ret;
	ret = max77601_rtc_write_commit(info, MAX77601_RTCSECA1,
					buf, RTC_BYTE_CNT);
	if (ret)
		return ret;
	return max77601_rtc_alarm_irq_enable(dev, alrm->enabled ? 1 : 0);
}

static int max77601_rtc_ajust_wday_reg(struct max77601_rtc_info *info)
{
	struct rtc_time tm;
	u8 buf[RTC_BYTE_CNT];
	unsigned long time, days;
	int wday, ret;
	ret = max77601_rtc_read_sync(info, MAX77601_RTCSEC, buf, RTC_BYTE_CNT);
	if (ret)
		return ret;
	ret = max77601_rtc_reg_to_tm(buf, &tm);
	if (ret)
		return ret;
	/* Convert Gregorian date to seconds since 01-01-1970 00:00:00 */
	rtc_tm_to_time(&tm, &time);
	days = time / (3600 * 24);
	/* Correct day of the week, 1970-01-01 was a Thursday */
	wday = (days + 4) % 7;
	/* Right, no need to ajust */
	if (tm.tm_wday == wday)
		return 0;
	/* Incorrect, Ajust... */
	tm.tm_wday = wday;
	/* Write back to RTC */
	ret = max77601_rtc_tm_to_reg(&tm, buf, 0);
	if (ret)
		return ret;
	ret = max77601_rtc_write_commit(info, MAX77601_RTCSEC,
					buf, RTC_BYTE_CNT);
	return ret;
}

static int max77601_rtc_device_init(struct max77601_rtc_info *info)
{
	u8 buf, mask, data;
	int ret = 0;

	/* Mask all interrupts */
	buf = MAX77601_RTC60SM | MAX77601_RTCA1M | MAX77601_RTCA2M
	    | MAX77601_SMPLM | MAX77601_RTC1SM;
	ret = max77601_rtc_write_commit(info, MAX77601_RTCINTM, &buf, 1);
	if (ret)
		return ret;
	/* UDF and RBUDF is cleared upon read */
	mask = data = MAX77601_FCUR;
	data = MAX77601_FCUR;
	ret = max77601_rtc_set_bits_commit(info, MAX77601_RTCUPDATE0,
					mask, data);
	if (ret)
		return ret;
	/* Enable BCD and HRMODE bit access */
	mask = MAX77601_BCDM | MAX77601_HRMODEM;
	data = MAX77601_BCDM | MAX77601_HRMODEM;
	ret = max77601_rtc_set_bits_commit(info, MAX77601_RTCCNTLM, mask, data);
	if (ret)
		return ret;
	/* Date Mode: Binary; Hour format: 24-Hour mode */
	buf = MAX77601_BCD_BINARY | MAX77601_HRMODE_24H;
	ret = max77601_rtc_write_commit(info, MAX77601_RTCCNTL, &buf, 1);
	if (ret)
		return ret;
	/* Disable BCD and HRMODE bit access */
	mask = MAX77601_BCDM | MAX77601_HRMODEM;
	data = 0;
	ret = max77601_rtc_set_bits_commit(info, MAX77601_RTCCNTLM, mask, data);
	/* Adjust week day */
	ret = max77601_rtc_ajust_wday_reg(info);
	return ret;
}

static const struct rtc_class_ops max77601_rtc_ops = {
	.read_time = max77601_rtc_read_time,
	.set_time = max77601_rtc_set_time,
	.read_alarm = max77601_rtc_read_alarm,
	.set_alarm = max77601_rtc_set_alarm,
	.alarm_irq_enable = max77601_rtc_alarm_irq_enable,
};

static int __devinit max77601_rtc_probe(struct platform_device *pdev)
{
	struct max77601_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct max77601_rtc_info *info;
	int ret = 0;

	info = kzalloc(sizeof(struct max77601_rtc_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->dev = &pdev->dev;
	info->i2c = chip->rtc;
	info->core_irq = chip->core_irq;
	dev_set_drvdata(&pdev->dev, info);

	device_init_wakeup(&pdev->dev, 1);
	mutex_init(&info->io_lock);

	ret = max77601_rtc_device_init(info);
	if (ret) {
		dev_err(info->dev, "device init failed!\n");
		goto err_dev_init;
	}
	/* Get IRQ */
	info->irq = platform_get_irq(pdev, 0);
	if (info->irq < 0) {
		dev_err(info->dev, "Failed to get IRQ\n");
		goto err_irq;
	}
	/* Request IRQ */
	ret = request_threaded_irq(info->irq, NULL,
				max77601_rtc_update_handler, IRQF_ONESHOT,
				"rtc-max77601-alarm", info);
	if (ret < 0) {
		dev_err(info->dev, "Failed to request IRQ: #%d\n", info->irq);
		goto err_irq;
	}
	info->rtc_dev = rtc_device_register("max77601-rtc", &pdev->dev,
					&max77601_rtc_ops, THIS_MODULE);
	if (IS_ERR(info->rtc_dev)) {
		dev_err(info->dev, "Failed to register RTC device\n");
		ret = PTR_ERR(info->rtc_dev);
		goto err_reg_rtc;
	}
	return 0;

err_reg_rtc:
	free_irq(info->irq, info);
err_irq:
err_dev_init:
	kfree(info);
	return ret;
}

static int __devexit max77601_rtc_remove(struct platform_device *pdev)
{
	struct max77601_rtc_info *info = platform_get_drvdata(pdev);
	if (info) {
		free_irq(info->irq, info);
		rtc_device_unregister(info->rtc_dev);
		kfree(info);
	}
	return 0;
}

#ifdef CONFIG_PM
static int max77601_rtc_suspend(struct device *dev)
{
	struct max77601_rtc_info *info = dev_get_drvdata(dev);
	if (device_may_wakeup(dev)) {
		enable_irq_wake(info->irq);
		enable_irq_wake(info->core_irq);
	}
	return 0;
}

static int max77601_rtc_resume(struct device *dev)
{
	struct max77601_rtc_info *info = dev_get_drvdata(dev);
	if (device_may_wakeup(dev)) {
		disable_irq_wake(info->core_irq);
		disable_irq_wake(info->irq);
	}
	return 0;
}

static const struct dev_pm_ops max77601_rtc_pm_ops = {
	.suspend = max77601_rtc_suspend,
	.resume = max77601_rtc_resume,
};
#endif
static struct platform_driver max77601_rtc_driver = {
	.driver = {
		.name = "max77601-rtc",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &max77601_rtc_pm_ops,
#endif
	},
	.probe = max77601_rtc_probe,
	.remove = __devexit_p(max77601_rtc_remove),
};

static int __init max77601_rtc_init(void)
{
	return platform_driver_register(&max77601_rtc_driver);
}

module_init(max77601_rtc_init);

static void __exit max77601_rtc_exit(void)
{
	platform_driver_unregister(&max77601_rtc_driver);
}

module_exit(max77601_rtc_exit);

MODULE_DESCRIPTION("Maxim max77601 RTC driver");
MODULE_AUTHOR("Yunfan Zhang <yfzhang@marvell.com>");
MODULE_LICENSE("GPL");
