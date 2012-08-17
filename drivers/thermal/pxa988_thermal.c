/*
 * linux/driver/thermal/pxa988_thermal.c
 *
 * Author:      Hong Feng <hongfeng@marvell.com>
 * Copyright:   (C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/io.h>
#include <linux/syscalls.h>
#include <linux/cpufreq.h>
#include <linux/smp.h>
#include <mach/regs-apbc.h>

/* debug: Use for sysfs set temp */
/* #define DEBUG_TEMPERATURE */
#define TRIP_POINTS_NUM	4
#define TRIP_POINTS_ACTIVE_NUM (TRIP_POINTS_NUM - 1)

#define THERMAL_VIRT_BASE (APB_VIRT_BASE + 0x13200)
#define THERMAL_REG(x) (THERMAL_VIRT_BASE + (x))
#define THERMAL_TS_CTRL THERMAL_REG(0x20)
#define THERMAL_TS_READ THERMAL_REG(0x24)
/* TS_CTRL */
#define TS_CTRL_TSEN_TEMP_ON (1 << 3)
#define TS_CTRL_RST_N_TSEN (1 << 2)
#define TS_CTRL_TSEN_LOW_RANGE (1 << 1)
#define TS_CTRL_TSEN_CHOP_EN (1 << 0)
/* TS_READ */
#define TS_READ_TS_ON (1 << 4)
#define TS_READ_OUT_DATA (0xf << 0)

#define COOLDEV_ACTIVE_MAX_STATE 1

struct pxa988_thermal_device {
	struct thermal_zone_device *therm_cpu;
	int temp_cpu;
	struct clk *therm_clk;
	struct thermal_cooling_device *cool_dev_active[TRIP_POINTS_ACTIVE_NUM];
	int irq;
};

struct cool_dev_priv {
	int current_state;
	int max_state;
};

static struct pxa988_thermal_device pxa988_thermal_dev;

static int cpu_thermal_trips_temp[TRIP_POINTS_NUM] = {
	85000,/* bind to active type */
	95000,/* bind to active type */
	105000,/* bind to active type */
	110000,/* bind to critical type */
};
static struct cool_dev_priv cool_dev_active_priv[TRIP_POINTS_ACTIVE_NUM] = {
	{0, COOLDEV_ACTIVE_MAX_STATE},
	{0, COOLDEV_ACTIVE_MAX_STATE},
	{0, COOLDEV_ACTIVE_MAX_STATE},
};

static int cool_dev_get_max_state(struct thermal_cooling_device *cdev,
		unsigned long *state)
{
	struct cool_dev_priv *priv = (struct cool_dev_priv *)cdev->devdata;
	*state = priv->max_state;
	return 0;
}

static int cool_dev_get_current_state(struct thermal_cooling_device *cdev,
		unsigned long *state)
{
	struct cool_dev_priv *priv = (struct cool_dev_priv *)cdev->devdata;
	*state = priv->current_state;
	return 0;
}

static int cool_dev_set_current_state(struct thermal_cooling_device *cdev,
		unsigned long state)
{
	int i;
	char *temp_info[2]    = { "TEMP=100000", NULL };
	static int drop_init_check;
	struct cool_dev_priv *priv = (struct cool_dev_priv *)cdev->devdata;
	if (state > COOLDEV_ACTIVE_MAX_STATE)
		state = COOLDEV_ACTIVE_MAX_STATE;
	/*
	 * This is extremely embarrass, thermal_zone_device_register
	 * will directly call into here, while this time kobject_uevent
	 * is not ready for use(will panic), so we drop first round check
	 */
	if (drop_init_check < TRIP_POINTS_ACTIVE_NUM) {
		drop_init_check++;
		goto out;
	}

	if (state == priv->current_state)
		goto out;
	priv->current_state = state;
	/* notify user for trip point cross */
	for (i = 0; i < TRIP_POINTS_ACTIVE_NUM; i++) {
		if (cdev == pxa988_thermal_dev.cool_dev_active[i]) {
			sprintf(temp_info[0], "TEMP=%d",
					pxa988_thermal_dev.temp_cpu);
			kobject_uevent_env(&((pxa988_thermal_dev.therm_cpu)->
				device.kobj), KOBJ_CHANGE, temp_info);

		}
	}
out:
	return 0;
}

static struct thermal_cooling_device_ops cool_dev_active_ops = {
	.get_max_state = cool_dev_get_max_state,
	.get_cur_state = cool_dev_get_current_state,
	.set_cur_state = cool_dev_set_current_state,
};

static int cpu_sys_bind(struct thermal_zone_device *tz,
		struct thermal_cooling_device *cdev)
{
	int i;
	for (i = 0; i < TRIP_POINTS_ACTIVE_NUM; i++) {
		if (cdev == pxa988_thermal_dev.cool_dev_active[i])
			break;
	}
	return thermal_zone_bind_cooling_device(tz, i, cdev);
}

#ifdef DEBUG_TEMPERATURE
static int g_test_temp = 108000;

static int thermal_temp_debug_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_test_temp);
}

static int thermal_temp_debug_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d\n", &g_test_temp);
	return count;
}

static DEVICE_ATTR(thermal_debug_temp, 0644, thermal_temp_debug_get,
		thermal_temp_debug_set);

static struct attribute *thermal_attrs[] = {
	&dev_attr_thermal_debug_temp.attr,
	NULL,
};

static struct attribute_group thermal_attr_grp = {
	.attrs = thermal_attrs,
};
#endif

/* This function decode 4bit long number of gray code into original binary */
static int gray_decode(unsigned int gray)
{
	int num, i, tmp;

	if (gray >= 16)
		return 0;

	num = gray & 0x8;
	tmp = num >> 3;
	for (i = 2; i >= 0; i--) {
		tmp = ((gray & (1 << i)) >> i) ^ tmp;
		num |= tmp << i;
	}
	return num;
}

#define RETRY_TIMES (10)
static int cpu_sys_get_temp(struct thermal_zone_device *thermal,
		unsigned long *temp)
{
	int i = 0;
	unsigned long ts_read, ts_ctrl;
	int gray_code = 0;
	int ret = 0;
#ifdef DEBUG_TEMPERATURE
	*temp = g_test_temp;
#else
	ts_read = __raw_readl(THERMAL_TS_READ);
	if (likely(ts_read & TS_READ_TS_ON)) {
		gray_code = ts_read & TS_READ_OUT_DATA;
		*temp = (gray_decode(gray_code) * 5 / 2 + 80) * 1000;
	} else {
		for (i = 0; i < RETRY_TIMES; i++) {
			ts_read = __raw_readl(THERMAL_TS_READ);
			if (ts_read & TS_READ_TS_ON) {
				gray_code = ts_read & TS_READ_OUT_DATA;
				break;
			}
			msleep(20);
		}
		if (RETRY_TIMES == i) {
			*temp = 0;
			ret = -1;
		} else
			*temp = (gray_decode(gray_code) * 5 / 2 + 80) * 1000;
	}
	/* restart measure */
	ts_ctrl = __raw_readl(THERMAL_TS_CTRL);
	ts_ctrl &= ~TS_CTRL_TSEN_TEMP_ON;
	__raw_writel(ts_ctrl, THERMAL_TS_CTRL);

	ts_ctrl = __raw_readl(THERMAL_TS_CTRL);
	ts_ctrl |= TS_CTRL_TSEN_TEMP_ON;
	__raw_writel(ts_ctrl, THERMAL_TS_CTRL);
#endif
	pxa988_thermal_dev.temp_cpu = *temp;
	return ret;
}

static int cpu_sys_get_trip_type(struct thermal_zone_device *thermal, int trip,
		enum thermal_trip_type *type)
{
	if ((trip >= 0) && (trip < TRIP_POINTS_ACTIVE_NUM))
		*type = THERMAL_TRIP_ACTIVE;
	else if (TRIP_POINTS_ACTIVE_NUM == trip)
		*type = THERMAL_TRIP_CRITICAL;
	else
		*type = (enum thermal_trip_type)(-1);
	return 0;
}

static int cpu_sys_get_trip_temp(struct thermal_zone_device *thermal, int trip,
		unsigned long *temp)
{
	if ((trip >= 0) && (trip < TRIP_POINTS_NUM))
		*temp = cpu_thermal_trips_temp[trip];
	else
		*temp = -1;
	return 0;
}

static int cpu_sys_set_trip_temp(struct thermal_zone_device *thermal, int trip,
		unsigned long temp)
{
	if ((trip >= 0) && (trip < TRIP_POINTS_NUM))
		cpu_thermal_trips_temp[trip] = temp;
	return 0;
}

static int cpu_sys_get_crit_temp(struct thermal_zone_device *thermal,
		unsigned long *temp)
{
	return cpu_thermal_trips_temp[TRIP_POINTS_NUM - 1];
}

static int cpu_sys_notify(struct thermal_zone_device *thermal, int count,
		enum thermal_trip_type trip_type)
{
	if (THERMAL_TRIP_CRITICAL == trip_type)
		pr_info("notify critical temp hit\n");
	else
		pr_err("unexpected temp notify\n");
	/*
	 * when THERMAL_TRIP_CRITICAL, return 0
	 * will trigger shutdown in opensource framework
	 */
	return 0;
}

static struct thermal_zone_device_ops cpu_thermal_ops = {
	.bind = cpu_sys_bind,
	.get_temp = cpu_sys_get_temp,
	.get_trip_type = cpu_sys_get_trip_type,
	.get_trip_temp = cpu_sys_get_trip_temp,
	.set_trip_temp = cpu_sys_set_trip_temp,
	.get_crit_temp = cpu_sys_get_crit_temp,
	.notify = cpu_sys_notify,
};

#ifdef CONFIG_PM
static int thermal_suspend(struct device *dev)
{
	clk_disable(pxa988_thermal_dev.therm_clk);
	return 0;
}

static int thermal_resume(struct device *dev)
{
	clk_enable(pxa988_thermal_dev.therm_clk);
	return 0;
}

static const struct dev_pm_ops thermal_pm_ops = {
	.suspend = thermal_suspend,
	.resume = thermal_resume,
};
#endif

static irqreturn_t pxa988_thermal_irq(int irq, void *devid)
{
	/*
	 * TODO: on Z0, we don't use irq,
	 * We'll try to use irq on A0
	 */
	return IRQ_HANDLED;
}

static int pxa988_thermal_probe(struct platform_device *pdev)
{
	int ret, irq, i;
	unsigned long ts_ctrl;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource defined\n");
		return -ENXIO;
	}
	ret = request_irq(irq, pxa988_thermal_irq, IRQF_DISABLED,
			pdev->name, NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		return -ENXIO;
	}
	pxa988_thermal_dev.irq = irq;

	/* The last trip point is critical */
	for (i = 0; i < TRIP_POINTS_ACTIVE_NUM; i++) {
		pxa988_thermal_dev.cool_dev_active[i] =
			thermal_cooling_device_register("cool_dev_active",
					(void *)&cool_dev_active_priv[i],
					&cool_dev_active_ops);
		if (IS_ERR(pxa988_thermal_dev.cool_dev_active[i])) {
			pr_err("Failed to register cooling device\n");
			ret = -EINVAL;
			goto failed_free_irq;
		}
	}

	pxa988_thermal_dev.therm_clk = clk_get(NULL, "THERMALCLK");
	if (IS_ERR(pxa988_thermal_dev.therm_clk)) {
		pr_err("Could not get thermal clock\n");
		ret = -ENXIO;
		goto failed_unregister_cooldev;
	}
	clk_enable(pxa988_thermal_dev.therm_clk);

	/*
	 * We start first measure due to
	 * thermal_zone_device_register will get_temp
	 */
#ifndef DEBUG_TEMPERATURE
	ts_ctrl = __raw_readl(THERMAL_TS_CTRL);
	ts_ctrl |= TS_CTRL_TSEN_CHOP_EN;
	/* we only care greater than 80 */
	ts_ctrl &= ~TS_CTRL_TSEN_LOW_RANGE;
	__raw_writel(ts_ctrl, THERMAL_TS_CTRL);

	ts_ctrl = __raw_readl(THERMAL_TS_CTRL);
	ts_ctrl |= TS_CTRL_RST_N_TSEN;
	__raw_writel(ts_ctrl, THERMAL_TS_CTRL);

	/* start first measure */
	ts_ctrl = __raw_readl(THERMAL_TS_CTRL);
	ts_ctrl |= TS_CTRL_TSEN_TEMP_ON;
	__raw_writel(ts_ctrl, THERMAL_TS_CTRL);
#endif
	pxa988_thermal_dev.therm_cpu = thermal_zone_device_register(
			"pxa988-thermal", TRIP_POINTS_NUM,
			NULL, &cpu_thermal_ops, 1, 1, 2000, 2000);
	if (IS_ERR(pxa988_thermal_dev.therm_cpu)) {
		pr_err("Failed to register thermal zone device\n");
		return -EINVAL;
	}
#ifdef DEBUG_TEMPERATURE
	sysfs_create_group(&((pxa988_thermal_dev.therm_cpu->device).kobj),
			&thermal_attr_grp);
#endif

	return 0;
failed_unregister_cooldev:
	for (i = 0; i < TRIP_POINTS_ACTIVE_NUM; i++)
		thermal_cooling_device_unregister(
				pxa988_thermal_dev.cool_dev_active[i]);
failed_free_irq:
	free_irq(pxa988_thermal_dev.irq, NULL);
	return ret;
}

static int pxa988_thermal_remove(struct platform_device *pdev)
{
	int i;
	clk_disable(pxa988_thermal_dev.therm_clk);
	thermal_zone_device_unregister(pxa988_thermal_dev.therm_cpu);
	for (i = 0; i < TRIP_POINTS_ACTIVE_NUM; i++)
		thermal_cooling_device_unregister(
				pxa988_thermal_dev.cool_dev_active[i]);
	free_irq(pxa988_thermal_dev.irq, NULL);
	pr_info("PXA988: Kernel Thermal management unregistered\n");
	return 0;
}

static struct platform_driver pxa988_thermal_driver = {
	.driver = {
		.name   = "thermal",
		.owner  = THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &thermal_pm_ops,
#endif
	},
	.probe          = pxa988_thermal_probe,
	.remove         = pxa988_thermal_remove,
};

static int __init pxa988_thermal_init(void)
{
	return platform_driver_register(&pxa988_thermal_driver);
}

static void __exit pxa988_thermal_exit(void)
{
	platform_driver_unregister(&pxa988_thermal_driver);
}

module_init(pxa988_thermal_init);
module_exit(pxa988_thermal_exit);

MODULE_AUTHOR("Marvell Semiconductor");
MODULE_DESCRIPTION("PXA988 SoC thermal driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pxa988-thermal");
