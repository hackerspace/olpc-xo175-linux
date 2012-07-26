/*
 * devfreq-vmeta: Generic Dynamic Voltage and Frequency Scaling (DVFS) Framework
 *		  for vMeta Device.
 *
 * Copyright (C) 2010 Marvell International Ltd.
 *	Xiaoguang Chen <chenxg@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/devfreq.h>
#include <linux/clk.h>
#include <plat/devfreq.h>

#ifdef CONFIG_VMETA_DEVFREQ_DEFAULT_GOV_PERFORMANCE
#define DEVFREQ_DEFAULT_GOVERNOR	(&devfreq_performance)
#elif defined(CONFIG_VMETA_DEVFREQ_DEFAULT_GOV_USERSPACE)
#define DEVFREQ_DEFAULT_GOVERNOR	(&devfreq_userspace)
#elif defined(CONFIG_VMETA_DEVFREQ_DEFAULT_GOV_POWERSAVE)
#define DEVFREQ_DEFAULT_GOVERNOR	(&devfreq_powersave)
#elif defined(CONFIG_VMETA_DEVFREQ_DEFAULT_GOV_SIMPLE_ONDEMAND)
#define DEVFREQ_DEFAULT_GOVERNOR	(&devfreq_simple_ondemand)
#endif

#define KHZ_TO_HZ	1000

struct vMeta_devfreq_data {
	struct devfreq *devfreq;
	struct clk *vclk;
};

static int vmeta_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct platform_device *pdev = container_of(dev, struct platform_device,
						    dev);
	struct vMeta_devfreq_data *data = platform_get_drvdata(pdev);
	int ret = 0;

	ret = clk_set_rate(data->vclk, *freq * KHZ_TO_HZ);
	if (!ret)
		*freq = clk_get_rate(data->vclk) / KHZ_TO_HZ;
	return ret;
}

static struct devfreq_dev_profile vMeta_devfreq_profile = {
	.target = vmeta_target,
};

static int vMeta_devfreq_probe(struct platform_device *pdev)
{
	struct vMeta_devfreq_data *data = NULL;
	struct devfreq_platform_data *pdata;
	int err = 0;
	struct device *dev = &pdev->dev;
	pdata = (struct devfreq_platform_data *)dev->platform_data;
	if (!pdata) {
		dev_err(dev, "No platform data!\n");
		goto out;
	}

	data = kzalloc(sizeof(struct vMeta_devfreq_data), GFP_KERNEL);

	if (data == NULL) {
		dev_err(dev, "Cannot allocate memory for vMeta devfreq!\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, data);
	data->vclk = clk_get(NULL, pdata->clk_name);
	if (IS_ERR(data->vclk)) {
		err = PTR_ERR(data->vclk);
		goto out;
	}
	vMeta_devfreq_profile.initial_freq =
			clk_get_rate(data->vclk) / KHZ_TO_HZ;

	data->devfreq = devfreq_add_device(dev, &vMeta_devfreq_profile,
					   DEVFREQ_DEFAULT_GOVERNOR, NULL);
	if (IS_ERR(data->devfreq)) {
		err = PTR_ERR(data->devfreq);
		goto out;
	}
	if (pdata->setup_freq_table)
		pdata->setup_freq_table(data->devfreq);
	else if (pdata->freq_table)
		devfreq_set_freq_table(data->devfreq, pdata->freq_table);

	return 0;
out:
	kfree(data);
	return err;
}

static int vMeta_devfreq_remove(struct platform_device *pdev)
{
	struct vMeta_devfreq_data *data = platform_get_drvdata(pdev);
	devfreq_remove_device(data->devfreq);
	kfree(data);
	return 0;
}

static struct platform_driver vMeta_devfreq_driver = {
	.probe = vMeta_devfreq_probe,
	.remove = vMeta_devfreq_remove,
	.driver = {
		   .name = "devfreq-vMeta",
		   .owner = THIS_MODULE,
		   },
};

static int __init vMeta_devfreq_init(void)
{
	return platform_driver_register(&vMeta_devfreq_driver);
}

static void __init vMeta_devfreq_exit(void)
{
	platform_driver_unregister(&vMeta_devfreq_driver);
}

module_init(vMeta_devfreq_init);
module_exit(vMeta_devfreq_exit);

MODULE_DESCRIPTION("vMeta devfreq device driver");
MODULE_LICENSE("GPL");
