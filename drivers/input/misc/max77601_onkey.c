/*
 * Max77601 onkey driver
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77601.h>
#include <linux/slab.h>
#include <linux/delay.h>

struct max77601_onkey_info {
	struct input_dev	*idev;
	struct i2c_client	*i2c;
	struct device		*dev;
	struct max77601_chip *chip;
	int irq;
};

static struct max77601_onkey_info *g_info;

void max77601_system_restart(void)
{
	int ret = 0;
	if (!g_info || !g_info->chip)
		return;
	pr_err("%s: rebooting...\n", __func__);
	/* 1. Enable SW reset wake up */
	ret = max77601_set_bits(g_info->chip, MAX77601_ONOFFCNFG2,
				MAX77601_SFT_RST_WK, MAX77601_SFT_RST_WK);
	if (ret)
		goto out;
	/* 2. Issue SW reset */
	ret = max77601_set_bits(g_info->chip, MAX77601_ONOFFCNFG1,
				MAX77601_SFT_RST, MAX77601_SFT_RST);
out:
	if (ret)
		pr_err("%s: Failed to reboot!\n", __func__);
	mdelay(500);
}

void max77601_system_poweroff(void)
{
	int ret;
	if (!g_info || !g_info->chip)
		return;
	pr_err("%s: powering off...\n", __func__);
	/* 1. Disable SW reset wake up */
	ret = max77601_set_bits(g_info->chip, MAX77601_ONOFFCNFG2,
				MAX77601_SFT_RST_WK, 0);
	if (ret)
		goto out;
	/* 2. Issue Power down */
	ret = max77601_set_bits(g_info->chip, MAX77601_ONOFFCNFG1,
				MAX77601_SFT_RST, MAX77601_SFT_RST);
out:
	if (ret)
		pr_err("%s: Failed to power off!\n", __func__);
	mdelay(500);
}

/*
 * MAX77601 gives us an interrupt when ONKEY(EN0) is pressed or released.
 * max77601_set_bits() operates I2C bus and may sleep. So implement
 * it in thread IRQ handler.
 */
static irqreturn_t max77601_onkey_handler(int irq, void *data)
{
	struct max77601_onkey_info *info = data;
	u8 status;
	int ret;

	ret = max77601_read(info->chip, MAX77601_ONOFFIRQ_REG, &status, 1);
	if (ret < 0)
		return IRQ_NONE;

	/* Event: 0 - up, 1- down */
	if (status & MAX77601_ONOFFIRQ_EN0_RM) {
		input_report_key(info->idev, KEY_POWER, 1);
		input_sync(info->idev);
		dev_dbg(info->dev, "Onkey: DOWN\n");
	}
	if (status & MAX77601_ONOFFIRQ_EN0_FM) {
		input_report_key(info->idev, KEY_POWER, 0);
		input_sync(info->idev);
		dev_dbg(info->dev, "Onkey: UP\n");
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
static int max77601_onkey_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max77601_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct max77601_onkey_info *info = platform_get_drvdata(pdev);

	if (device_may_wakeup(dev)) {
		enable_irq_wake(chip->core_irq);
		enable_irq_wake(info->irq);
	}

	return 0;
}

static int max77601_onkey_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct max77601_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct max77601_onkey_info *info = platform_get_drvdata(pdev);

	if (device_may_wakeup(dev)) {
		disable_irq_wake(chip->core_irq);
		disable_irq_wake(info->irq);
	}

	return 0;
}

static const struct dev_pm_ops max77601_onkey_pm_ops = {
	.suspend	= max77601_onkey_suspend,
	.resume		= max77601_onkey_resume,
};
#endif

static int __devinit max77601_onkey_probe(struct platform_device *pdev)
{
	struct max77601_chip *chip;
	struct max77601_onkey_info *info;
	int irq, error;
	u8 status, mask = MAX77601_ONOFFIRQ_EN0_RM | MAX77601_ONOFFIRQ_EN0_FM;

	if (!pdev)
		return -EINVAL;

	chip = dev_get_drvdata(pdev->dev.parent);
	if (!chip) {
		dev_err(&pdev->dev, "Unvalid chip data\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "No IRQ resource 0\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(struct max77601_onkey_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->i2c = chip->i2c;
	info->dev = &pdev->dev;
	irq += chip->irq_base;
	info->chip = chip;
	g_info = info;

	/* Mask EN0 Rising , falling edge interrupt */
	max77601_set_bits(chip, MAX77601_ONOFFIRQM_REG, mask, mask);
	/* Clear pending interrupt */
	max77601_read(chip, MAX77601_ONOFFIRQ_REG, &status, 1);
	/* Request IRQ */
	error = request_threaded_irq(irq, NULL, max77601_onkey_handler,
				     IRQF_ONESHOT, "onkey", info);
	if (error < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			irq, error);
		goto out;
	}

	info->idev = input_allocate_device();
	if (!info->idev) {
		dev_err(chip->dev, "Failed to allocate input dev\n");
		error = -ENOMEM;
		goto out_irq;
	}

	info->idev->name = "max77601_on";
	info->idev->phys = "max77601_on/input0";
	info->idev->id.bustype = BUS_I2C;
	info->idev->dev.parent = &pdev->dev;
	info->irq = irq;
	info->idev->evbit[0] = BIT_MASK(EV_KEY);
	info->idev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);

	error = input_register_device(info->idev);
	if (error) {
		dev_err(chip->dev, "Can't register input device: %d\n", error);
		goto out_reg;
	}

	platform_set_drvdata(pdev, info);
	device_init_wakeup(&pdev->dev, 1);

	/* Unmask EN0 Rising , falling edge interrupt */
	max77601_set_bits(chip, MAX77601_ONOFFIRQM_REG, mask, 0x0);

	return 0;

out_reg:
	input_free_device(info->idev);

out_irq:
	free_irq(info->irq, info);
out:
	kfree(info);
	return error;
}

static int __devexit max77601_onkey_remove(struct platform_device *pdev)
{
	struct max77601_onkey_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq, info);
	input_unregister_device(info->idev);
	kfree(info);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver max77601_onkey_driver = {
	.driver		= {
		.name	= "max77601-onkey",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &max77601_onkey_pm_ops,
#endif
	},
	.probe		= max77601_onkey_probe,
	.remove		= __devexit_p(max77601_onkey_remove),
};

static int __init max77601_onkey_init(void)
{
	return platform_driver_register(&max77601_onkey_driver);
}
module_init(max77601_onkey_init);

static void __exit max77601_onkey_exit(void)
{
	platform_driver_unregister(&max77601_onkey_driver);
}
module_exit(max77601_onkey_exit);


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MAX77601 Onkey Driver");
MODULE_VERSION("1.0");
