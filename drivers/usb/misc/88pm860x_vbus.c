/*
 * 88pm860x VBus driver for Marvell USB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mfd/88pm860x.h>

struct pm860x_vbus_info {
	struct pm860x_chip	*chip;
	int			irq;
};

static int __devinit pm860x_vbus_probe(struct platform_device *pdev)
{
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_vbus_info *vbus;
	int ret;

	vbus = kzalloc(sizeof(struct pm860x_vbus_info), GFP_KERNEL);
	if (!vbus)
		return -ENOMEM;

	vbus->chip = chip;

	vbus->irq = platform_get_irq(pdev, 0);
	if (vbus->irq < 0) {
		dev_err(&pdev->dev, "failed to get vbus irq\n");
		ret = -ENXIO;
		goto out;
	}

	platform_set_drvdata(pdev, vbus);
	device_init_wakeup(&pdev->dev, 1);

	return 0;

out:
	kfree(vbus);
	return ret;
}

static int __devexit pm860x_vbus_remove(struct platform_device *pdev)
{
	struct pm860x_vbus_info *vbus = platform_get_drvdata(pdev);

	if (vbus) {
		platform_set_drvdata(pdev, NULL);
		kfree(vbus);
	}

	return 0;
}

#ifdef CONFIG_PM
static int pm860x_vbus_suspend(struct device *dev)
{
	struct pm860x_vbus_info *vbus = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		enable_irq_wake(vbus->chip->core_irq);
		enable_irq_wake(vbus->irq);
	}

	return 0;
}

static int pm860x_vbus_resume(struct device *dev)
{
	struct pm860x_vbus_info *vbus = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		disable_irq_wake(vbus->chip->core_irq);
		disable_irq_wake(vbus->irq);
	}

	return 0;
}

static const struct dev_pm_ops pm860x_vbus_pm_ops = {
	.suspend	= pm860x_vbus_suspend,
	.resume		= pm860x_vbus_resume,
};
#endif

static struct platform_driver pm860x_vbus_driver = {
	.driver		= {
		.name	= "88pm860x-vbus",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pm860x_vbus_pm_ops,
#endif
	},
	.probe		= pm860x_vbus_probe,
	.remove		= __devexit_p(pm860x_vbus_remove),
};

static int __init pm860x_vbus_init(void)
{
	return platform_driver_register(&pm860x_vbus_driver);
}
module_init(pm860x_vbus_init);

static void __exit pm860x_vbus_exit(void)
{
	platform_driver_unregister(&pm860x_vbus_driver);
}
module_exit(pm860x_vbus_exit);

MODULE_DESCRIPTION("VBUS driver for Marvell Semiconductor 88PM860x");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:88pm860x-vbus");
