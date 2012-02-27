/*
 * 88pm80x VBus driver for Marvell USB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mfd/88pm80x.h>
#include <plat/usb.h>
#include <linux/delay.h>

#define STATUS2_VBUS        (1 << 4)
#define GPADC2_MEAS1		0x79
#define GPADC2_MEAS2		0x7A
#define GPADC2_LOW_TH		0x61
#define GPADC2_UPP_TH		0x69
#define MEAS_ENABLE1		0x50
#define MEAS_GP2_EN		(1 << 6)
#define GPADC_MISC1		0x57
#define GPFSM_EN		(1 << 0)
#define MISC1_GPIO1_DIR		(1 << 3)
#define MISC1_GPIO1_VAL		(1 << 4)
#define MISC1_GPIO2_DIR		(1 << 5)
#define MISC1_GPIO2_VAL		(1 << 6)

struct pm80x_vbus_info {
	struct pm80x_chip	*chip;
	int			irq;
};

int pm80x_read_vbus_val(void)
{
	int ret;
	ret = pm80x_codec_reg_read(
		(PM80X_BASE_PAGE << 8) | PM800_STATUS_1);
	if (ret & (1 << 2))
		ret = VBUS_HIGH;
	else
		ret = VBUS_LOW;
	return ret;
}
EXPORT_SYMBOL(pm80x_read_vbus_val);

int pm80x_read_id_val(void)
{
	int ret, data;
	ret = pm80x_codec_reg_read(
		(PM80X_GPADC_PAGE << 8) | PM800_GPADC2_MEAS1);
	data = ret << 4;
	ret = pm80x_codec_reg_read(
		(PM80X_GPADC_PAGE << 8) | PM800_GPADC2_MEAS2);
	data |= ret & 0x0F;
	if (data > 0x10) {
		ret = 1;
		pm80x_codec_reg_write(
			(PM80X_GPADC_PAGE << 8) | PM800_GPADC2_LOW_TH,
			0x10);
		pm80x_codec_reg_write(
			(PM80X_GPADC_PAGE << 8) | PM800_GPADC2_UPP_TH,
			0xff);
	} else {
		ret = 0;
		pm80x_codec_reg_write(
			(PM80X_GPADC_PAGE << 8) | PM800_GPADC2_LOW_TH,
			0);
		pm80x_codec_reg_write(
			(PM80X_GPADC_PAGE << 8) | PM800_GPADC2_UPP_TH,
			0x10);
	}

	return ret;
};
EXPORT_SYMBOL(pm80x_read_id_val);

void pm80x_init_id(void)
{
	pm80x_codec_reg_set_bits(
		(PM80X_GPADC_PAGE << 8) | PM800_GPADC_MEAS_EN2,
		PM800_MEAS_GP2_EN,
		PM800_MEAS_GP2_EN);
	pm80x_codec_reg_set_bits(
		(PM80X_GPADC_PAGE << 8) | PM800_GPADC_MISC_CONFIG2,
		PM800_GPADC_MISC_GPFSM_EN,
		PM800_GPADC_MISC_GPFSM_EN);
}
EXPORT_SYMBOL(pm80x_init_id);

int pm80x_set_vbus(unsigned int vbus)
{
	unsigned int data = 0, mask;

	mask = 	PM800_GPIO3_GPIO_MODE(0x01) | PM800_GPIO3_VAL;

	if (vbus == VBUS_HIGH)
		data = mask;

	pm80x_codec_reg_set_bits(
		(PM80X_BASE_PAGE << 8) | PM800_GPIO_2_3_CNTRL,
		mask, data);

	mdelay(20);

	if (pm80x_read_vbus_val() != vbus)
		pr_info("vbus set failed %x\n", vbus);
	else
		pr_info("vbus set done %x\n", vbus);

	return 0;
}
EXPORT_SYMBOL(pm80x_set_vbus);

static int __devinit pm80x_vbus_probe(struct platform_device *pdev)
{
	struct pm80x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm80x_vbus_info *vbus;
	int ret;

	vbus = kzalloc(sizeof(struct pm80x_vbus_info), GFP_KERNEL);
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

static int __devexit pm80x_vbus_remove(struct platform_device *pdev)
{
	struct pm80x_vbus_info *vbus = platform_get_drvdata(pdev);

	if (vbus) {
		platform_set_drvdata(pdev, NULL);
		kfree(vbus);
	}

	return 0;
}

#ifdef CONFIG_PM
static int pm80x_vbus_suspend(struct device *dev)
{
	struct pm80x_vbus_info *vbus = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		enable_irq_wake(vbus->chip->pm800_chip->irq);
		enable_irq_wake(vbus->irq);
	}

	return 0;
}

static int pm80x_vbus_resume(struct device *dev)
{
	struct pm80x_vbus_info *vbus = dev_get_drvdata(dev);

	if (device_may_wakeup(dev)) {
		disable_irq_wake(vbus->chip->pm800_chip->irq);
		disable_irq_wake(vbus->irq);
	}

	return 0;
}

static const struct dev_pm_ops pm80x_vbus_pm_ops = {
	.suspend	= pm80x_vbus_suspend,
	.resume		= pm80x_vbus_resume,
};
#endif

static struct platform_driver pm80x_vbus_driver = {
	.driver		= {
		.name	= "88pm80x-vbus",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pm80x_vbus_pm_ops,
#endif
	},
	.probe		= pm80x_vbus_probe,
	.remove		= __devexit_p(pm80x_vbus_remove),
};

static int __init pm80x_vbus_init(void)
{
	return platform_driver_register(&pm80x_vbus_driver);
}
module_init(pm80x_vbus_init);

static void __exit pm80x_vbus_exit(void)
{
	platform_driver_unregister(&pm80x_vbus_driver);
}
module_exit(pm80x_vbus_exit);

MODULE_DESCRIPTION("VBUS driver for Marvell Semiconductor 88PM80x");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:88pm80x-vbus");
