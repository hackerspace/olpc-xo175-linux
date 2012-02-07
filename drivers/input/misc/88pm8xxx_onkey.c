/*
 * 88pm80x_onkey.c - Marvell 88PM80x ONKEY driver
 *
 * Copyright (C) 2009-2010 Marvell International Ltd.
 *      Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/88pm80x.h>
#include <linux/mfd/88pm860x.h>
#include <linux/slab.h>

#define PM800_LONG_ONKEY_EN		(1 << 0)
#define PM800_LONG_KEY_DELAY		(8)	/* 1 .. 16 seconds */
#define PM800_LONKEY_PRESS_TIME		((PM800_LONG_KEY_DELAY-1) << 4)
#define PM800_LONKEY_PRESS_TIME_MASK	(0xF0)
#define PM800_SW_PDOWN			(1 << 5)

#define PM8607_ONKEY_STATUS		(1 << 0)
#define PM8607_LONG_ONKEY_EN		(1 << 1)
#define PM8607_WAKEUP				0x0b
#define PM8607_SW_PDOWN			(1 << 7)

struct pm8xxx_onkey_info {
	struct input_dev	*idev;
	struct pm80x_chip	*chip80x;
	struct pm860x_chip	*chip860x;
	struct i2c_client	*i2c;
	int			irq;
	int 			core_irq;
	int 			pmic_id;
};

static struct pm8xxx_onkey_info *pm8xxx_info;

/* 88PM8xxx poweroff function */
void pm8xxx_system_poweroff(void)
{
	printk(KERN_INFO"turning off power....\n");
	if (pm8xxx_info->pmic_id <= PM8607_CHIP_END)
		pm860x_set_bits(pm8xxx_info->i2c, PM8607_RESET_OUT, PM8607_SW_PDOWN, PM8607_SW_PDOWN);
	else if (pm8xxx_info->pmic_id <= PM800_CHIP_END)
		pm80x_set_bits(pm8xxx_info->i2c, PM800_WAKEUP1, PM800_SW_PDOWN, PM800_SW_PDOWN);
}

/* 88PM8xxx gives us an interrupt when ONKEY is held */
static irqreturn_t pm8xxx_onkey_handler(int irq, void *data)
{
	struct pm8xxx_onkey_info *info = data;
	int ret = 0;
	if (info->pmic_id <= PM8607_CHIP_END) {
		ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
		ret &= PM8607_ONKEY_STATUS;
	} else if (info->pmic_id <= PM800_CHIP_END) {
		ret = pm80x_reg_read(info->i2c, PM800_STATUS_1);
		ret &= PM800_ONKEY_STS1;
	}
	input_report_key(info->idev, KEY_POWER, ret);
	input_sync(info->idev);

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
static int pm8xxx_onkey_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pm8xxx_onkey_info *info = platform_get_drvdata(pdev);
	if (device_may_wakeup(dev)) {
		enable_irq_wake(info->core_irq);
		enable_irq_wake(info->irq);
	}
	return 0;
}

static int pm8xxx_onkey_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pm8xxx_onkey_info *info = platform_get_drvdata(pdev);
	if (device_may_wakeup(dev)) {
		disable_irq_wake(info->core_irq);
		disable_irq_wake(info->irq);
	}
	return 0;
}


static const struct dev_pm_ops pm8xxx_onkey_pm_ops = {
	.suspend	= pm8xxx_onkey_suspend,
	.resume		= pm8xxx_onkey_resume,
};
#endif

static int __devinit pm8xxx_onkey_probe(struct platform_device *pdev)
{

	void *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm8xxx_onkey_info *info;
	int irq, ret;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(struct pm8xxx_onkey_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->pmic_id = get_pmic_version(chip);
	if (info->pmic_id <= PM8607_CHIP_END) {
		info->chip860x = (struct pm860x_chip *) chip;
		info->i2c = (info->chip860x->id == CHIP_PM8607) ? info->chip860x->client : info->chip860x->companion;
		info->irq = irq;
		info->core_irq = info->chip860x->core_irq;
	} else if (info->pmic_id <= PM800_CHIP_END) {
		info->chip80x = (struct pm80x_chip *) chip;
		info->i2c = info->chip80x->base_page;
		info->irq = irq + info->chip80x->pm800_chip->irq_base;
		info->core_irq = info->chip80x->pm800_chip->irq;
	}
	pm8xxx_info = info;
	info->idev = input_allocate_device();
	if (!info->idev) {
		dev_err(&pdev->dev, "Failed to allocate input dev\n");
		ret = -ENOMEM;
		goto out;
	}

	info->idev->name = "88pm8xxx_on";
	info->idev->phys = "88pm8xxx_on/input0";
	info->idev->id.bustype = BUS_I2C;
	info->idev->dev.parent = &pdev->dev;
	info->idev->evbit[0] = BIT_MASK(EV_KEY);
	info->idev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);

	ret = input_register_device(info->idev);
	if (ret) {
		dev_err(&pdev->dev, "Can't register input device: %d\n", ret);
		goto out_reg;
	}

	ret = request_threaded_irq(info->irq, NULL, pm8xxx_onkey_handler,
					IRQF_ONESHOT, "onkey", info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq, ret);
		goto out_irq;
	}

	platform_set_drvdata(pdev, info);
	if (info->pmic_id <= PM8607_CHIP_END) {
		pm860x_set_bits(info->i2c, PM8607_WAKEUP, 3, PM8607_LONG_ONKEY_EN);
	} else if (info->pmic_id <= PM800_CHIP_END) {
		/* Enable long onkey detection */
		pm80x_set_bits(info->i2c, PM800_RTC_MISC4,
			PM800_LONG_ONKEY_EN, PM800_LONG_ONKEY_EN);
		/* Set 8-second interval*/
		pm80x_set_bits(info->i2c, PM800_RTC_MISC3,
			PM800_LONKEY_PRESS_TIME_MASK, PM800_LONKEY_PRESS_TIME);
	}
	device_init_wakeup(&pdev->dev, 1);
	return 0;

out_irq:
	input_unregister_device(info->idev);
	kfree(info);
	return ret;

out_reg:
	input_free_device(info->idev);
out:
	kfree(info);
	return ret;
}

static int __devexit pm8xxx_onkey_remove(struct platform_device *pdev)
{
	struct pm8xxx_onkey_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq, info);
	input_unregister_device(info->idev);
	kfree(info);
	return 0;
}


static struct platform_driver pm8xxx_onkey_driver = {
	.driver		= {
		.name	= "88pm8xxx-onkey",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pm8xxx_onkey_pm_ops,
#endif
	},
	.probe		= pm8xxx_onkey_probe,
	.remove		= __devexit_p(pm8xxx_onkey_remove),
};

static int __init pm8xxx_onkey_init(void)
{
	return platform_driver_register(&pm8xxx_onkey_driver);
}
module_init(pm8xxx_onkey_init);

static void __exit pm8xxx_onkey_exit(void)
{
	platform_driver_unregister(&pm8xxx_onkey_driver);
}
module_exit(pm8xxx_onkey_exit);

MODULE_DESCRIPTION("Marvell 88PM80x ONKEY driver");
MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_LICENSE("GPL");
