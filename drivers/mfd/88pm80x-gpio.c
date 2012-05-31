#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/88pm80x.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <mach/mfp.h>
#include <mach/gpio.h>

struct pm80x_gpio_info {
	struct pm80x_chip	*chip;
	struct device 		*dev;
	struct i2c_client		*i2c;
	struct delayed_work	interrupt_work;
	int				interval;
	int 				irq_gpio00;
	int 				irq_gpio01;
	int 				irq_gpio02;
	int 				irq_gpio03;
	int 				irq_gpio04;
	int				gpio04_flag;
	int				headset_gpio;
	int				headset_gpio_val;
};
#define GPIO4_MODE 		(0x08)

#ifdef CONFIG_SWITCH_88PM80X_HEADSET
extern void pm80x_headphone_handler(int status);
#else
static void pm80x_headphone_handler(int status) {}
#endif

static void pm80x_gpio04_work(struct work_struct *work)
{
	struct pm80x_gpio_info *info =
	    container_of(work, struct pm80x_gpio_info,
			 interrupt_work.work);
	int value = 0, mask = 0;
	mask = info->headset_gpio_val;
	value = pm80x_reg_read(info->i2c, info->headset_gpio);

	if (info->gpio04_flag)
		pm80x_headphone_handler(value & mask);
	else
		pm80x_headphone_handler(~value & mask);
}

static irqreturn_t pm80x_gpio04_handler(int irq, void *data)
{
	struct pm80x_gpio_info *info = data;
	cancel_delayed_work_sync(&info->interrupt_work);
	schedule_delayed_work(&info->interrupt_work, info->interval);
	return IRQ_HANDLED;
}
static int pm80x_gpio_probe(struct platform_device *pdev)
{
	struct pm80x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm80x_gpio_info *info;
	struct pm80x_platform_data *pm80x_pdata;
	int irq_gpio04;
	int ret;
	int val;

	if (pdev->dev.parent->platform_data) {
		pm80x_pdata = pdev->dev.parent->platform_data;
	} else {
		pr_debug("Invalid pm80x platform data!\n");
		return -EINVAL;
	}

	if (!pm80x_pdata->headset) {
		dev_err(&pdev->dev, "No headset platform info!\n");
		return -EINVAL;
	}
	irq_gpio04 = platform_get_irq(pdev, pm80x_pdata->headset->gpio);

	if (irq_gpio04 < 0) {
		dev_err(&pdev->dev, "No IRQ resource for gpio04!\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(struct pm80x_gpio_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->chip = chip;
	info->dev = &pdev->dev;
	info->irq_gpio04 = irq_gpio04;
	info->i2c = chip->base_page;
	info->gpio04_flag = pm80x_pdata->headset_flag;
	info->headset_gpio = pm80x_pdata->headset->gpio_ctl;
	info->headset_gpio_val = pm80x_pdata->headset->gpio_val_bit;
	ret = request_threaded_irq(info->irq_gpio04, NULL, pm80x_gpio04_handler,
					IRQF_ONESHOT, "gpio-04", info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq_gpio04, ret);
	}
	platform_set_drvdata(pdev, info);
	info->interval = HZ/8;
	INIT_DELAYED_WORK(&info->interrupt_work, pm80x_gpio04_work);
	/*set gpio04 as input mode. */
	pm80x_set_bits(info->i2c, PM800_INT_ENA_4,
			pm80x_pdata->headset->gpio_enable_irq,
			pm80x_pdata->headset->gpio_enable_irq);
	val = pm80x_reg_read(info->i2c, pm80x_pdata->headset->gpio_ctl);
	val &= pm80x_pdata->headset->gpio_set_mask;
	val |= pm80x_pdata->headset->gpio_set_val;
	pm80x_reg_write(info->i2c, pm80x_pdata->headset->gpio_ctl , val);
	schedule_delayed_work(&info->interrupt_work, info->interval);
	return 0;
}

static int pm80x_gpio_remove(struct platform_device *pdev)
{
	struct pm80x_gpio_info *info = platform_get_drvdata(pdev);
	cancel_delayed_work_sync(&info->interrupt_work);
	free_irq(info->irq_gpio04, info);
	kfree(info);
	return 0;
}

static int pm80x_gpio_resume(struct platform_device *pdev)
{
	return 0;
}

static int pm80x_gpio_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static struct platform_driver pm80x_gpio_driver = {
	.probe 	= pm80x_gpio_probe,
	.remove 	= __devexit_p(pm80x_gpio_remove),
	.suspend	= pm80x_gpio_suspend,
	.resume 	= pm80x_gpio_resume,
	.driver	= {
		.name	= "88pm80x-gpio",
		.owner	= THIS_MODULE,
	},
};

static struct miscdevice pm80x_gpio_miscdev = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= "88pm80x_gpio"
};

static int pm80x_gpio_init(void)
{
	int ret = -EINVAL;
	ret = misc_register(&pm80x_gpio_miscdev);
	if (ret < 0)
		return ret;
	ret = platform_driver_register(&pm80x_gpio_driver);
	return ret;
}
late_initcall(pm80x_gpio_init);

static void __exit pm80x_gpio_exit(void)
{
	platform_driver_unregister(&pm80x_gpio_driver);
	misc_deregister(&pm80x_gpio_miscdev);
}
module_exit(pm80x_gpio_exit);
