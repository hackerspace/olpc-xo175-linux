/*
 *	drivers/switch/88pm805_headset.c
 *
 *	headset & hook detect driver for pm805
 *
 *	Copyright (C) 2010, Marvell Corporation (wzch@Marvell.com)
 *	Author: Wenzeng Chen <wzch@marvell.com>
 *				 Mike Lockwood <lockwood@android.com>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *	published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/88pm80x.h>
#include <linux/mfd/88pm8xxx-headset.h>
#include <linux/init.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
struct pm80x_headset_info {
	struct pm80x_chip *chip;
	struct device *dev;
	struct i2c_client *i2c;
	int irq_headset;
	int irq_hook;
	int status;
	int gpio_num;
	struct work_struct work_headset, work_hook;
	struct delayed_work delayed_work;
	struct headset_switch_data *psw_data_headset, *psw_data_hook;
};
struct headset_switch_data {
	struct switch_dev sdev;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	int state;
};
static struct device *hsdetect_dev;
static struct pm80x_headset_info *pm80x_info;
static struct PM8XXX_HS_IOCTL hs_detect;

void pm80x_headphone_handler(int status)
{
	if (!pm80x_info)
		return;

	if (pm80x_info->gpio_num == 3) {
		if (!pm80x_info || !pm80x_info->psw_data_headset
			|| !pm80x_info->chip
			|| !pm80x_info->chip->pm805_wqueue)
			return;

		pm80x_info->status = status;
		queue_work(pm80x_info->chip->pm805_wqueue,
			&pm80x_info->work_headset);
	} else {
		if (!pm80x_info || !pm80x_info->psw_data_headset
			|| !pm80x_info->chip
			|| !pm80x_info->chip->pm800_wqueue)
			return;

		pm80x_info->status = status;
		queue_work(pm80x_info->chip->pm800_wqueue,
			&pm80x_info->work_headset);
	}
}

static irqreturn_t pm80x_hook_handler(int irq, void *data)
{
	struct pm80x_headset_info *info = data;

	/* hook interrupt */
	if (info->psw_data_hook != NULL) {
		if (info->gpio_num == 3)
			queue_work(info->chip->pm805_wqueue,
				   &info->work_hook);
		else
			queue_work(info->chip->pm800_wqueue,
				   &info->work_hook);

	}
	return IRQ_HANDLED;
}

#define PM805_STATUS_HEADSET		(1 << 0)
#define PM805_STATUS_HOOK			(1 << 1)
#define PM805_POWER_STANBY			(1 << 0)
#define PM805_MIC_AUTO_DET			(1 << 0)
#define PM805_EXTMIC_BIAS			(1 << 6)
#define PM805_MIC_TIM				(3 << 0)
static void pm80x_headset_switch_work(struct work_struct *work)
{
	struct pm80x_headset_info *info =
	    container_of(work, struct pm80x_headset_info, work_headset);
	struct headset_switch_data *switch_data;
	unsigned char value;
	if (info == NULL) {
		pr_debug("Invalid headset info!\n");
		return;
	}
	switch_data = info->psw_data_headset;
	if (switch_data == NULL) {
		pr_debug("Invalid headset switch data!\n");
		return;
	}
	/* headset detected */
	if (info->status) {
		switch_data->state = PM8XXX_HEADSET_ADD;
				/* for telephony */
		kobject_uevent(&hsdetect_dev->kobj, KOBJ_ADD);
		hs_detect.hsdetect_status = PM8XXX_HEADSET_ADD;
		 /*enalbe mic auto detect */
		 pm80x_set_bits(info->i2c, PM805_MIC_DET1, PM805_MIC_AUTO_DET, PM805_MIC_AUTO_DET);
		 pm80x_set_bits(info->i2c, PM805_DWS_SETTING, PM805_EXTMIC_BIAS, PM805_EXTMIC_BIAS);
		msleep(500);
		value = (unsigned char)pm80x_reg_read(info->i2c, PM805_MIC_DET_STATUS1);

		/* mic detected, now in 88pm805 there is a problem, when Mic detected,
		 * PM805_MIC_DET_STATUS1 read as 0x1; Mic not detected,
		 * PM805_MIC_DET_STATUS1 read as 0xf;
		 */
		if (value == PM805_STATUS_HEADSET) {
			switch_data->state = PM8XXX_HEADSET_ADD;
			/*enable short button interrupt*/
			pm80x_set_bits(info->i2c, PM805_INT_MASK2, PM805_SHRT_BTN_DET, PM805_SHRT_BTN_DET);
		} else {
			switch_data->state = PM8XXX_HEADPHONE_ADD;
			hs_detect.hsmic_status = PM8XXX_HS_MIC_REMOVE;
			pm80x_set_bits(info->i2c, PM805_MIC_DET1, PM805_MIC_AUTO_DET, 0);
			pm80x_set_bits(info->i2c, PM805_DWS_SETTING, PM805_EXTMIC_BIAS, 0);
		}

	} else {
		/* disable mic bias */
		switch_data->state = PM8XXX_HEADSET_REMOVE;
		pm80x_set_bits(info->i2c, PM805_MIC_DET1, PM805_MIC_AUTO_DET, 0);
		pm80x_set_bits(info->i2c, PM805_INT_MASK2, PM805_SHRT_BTN_DET, 0);
		/*for telephony */
		kobject_uevent(&hsdetect_dev->kobj, KOBJ_REMOVE);
		hs_detect.hsdetect_status = PM8XXX_HEADSET_REMOVE;
		hs_detect.hsmic_status = PM8XXX_HS_MIC_ADD;
	}
	pr_info("headset_switch_work to %d\n", switch_data->state);
	switch_set_state(&switch_data->sdev, switch_data->state);
}

static void pm80x_hook_switch_work(struct work_struct *work)
{
	struct pm80x_headset_info *info =
	    container_of(work, struct pm80x_headset_info, work_hook);
	struct headset_switch_data *switch_data;
	unsigned char value;
	if (info == NULL) {
		pr_debug("Invalid headset info!\n");
		return;
	}
	switch_data = info->psw_data_hook;
	if (switch_data == NULL) {
		pr_debug("Invalid hook switch data!\n");
		return;
	}
	value = (unsigned char)pm80x_reg_read(info->i2c, PM805_MIC_DET_STATUS1);
	/* check whether it's hardware jitter during headset unplug */
	if (!(value & PM805_MIC_DET_STATUS1)) {
		pr_info("fake hook interrupt\n");
		return;
	}
	value &= PM805_STATUS_HOOK;
	/* hook pressed */
	if (value) {
		switch_data->state = PM8XXX_HOOKSWITCH_PRESSED;
		/*for telephony */
		kobject_uevent(&hsdetect_dev->kobj, KOBJ_ONLINE);
		hs_detect.hookswitch_status = PM8XXX_HOOKSWITCH_PRESSED;
	} else {
		/* hook released */
		switch_data->state = PM8XXX_HOOKSWITCH_RELEASED;
		/*for telephony */
		kobject_uevent(&hsdetect_dev->kobj, KOBJ_OFFLINE);
		hs_detect.hookswitch_status = PM8XXX_HOOKSWITCH_RELEASED;
	}
	pr_info("hook state switch to %d\n", switch_data->state);
	switch_set_state(&switch_data->sdev, switch_data->state);
}

static ssize_t switch_headset_print_state(struct switch_dev *sdev,
					    char *buf)
{
	struct headset_switch_data *switch_data =
	    container_of(sdev, struct headset_switch_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;
	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static int pm80x_headset_switch_probe(struct platform_device *pdev)
{
	struct pm80x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm80x_platform_data *pm80x_pdata;
	struct pm80x_headset_info *info;
	struct gpio_switch_platform_data *pdata_headset =
	    pdev->dev.platform_data;
	struct gpio_switch_platform_data *pdata_hook = pdata_headset + 1;
	struct headset_switch_data *switch_data_headset =
	    NULL, *switch_data_hook = NULL;
	int irq_headset, irq_hook, ret = 0;

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
	if (pdata_headset == NULL || pdata_hook == NULL) {
		pr_debug("Invalid gpio switch platform data!\n");
		return -EBUSY;
	}
	irq_headset = platform_get_irq(pdev, 0);
	if (irq_headset < 0) {
		dev_err(&pdev->dev, "No IRQ resource for headset!\n");
		return -EINVAL;
	}
	irq_hook = platform_get_irq(pdev, 1);
	if (irq_hook < 0) {
		dev_err(&pdev->dev, "No IRQ resource for hook!\n");
		return -EINVAL;
	}
	info = kzalloc(sizeof(struct pm80x_headset_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->chip = chip;
	info->gpio_num = pm80x_pdata->headset->gpio;
	info->i2c = chip->pm805_chip->client;
	info->dev = &pdev->dev;
	info->irq_headset = irq_headset + chip->pm805_chip->irq_base;
	info->irq_hook = irq_hook + chip->pm805_chip->irq_base;
	switch_data_headset =
	    kzalloc(sizeof(struct headset_switch_data), GFP_KERNEL);
	if (!switch_data_headset)
		return -ENOMEM;
	switch_data_hook =
	    kzalloc(sizeof(struct headset_switch_data), GFP_KERNEL);
	if (!switch_data_hook)
		return -ENOMEM;
	switch_data_headset->sdev.name = pdata_headset->name;
	switch_data_headset->name_on = pdata_headset->name_on;
	switch_data_headset->name_off = pdata_headset->name_off;
	switch_data_headset->state_on = pdata_headset->state_on;
	switch_data_headset->state_off = pdata_headset->state_off;
	switch_data_headset->sdev.print_state = switch_headset_print_state;
	info->psw_data_headset = switch_data_headset;
	switch_data_hook->sdev.name = pdata_hook->name;
	switch_data_hook->name_on = pdata_hook->name_on;
	switch_data_hook->name_off = pdata_hook->name_off;
	switch_data_hook->state_on = pdata_hook->state_on;
	switch_data_hook->state_off = pdata_hook->state_off;
	switch_data_hook->sdev.print_state = switch_headset_print_state;
	info->psw_data_hook = switch_data_hook;
	ret = switch_dev_register(&switch_data_headset->sdev);
	if (ret < 0)
		goto err_switch_dev_register;
	ret = switch_dev_register(&switch_data_hook->sdev);
	if (ret < 0)
		goto err_switch_dev_register;
	ret =
	    request_threaded_irq(info->irq_hook, NULL, pm80x_hook_handler,
				 IRQF_ONESHOT, "hook", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			 info->irq_hook, ret);
		goto out_irq_hook;
	}
	platform_set_drvdata(pdev, info);
	INIT_WORK(&info->work_headset, pm80x_headset_switch_work);
	INIT_WORK(&info->work_hook, pm80x_hook_switch_work);
	hsdetect_dev = &pdev->dev;
	hs_detect.hsdetect_status = 0;
	hs_detect.hookswitch_status = 0;

	pm80x_info = info;

	/*default 4_POLES */
	hs_detect.hsmic_status = PM8XXX_HS_MIC_ADD;

	/* Perform initial detection */
	/*enable elba power */
	pm80x_set_bits(info->i2c, PM805_MAIN_POWERUP, PM805_POWER_STANBY,
					PM805_POWER_STANBY);

	/* set polling time */
	pm80x_set_bits(info->i2c, PM805_MIC_DET2, PM805_MIC_TIM, PM805_MIC_TIM);
	return 0;
err_switch_dev_register:
	kfree(switch_data_headset);
	kfree(switch_data_hook);
out_irq_hook:
	free_irq(info->irq_headset, info);
	kfree(info);
	return ret;
}

static int __devexit pm80x_headset_switch_remove(struct platform_device *pdev)

{
	struct pm80x_headset_info *info = platform_get_drvdata(pdev);
	struct headset_switch_data *switch_data_headset =
	    info->psw_data_headset;
	struct headset_switch_data *switch_data_hook = info->psw_data_hook;
	/* disable headset detection */
	pm80x_set_bits(info->i2c, PM805_MIC_DET1, PM805_MIC_AUTO_DET, 0);
	cancel_work_sync(&info->work_headset);
	cancel_work_sync(&info->work_hook);
	free_irq(info->irq_hook, info);
	free_irq(info->irq_headset, info);
	switch_dev_unregister(&switch_data_hook->sdev);
	switch_dev_unregister(&switch_data_headset->sdev);
	kfree(switch_data_hook);
	kfree(switch_data_headset);
	kfree(info);
	return 0;
}

static int pm80x_headset_switch_suspend(struct platform_device *pdev,
					  pm_message_t state)
{
	return 0;
}

static int pm80x_headset_switch_resume(struct platform_device *pdev)
{
	return 0;
}

static long pm80x_hsdetect_ioctl(struct file *file, unsigned int cmd,
				   unsigned long arg)
{
	struct PM8XXX_HS_IOCTL hs_ioctl;
	if (copy_from_user(&hs_ioctl,
			     (void *)arg, sizeof(struct PM8XXX_HS_IOCTL)))
		return -EFAULT;
	switch (cmd) {
	case PM8XXX_HSDETECT_STATUS:
		hs_ioctl.hsdetect_status = hs_detect.hsdetect_status;
		hs_ioctl.hookswitch_status = hs_detect.hookswitch_status;

#if defined(ENABLE_HS_DETECT_POLES)
		    hs_ioctl.hsmic_status = hs_detect.hsmic_status;

#endif /* */
		pr_info("hsdetect_ioctl PM8XXX_HSDETECT_STATUS\n");
		break;
	case PM8XXX_HOOKSWITCH_STATUS:
		hs_ioctl.hookswitch_status = hs_detect.hookswitch_status;
		hs_ioctl.hsdetect_status = hs_detect.hsdetect_status;

#if defined(ENABLE_HS_DETECT_POLES)
		    hs_ioctl.hsmic_status = hs_detect.hsmic_status;

#endif /* */
		pr_info("hsdetect_ioctl PM860X_HOOKSWITCH_STATUS\n");
		break;
	default:
		return -ENOTTY;
	}
	return copy_to_user((void *)arg, &hs_ioctl,
			     sizeof(struct PM8XXX_HS_IOCTL));
}

static struct platform_driver pm80x_headset_switch_driver = {
	.probe = pm80x_headset_switch_probe,
	.remove = __devexit_p(pm80x_headset_switch_remove),
	.suspend = pm80x_headset_switch_suspend,
	.resume = pm80x_headset_switch_resume,
	.driver = {
			.name = "88pm80x-headset",
			.owner = THIS_MODULE,
		},
};

static const struct file_operations pm80x_hsdetect_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = pm80x_hsdetect_ioctl,
};

static struct miscdevice pm80x_hsdetect_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "88pm80x-headset",
	.fops = &pm80x_hsdetect_fops,
};

static int __init headset_switch_init(void)
{
	int ret = -EINVAL;
	ret = misc_register(&pm80x_hsdetect_miscdev);
	if (ret < 0)
		return ret;
	ret = platform_driver_register(&pm80x_headset_switch_driver);
	return ret;
}

module_init(headset_switch_init);
static void __exit headset_switch_exit(void)
{
	platform_driver_unregister(&pm80x_headset_switch_driver);
	misc_deregister(&pm80x_hsdetect_miscdev);
}

module_exit(headset_switch_exit);

MODULE_DESCRIPTION("Marvell 88PM805 Headset driver");
MODULE_AUTHOR("Wenzeng Chen <wzch@marvell.com>");
MODULE_LICENSE("GPL");
