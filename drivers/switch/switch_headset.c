/*
 *	drivers/switch/switch_headset.c
 *
 *	headset detect driver for Android
 *
 *	Copyright (C) 2010, Marvell Corporation
 *	Author: Jiangang Jing <jgjing@marvell.com>
 *		Libin Yang <lbyang@marvell.com>
 *	Author: Raul Xiong <xjian@marvell.com>
 *		Mike Lockwood <lockwood@android.com>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *	published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <mach/hardware.h>
#include <mach/cputype.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "switch_headset.h"

static struct headset_switch_data *hs_switch_data;

/*
 * return back >=0, then set the status;
 * return back = -EAGAIN, then get the status from gpio level;
 * others mean there have some error, ignore it.
 */
int (*headset_detect_func)(void) = NULL;

static void headset_switch_work(struct work_struct *work)
{
	int state = 0;
	struct headset_switch_data *switch_data =
		container_of(work, struct headset_switch_data, work);

	if (headset_detect_func) {
		/* HW detection code here */
		state = headset_detect_func();
		if (state >= 0) {
			switch_set_state(&switch_data->sdev, state);
			return;
		} else if (state != -EAGAIN) {
			printk(KERN_INFO "%s: the states is %d\n",
				__func__, state);
			return;
		}
	}

	state = gpio_get_value(switch_data->gpio);
	if (!switch_data->invert)
		state = state ? 2 : 0;
	else
		state = state ? 0 : 2;
	switch_set_state(&switch_data->sdev, state);
	return;
}

static irqreturn_t headset_irq_handler(int irq, void *dev_id)
{
	struct headset_switch_data *switch_data =
	    (struct headset_switch_data *)dev_id;
	int state = 0;

	if (headset_detect_func) {
		/* HW detection code here */
		state = headset_detect_func();
		pr_info("state %d\n", state);
		if (state >= 0) {
			switch_set_state(&switch_data->sdev, state);
			return IRQ_HANDLED;
		} else if (state != -EAGAIN) {
			printk(KERN_INFO "%s: the states is %d\n",
				__func__, state);
			return IRQ_HANDLED;
		}
	}

	state = gpio_get_value(switch_data->gpio);
	if (!switch_data->invert)
		state = state ? 2 : 0;
	else
		state = state ? 0 : 2;
	switch_set_state(&switch_data->sdev, state);
	return IRQ_HANDLED;
}

static ssize_t switch_headset_print_state(struct switch_dev *sdev, char *buf)
{
	struct headset_switch_data	*switch_data =
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

static int headset_switch_probe(struct platform_device *pdev)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct headset_switch_data *switch_data;
	int ret = 0;
	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct headset_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = pdata->name;
	switch_data->gpio = pdata->gpio;
	switch_data->invert = pdata->invert;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_headset_print_state;
	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	INIT_WORK(&switch_data->work, headset_switch_work);

	switch_data->irq = gpio_to_irq(switch_data->gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;

	}

	ret = request_threaded_irq(switch_data->irq, NULL, headset_irq_handler,
				   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				   pdev->name, switch_data);
	if (ret < 0)
		goto err_request_irq;

	headset_switch_work(&switch_data->work);
	hs_switch_data = switch_data;

	return 0;

err_request_irq:
err_detect_irq_num_failed:
	switch_dev_unregister(&switch_data->sdev);

err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int __devexit headset_switch_remove(struct platform_device *pdev)
{
	struct headset_switch_data *switch_data = hs_switch_data;

	cancel_work_sync(&switch_data->work);

	gpio_free(switch_data->gpio);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

#ifdef CONFIG_PM
static int headset_switch_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;

	if (pdata->stay_on)
		return 0;

	switch_set_state(&hs_switch_data->sdev, 0);
	return 0;
}

static int headset_switch_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define headset_switch_suspend	NULL
#define headset_switch_resume	NULL
#endif

static struct platform_driver headset_switch_driver = {
	.probe		= headset_switch_probe,
	.remove		= __devexit_p(headset_switch_remove),
	.driver		= {
		.name	= "headset",
		.owner	= THIS_MODULE,
	},
#ifdef CONFIG_PM
	.suspend	= headset_switch_suspend,
	.resume		= headset_switch_resume,
#endif
};

static int __init headset_switch_init(void)
{
	return platform_driver_register(&headset_switch_driver);
}

static void __exit headset_switch_exit(void)
{
	platform_driver_unregister(&headset_switch_driver);
}

late_initcall(headset_switch_init);
module_exit(headset_switch_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("headset Switch driver");
MODULE_LICENSE("GPL");
