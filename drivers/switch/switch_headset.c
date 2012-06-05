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
#include <linux/switch_headset.h>

struct headset_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	int invert;
	const char *state_on;
	const char *state_off;
	int irq;
	struct workqueue_struct *hs_workq;
	struct delayed_work det_work;
};

enum {
	HS_REMOVE = 0x0,	/* HS is removal */
	HS_INSERT = 0x1,	/* HS(with Mic) is inserted */
	HP_INSERT = 0x2,	/* HP(without Mic) is inserted */
};

int (*headset_detect_func)(void) = NULL;
int (*get_mic_state)(void) = NULL;

static void print_hs_state(int old, int new)
{
	char *str = "UNKNOW";

	if (new == HS_INSERT)
		str = "Headset(Mic) Inserted";
	else if (new == HP_INSERT)
		str = "Headphone Inserted";
	else if ((old == HS_INSERT) && (new == HS_REMOVE))
		str = "Headset(Mic) Removed";
	else if ((old == HP_INSERT) && (new == HS_REMOVE))
		str = "Headphone Removed";

	pr_info("%s: state=%d\n", str, new);
}

static void headset_det_work_func(struct work_struct *work)
{
	struct headset_switch_data *switch_data =
		container_of(work, struct headset_switch_data, det_work.work);
	static int hs_state = HS_REMOVE;
	int state, gpio, hs_det = 0, mic = 0;

	/* HS detection is totally handled by WM8994 */
	if (headset_detect_func) {
		/* HW detection code here */
		state = headset_detect_func();
		if (state >= 0 && state != hs_state) {
			pr_info("Headset State=%d\n", state);
			hs_state = state;
			switch_set_state(&switch_data->sdev, state);
		}
		return;
	} else {
		/* HS insert is detected by GPIO;
		 * MIC is detected by WM8994 */
		/* GPIO: 0: plugged in; 1: plugged out */
		gpio = !!gpio_get_value(switch_data->gpio);
		/* hs_det: 1=>insert; 0=>remove */
		hs_det = switch_data->invert ? gpio : !gpio;
		state = HP_INSERT;
		if (hs_det) {
			/* If HS inserted, get Mic state */
			if (get_mic_state)
				mic = get_mic_state();
			/* If there is Mic, Headset(Mic) inserted */
			if (mic)
				state = HS_INSERT;
		} else
			state = HS_REMOVE;
	}
	/* If state is changed, report switch event */
	if (state != hs_state) {
		print_hs_state(hs_state, state);
		hs_state = state;
		switch_set_state(&switch_data->sdev, state);
	}
	return;
}

static irqreturn_t headset_irq_handler(int irq, void *dev_id)
{
	struct headset_switch_data *switch_data =
			(struct headset_switch_data *)dev_id;

	/* Debounce here in order to report stable status */
	cancel_delayed_work_sync(&switch_data->det_work);
	queue_delayed_work(switch_data->hs_workq,
				&switch_data->det_work, HZ / 3);

	return IRQ_HANDLED;
}

static ssize_t switch_headset_print_state(struct switch_dev *sdev, char *buf)
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

static int headset_switch_probe(struct platform_device *pdev)
{
	struct switch_headset_pdata *pdata = pdev->dev.platform_data;
	struct headset_switch_data *switch_data;
	int ret = 0;

	if (!pdata) {
		dev_err(&pdev->dev, "pdata is not available\n");
		return -EINVAL;
	}

	switch_data = kzalloc(sizeof(struct headset_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = pdata->name;
	switch_data->gpio = pdata->gpio;
	switch_data->invert = pdata->invert;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_headset_print_state;

	platform_set_drvdata(pdev, switch_data);

	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register switch dev\n");
		goto err_switch_dev_register;
	}

	switch_data->hs_workq =
			create_singlethread_workqueue("HS DET Work Queue");
	if (!switch_data->hs_workq) {
		dev_err(&pdev->dev, "failed to create work queue\n");
		ret = -ENOMEM;
		goto err_create_workq;
	}

	INIT_DELAYED_WORK(&switch_data->det_work, headset_det_work_func);

	ret = gpio_request(switch_data->gpio, pdev->name);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request gpio\n");
		goto err_request_gpio;
	}

	ret = gpio_direction_input(switch_data->gpio);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to set gpio as input\n");
		goto err_set_gpio_input;
	}

	switch_data->irq = gpio_to_irq(switch_data->gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		dev_err(&pdev->dev, "gpio irq is invalid\n");
		goto err_irq;
	}

	ret = request_threaded_irq(switch_data->irq, NULL, headset_irq_handler,
				   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
				   | IRQF_ONESHOT,
				   pdev->name, switch_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request irq\n");
		goto err_irq;
	}

	/* Detect HS plug status during boot up */
	queue_delayed_work(switch_data->hs_workq, &switch_data->det_work, HZ);

	return 0;

err_irq:
err_set_gpio_input:
	gpio_free(switch_data->gpio);
err_request_gpio:
	destroy_workqueue(switch_data->hs_workq);
err_create_workq:
	switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int __devexit headset_switch_remove(struct platform_device *pdev)
{
	struct headset_switch_data *switch_data = platform_get_drvdata(pdev);

	free_irq(switch_data->irq, switch_data);
	cancel_delayed_work_sync(&switch_data->det_work);
	destroy_workqueue(switch_data->hs_workq);

	gpio_free(switch_data->gpio);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

#ifdef CONFIG_PM
static int headset_switch_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	struct headset_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&switch_data->det_work);
	return 0;
}

static int headset_switch_resume(struct platform_device *pdev)
{
	struct headset_switch_data *switch_data = platform_get_drvdata(pdev);

	/* Fix missing irq if headset is disconected when suspend */
	queue_delayed_work(switch_data->hs_workq, &switch_data->det_work, HZ);
	return 0;
}
#else
#define headset_switch_suspend	NULL
#define headset_switch_resume	NULL
#endif

static struct platform_driver headset_switch_driver = {
	.probe = headset_switch_probe,
	.remove = __devexit_p(headset_switch_remove),
	.driver = {
		.name = "headset",
		.owner = THIS_MODULE,
	},
#ifdef CONFIG_PM
	.suspend = headset_switch_suspend,
	.resume = headset_switch_resume,
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
