/*
 * Maxim 8900A, 8900B, 8900C charger driver.
 *
 * Copyright (C) 2011 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

#include <linux/max8900_charger.h>

struct max8900_data {
	struct max8900_pdata *pdata;
	struct device *dev;
	struct power_supply ac_charger;
	struct power_supply usb_charger;

	unsigned online:1;
	unsigned charge_type:2;	/* see for enumeration charger_type */
	unsigned state:5;		/* see for enumeration max8900_state */
};

struct max8900_status_state_map {
	unsigned char status;
	unsigned char state;
};

static struct max8900_status_state_map status_state_map[] = {
		/* MAX8900A */
	{
		.status	= 0x10,
		.state	= MAX8900_UNDEFINED_STATE,
	},
	{
		.status = 0x11,
		.state	= MAX8900_CHARGING_STATE,
	},
	{
		.status = 0x12,
		.state	= MAX8900_UNSPEC_FAULT_STATE,
	},
	{
		.status = 0x13,
		.state	= MAX8900_DONE_STATE,
	},

	/* MAX8900B and MAX8900C states */
	{
		.status = 0x00,
		.state	= MAX8900_BATTERY_COLD_STATE,
	},
	{
		.status = 0x01,
		.state	= MAX8900_VIN_HIGH_STATE,
	},
	{
		.status = 0x02,
		.state	= MAX8900_CHARGING_STATE,
	},
	{
		.status = 0x03,
		.state	= MAX8900_BATTERY_HOT_STATE,
	},
	{
		.status = 0x04,
		.state	= MAX8900_DONE_STATE,
	},
	{
		.status = 0x05,
		.state	= MAX8900_UNDEFINED_STATE,
	},
	{
		.status = 0x06,
		.state	= MAX8900_TIMER_FAULT_STATE,
	},
	{
		.status = 0x07,
		.state	= MAX8900_UNSPEC_FAULT_STATE,
	},
};

static unsigned char max8900_get_state_by_status(unsigned char status)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(status_state_map); i++) {
		if (status == status_state_map[i].status)
			return status_state_map[i].state;
	}

	return MAX8900_UNDEFINED_STATE;
}

static char *max8900_supplied_to[] = {
	"battery",
};

static struct max8900_data *global_data;

int max8900_set_charger_online(bool online)
{
	if (!global_data)
		return -EINVAL;

	global_data->online = online;
	max8900_set_charge_mode(MAX8900_NORMAL_CHARGE_MODE);

	power_supply_changed(&global_data->ac_charger);
	power_supply_changed(&global_data->usb_charger);

	return 0;
}
EXPORT_SYMBOL(max8900_set_charger_online);

int max8900_set_charger_type(enum max8900_charger_type type)
{
	if (!global_data)
		return -EINVAL;

	if (type == MAX8900_USB_CHARGER)
		max8900_set_charge_mode(MAX8900_NORMAL_CHARGE_MODE);
	else if (type == MAX8900_AC_CHARGER)
		max8900_set_charge_mode(MAX8900_FAST_CHARGE_MODE);
	else
		return -EINVAL;

	global_data->online = 1;
	global_data->charge_type = type;

	power_supply_changed(&global_data->ac_charger);
	power_supply_changed(&global_data->usb_charger);

	return 0;
}
EXPORT_SYMBOL(max8900_set_charger_type);

int max8900_set_charge_mode(enum max8900_charge_mode mode)
{
	int cen = 1;
	int seti = 0;

	if (!global_data || !global_data->pdata)
		return -EINVAL;

	switch (mode) {
	case MAX8900_DISABLED_CHARGE_MODE:
		cen = 1;
		seti = 0;
		break;
	case MAX8900_NORMAL_CHARGE_MODE:
		cen = 0;
		seti = 0;
		break;
	case MAX8900_FAST_CHARGE_MODE:
		cen = 0;
		seti = 1;
		break;
	default:
		return -EINVAL;
	}

	if (gpio_is_valid(global_data->pdata->gpio_cen))
		gpio_set_value(global_data->pdata->gpio_cen, cen);
	else
		return -EINVAL;
	if (gpio_is_valid(global_data->pdata->gpio_seti))
		gpio_set_value(global_data->pdata->gpio_seti, seti);
	else
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL(max8900_set_charge_mode);

static enum power_supply_property max8900_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int max8900_get_ac_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct max8900_data *data = container_of(psy, struct max8900_data,
						ac_charger);

	switch (psp)	{
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->online &&
		(data->charge_type == MAX8900_AC_CHARGER);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property max8900_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int max8900_get_usb_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct max8900_data *data = container_of(psy, struct max8900_data,
						usb_charger);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->online &&
		(data->charge_type == MAX8900_USB_CHARGER);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int max8900_validate_gpio(struct max8900_pdata *pdata)
{
	if (!pdata)
		return -EINVAL;

	if (!gpio_is_valid(pdata->gpio_cen) ||
		!gpio_is_valid(pdata->gpio_seti) ||
		!gpio_is_valid(pdata->gpio_stat1) ||
		!gpio_is_valid(pdata->gpio_stat2) ||
		((pdata->type != MAX8900A_TYPE) &&
		!gpio_is_valid(pdata->gpio_stat3)))
		return -EINVAL;

	return 0;
}

static int max8900_request_gpio(struct max8900_pdata *pdata)
{
	if (!pdata)
		return -EINVAL;

	if (gpio_request(pdata->gpio_cen, "Enable charge") ||
		gpio_request(pdata->gpio_seti, "Enable fast charging") ||
		gpio_request(pdata->gpio_stat1, "Status 1") ||
		gpio_request(pdata->gpio_stat2, "Status 2"))
		return -EINVAL;

	gpio_direction_output(pdata->gpio_cen, 0);
	gpio_direction_output(pdata->gpio_seti, 0);

	gpio_direction_input(pdata->gpio_stat1);
	gpio_direction_input(pdata->gpio_stat2);

	if (pdata->type != MAX8900A_TYPE) {
		if (gpio_request(pdata->gpio_stat3, "Status 3"))
			return -EINVAL;
		gpio_direction_input(pdata->gpio_stat3);
	}

	return 0;
}

static void max8900_free_gpio(struct max8900_pdata *pdata)
{
	if (!pdata)
		return;

	gpio_free(pdata->gpio_cen);
	gpio_free(pdata->gpio_seti);
	gpio_free(pdata->gpio_stat1);
	gpio_free(pdata->gpio_stat2);

	if (pdata->type != MAX8900A_TYPE)
		gpio_free(pdata->gpio_stat3);
}

static irqreturn_t max8900_status(int irq, void *irq_data)
{
	struct max8900_data *data = irq_data;
	struct max8900_pdata *pdata = data->pdata;
	unsigned old_state = data->state;

	bool cen = gpio_get_value(pdata->gpio_cen);
	bool stat1 = gpio_get_value(pdata->gpio_stat1);
	bool stat2 = gpio_get_value(pdata->gpio_stat2);
	bool stat3 = false;

	if (cen)
		data->state = MAX8900_CHARGER_DISABLED_STATE;
	else {
		if (pdata->type == MAX8900A_TYPE) {
			data->state = max8900_get_state_by_status(0x10 |
					(stat1<<1) | stat2);
		} else {
			stat3 = gpio_get_value(pdata->gpio_stat3);
			data->state = max8900_get_state_by_status((stat1<<2) |
					(stat2<<1) | stat3);
		}
	}

	if (data->state != old_state) {
		if (pdata->notify_state_changed)
			pdata->notify_state_changed(data->state);

		power_supply_changed(&data->ac_charger);
		power_supply_changed(&data->usb_charger);
	}

	return IRQ_HANDLED;
}

static int __devinit max8900_request_status_irq(struct max8900_data *data)
{
	int ret = 0;
	struct max8900_pdata *pdata;

	if (!data || !data->pdata)
		return EINVAL;

	pdata = data->pdata;

	ret = request_threaded_irq(gpio_to_irq(pdata->gpio_stat1),
				NULL, max8900_status,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"MAX8900 status", data);
	if (ret)
		return ret;

	ret = request_threaded_irq(gpio_to_irq(pdata->gpio_stat2),
				NULL, max8900_status,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"MAX8900 status", data);
	if (ret)
		goto error_unregister_stat1_irq;

	if (data->pdata->type != MAX8900A_TYPE) {
		ret = request_threaded_irq(gpio_to_irq(pdata->gpio_stat3),
					NULL, max8900_status,
					IRQF_TRIGGER_FALLING |
					IRQF_TRIGGER_RISING,
					"MAX8900 status", data);
		if (ret)
			goto error_unregister_stat2_irq;
	}

	return 0;

error_unregister_stat2_irq:
	free_irq(gpio_to_irq(pdata->gpio_stat2), data);
error_unregister_stat1_irq:
	free_irq(gpio_to_irq(pdata->gpio_stat1), data);
	return ret;
}

static __devexit void max8900_free_status_irq(struct max8900_data *data)
{
	struct max8900_pdata *pdata;

	if (!data || !data->pdata)
		return;

	pdata = data->pdata;

	free_irq(gpio_to_irq(pdata->gpio_stat1), data);
	free_irq(gpio_to_irq(pdata->gpio_stat2), data);
	if (pdata->type != MAX8900A_TYPE)
		free_irq(gpio_to_irq(pdata->gpio_stat3), data);
}

static __devinit int max8900_probe(struct platform_device *pdev)
{
	struct max8900_data *data;
	struct device *dev = &pdev->dev;
	struct max8900_pdata *pdata = pdev->dev.platform_data;
	int ret = 0;

	if (!pdata) {
		dev_err(&pdev->dev, "No platform data!\n");
		return -EINVAL;
	}

	ret = max8900_validate_gpio(pdata);
	if (ret) {
		dev_err(&pdev->dev, "Invalid GPIO pin!\n");
		return -EINVAL;
	}

	ret = max8900_request_gpio(pdata);
	if (ret) {
		dev_err(dev, "Request GPIO failed!\n");
		goto error_release_source;
	}

	data = kzalloc(sizeof(struct max8900_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Can not allocate memory!\n");
		ret = -ENOMEM;
		goto error_release_source;
	}

	data->dev = dev;
	data->pdata = pdata;
	data->online = 0;
	data->charge_type = MAX8900_UNDEFINED_CHARGER;
	global_data = data;

	data->ac_charger.name = "ac";
	data->ac_charger.type = POWER_SUPPLY_TYPE_MAINS;
	data->ac_charger.supplied_to = max8900_supplied_to;
	data->ac_charger.num_supplicants = ARRAY_SIZE(max8900_supplied_to);
	data->ac_charger.get_property = max8900_get_ac_property;
	data->ac_charger.properties = max8900_ac_props;
	data->ac_charger.num_properties = ARRAY_SIZE(max8900_ac_props);

	ret = power_supply_register(dev, &data->ac_charger);
	if (ret) {
		dev_err(dev, "AC power supply registeration failed!\n");
		goto error_free_mem;
	}

	data->usb_charger.name = "usb";
	data->usb_charger.type = POWER_SUPPLY_TYPE_USB;
	data->usb_charger.supplied_to = max8900_supplied_to;
	data->usb_charger.num_supplicants = ARRAY_SIZE(max8900_supplied_to);
	data->usb_charger.get_property = max8900_get_usb_property;
	data->usb_charger.properties = max8900_usb_props;
	data->usb_charger.num_properties = ARRAY_SIZE(max8900_usb_props);

	ret = power_supply_register(dev, &data->usb_charger);
	if (ret) {
		dev_err(dev, "USB power supply registeration failed!\n");
		goto error_unregister_ps_ac;
	}

	ret = max8900_request_status_irq(data);
	if (ret) {
		dev_err(dev, "IRQ request failed!\n");
		goto error_unregister_ps_usb;
	}

	/* Read initial status */
	max8900_status(0, data);

	platform_set_drvdata(pdev, data);
	return 0;

error_unregister_ps_usb:
	power_supply_unregister(&data->usb_charger);
error_unregister_ps_ac:
	power_supply_unregister(&data->ac_charger);
error_free_mem:
	kfree(data);
error_release_source:
	max8900_free_gpio(pdata);
	return ret;
}

static __devexit int max8900_remove(struct platform_device *pdev)
{
	struct max8900_data *data = platform_get_drvdata(pdev);

	if (data) {
		struct max8900_pdata *pdata = data->pdata;
		power_supply_unregister(&data->ac_charger);
		power_supply_unregister(&data->usb_charger);
		max8900_free_status_irq(data);
		max8900_free_gpio(pdata);
		global_data = NULL;
		kfree(data);
	}

	return 0;
}

static struct platform_driver max8900_driver = {
	.probe	= max8900_probe,
	.remove	= __devexit_p(max8900_remove),
	.driver	= {
		.name	= "max8900-charger",
		.owner	= THIS_MODULE,
	}
};

static int __init max8900_init(void)
{
	return platform_driver_register(&max8900_driver);
}

static void __exit max8900_exit(void)
{
	platform_driver_unregister(&max8900_driver);
}

module_init(max8900_init);
module_exit(max8900_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MAX8900 charger driver");
MODULE_ALIAS("max8900-charger");
