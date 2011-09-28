/*
 * max8903_charger.c - Maxim 8903 USB/Adapter Charger Driver
 *
 * Copyright (C) 2011 Samsung Electronics
 * MyungJoo Ham <myungjoo.ham@samsung.com>
 * Copyright (C) 2011 Marvell Technology Ltd.
 * Yunfan Zhang <yfzhang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/mfd/max8925.h>
#include <linux/power/max8903_charger.h>

struct max8903_data {
	struct max8903_pdata *pdata;
	struct device *dev;
	struct power_supply psy;
	struct notifier_block chg_notif;
	bool dc_in;		/* usb charger attached */
	bool usb_in;	/* DC charger attached */
	bool fault;
};

/* Set GPIO asserted: en(1: asserted; 0: deasserted)
 * Return zero on success */
static int max8903_gpio_set_assert(struct max8903_data *data,
				int pin_id, bool en)
{
	struct max8903_pdata *pdata;
	struct max8903_gpio *gpio;
	int i = 0;
	if (!data || !data->pdata)
		return -EINVAL;
	pdata = data->pdata;
	for (i = 0; i < pdata->gpio_nums; i++) {
		gpio = &pdata->gpios[i];
		if (gpio->id == pin_id) {
			if (gpio->active_low)
				en = !en;
			gpio_direction_output(gpio->gpio, en);
			return 0;
		}
	}
	return -EINVAL;
}

/* Set current-limit, 1: DC(2A); 0: USB(500/100mA) */
static int max8903_set_dc_mode(struct max8903_data *data, int mode)
{
	return max8903_gpio_set_assert(data, MAX8903_PIN_DCM, !!mode);
}
/* Set USB current-limit, 1: 500mA; 0: 100mA */
static int max8903_set_iusb(struct max8903_data *data, int mode)
{
	return max8903_gpio_set_assert(data, MAX8903_PIN_IUSB, !!mode);
}
/* Set USB suspend, 1: suspended; 0: active */
static int max8903_set_usus(struct max8903_data *data, int mode)
{
	return max8903_gpio_set_assert(data, MAX8903_PIN_USUS, !!mode);
}
/* Charger enable */
static int max8903_set_cen(struct max8903_data *data, int mode)
{
	return max8903_gpio_set_assert(data, MAX8903_PIN_CEN_N, !!mode);
}

/* Detect whether state pin is asserted */
static bool max8903_gpio_is_assert(struct max8903_data *data, int pin_id)
{
	struct max8903_pdata *pdata;
	struct max8903_gpio *gpio;
	int i = 0, ret = 0;
	bool is_assert = 0;
	if (!data || !data->pdata)
		return -EINVAL;
	pdata = data->pdata;
	for (i = 0; i < pdata->gpio_nums; i++) {
		gpio = &pdata->gpios[i];
		if (gpio->id == pin_id) {
			ret = !!gpio_get_value(gpio->gpio);
			is_assert = gpio->active_low ?
				ret ? false : true		/* active low */
				: ret ? true : false;	/* avtive high */
			break;
		}
	}
	return is_assert;
}
/* Is DC attached? */
static bool max8903_is_dcin(struct max8903_data *data)
{
	return max8903_gpio_is_assert(data, MAX8903_PIN_DOK_N);
}
/* IS USB attached? */
static bool max8903_is_usbin(struct max8903_data *data)
{
	return max8903_gpio_is_assert(data, MAX8903_PIN_UOK_N);
}
/* IS fault? */
static bool max8903_is_fault(struct max8903_data *data)
{
	return max8903_gpio_is_assert(data, MAX8903_PIN_FLT_N);
}

/* DC-OK irq handler */
static irqreturn_t max8903_dcin_handler(int irq, void *_data)
{
	struct max8903_data *data = _data;
	enum power_supply_type old_type;
	bool dc_in;

	dc_in = max8903_is_dcin(data);

	if (dc_in == data->dc_in)
		return IRQ_HANDLED;

	data->dc_in = dc_in;

	/* Set Current-Limit-Mode 1:DC 0:USB */
	max8903_set_dc_mode(data, dc_in ? 1 : 0);

	/* Charger Enable/Disable */
	max8903_set_cen(data, data->dc_in ? 1 :
			(data->usb_in ? 1 : 0));

	dev_dbg(data->dev, "DC-IN Charger %s.\n", dc_in ?
			"Connected" : "Disconnected");

	old_type = data->psy.type;

	if (data->dc_in)
		data->psy.type = POWER_SUPPLY_TYPE_MAINS;
	else if (data->usb_in)
		data->psy.type = POWER_SUPPLY_TYPE_USB;

	if (old_type != data->psy.type)
		power_supply_changed(&data->psy);

	return IRQ_HANDLED;
}

/* USB-OK irq handler */
static irqreturn_t max8903_usbin_handler(int irq, void *_data)
{
	struct max8903_data *data = _data;
	enum power_supply_type old_type;
	bool usb_in;

	usb_in = max8903_is_usbin(data);

	if (usb_in == data->usb_in)
		return IRQ_HANDLED;

	data->usb_in = usb_in;

	/* Do not touch Current-Limit-Mode */

	/* Charger Enable/Disable */
	max8903_set_cen(data, data->usb_in ? 1 :
			(data->dc_in ? 1 : 0));

	dev_dbg(data->dev, "USB Charger %s.\n", usb_in ?
			"Connected" : "Disconnected");

	old_type = data->psy.type;

	if (data->dc_in)
		data->psy.type = POWER_SUPPLY_TYPE_MAINS;
	else
		data->psy.type = POWER_SUPPLY_TYPE_USB;

	if (old_type != data->psy.type)
		power_supply_changed(&data->psy);

	return IRQ_HANDLED;
}

/* Fault irq handler */
static irqreturn_t max8903_fault_handler(int irq, void *_data)
{
	struct max8903_data *data = _data;
	bool fault;

	fault = max8903_is_fault(data);

	if (fault == data->fault)
		return IRQ_HANDLED;

	data->fault = fault;

	if (fault)
		dev_err(data->dev, "Charger suffers a fault and stops.\n");
	else
		dev_err(data->dev, "Charger recovered from a fault.\n");

	return IRQ_HANDLED;
}

static int max8903_gpio_init(struct max8903_data *data)
{
	struct max8903_pdata *pdata = data->pdata;
	struct max8903_gpio *gpio;
	int i = 0, ret = 0;
	if (!pdata)
		return -EINVAL;
	/* GPIOs setup */
	for (i = 0; i < pdata->gpio_nums; i++) {
		gpio = &pdata->gpios[i];
		if (gpio && (gpio->id < MAX8903_PIN_END)
			&& gpio_is_valid(gpio->gpio)) {
			/* Request gpios */
			ret = gpio_request(gpio->gpio, gpio->desc);
			if (ret) {
				dev_err(data->dev, "failed to request GPIO%d\n",
						gpio->gpio);
				goto err_gpio_request;
			}
			if (gpio->id == MAX8903_PIN_DOK_N ||
				gpio->id == MAX8903_PIN_UOK_N ||
				gpio->id == MAX8903_PIN_CHG_N ||
				gpio->id == MAX8903_PIN_FLT_N) {
				/* Set status GPIOs as Input */
				gpio_direction_input(gpio->gpio);
			}
		} else {
			dev_err(data->dev, "GPIO is not invalid\n");
			goto err_gpio_invalid;
		}
	}
	/* Enable charger by default to ensure boot up w/o battery */
	max8903_set_dc_mode(data, 1);
	max8903_set_iusb(data, 1);
	max8903_set_usus(data, 0);
	max8903_set_cen(data, 1);
	return 0;
err_gpio_invalid:
err_gpio_request:
	if (i >= 1) {
		do {
			i--;
			gpio = &pdata->gpios[i];
			gpio_free(gpio->gpio);
		} while (i > 0);
	}
	return ret;
}

static void max8903_gpio_deinit(struct max8903_data *data)
{
	struct max8903_pdata *pdata = data->pdata;
	struct max8903_gpio *gpio;
	int i = 0;
	for (i = 0; i < pdata->gpio_nums; i++) {
		gpio = &pdata->gpios[i];
		gpio_free(gpio->gpio);
	}
}

static int max8903_request_irq(struct max8903_data *data)
{
	struct max8903_pdata *pdata = data->pdata;
	struct device *dev = data->dev;
	struct max8903_gpio *gpio;
	int i = 0, ret = 0;
	/* GPIOs already checked by max8903_gpio_init() */
	for (i = 0; i < pdata->gpio_nums; i++) {
		gpio = &pdata->gpios[i];
		if (gpio->id == MAX8903_PIN_DOK_N) {
			ret = request_threaded_irq(gpio_to_irq(gpio->gpio),
				NULL, max8903_dcin_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"MAX8903 DC_IN", data);
			if (ret) {
				dev_err(dev, "fail to request irq %d,errno(%d)\n",
						gpio_to_irq(gpio->gpio), ret);
				goto err_irq_dc_in;
			}
		} else if (gpio->id == MAX8903_PIN_UOK_N) {
			ret = request_threaded_irq(gpio_to_irq(gpio->gpio),
				NULL, max8903_usbin_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"MAX8903 USB_IN", data);
			if (ret) {
				dev_err(dev, "fail to request irq %d,errno(%d)\n",
						gpio_to_irq(gpio->gpio), ret);
				goto err_irq_usb_in;
			}
		} else if (gpio->id == MAX8903_PIN_FLT_N) {
			ret = request_threaded_irq(gpio_to_irq(gpio->gpio),
				NULL, max8903_fault_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"MAX8903 Fault", data);
			if (ret) {
				dev_err(dev, "fail to request irq %d,errno(%d)\n",
						gpio_to_irq(gpio->gpio), ret);
				goto err_irq_fault;
			}
		}
	}
	return 0;
err_irq_fault:
err_irq_usb_in:
err_irq_dc_in:
	if (i >= 1)	{
		do {
			i--;
			gpio = &pdata->gpios[i];
			if (gpio->id == MAX8903_PIN_DOK_N ||
				gpio->id == MAX8903_PIN_UOK_N ||
				gpio->id == MAX8903_PIN_FLT_N) {
				/* backtrace to free irq */
				free_irq(gpio_to_irq(gpio->gpio), data);
			}
		} while (i > 0);
	}
	return ret;
}

static void max8903_free_irq(struct max8903_data *data)
{
	struct max8903_pdata *pdata = data->pdata;
	struct max8903_gpio *gpio;
	int i = 0;
	for (i = 0; i < pdata->gpio_nums; i++) {
		gpio = &pdata->gpios[i];
		if (gpio->id == MAX8903_PIN_DOK_N ||
			gpio->id == MAX8903_PIN_UOK_N ||
			gpio->id == MAX8903_PIN_FLT_N) {
			/* Free all irqs */
			free_irq(gpio_to_irq(gpio->gpio), data);
		}
	}
}

static int max8903_start_charging(struct max8903_data *data)
{
	if (!data)
		return -EINVAL;
	if (data->dc_in) {
		/* Enable 2A DC mode */
		max8903_set_dc_mode(data, 1);
	} else if (data->usb_in) {
		/* Disable DC mode, 500/100mA now */
		max8903_set_dc_mode(data, 0);
		/* USB input active */
		max8903_set_usus(data, 0);
		/* NOTE: set to 500mA if usb attached */
		max8903_set_iusb(data, 1);
	}
	/* Enable charger */
	max8903_set_cen(data, 1);
	return 0;
}

static int max8903_stop_charging(struct max8903_data *data)
{
	if (!data)
		return -EINVAL;
	/* Disable charger */
	max8903_set_cen(data, 0);
	/* Disable DC mode */
	max8903_set_dc_mode(data, 0);
	/* Limit to 100mA */
	max8903_set_iusb(data, 0);
	/* USB input suspend */
	max8903_set_usus(data, 1);
	return 0;
}

static int max8903_chg_notifier_callback(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct max8903_data *di =
			container_of(nb, struct max8903_data, chg_notif);
	struct chg_data *chg_data = data;
	if (!di || !chg_data)
		return 0;

	/* Get charger type */
	switch (chg_data->charger_type) {
	case POWER_SUPPLY_TYPE_MAINS:
	case POWER_SUPPLY_TYPE_USB_DCP:
	case POWER_SUPPLY_TYPE_USB_CDP:
	case POWER_SUPPLY_TYPE_USB_ACA:
		di->dc_in = 1;
		break;
	case POWER_SUPPLY_TYPE_USB:
		di->usb_in = 1;
		break;
	case POWER_SUPPLY_TYPE_BATTERY:
	default:
		di->dc_in = 0;
		di->usb_in = 0;
		break;
	}
	/* Update charger type */
	if (di->dc_in)
		di->psy.type = POWER_SUPPLY_TYPE_MAINS;
	else
		di->psy.type = POWER_SUPPLY_TYPE_USB;
	/* Handle start/stop charging event */
	switch (event) {
	case CHG_EVENT_START:
		max8903_start_charging(di);
		break;
	case CHG_EVENT_STOP:
		di->dc_in = 0;
		di->usb_in = 0;
		max8903_stop_charging(di);
		break;
	default:
		return 0;
	}
	power_supply_changed(&di->psy);
	return NOTIFY_OK;
}

static enum power_supply_property max8903_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE, /* External power source */
	POWER_SUPPLY_PROP_HEALTH, /* Fault or OK */
};

static int max8903_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct max8903_data *data = container_of(psy,
			struct max8903_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (data->dc_in || data->usb_in);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		if (data->fault)
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static char *max8903_supplied_to[] = {
	"max8925-battery",
	"max17042-battery",
	"bq27410-battery",
};

static int max8903_powersupply_init(struct max8903_data *data)
{
	struct max8903_pdata *pdata = data->pdata;
	int ret = 0;

	if (pdata->supplied_to) {
		data->psy.supplied_to = pdata->supplied_to;
		data->psy.num_supplicants = pdata->num_supplicants;
	} else {
		data->psy.supplied_to = max8903_supplied_to;
		data->psy.num_supplicants = ARRAY_SIZE(max8903_supplied_to);
	}
	data->psy.name = "max8903-charger";
	if (pdata->dc_valid)
		data->psy.type = POWER_SUPPLY_TYPE_MAINS;
	else
		data->psy.type = POWER_SUPPLY_TYPE_USB;
	data->psy.get_property = max8903_get_property;
	data->psy.properties = max8903_charger_props;
	data->psy.num_properties = ARRAY_SIZE(max8903_charger_props);

	ret = power_supply_register(data->dev, &data->psy);
	return ret;
}

static void max8903_powersupply_deinit(struct max8903_data *data)
{
	power_supply_unregister(&data->psy);
}

static __devinit int max8903_probe(struct platform_device *pdev)
{
	struct max8903_data *data;
	struct device *dev = &pdev->dev;
	struct max8903_pdata *pdata = pdev->dev.platform_data;
	int ret = 0;

	data = kzalloc(sizeof(struct max8903_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Cannot allocate memory.\n");
		return -ENOMEM;
	}
	data->pdata = pdata;
	data->dev = dev;
	platform_set_drvdata(pdev, data);

	if (pdata->dc_valid == false && pdata->usb_valid == false) {
		dev_err(dev, "No valid power sources.\n");
		ret = -EINVAL;
		goto err_power_source;
	}
	/* Init GPIOs */
	ret = max8903_gpio_init(data);
	if (ret) {
		dev_err(dev, "failed to setup gpio.\n");
		goto err_gpio_setup;
	}
	/* Register power supply */
	ret = max8903_powersupply_init(data);
	if (ret) {
		dev_err(dev, "failed: power supply register.\n");
		goto err_powersupply_init;
	}
	/* Register charger event notifier */
	data->chg_notif.notifier_call = max8903_chg_notifier_callback;
	max8925_chg_register_client(&data->chg_notif);

	ret = max8903_request_irq(data);
	if (ret) {
		dev_err(dev, "failed to request.\n");
		goto err_request_irq;
	}
	dev_info(dev, "charger is enabled.\n");
	return 0;
err_request_irq:
	max8925_chg_unregister_client(&data->chg_notif);
	max8903_powersupply_deinit(data);
err_powersupply_init:
	max8903_gpio_deinit(data);
err_gpio_setup:
err_power_source:
	kfree(data);
	return ret;
}

static __devexit int max8903_remove(struct platform_device *pdev)
{
	struct max8903_data *data = platform_get_drvdata(pdev);
	if (data) {
		max8903_free_irq(data);
		max8925_chg_unregister_client(&data->chg_notif);
		max8903_powersupply_deinit(data);
		max8903_gpio_deinit(data);
		kfree(data);
	}
	return 0;
}

static struct platform_driver max8903_driver = {
	.probe	= max8903_probe,
	.remove	= __devexit_p(max8903_remove),
	.driver = {
		.name	= "max8903-charger",
		.owner	= THIS_MODULE,
	},
};

static int __init max8903_init(void)
{
	return platform_driver_register(&max8903_driver);
}
module_init(max8903_init);

static void __exit max8903_exit(void)
{
	platform_driver_unregister(&max8903_driver);
}
module_exit(max8903_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MAX8903 Charger Driver");
MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_ALIAS("max8903-charger");
