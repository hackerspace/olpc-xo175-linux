/*
 * Battery driver for Maxim MAX8925
 *
 * Copyright (c) 2009-2010 Marvell International Ltd.
 *	Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/mfd/max8925.h>
#include <linux/notifier.h>
#include <plat/usb.h>

/* registers in GPM */
#define MAX8925_OUT5VEN			0x54
#define MAX8925_OUT3VEN			0x58
#define MAX8925_CHG_CNTL1		0x7c

/* bits definition */
#define MAX8925_CHG_STAT_VSYSLOW	(1 << 0)
#define MAX8925_CHG_STAT_MODE_MASK	(3 << 2)
#define MAX8925_CHG_STAT_EN_MASK	(1 << 4)
#define MAX8925_CHG_MBDET		(1 << 1)
#define MAX8925_CHG_AC_RANGE_MASK	(3 << 6)

/* registers in ADC */
#define MAX8925_ADC_RES_CNFG1		0x06
#define MAX8925_ADC_AVG_CNFG1		0x07
#define MAX8925_ADC_ACQ_CNFG1		0x08
#define MAX8925_ADC_ACQ_CNFG2		0x09
/* 2 bytes registers in below. MSB is 1st, LSB is 2nd. */
#define MAX8925_ADC_AUX2		0x62
#define MAX8925_ADC_VCHG		0x64
#define MAX8925_ADC_VBBATT		0x66
#define MAX8925_ADC_VMBATT		0x68
#define MAX8925_ADC_ISNS		0x6a
#define MAX8925_ADC_THM			0x6c
#define MAX8925_ADC_TDIE		0x6e
#define MAX8925_CMD_AUX2		0xc8
#define MAX8925_CMD_VCHG		0xd0
#define MAX8925_CMD_VBBATT		0xd8
#define MAX8925_CMD_VMBATT		0xe0
#define MAX8925_CMD_ISNS		0xe8
#define MAX8925_CMD_THM			0xf0
#define MAX8925_CMD_TDIE		0xf8

enum {
	MEASURE_AUX2,
	MEASURE_VCHG,
	MEASURE_VBBATT,
	MEASURE_VMBATT,
	MEASURE_ISNS,
	MEASURE_THM,
	MEASURE_TDIE,
	MEASURE_MAX,
};

struct max8925_power_info {
	struct device *dev;
	struct max8925_chip *chip;
	struct i2c_client *gpm;
	struct i2c_client *adc;
	struct delayed_work chg_event_work;
	struct notifier_block notif;

	struct power_supply battery;
	int irq_base;
	unsigned bat_online:1;
	unsigned chg_mode:2;
	unsigned batt_detect:1;	/* detecing MB by ID pin */
	unsigned topoff_threshold:2;
	unsigned fast_charge:3;	/* fast charging current */
	/* MAX8925 battery monitor enable */
	unsigned bat_max8925_en:1;
	unsigned supply_type; /* power supply type */
	/* Charger port PMIC connection config */
	unsigned chg_port_config;
	/* Charger event */
	unsigned chg_event;
	int (*set_led)(int);
};

struct capacity_percent {
	int voltage;
	int percent;
};

static int start_measure(struct max8925_power_info *info, int type);
/* Battery resistance offset */
#define BATTERY_R_CHARGING		330	/* charging offset */
#define BATTERY_R_DISCHARGING	60	/* discharging offset */
/* High level voltage threshold */
#define CHG_HIGH_LEVEL_TH	(4500)	/* mA */

static struct capacity_percent cap_table[] = {
	{3200, 0}, {3320, 10}, {3420, 20}, {3500, 30}, {3550, 40},
	{3600, 50}, {3650, 60}, {3750, 70}, {3850, 80}, {3950, 90},
	{4100, 100},
};

static struct max8925_power_info *pw_info;
/* Notifier: notify PMIC event(attach/detach;start/stop charging...) */
static BLOCKING_NOTIFIER_HEAD(chg_notifier_list);

int max8925_chg_register_client(struct notifier_block *nb)
{
	struct max8925_power_info *info = pw_info;
	struct chg_data data;
	int ret = 0;
	ret = blocking_notifier_chain_register(&chg_notifier_list, nb);
	if (ret)
		return ret;
	if (info) {
		memset(&data, 0, sizeof(struct chg_data));
		data.charger_type = info->supply_type;
		return blocking_notifier_call_chain(&chg_notifier_list,
						info->chg_event, &data);
	}
	return 0;
}
EXPORT_SYMBOL(max8925_chg_register_client);

int max8925_chg_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&chg_notifier_list, nb);
}
EXPORT_SYMBOL(max8925_chg_unregister_client);

static int max8925_chg_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&chg_notifier_list, val, v);
}

static int max8925_start_charge(struct max8925_power_info *info)
{
	struct chg_data data;

	memset(&data, 0, sizeof(struct chg_data));
	data.charger_type = info->supply_type;
	info->chg_event = CHG_EVENT_START;
	return max8925_chg_notifier_call_chain(info->chg_event, &data);
}

static int max8925_stop_charge(struct max8925_power_info *info)
{
	struct chg_data data;

	memset(&data, 0, sizeof(struct chg_data));
	data.charger_type = info->supply_type;
	info->chg_event = CHG_EVENT_STOP;
	return max8925_chg_notifier_call_chain(info->chg_event, &data);
}

static int max8925_charger_attach(struct max8925_power_info *info)
{
	max8925_start_charge(info);
	/* Enable indicator led */
	if (info->set_led)
		info->set_led(1);

	return 0;
}

static int max8925_charger_detach(struct max8925_power_info *info)
{
	max8925_stop_charge(info);
	/* Disable indicator led */
	if (info->set_led)
		info->set_led(0);

	return 0;
}

static void chg_event_work_func(struct work_struct *work)
{
	struct max8925_power_info *info =
	    container_of(work, struct max8925_power_info,
				chg_event_work.work);
	/* Detect attach or detach state by voltage */
	if ((start_measure(info, MEASURE_VCHG) * 2) > CHG_HIGH_LEVEL_TH)
		info->chg_event = CHG_EVENT_ATTACH;
	else
		info->chg_event = CHG_EVENT_DETACH;
	/* Broadcast attach/detach event to listeners */
	max8925_chg_notifier_call_chain(info->chg_event, NULL);
	/* If USB port, charging event will be triggered by usb driver */
	if (info->chg_port_config == CHG_PORT_WALL) {
		if (info->chg_event == CHG_EVENT_ATTACH) {
			info->supply_type = POWER_SUPPLY_TYPE_MAINS;
			max8925_charger_attach(info);
		} else if (info->chg_event == CHG_EVENT_DETACH) {
			info->supply_type = POWER_SUPPLY_TYPE_BATTERY;
			max8925_charger_detach(info);
		}
	}
}

/* vbus event handler */
static int max8925_vbus_notifier_callback(struct notifier_block *nb,
				unsigned long event, void *chg_data)
{
	struct max8925_power_info *info =
		container_of(nb, struct max8925_power_info, notif);
	/* Parse vbus event passed by usb driver */
	switch (event) {
	case DEFAULT_CHARGER:
	case VBUS_CHARGER:
		/* Standard Downstream Port */
		info->supply_type = POWER_SUPPLY_TYPE_USB;
		break;
	case AC_CHARGER_STANDARD:
		/* Dedicated Charging Port */
		info->supply_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case AC_CHARGER_OTHER:
		/* Adapter Charger */
		info->supply_type = POWER_SUPPLY_TYPE_MAINS;
		break;
	case NULL_CHARGER:
	default:
		/* No charger */
		info->supply_type = POWER_SUPPLY_TYPE_BATTERY;
		break;
	}

	if (info->supply_type == POWER_SUPPLY_TYPE_BATTERY) {
		/* Stop charging */
		max8925_charger_detach(info);
	} else {
		/* Start charing */
		max8925_charger_attach(info);
	}

	return NOTIFY_OK;
}

static irqreturn_t max8925_charger_handler(int irq, void *data)
{
	struct max8925_power_info *info = data;
	struct max8925_chip *chip;
	if (!info) {
		dev_err(info->dev, "max8925 power info is not available!\n");
		return IRQ_HANDLED;
	}
	chip = info->chip;

	switch (irq - chip->irq_base) {
	case MAX8925_IRQ_VCHG_DC_R:
		/* Charger attached */
		dev_dbg(chip->dev, "Charger inserted\n");
		break;
	case MAX8925_IRQ_VCHG_DC_F:
		/* Charger detached */
		dev_dbg(chip->dev, "Charger is removal\n");
		break;
	default:
		break;
	}
	/* Detect actual charger state in chg_event_work */
	cancel_delayed_work_sync(&info->chg_event_work);
	schedule_delayed_work(&info->chg_event_work, HZ / 2);
	return IRQ_HANDLED;
}

static int start_measure(struct max8925_power_info *info, int type)
{
	unsigned char buf[2] = { 0, 0 };
	int meas_reg = 0, meas_cmd = 0, ret = 0;

	switch (type) {
	case MEASURE_VCHG:
		meas_cmd = MAX8925_CMD_VCHG;
		meas_reg = MAX8925_ADC_VCHG;
		break;
	case MEASURE_VBBATT:
		meas_cmd = MAX8925_CMD_VBBATT;
		meas_reg = MAX8925_ADC_VBBATT;
		break;
	case MEASURE_VMBATT:
		meas_cmd = MAX8925_CMD_VMBATT;
		meas_reg = MAX8925_ADC_VMBATT;
		break;
	case MEASURE_ISNS:
		meas_cmd = MAX8925_CMD_ISNS;
		meas_reg = MAX8925_ADC_ISNS;
		break;
	default:
		return -EINVAL;
	}

	max8925_reg_write(info->adc, meas_cmd, 0);
	max8925_bulk_read(info->adc, meas_reg, 2, buf);
	ret = ((buf[0] << 8) | buf[1]) >> 4;

	return ret;
}

static int get_capacity(int data)
{
	int i, size;
	int ret = -EINVAL;

	size = ARRAY_SIZE(cap_table);
	if (data >= cap_table[size - 1].voltage) {
		ret = 100;
		return ret;
	} else if (data <= cap_table[0].voltage) {
		ret = 0;
		return ret;
	}
	for (i = size - 2; i >= 0; i--) {
		if (data < cap_table[i].voltage)
			continue;
		ret = (data - cap_table[i].voltage) * 10
		    / (cap_table[i + 1].voltage - cap_table[i].voltage);
		ret += cap_table[i].percent;
		break;
	}
	return ret;
}

static int max8925_get_bat_current(struct max8925_power_info *info)
{
	int ret = 0;

	ret = start_measure(info, MEASURE_ISNS);
	if (ret < 0) {
		pr_debug("fail to measure battery current!\n");
		return 0;
	}
	/* NOTE: r_sns is .02 here */
	ret = (ret * 6250) / 4096 - 3125;	/* mA */

	return ret;
}

static int max8925_get_bat_voltage(struct max8925_power_info *info)
{
	int cur = 0, vol = 0, offset = 0;
	int ret = 0;
	/* get battery current */
	ret = start_measure(info, MEASURE_VMBATT);
	if (ret < 0) {
		pr_debug("fail to measure battery voltage!\n");
		return 0;
	}

	vol = ret << 1;		/* unit is mV */

	/* count in offset voltage caused by battery resistance */
	cur = max8925_get_bat_current(info);
	if (cur >= 0)
		offset = cur * BATTERY_R_CHARGING / 1000;
	else
		offset = cur * BATTERY_R_DISCHARGING / 1000;
	/* calculate actual battery voltage */
	vol -= offset;

	if (vol < 0) {
		pr_debug("battery voltage is abnormal!\n");
		vol = 0;
	}

	return vol;
}

static int max8925_bat_get_prop(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct max8925_power_info *info =
			container_of(psy, struct max8925_power_info, battery);
	int ret = 0;

	if (!info) {
		dev_err(info->dev, "max8925 power info is not availible!");
		return -EINVAL;
	}
	/* Unit:
	 * All voltages, currents, charges, energies, time and temperatures
	 * in µV, µA, µAh, µWh, seconds and tenths of degree Celsius.
	 * */
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = info->bat_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (info->bat_online) {
			val->intval = max8925_get_bat_voltage(info) * 1000;
			break;
		}
		ret = -ENODATA;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (info->bat_online) {
			val->intval = max8925_get_bat_current(info) * 1000;
			ret = 0;
			break;
		}
		ret = -ENODATA;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (!info->bat_online) {
			ret = -ENODATA;
			break;
		}
		ret = max8925_get_bat_current(info);
		if (ret < -5)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (ret > 5)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (!info->bat_online) {
			ret = -ENODATA;
			break;
		}
		ret = max8925_get_bat_voltage(info);
		val->intval = get_capacity(ret);
		ret = 0;
		break;
	default:
		ret = -ENODEV;
		break;
	}
	return ret;
}

static enum power_supply_property max8925_battery_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
};

#define REQUEST_IRQ(_irq, _name)	\
do {	\
	ret = request_threaded_irq(chip->irq_base + _irq, NULL,	\
				    max8925_charger_handler,	\
				    IRQF_ONESHOT, _name, info);	\
	if (ret)	\
		dev_err(chip->dev, "Failed to request IRQ #%d: %d\n",	\
			_irq, ret);	\
} while (0)

#define FREE_IRQ(_irq)	\
do {	\
	free_irq(chip->irq_base + _irq, info);	\
} while (0)

static int max8925_init_charger(struct max8925_chip *chip,
				struct max8925_power_info *info)
{
	int ret = 0;

	if (info->batt_detect) {
		ret = max8925_reg_read(info->gpm, MAX8925_CHG_STATUS);
		info->bat_online = (ret & MAX8925_CHG_MBDET) ? 0 : 1;
	} else
		info->bat_online = 1;

	/* Disable max8925 charge by default */
	max8925_set_bits(info->gpm, MAX8925_CHG_CNTL1, 1 << 7, 1 << 7);

	return 0;
}

static void max8925_bat_external_power_changed(struct power_supply *psy)
{
	struct max8925_power_info *info =
			container_of(psy, struct max8925_power_info, battery);
	power_supply_changed(&info->battery);
}

static int max8925_power_powersupply_init(struct max8925_power_info *info)
{
	int ret = 0;

	/* Register Battery */
	if (info->bat_max8925_en) {
		info->battery.name = "max8925-battery";
		info->battery.type = POWER_SUPPLY_TYPE_BATTERY;
		info->battery.properties = max8925_battery_props;
		info->battery.num_properties =
		    ARRAY_SIZE(max8925_battery_props);
		info->battery.get_property = max8925_bat_get_prop;
		info->battery.external_power_changed =
				max8925_bat_external_power_changed;
		ret = power_supply_register(info->dev, &info->battery);
	}
	return ret;
}

static int max8925_power_powersupply_deinit(struct max8925_power_info *info)
{
	if (info->bat_max8925_en)
		power_supply_unregister(&info->battery);
	return 0;
}

static int max8925_power_probe(struct platform_device *pdev)
{
	struct max8925_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct max8925_power_pdata *pdata = NULL;
	struct max8925_power_info *info;
	int ret = 0;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "platform data isn't assigned to "
			"power supply\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(struct max8925_power_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->dev = &pdev->dev;
	info->chip = chip;
	info->gpm = chip->i2c;
	info->adc = chip->adc;
	/* Init platform data */
	info->batt_detect = pdata->batt_detect;
	info->topoff_threshold = pdata->topoff_threshold;
	info->fast_charge = pdata->fast_charge;
	info->set_led = pdata->set_led;

	info->bat_max8925_en = pdata->bat_max8925_en;
	info->chg_port_config = pdata->chg_port_config;

	info->bat_online = 0;
	info->supply_type = POWER_SUPPLY_TYPE_BATTERY;
	pw_info = info;
	platform_set_drvdata(pdev, info);

	/* Init max8925 charger */
	max8925_init_charger(chip, info);
	/* Register power supply items */
	ret = max8925_power_powersupply_init(info);
	if (ret) {
		dev_err(&pdev->dev, "failed to register power supply\n");
		goto err_power_supply_init;
	}
	INIT_DELAYED_WORK(&info->chg_event_work, chg_event_work_func);
	/* Request IRQs */
	REQUEST_IRQ(MAX8925_IRQ_VCHG_DC_F, "dc-remove");
	if (ret)
		goto err_request_dc_f;
	REQUEST_IRQ(MAX8925_IRQ_VCHG_DC_R, "dc-insert");
	if (ret)
		goto err_request_dc_r;

	if (info->chg_port_config == CHG_PORT_USB) {
		info->notif.notifier_call = max8925_vbus_notifier_callback;
#ifdef CONFIG_USB_PXA_U2O
		mv_udc_register_client(&info->notif);
#endif
	}
	/* Check if charger connected before boot up: >4.5V
	 * CHG_PORT_WALL: AC charger attached, start charging.
	 * CHG_PORT_USB: Charger type detected by usb gadget driver. */
	if (info->chg_port_config == CHG_PORT_WALL) {
		if ((start_measure(info, MEASURE_VCHG) * 2)
			> CHG_HIGH_LEVEL_TH) {
			info->supply_type = POWER_SUPPLY_TYPE_MAINS;
			max8925_charger_attach(info);
		}
	}

	return 0;

err_request_dc_r:
	FREE_IRQ(MAX8925_IRQ_VCHG_DC_R);
err_request_dc_f:
	max8925_power_powersupply_deinit(info);
err_power_supply_init:
	kfree(info);
	return ret;
}

static int max8925_power_remove(struct platform_device *pdev)
{
	struct max8925_power_info *info = platform_get_drvdata(pdev);
	struct max8925_chip *chip = info->chip;

	/* Disable charger */
	max8925_stop_charge(info);

	FREE_IRQ(MAX8925_IRQ_VCHG_DC_R);
	FREE_IRQ(MAX8925_IRQ_VCHG_DC_F);

	cancel_delayed_work_sync(&info->chg_event_work);
#ifdef CONFIG_USB_PXA_U2O
	mv_udc_unregister_client(&info->notif);
#endif
	max8925_power_powersupply_deinit(info);
	kfree(info);

	return 0;
}

static struct platform_driver max8925_power_driver = {
	.probe = max8925_power_probe,
	.remove = max8925_power_remove,
	.driver = {
		.name = "max8925-power",
	},
};

static int __init max8925_power_init(void)
{
	return platform_driver_register(&max8925_power_driver);
}

module_init(max8925_power_init);

static void __exit max8925_power_exit(void)
{
	platform_driver_unregister(&max8925_power_driver);
}

module_exit(max8925_power_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Power supply driver for MAX8925");
MODULE_ALIAS("platform:max8925-power");
