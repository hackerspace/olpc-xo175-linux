/*
 * Fuel gauge driver for Maxim 17042 / 8966 / 8997
 *  Note that Maxim 8966 and 8997 are mfd and this is its subdevice.
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
 * This driver is based on max17040_battery.c
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/power/max17042_battery.h>

struct max17042_battery_params {
	int status;
	int present;
	int volt;
	int cur;
	int cap;
	int temp;
	int health;
	int tech;
};

struct max17042_device_info {
	struct device *dev;
	struct power_supply bat;
	struct i2c_client *client;
	struct max17042_battery_params bat_params;
	struct delayed_work bat_monitor_work;
	struct wake_lock alert_wake_lock;
	unsigned int bat_design_cap;
	unsigned int bat_ichg_term;
	unsigned int r_sns;
	u8 rsvd_cap;
	unsigned int interval;
	int alert_gpio;
	bool alert_gpio_en;
	/* Charging indicator led */
	int (*is_charging_led)(int);
};

#define MAX17042_DEFAULT_DCAP	(1400 * 2)	/* Design Capacity: mAh */
#define MAX17042_DEFAULT_ICHG_TERM	(20)	/* Charge termination current */
#define MAX17042_DEFAULT_R_SNS		(10000)	/* mirco-ohms */
#define MAX17042_DEFAULT_INTERVAL	(60 * HZ)

static int max17042_read_reg(struct i2c_client *client, u8 reg, u16 *data)
{
	int ret = 0;

	if (!client || !data)
		return -EINVAL;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	}
	*data = ret;
	return 0;
}

static int max17042_write_reg(struct i2c_client *client, u8 reg, u16 data)
{
	int ret = 0;

	if (!client)
		return -EINVAL;

	ret = i2c_smbus_write_word_data(client, reg, data);
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			data, reg, ret);
		return ret;
	}
	return 0;
}

/* Capacity: % */
static int max17042_get_capacity(struct max17042_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = max17042_read_reg(di->client, MAX17042_RepSOC, &val);
	if (ret < 0)
		return ret;
	ret = val >> 8;
	return ret;
}

/* Voltage: µV */
static int max17042_get_voltage(struct max17042_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = max17042_read_reg(di->client, MAX17042_VCELL, &val);
	if (ret < 0)
		return ret;
	ret = (val >> 3) * 625;
	return ret;
}

/* Current: µA */
static int max17042_get_current(struct max17042_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = max17042_read_reg(di->client, MAX17042_Current, &val);
	if (ret < 0)
		return ret;
	if (0x8000 & val)
		ret = ((~val & 0x7FFF) + 1) * -1;
	else
		ret = val;
	ret *= (1562500 / di->r_sns);
	return ret;
}

/* Temperature: 0.1C */
static int max17042_get_temperature(struct max17042_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = max17042_read_reg(di->client, MAX17042_TEMP, &val);
	if (ret < 0)
		return ret;
	ret = (val >> 8) * 10;
	return ret;
}

static int max17042_get_status(struct max17042_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = max17042_read_reg(di->client, MAX17042_STATUS, &val);
	if (ret < 0)
		return ret;
	return val;
}

static int max17042_get_design_cap(struct max17042_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = max17042_read_reg(di->client, MAX17042_DesignCap, &val);
	if (ret < 0)
		return ret;
	ret = (val * 5000) / di->r_sns;
	return ret;
}

static int max17042_get_ichg_term(struct max17042_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = max17042_read_reg(di->client, MAX17042_ICHGTerm, &val);
	if (ret < 0)
		return ret;
	ret = val * (1562500 / di->r_sns);
	return ret;
}

static int max17042_get_config(struct max17042_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = max17042_read_reg(di->client, MAX17042_CONFIG, &val);
	if (ret < 0)
		return ret;
	return val;
}

static int max17042_get_soc_alert(struct max17042_device_info *di)
{
	int ret = 0;
	u16 val = 0;
	ret = max17042_read_reg(di->client, MAX17042_SALRT_Th, &val);
	if (ret < 0)
		return ret;
	return val;
}

static int max17042_set_design_cap(struct max17042_device_info *di, int val)
{
	u16 data = 0;
	data = val * (di->r_sns / 5000);
	return max17042_write_reg(di->client, MAX17042_DesignCap, data);
}

static int max17042_set_ichg_term(struct max17042_device_info *di, int val)
{
	u16 data = 0;
	data = val / (1562500 / di->r_sns);
	return max17042_write_reg(di->client, MAX17042_ICHGTerm, data);
}

static int max17042_set_config(struct max17042_device_info *di, int val)
{
	return max17042_write_reg(di->client, MAX17042_CONFIG, val);
}

static int max17042_set_soc_alert(struct max17042_device_info *di, int val)
{
	return max17042_write_reg(di->client, MAX17042_SALRT_Th, val);
}

/* Update battery status */
static void max17042_bat_update_status(struct max17042_device_info *di)
{
	int cap = 0, rsvd_cap = di->rsvd_cap;

	/* Battery presence state */
	if (max17042_get_status(di) & MAX17042_STATUS_BST)
		di->bat_params.present = 0;
	else
		di->bat_params.present = 1;
	/* NOTE: hardcode battery type[Lion] and health[Good] */
	di->bat_params.health = POWER_SUPPLY_HEALTH_GOOD;
	di->bat_params.tech = POWER_SUPPLY_TECHNOLOGY_LION;
	/* Battery temperature */
	di->bat_params.temp = max17042_get_temperature(di);

	/* Capacity: % */
	cap = max17042_get_capacity(di);
	if (cap > 100)
		cap = 100;
	else if (cap < 0)
		cap = 0;
	if (rsvd_cap > 0 && rsvd_cap <= 20 && cap <= 80) {
		/* Extend capacity from [80 ~ rsvd_cap]% to [80 ~ 0]%.
		 * if cap <= rsvd_cap, cap = 0 */
		if (cap >= rsvd_cap)
			cap = cap - (rsvd_cap * rsvd_cap) / cap;
		else
			cap = 0;
		di->bat_params.cap = cap;
	} else {
		/* actual capacity */
		di->bat_params.cap = cap;
	}

	/* Voltage */
	di->bat_params.volt = max17042_get_voltage(di);
	/* Charging status */
	di->bat_params.cur = max17042_get_current(di);
	if (di->bat_params.cur > 50000)
		di->bat_params.status = POWER_SUPPLY_STATUS_CHARGING;
	else if (di->bat_params.cur < -30000)
		di->bat_params.status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (di->bat_params.cap >= 100)
		di->bat_params.status = POWER_SUPPLY_STATUS_FULL;
	else
		di->bat_params.status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	/* Charging led */
	if (di->is_charging_led) {
		if (di->bat_params.status == POWER_SUPPLY_STATUS_CHARGING)
			di->is_charging_led(1);
		else
			di->is_charging_led(0);
	}
}

static irqreturn_t max17042_alert_irq_handler(int irq, void *data)
{
	struct max17042_device_info *di = data;

	dev_err(di->dev, "Low Battery Alert!!!\n");
	/* Leave enough time for android to power off */
	wake_lock_timeout(&di->alert_wake_lock, 5 * HZ);
	cancel_delayed_work_sync(&di->bat_monitor_work);
	schedule_delayed_work(&di->bat_monitor_work, HZ / 10);
	return IRQ_HANDLED;
}

static int max17042_fuel_guage_setup(struct max17042_device_info *di,
				struct max17042_platform_data *pdata)
{
	int ret = 0;

	/* Design Capacity */
	if (pdata->bat_design_cap)
		di->bat_design_cap = pdata->bat_design_cap;
	else
		di->bat_design_cap = MAX17042_DEFAULT_DCAP;
	/* Charge termination current */
	if (pdata->bat_ichg_term)
		di->bat_ichg_term = pdata->bat_ichg_term;
	else
		di->bat_ichg_term = MAX17042_DEFAULT_ICHG_TERM;
	/* Current sensor resistor */
	if (pdata->r_sns)
		di->r_sns = pdata->r_sns;
	else
		di->r_sns = MAX17042_DEFAULT_R_SNS;
	/* Battery monitor interval */
	if (pdata->monitor_interval)
		di->interval = pdata->monitor_interval * HZ;
	else
		di->interval = MAX17042_DEFAULT_INTERVAL;
	/* Reserved capacity */
	if (pdata->rsvd_cap >= 0 && pdata->rsvd_cap <= 20)
		di->rsvd_cap = pdata->rsvd_cap;
	else
		di->rsvd_cap = 0;
	/* Low battery alert GPIO */
	di->alert_gpio_en = pdata->alert_gpio_en;
	di->alert_gpio = pdata->alert_gpio;
	/* Charging indicator led */
	di->is_charging_led = pdata->is_charging_led;

	/* Set SOC alert threshold, low battery(0%) protection */
	ret = max17042_set_soc_alert(di, 0xFF00 | (di->rsvd_cap + 1));
	if (ret < 0)
		return ret;
	ret = max17042_get_soc_alert(di);
	if (ret < 0)
		return ret;
	dev_info(di->dev, "SOC Alert Threshold: 0x%04x\n", ret);
	/* Enable alert output */
	ret = max17042_get_config(di);
	if (ret < 0)
		return ret;
	ret = max17042_set_config(di, ret | MAX17042_CONFIG_AEN);
	if (ret < 0)
		return ret;
	ret = max17042_get_config(di);
	if (ret < 0)
		return ret;
	dev_info(di->dev, "Config Reg: 0x%04x\n", ret);

	/* Configure Design Capacity */
	ret = max17042_set_design_cap(di, di->bat_design_cap);
	if (ret < 0)
		return ret;
	dev_info(di->dev, "DesignCap = %dmAh\n", max17042_get_design_cap(di));
	/* Configure Charge Termination Current */
	ret = max17042_set_ichg_term(di, di->bat_ichg_term);
	if (ret < 0)
		return ret;
	dev_info(di->dev, "ICHGTerm = %duA\n", max17042_get_ichg_term(di));

	return 0;
}

static void max17042_bat_monitor_work_func(struct work_struct *work)
{
	struct max17042_device_info *di =
			container_of(work, struct max17042_device_info,
						bat_monitor_work.work);

	max17042_bat_update_status(di);
	power_supply_changed(&di->bat);
	/* Reschedule for the next time */
	schedule_delayed_work(&di->bat_monitor_work, di->interval);
}

static enum power_supply_property max17042_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static int max17042_bat_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct max17042_device_info *di =
		container_of(psy, struct max17042_device_info, bat);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->bat_params.present;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->bat_params.status;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = di->bat_params.cap;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->bat_params.temp;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = di->bat_params.health;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = di->bat_params.tech;
		break;
	/* NOTE: voltage&current keep changing, read real-time value here */
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = max17042_get_voltage(di);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = max17042_get_current(di);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void max17042_bat_external_power_changed(struct power_supply *psy)
{
	struct max17042_device_info *di =
			container_of(psy, struct max17042_device_info, bat);
	cancel_delayed_work_sync(&di->bat_monitor_work);
	schedule_delayed_work(&di->bat_monitor_work, HZ / 2);
}

static int max17042_powersupply_init(struct max17042_device_info *di)
{
	int ret = 0;

	/* register ac battery props */
	di->bat.name = "max17042-battery";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = max17042_bat_props;
	di->bat.num_properties = ARRAY_SIZE(max17042_bat_props);
	di->bat.get_property = max17042_bat_get_property;
	di->bat.external_power_changed = max17042_bat_external_power_changed;

	ret = power_supply_register(di->dev, &di->bat);
	return ret;
}

static int __devinit max17042_battery_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17042_device_info *di;
	struct max17042_platform_data *pdata;
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;
	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "missing platform data\n");
		return -EINVAL;
	}
	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}
	i2c_set_clientdata(client, di);
	di->client = client;
	di->dev = &client->dev;

	ret = max17042_fuel_guage_setup(di, pdata);
	if (ret < 0)
		goto err_chip_setup;
	/* Update battery status */
	max17042_bat_update_status(di);
	/* Register power supply device for battery */
	ret = max17042_powersupply_init(di);
	if (ret) {
		dev_err(&client->dev, "failed to register battery\n");
		goto err_power_supply_reg;
	}
	wake_lock_init(&di->alert_wake_lock, WAKE_LOCK_SUSPEND,
					"max17042-battery");
	INIT_DELAYED_WORK(&di->bat_monitor_work,
			max17042_bat_monitor_work_func);
	schedule_delayed_work(&di->bat_monitor_work, di->interval);
	/* Alert GPIO setup */
	if (di->alert_gpio_en) {
		ret = gpio_request(di->alert_gpio, "Low Battery Alert");
		if (ret) {
			dev_err(di->dev, "failed to request GPIO%d\n",
					di->alert_gpio);
			goto err_gpio_request;
		}
		gpio_direction_input(di->alert_gpio);
		gpio_free(di->alert_gpio);
		ret = request_threaded_irq(gpio_to_irq(di->alert_gpio),
				NULL, max17042_alert_irq_handler,
				IRQF_ONESHOT | IRQF_NO_SUSPEND
				| IRQF_TRIGGER_FALLING,
				"Low Battery Alert IRQ", di);
		if (ret) {
			dev_err(di->dev, "failed to request alert irq\n");
			goto err_request_irq;
		}
	}

	dev_info(&client->dev, "fuel guage enabled\n");
	return 0;

err_request_irq:
err_gpio_request:
	cancel_delayed_work_sync(&di->bat_monitor_work);
	wake_lock_destroy(&di->alert_wake_lock);
	power_supply_unregister(&di->bat);
err_power_supply_reg:
err_chip_setup:
	kfree(di);
	return ret;
}

static int __devexit max17042_battery_remove(struct i2c_client *client)
{
	struct max17042_device_info *di = i2c_get_clientdata(client);

	if (di->alert_gpio_en)
		free_irq(gpio_to_irq(di->alert_gpio), di);
	cancel_delayed_work_sync(&di->bat_monitor_work);
	wake_lock_destroy(&di->alert_wake_lock);
	power_supply_unregister(&di->bat);
	kfree(di);
	return 0;
}

#ifdef CONFIG_PM
static int max17042_battery_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max17042_device_info *di = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&di->bat_monitor_work);
	return 0;
}

static int max17042_battery_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max17042_device_info *di = i2c_get_clientdata(client);

	schedule_delayed_work(&di->bat_monitor_work, HZ / 2);
	return 0;
}

static const struct dev_pm_ops max17042_pm_ops = {
	.suspend = max17042_battery_suspend,
	.resume = max17042_battery_resume,
};
#endif

static const struct i2c_device_id max17042_id[] = {
	{"max17042", -1},
	{},
};
MODULE_DEVICE_TABLE(i2c, max17042_id);

static struct i2c_driver max17042_battery_driver = {
	.driver = {
		.name = "max17042-battery",
#ifdef CONFIG_PM
		.pm = &max17042_pm_ops,
#endif
	},
	.probe = max17042_battery_probe,
	.remove = __devexit_p(max17042_battery_remove),
	.id_table = max17042_id,
};

static int __init max17042_battery_init(void)
{
	int ret;
	ret = i2c_add_driver(&max17042_battery_driver);
	if (ret)
		pr_err("Unable to register max17042 fuel gauge driver");
	return ret;
}

module_init(max17042_battery_init);

static void __exit max17042_battery_exit(void)
{
	i2c_del_driver(&max17042_battery_driver);
}

module_exit(max17042_battery_exit);

MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_DESCRIPTION("MAX17042 Fuel Gauge");
MODULE_LICENSE("GPL");
