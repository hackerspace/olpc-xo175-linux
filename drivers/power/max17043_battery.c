#include <linux/i2c.h>
#include <linux/init.h>
#include <plat/pm.h>
#include <linux/power_supply.h>
#include <linux/max17043_battery.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/delay.h>

#ifdef CONFIG_MFD_88PM80X
#include <linux/mfd/88pm80x.h>
#endif

#define VOL_HIGH_THL	2500
#define VOL_LOW_THL	1500

#define TBAT_100D			8
#define TBAT_50D			30
#define TBAT_25D			240
#define TBAT_NEG_20D		1000
#define LOW_BAT			0x11	/* 15%*/
#define MAX17043_DEFAULT_DCAP	(1500)	/* Design Capacity: mAh */
#define uAh_to_uWh(val)	(val * 37 / 10)	/* Nominal voltage: 3.7v */

struct max17043_device_info {
	struct device *dev;
	struct i2c_client *client;
	struct power_supply battery;
	struct delayed_work battery_monitor_work;
	struct wake_lock alert_wake_lock;
	int present;
	int status;
	int capacity;
	int charge_full;	/* charge: µAh */
	int charge_now;
	int energy_full;	/* energy: µWh */
	int energy_now;
	int temp;
	int voltage;
	int healthy;
	int interval;
	int tech;
	bool alert_gpio_en;
	int alert_gpio;
	int bat_design_cap;
};
static int max17043_read_reg(struct i2c_client *client, u8 reg, u16 * data)
{
	int ret = 0;
	u16 value;
	if (!client || !data)
		return -EINVAL;
	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	}
	value = (u16) ret;
	value = ((value & 0xFF) << 8) | ((value & 0xFF00) >> 8);
	*data = value;
	return 0;
}

static int max17043_write_reg(struct i2c_client *client, u8 reg, u16 data)
{
	int ret = 0;

	if (!client)
		return -EINVAL;
	data = ((data & 0xFF) << 8) | ((data & 0xFF00) >> 8);
	ret = i2c_smbus_write_word_data(client, reg, data);
	if (ret < 0) {
		dev_err(&client->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			data, reg, ret);
		return ret;
	}
	return 0;
}

static int max17043_get_capacity(struct max17043_device_info *info)
{
	int ret = 0;
	u16 val = 0;
	ret = max17043_read_reg(info->client, MAX17043_SOC, &val);
	if (ret < 0)
		return ret;
	val = val >> 8;
	return val;
}

static int max17043_get_voltage(struct max17043_device_info *info)
{
	int ret = 0;
	u16 val = 0;
	ret = max17043_read_reg(info->client, MAX17043_VCELL, &val);
	if (ret < 0)
		return ret;
	val = (val >> 4) * 125 / 100;
	/*mV */
	return val;
}

static int max17043_quick_start(struct max17043_device_info *info)
{
	int ret = 0;
	ret = max17043_write_reg(info->client, MAX17043_MODE, 0x4000);
	return ret;
}

static enum power_supply_property max17043_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

#ifdef CONFIG_MFD_88PM80X
static int is_battery_removed(void)
{
	int data;
	data = pm80x_codec_reg_read((PM80X_BASE_PAGE << 8) | PM800_RTC_MISC5);
	return data & 0xF0;
}

static int max17043_get_GPADC_voltage(void)
{
	unsigned char buf[2];
	int data = 0;
	buf[1] = pm80x_codec_reg_read((PM80X_GPADC_PAGE << 8) |
				      PM800_VBAT_MEAS2);
	buf[0] =
	    pm80x_codec_reg_read((PM80X_GPADC_PAGE << 8) | PM800_VBAT_MEAS1);

	data = ((buf[0] & 0xFF) << 4) | (buf[1] & 0x0F);
	/*measure(mv) = value * 4*1.4 *1000/(2^12) */
	data = ((data & 0xFFF) * 5600) >> 12;
	return data;
}
#else
static int is_battery_removed(void)
{
	return 0;
}

static int max17043_get_GPADC_voltage(void)
{
	/*return a fake voltage */
	return 0;
}
#endif
static int max17043_check_battery_exist(struct max17043_device_info *info)
{
	int voltage;
	voltage = max17043_get_GPADC_voltage();
	if (voltage <= VOL_HIGH_THL)
		info->present = 1;
	else
		info->present = 0;

	return info->present;
}

static int max17043_get_battery_temp(struct max17043_device_info *info)
{
	int gpadc_voltage, bat_voltage, resistor;
	int temp;
	gpadc_voltage = max17043_get_GPADC_voltage();
	bat_voltage = max17043_get_voltage(info);
	if (bat_voltage <= gpadc_voltage)
		return 0;
	if (gpadc_voltage == 0)
		return 0;
	resistor = (100 * gpadc_voltage) / (bat_voltage - gpadc_voltage);

	if (resistor < TBAT_100D)
		temp = 100;
	else if (resistor < TBAT_50D)
		temp = 50;
	else if (resistor < TBAT_25D)
		temp = 25;
	else if (resistor < TBAT_NEG_20D)
		temp = -20;
	else
		temp = -40;
	return temp;
}

static int get_charger_status(struct max17043_device_info *info)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int i, ret = 0;

	for (i = 0; i < info->battery.num_supplicants; i++) {
		psy = power_supply_get_by_name(info->battery.supplied_to[i]);
		if (!psy || !psy->get_property)
			continue;
		ret = psy->get_property(psy, POWER_SUPPLY_PROP_STATUS, &val);
		if (ret == 0)
			return val.intval;
	}
	return 0;
}

static int is_charger_online(struct max17043_device_info *info)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int i, ret = 0;

	for (i = 0; i < info->battery.num_supplicants; i++) {
		psy = power_supply_get_by_name(info->battery.supplied_to[i]);
		if (!psy || !psy->get_property)
			continue;
		ret = psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val);
		if (ret == 0 && val.intval == 1)
			return 1;
	}
	return 0;
}

static char *max17043_supply_to[] = {
	"ac",
	"usb",
};

static void max17043_battery_update_status(struct max17043_device_info *info)
{
	int cap = 0;
	info->healthy = POWER_SUPPLY_HEALTH_GOOD;
	info->tech = POWER_SUPPLY_TECHNOLOGY_LION;
	info->temp = max17043_get_battery_temp(info);

	cap = max17043_get_capacity(info);
	if (!is_charger_online(info)) {
		if (cap > info->capacity) {
			cap = info->capacity;
			max17043_quick_start(info);
			msleep(2000);
			cap = max17043_get_capacity(info);
			pr_info("old cap = %d, new cap = %d\n", info->capacity,
				cap);
		}
	}

	if (cap > 100)
		cap = 100;
	else if (cap < 0)
		cap = 0;
	info->capacity = cap;

	if (is_charger_online(info))
		info->status = get_charger_status(info);
	else
		info->status = POWER_SUPPLY_STATUS_DISCHARGING;
	if (info->capacity >= 100)
		info->status = POWER_SUPPLY_STATUS_FULL;

	info->voltage = max17043_get_voltage(info);
	/*charge: uAh*/
	info->charge_full = info->bat_design_cap * 1000;
	info->charge_now = info->bat_design_cap * info->capacity * 10;
	/*Energy: uWh*/
	info->energy_full = uAh_to_uWh(info->charge_full);
	info->energy_now = uAh_to_uWh(info->charge_now);
}

static irqreturn_t max17043_alert_irq_handler(int irq, void *data)
{
	struct max17043_device_info *info = data;
	u16 value = 0;
	max17043_read_reg(info->client, MAX17043_CONFIG, &value);
	/*clear alrt bit */
	value &= ~MAX17043_ALERT;
	max17043_write_reg(info->client, MAX17043_CONFIG, value);
	pr_info("low battery alert\n");
	/* Leave enough time for android to power off */
	wake_lock_timeout(&info->alert_wake_lock, 5 * HZ);
	cancel_delayed_work_sync(&info->battery_monitor_work);
	schedule_delayed_work(&info->battery_monitor_work, HZ / 10);
	return IRQ_HANDLED;
}

static void max17043_battery_external_power_changed(struct power_supply *psy)
{
	struct max17043_device_info *info =
	    container_of(psy, struct max17043_device_info, battery);
	cancel_delayed_work(&info->battery_monitor_work);
	schedule_delayed_work(&info->battery_monitor_work, HZ / 2);
}

static void max17043_battery_monitor_work_func(struct work_struct *work)
{
	struct max17043_device_info *info =
	    container_of(work, struct max17043_device_info,
			 battery_monitor_work.work);
	max17043_battery_update_status(info);
	power_supply_changed(&info->battery);
	schedule_delayed_work(&info->battery_monitor_work, info->interval);
}

static int max17043_battery_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct max17043_device_info *info =
	    container_of(psy, struct max17043_device_info, battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = info->present;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = info->status;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (info->present)
			val->intval = info->capacity;
		else
			val->intval = 80;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = info->charge_full;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = info->charge_now;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		val->intval = info->energy_full;
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		val->intval = info->energy_now;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (info->present && is_charger_online(info))
			val->intval = info->temp;
		else
			val->intval = 25;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = info->healthy;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = info->tech;
		break;
		/* NOTE: voltage keep changing, read real-time value here */
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = max17043_get_voltage(info);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int max17043_powersupply_init(struct max17043_device_info *info)
{
	int ret = 0;
	info->battery.name = "battery";
	info->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	info->battery.properties = max17043_bat_props;
	info->battery.num_properties = ARRAY_SIZE(max17043_bat_props);
	info->battery.get_property = max17043_battery_get_property;
	info->battery.external_power_changed =
	    max17043_battery_external_power_changed;
	info->battery.supplied_to = max17043_supply_to;
	info->battery.num_supplicants = ARRAY_SIZE(max17043_supply_to);
	ret = power_supply_register(info->dev, &info->battery);
	return ret;
}

static int __devinit max17043_battery_probe(struct i2c_client *client,
					    const struct i2c_device_id *id)
{
	struct max17043_device_info *info;
	struct max17043_battery_pdata *pdata;
	int ret = 0;
	u16 version, config;
	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "missing plarform data\n");
		return -EINVAL;
	}
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}
	info->interval = pdata->interval * HZ;
	info->client = client;
	info->dev = &client->dev;
	info->alert_gpio_en = pdata->gpio_en;
	info->alert_gpio = pdata->gpio;
	if (pdata->bat_design_cap)
		info->bat_design_cap = pdata->bat_design_cap;
	else
		info->bat_design_cap = MAX17043_DEFAULT_DCAP;
	i2c_set_clientdata(client, info);

	max17043_read_reg(info->client, MAX17043_VERSION, &version);
	if (version != 0x03) {
		ret = -EINVAL;
		goto err_check_version;
	}

	max17043_read_reg(info->client, MAX17043_CONFIG, &config);
	/*clear alert bit*/
	config &= ~MAX17043_ALERT;
	/*set low battery alert*/
	config = (config & 0xFFE0) | LOW_BAT;
	max17043_write_reg(info->client, MAX17043_CONFIG, config);

	info->capacity = max17043_get_capacity(info);
	if (info->capacity > 100)
		info->capacity = 100;
	else if (info->capacity < 0)
		info->capacity = 0;

	max17043_powersupply_init(info);
	max17043_check_battery_exist(info);
	max17043_battery_update_status(info);
	wake_lock_init(&info->alert_wake_lock, WAKE_LOCK_SUSPEND,
		       "max17043-battery");
	INIT_DELAYED_WORK(&info->battery_monitor_work,
			  max17043_battery_monitor_work_func);
	schedule_delayed_work(&info->battery_monitor_work, info->interval);

	if (info->alert_gpio_en) {
		ret = gpio_request(info->alert_gpio, "Low Battery Alert");
		if (ret) {
			dev_err(info->dev, "failed to request GPIO%d\n",
				info->alert_gpio);
			goto err_gpio_request;
		}
		gpio_direction_input(info->alert_gpio);
		gpio_free(info->alert_gpio);
		ret = request_threaded_irq(gpio_to_irq(info->alert_gpio),
					   NULL, max17043_alert_irq_handler,
					   IRQF_ONESHOT | IRQF_NO_SUSPEND
					   | IRQF_TRIGGER_FALLING,
					   "Low Battery Alert IRQ", info);
		if (ret) {
			dev_err(info->dev, "failed to request alert irq\n");
			goto err_request_irq;
		}
	}

	dev_info(&client->dev, "fuel guage enable\n");
	return 0;

err_request_irq:
err_gpio_request:
	cancel_delayed_work_sync(&info->battery_monitor_work);
	power_supply_unregister(&info->battery);
err_check_version:
	kfree(info);
	return ret;
}

static int __devexit max17043_battery_remove(struct i2c_client *client)
{
	struct max17043_device_info *info = i2c_get_clientdata(client);

	if (info->alert_gpio_en)
		free_irq(gpio_to_irq(info->alert_gpio), info);
	cancel_delayed_work_sync(&info->battery_monitor_work);
	power_supply_unregister(&info->battery);
	kfree(info);
	return 0;
}

#ifdef CONFIG_PM
static int max17043_battery_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max17043_device_info *info = i2c_get_clientdata(client);
	u16 config;
	max17043_read_reg(info->client, MAX17043_CONFIG, &config);
	/*set sleep bit*/
	config |= MAX17043_sleep;
	max17043_write_reg(info->client, MAX17043_CONFIG, config);
	cancel_delayed_work_sync(&info->battery_monitor_work);
	return 0;
}

static int max17043_battery_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct max17043_device_info *info = i2c_get_clientdata(client);
	u16 config;
	max17043_read_reg(info->client, MAX17043_CONFIG, &config);
	/*clear sleep bit*/
	config &= ~MAX17043_sleep;
	max17043_write_reg(info->client, MAX17043_CONFIG, config);
	schedule_delayed_work(&info->battery_monitor_work, HZ / 2);
	return 0;
}

static const struct dev_pm_ops max17043_pm_ops = {
	.suspend = max17043_battery_suspend,
	.resume = max17043_battery_resume,
};
#endif

static const struct i2c_device_id max17043_id[] = {
	{"max17043", -1},
};

static struct i2c_driver max17043_battery_driver = {
	.driver = {
		   .name = "max17043-battery",
#ifdef CONFIG_PM
		   .pm = &max17043_pm_ops,
#endif
		   },
	.probe = max17043_battery_probe,
	.remove = __devexit_p(max17043_battery_remove),
	.id_table = max17043_id,
};

static int __init max17043_battery_init(void)
{
	int ret;
	ret = i2c_add_driver(&max17043_battery_driver);
	if (ret)
		pr_err("Unable to register max17043 fuel gauge driver");
	return ret;
}

module_init(max17043_battery_init);

static void __exit max17043_battery_exit(void)
{
	i2c_del_driver(&max17043_battery_driver);
}

module_exit(max17043_battery_exit);

MODULE_AUTHOR("Wenzeng Chen<wzch@marvell.com>");
MODULE_DESCRIPTION("MAX17043 Fuel Gauge");
MODULE_LICENSE("GPL");
