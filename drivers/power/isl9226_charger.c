/*
 * ISL9226 system voltage regulator and battery charger driver.
 *
 * Copyright (c) 2011 Marvell Technology Ltd.
 * Wenzeng Chen <wzch@marvell.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/power/isl9226.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/wakelock.h>
#include <linux/proc_fs.h>
#include <plat/pm.h>
#include <plat/usb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

struct isl9226_device_info {
	struct device *dev;
	struct i2c_client *client;
	struct power_supply ac;
	struct power_supply usb;
	struct notifier_block chg_notif;
	int ac_chg_online;
	int usb_chg_online;
	unsigned char chg_cur_in;
	unsigned char chg_cur_out;
	unsigned char prechg_cur;
	unsigned char prechg_vol;
	unsigned char eoc_cur;
	unsigned char usb_chg_cur;
	unsigned char default_chg_cur;
	unsigned char ac_chg_cur;
};
static const unsigned int input_current_table[] = {
	95, 475, 855, 950, 1425, 2900, 2900, 2900,
};
static const unsigned int eoc_current_table[] = {
	50, 75, 100,
};
static const unsigned int prechg_current_table[] = {
	130, 150, 180,
};
static const unsigned int prechg_voltage_table[] = {
	3000, 2900, 2800, 2700,
};
static char *isl9226_supply_to[] = {
	"battery",
};

static enum power_supply_property isl9226_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property isl9226_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int isl9226_get_input_current(unsigned int cur)
{
	int i, ret = -ENOENT;
	for (i = 0; i < sizeof(input_current_table); i++) {
		if (cur == input_current_table[i]) {
			ret = i;
			break;
		}
	}
	if (ret < 0) {
		ret = 0;
		pr_err("invalid precharge current %d mA\n", cur);
	}
	return ret;
}

static int isl9226_get_eoc_current(unsigned int cur)
{
	int i, ret = -ENOENT;
	for (i = 0; i < sizeof(eoc_current_table); i++) {
		if (cur == eoc_current_table[i]) {
			ret = i;
			break;
		}
	}
	if (ret < 0) {
		ret = 0;
		pr_err("invalid end of charge current %d mA\n", cur);
	}
	return ret;
}

static int isl9226_get_prechg_current(unsigned int cur)
{
	int i, ret = -ENOENT;
	for (i = 0; i < sizeof(prechg_current_table); i++) {
		if (cur == prechg_current_table[i]) {
			ret = i;
			break;
		}
	}
	if (ret < 0) {
		ret = 0;
		pr_err("invalid precharge current %d mA\n", cur);
	}
	return ret;
}

static int isl9226_get_prechg_voltage(unsigned int vol)
{
	int i, ret = -ENOENT;
	for (i = 0; i < sizeof(prechg_voltage_table); i++) {
		if (vol == prechg_voltage_table[i]) {
			ret = i;
			break;
		}
	}
	if (ret < 0) {
		ret = 0;
		pr_err("invalid precharge voltage %d mV\n", vol);
	}
	return ret;
}

static int isl9226_write_reg(struct i2c_client *client, unsigned char reg,
			     unsigned char data)
{
	int ret = 0;
	if (!client)
		return -EINVAL;
	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0) {
		dev_err(&client->dev, "isl9226: failed to write reg-0x%02x\n",
			reg);
		return ret;
	}
	return 0;
}

static int isl9226_read_reg(struct i2c_client *client, unsigned char reg)
{
	int ret = 0;
	if (!client)
		return -EINVAL;
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "isl9226: failed to read reg-0x%02x\n",
			reg);
		return ret;
	}
	return (unsigned char)ret;
}

static int isl9226_set_bits(struct i2c_client *client, unsigned char reg,
			    unsigned char mask, unsigned char data)
{
	unsigned char valget, valset;
	int ret;

	/*need to add mutex protection */
	valget = isl9226_read_reg(client, reg);
	valset = valget;
	valset &= ~mask;
	valset |= data;
	ret = isl9226_write_reg(client, reg, valset);
	return ret;
}

static int isl9226_has_supplicant(struct isl9226_device_info *info)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int i, ret = 0;

	for (i = 0; i < info->ac.num_supplicants; i++) {
		psy = power_supply_get_by_name(info->ac.supplied_to[i]);
		if (!psy || !psy->get_property)
			continue;
		ret = psy->get_property(psy, POWER_SUPPLY_PROP_PRESENT, &val);
		if (ret == 0 && val.intval == 1)
			return 1;
	}
	for (i = 0; i < info->usb.num_supplicants; i++) {
		psy = power_supply_get_by_name(info->usb.supplied_to[i]);
		if (!psy || !psy->get_property)
			continue;
		ret = psy->get_property(psy, POWER_SUPPLY_PROP_PRESENT, &val);
		if (ret == 0 && val.intval == 1)
			return 1;
	}
	return 0;
}

/*to save power isl9226 will power off after USB/AC
*cable pulg out, so set all the register
*/
static int isl9226_start_charging(struct isl9226_device_info *info)
{
	unsigned char value;
	if (!isl9226_has_supplicant(info))
		return 0;
	value =
	    ((info->eoc_cur & 0x3) << 6) | ((info->prechg_cur & 0x03) << 4) |
	    (info->chg_cur_out & 0x07);

	/*set EOC, precharge and output current */
	isl9226_write_reg(info->client, CCTRL1, value);

	/*set input current */
	isl9226_write_reg(info->client, CCTRL2, info->chg_cur_in & 0x07);

	/*set precharge voltage, charge voltage 4.2v */
	isl9226_write_reg(info->client, VCTRL1, ((info->prechg_vol & 0x03) << 6) | 0x13);

	/*enable charge, auto-recharge, auto-stop */
	isl9226_write_reg(info->client, OPMOD, 0x27);
	return 0;
}

static void isl9226_stop_charging(struct isl9226_device_info *info)
{
	if (info->usb_chg_online || info->ac_chg_online)
		isl9226_write_reg(info->client, OPMOD, 0);
}

static int isl9226_chg_notifier_callback(struct notifier_block *nb,
					 unsigned long type, void *chg_event)
{
	struct isl9226_device_info *info =
	    container_of(nb, struct isl9226_device_info, chg_notif);
	switch (type) {
	case NULL_CHARGER:
		info->ac_chg_online = 0;
		info->usb_chg_online = 0;
		break;
	case DEFAULT_CHARGER:
		info->chg_cur_in = info->default_chg_cur;
		info->chg_cur_out = 0;
		info->usb_chg_online = 1;
		break;
	case VBUS_CHARGER:
		info->chg_cur_in = info->usb_chg_cur;
		/*set output current higher than input */
		if (info->usb_chg_cur <= 500)
			info->chg_cur_out = 0;
		else
			info->chg_cur_out = 3;
		info->usb_chg_online = 1;
		break;
	case AC_CHARGER_STANDARD:
	case AC_CHARGER_OTHER:
		info->chg_cur_in = info->ac_chg_cur;
		/*set output current higher than input */
		if (info->ac_chg_cur <= 1000)
			info->chg_cur_out = 4;
		else
			info->chg_cur_out = 6;
		info->ac_chg_online = 1;
		break;
	default:
		break;
	}
	if (info->usb_chg_online || info->ac_chg_online) {
		isl9226_start_charging(info);
	} else {
		/*cable plug out, isl9226 power down, i2c read write error */
		isl9226_stop_charging(info);
	}
	power_supply_changed(&info->ac);
	power_supply_changed(&info->usb);
	return 0;
}

static int isl9226_ac_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct isl9226_device_info *info =
	    container_of(psy, struct isl9226_device_info, ac);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = info->ac_chg_online;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int isl9226_usb_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct isl9226_device_info *info =
	    container_of(psy, struct isl9226_device_info, usb);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = info->usb_chg_online;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int isl9226_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct isl9226_device_info *info;
	struct isl9226_charger_pdata *pdata;
	int ret = 0;
	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "missing platform data\n");
		return -EINVAL;
	}
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}
	info->client = client;
	info->dev = &client->dev;
	i2c_set_clientdata(client, info);
	info->eoc_cur = isl9226_get_eoc_current(pdata->eoc_current);
	info->prechg_cur = isl9226_get_prechg_current(pdata->prechg_current);
	info->prechg_vol = isl9226_get_prechg_voltage(pdata->prechg_voltage);
	info->usb_chg_cur = isl9226_get_input_current(pdata->usb_input_current);
	info->ac_chg_cur = isl9226_get_input_current(pdata->ac_input_current);
	info->default_chg_cur =
	    isl9226_get_input_current(pdata->default_input_current);
	info->ac.name = "ac";
	info->ac.type = POWER_SUPPLY_TYPE_MAINS;
	info->ac.supplied_to = isl9226_supply_to;
	info->ac.num_supplicants = ARRAY_SIZE(isl9226_supply_to);
	info->ac.get_property = isl9226_ac_get_property;
	info->ac.properties = isl9226_ac_props;
	info->ac.num_properties = ARRAY_SIZE(isl9226_ac_props);

	ret = power_supply_register(info->dev, &info->ac);
	if (ret) {
		dev_err(&client->dev,
			"AC power supply resisteration failed! \n");
		goto out;
	}
	info->usb.name = "usb";
	info->usb.type = POWER_SUPPLY_TYPE_USB;
	info->usb.supplied_to = isl9226_supply_to;
	info->usb.num_supplicants = ARRAY_SIZE(isl9226_supply_to);
	info->usb.get_property = isl9226_usb_get_property;
	info->usb.properties = isl9226_usb_props;
	info->usb.num_properties = ARRAY_SIZE(isl9226_usb_props);
	ret = power_supply_register(info->dev, &info->usb);
	if (ret) {
		dev_err(&client->dev,
			"USB power supply resisteration failed! \n");
		goto out;
	}

	info->chg_notif.notifier_call = isl9226_chg_notifier_callback;
#ifdef CONFIG_USB_GADGET_PXA_U2O
	ret = mv_udc_register_client(&info->chg_notif);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register  client: %d\n", ret);
		goto out;
	}
#endif

	dev_info(&client->dev, "isl9226 probe finished\n");
	return 0;
out:
	kfree(info);
	return ret;
}

static int isl9226_charger_remove(struct i2c_client *client)
{
	struct isl9226_device_info *info = i2c_get_clientdata(client);
	isl9226_stop_charging(info);
	mv_udc_unregister_client(&info->chg_notif);
	power_supply_unregister(&info->usb);
	power_supply_unregister(&info->ac);
	kfree(info);
	return 0;
}

#ifdef CONFIG_PM
static int isl9226_charger_suspend(struct device *dev)
{
	return 0;
}

static int isl9226_charger_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops isl9226_pm_ops = {
	.suspend = isl9226_charger_suspend,
	.resume = isl9226_charger_resume,
};

#endif
static void isl9226_charger_shutdown(struct i2c_client *client)
{
	struct isl9226_device_info *info = i2c_get_clientdata(client);

	/* stop Charger */
	isl9226_stop_charging(info);
}

static const struct i2c_device_id isl9226_id[] = {
	{"isl9226", -1},
};

static struct i2c_driver isl9226_charger_driver = {
	.driver = {.name = "isl9226-charger",
#ifdef CONFIG_PM
		   .pm = &isl9226_pm_ops,
#endif
		   },
	.probe = isl9226_charger_probe,
	.remove = isl9226_charger_remove,
	.shutdown = isl9226_charger_shutdown,
	.id_table = isl9226_id,
};

static int __init isl9226_charger_init(void)
{
	int ret;
	ret = i2c_add_driver(&isl9226_charger_driver);
	if (ret)
		pr_err("Unable to register isl9226 charger driver");
	return ret;
}

module_init(isl9226_charger_init);
static void __exit isl9226_charger_exit(void)
{
	i2c_del_driver(&isl9226_charger_driver);
}

module_exit(isl9226_charger_exit);

MODULE_AUTHOR("Wenzeng Chen <wzch@marvell.com>");
MODULE_DESCRIPTION("isl9226 battery charger driver");
MODULE_LICENSE("GPL");
