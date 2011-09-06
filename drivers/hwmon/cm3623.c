/*
*  cm3623.c - CM3623 light and proximity sensor driver
*
*  Copyright (C) Yifan Zhang  zhangyf@marvell.com
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation; either version 2 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the Free Software
*  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <mach/axis_sensor.h>
#include "cm3623.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend cm_early_suspend;
#endif
/* avoid android accessing cm3623 during suspend */

static atomic_t suspend_flag = ATOMIC_INIT(0);

static struct delayed_work cm3623_als_input_work;
static struct delayed_work cm3623_ps_input_work;
/*global struct store all input_dev and i2c_client*/
static struct i2c_cm3623 cm3623_dev = {
	.g_client_ps_cmd_or_data = NULL,
	.g_client_als_cmd_or_msb = NULL,
	.g_client_init_or_lsb = NULL,
	.g_client_int = NULL,
	.g_client_ps_threshold = NULL,
	.als_user_count = 0,
	.ps_user_count = 0,
	.ps_sample_interval = PS_SAMPLE_INTERVAL,
	.als_sample_interval = ALS_SAMPLE_INTERVAL
};

/*Get the value for proximity sensor*/
static int cm3623_get_ps(unsigned char *data)
{
	return i2c_master_recv(cm3623_dev.g_client_ps_cmd_or_data, data, 1);
}

/*Get the value for ambient light sensor*/
static int cm3623_get_als(unsigned short *data)
{
	unsigned char m_data, l_data;
	int ret;

	ret = i2c_master_recv(cm3623_dev.g_client_als_cmd_or_msb, &m_data, 1);
	if (ret < 0)
		return ret;
	ret = i2c_master_recv(cm3623_dev.g_client_init_or_lsb, &l_data, 1);
	*data = l_data | (m_data<<8);
	return ret;
}

/*
 * register proximity sensor i2c clinet, return 0 for success,
 * non-zero for fail
 */
static int ps_register_device(struct i2c_client *client)
{
	int err;

	cm3623_dev.g_ps = input_allocate_device();
	cm3623_dev.g_ps->name       = "CM3623 Proximity sensor";
	cm3623_dev.g_ps->phys       = "Proximity-sensor/input0";
	cm3623_dev.g_ps->id.bustype = BUS_I2C;
	cm3623_dev.g_ps->id.vendor  = 0;
	cm3623_dev.g_ps->dev.parent = &client->dev;

	__set_bit(EV_ABS, cm3623_dev.g_ps->evbit);
	__set_bit(ABS_DISTANCE, cm3623_dev.g_ps->absbit);
	__set_bit(ABS_MISC, cm3623_dev.g_ps->absbit);
	__set_bit(EV_SYN, cm3623_dev.g_ps->evbit);
	input_set_abs_params(cm3623_dev.g_ps, ABS_DISTANCE, 0, 255, 0, 0);
	input_set_abs_params(cm3623_dev.g_ps, ABS_MISC, -100, 100, 0, 0);

	err = input_register_device(cm3623_dev.g_ps);
	if (err) {
		dev_err(&client->dev,
			"ps_register_device input driver error\n");
		input_free_device(cm3623_dev.g_ps);
		cm3623_dev.g_ps = NULL;
	}
	return err;
}

/*
 * register light sensor i2c clinet, return 0 for success,
 * non-zero for fail
 */
static int als_register_device(struct i2c_client *client)
{
	int err;

	cm3623_dev.g_als = input_allocate_device();
	cm3623_dev.g_als->name       = "CM3623 Ambient Light sensor";
	cm3623_dev.g_als->phys       = "ligt-sensor/input0";
	cm3623_dev.g_als->id.bustype = BUS_I2C;
	cm3623_dev.g_als->id.vendor  = 0;
	cm3623_dev.g_als->dev.parent = &client->dev;

	__set_bit(EV_ABS, cm3623_dev.g_als->evbit);
	__set_bit(ABS_X, cm3623_dev.g_als->absbit);
	__set_bit(EV_SYN, cm3623_dev.g_als->evbit);
	__set_bit(ABS_MISC, cm3623_dev.g_als->absbit);
	input_set_abs_params(cm3623_dev.g_als, ABS_X, 0, 65535, 0, 0);
	input_set_abs_params(cm3623_dev.g_als, ABS_MISC, -100, 100, 0, 0);
	err = input_register_device(cm3623_dev.g_als);
	if (err) {
		dev_err(&client->dev, "regist input driver error\n");
		input_free_device(cm3623_dev.g_als);
		cm3623_dev.g_als = NULL;
	}

	return err;
}

static int active_set(struct device *dev, struct device_attribute *attr,
		      const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	int ret = 0;
	int enable = strcmp(buf, "1\n") ? 0 : 1;

	mutex_lock(&cm3623_dev.active_lock);

	if (enable && !cm3623_dev.als_user_count && !cm3623_dev.ps_user_count) {
		if (cm3623_dev.pdata && (cm3623_dev.pdata)->set_power)
			(cm3623_dev.pdata)->set_power(1);
	}

	if (!strcmp(client->name, "cm3623_als_msb")) {
		if (enable) {
			if ((cm3623_dev.als_user_count == 0) &&
			(!atomic_read(&suspend_flag))) {
				cm3623_dev.g_als_state &= ~ALS_SD_ALS_SHUTDOWN;
				ret = i2c_master_send(cm3623_dev.g_client_als_cmd_or_msb,
					&cm3623_dev.g_als_state, 1);
				pr_debug("%s/%d: cm3623_dev.g_als_state = %02X, ret = %d\n",
					__func__, __LINE__,
					cm3623_dev.g_als_state, ret);
				if (ret < 0) {
					printk(KERN_WARNING
						"%s: cannot activate cm3623 ALS (%d)\n",
						__func__, ret);
					goto out;
				}
				schedule_delayed_work(&cm3623_als_input_work,
					msecs_to_jiffies(cm3623_dev.als_sample_interval));
				dev_info(dev, "status on\n");
			}
			cm3623_dev.als_user_count++;
		} else if (cm3623_dev.als_user_count > 0) {
			if ((cm3623_dev.als_user_count == 1) &&
			(!atomic_read(&suspend_flag))) {
				cancel_delayed_work_sync(&cm3623_als_input_work);
				cm3623_dev.g_als_state |= ALS_SD_ALS_SHUTDOWN;
				ret = i2c_master_send(cm3623_dev.g_client_als_cmd_or_msb,
					&cm3623_dev.g_als_state, 1);
				pr_debug("%s/%d: cm3623_dev.g_als_state = %02X, ret = %d\n",
					__func__, __LINE__,
					cm3623_dev.g_als_state, ret);
				if (ret < 0) {
					printk(KERN_WARNING "%s: fail to stop cm3623 ALS (%d)\n",
						__func__, ret);
					goto out;
				}
				dev_info(dev, "status off\n");
			}
			cm3623_dev.als_user_count--;
		}
	} else if (!strcmp(client->name, "cm3623_ps")) {
		if (enable) {
			if ((cm3623_dev.ps_user_count == 0) &&
			(!atomic_read(&suspend_flag))) {
				cm3623_dev.g_ps_state &= ~PS_SD_PS_SHUTDOWN;
				ret = i2c_master_send(cm3623_dev.g_client_ps_cmd_or_data,
					&cm3623_dev.g_ps_state, 1);
				pr_debug("%s/%d: cm3623_dev.g_ps_state = %02X, ret = %d\n",
					__func__, __LINE__,
					cm3623_dev.g_ps_state, ret);
				if (ret < 0) {
					printk(KERN_WARNING "%s: cannot activate cm3623 PS (%d)\n",
						__func__, ret);
					goto out;
				}
				schedule_delayed_work(&cm3623_ps_input_work,
					msecs_to_jiffies(cm3623_dev.ps_sample_interval));
				dev_info(dev, "status on\n");
			}
			cm3623_dev.ps_user_count++;
		} else if (cm3623_dev.ps_user_count > 0) {
			if ((cm3623_dev.ps_user_count == 1) &&
			(!atomic_read(&suspend_flag))) {
				cancel_delayed_work_sync(&cm3623_ps_input_work);
				cm3623_dev.g_ps_state |= PS_SD_PS_SHUTDOWN;
				ret = i2c_master_send(cm3623_dev.g_client_ps_cmd_or_data,
					&cm3623_dev.g_ps_state, 1);
				pr_debug("%s/%d: cm3623_dev.g_ps_state = %02X, ret = %d\n",
					__func__, __LINE__,
					cm3623_dev.g_ps_state, ret);
				if (ret < 0) {
					printk(KERN_WARNING "%s: cannot stop cm3623 PS (%d)\n",
						__func__, ret);
					goto out;
				}
				dev_info(dev, "status off\n");
			}
			cm3623_dev.ps_user_count--;
		}
	}

	if (!enable && !cm3623_dev.als_user_count &&
	!cm3623_dev.ps_user_count) {
		if (cm3623_dev.pdata && (cm3623_dev.pdata)->set_power)
			(cm3623_dev.pdata)->set_power(0);
	}

out:
	mutex_unlock(&cm3623_dev.active_lock);
	return count;
}

static int active_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);

	pr_debug("%s: client->name = %s\n", __func__, client->name);

	if (!strcmp(client->name, "cm3623_als_msb")) {
		if (cm3623_dev.als_user_count > 0)
			return sprintf(buf, "1\n");
		else
			return sprintf(buf, "0\n");
	} else {
		if (cm3623_dev.ps_user_count > 0)
			return sprintf(buf, "1\n");
		else
			return sprintf(buf, "0\n");
	}
}
static int interval_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);

	pr_debug("%s: client->name = %s\n", __func__, client->name);

	if (!strcmp(client->name, "cm3623_ps"))
		return sprintf(buf, "%d\n", cm3623_dev.ps_sample_interval);
	else
		return sprintf(buf, "%d\n", cm3623_dev.als_sample_interval);
}

static int set_ps_interval(int val)
{
	if (val < 100)
		val = 100;
	else if (val > 800)
		val = 800;

	cm3623_dev.ps_sample_interval = val;
	return 0;
}

static int set_als_interval(int val)
{
	int ret;

	cm3623_dev.g_als_state &= ~ALS_IT_ALS;
	if (val < 150) {
		val = 100;
		cm3623_dev.g_als_state |= ALS_INTEGRATE_TIME_100;
	} else if (val < 300) {
		val = 200;
		cm3623_dev.g_als_state |= ALS_INTEGRATE_TIME_200;
	} else if (val < 600) {
		val = 400;
		cm3623_dev.g_als_state |= ALS_INTEGRATE_TIME_400;
	} else {
		val = 800;
		cm3623_dev.g_als_state |= ALS_INTEGRATE_TIME_800;
	}

	ret = i2c_master_send(cm3623_dev.g_client_als_cmd_or_msb,
		&cm3623_dev.g_als_state, 1);
	pr_debug("%s/%d: cm3623_dev.g_als_state = %02X, ret = %d\n",
		__func__, __LINE__, cm3623_dev.g_als_state, ret);

	cm3623_dev.als_sample_interval = val;
	return ret;
}

static int interval_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned int val = 0;
	char msg[13];

	pr_debug("%s: client->name = %s\n", __func__, client->name);
	if (count > sizeof(msg)-1)
		count = sizeof(msg)-1;
	memcpy(msg, buf, count);
	msg[count] = '\0';
	val = (unsigned int)simple_strtoul(msg, NULL, 10);

	if (!strcmp(client->name, "cm3623_ps")) {
		set_ps_interval(val);
		pr_debug("cm3623 proximity sensor sample interval is %d ms\n",
			cm3623_dev.ps_sample_interval);
	} else {
		set_als_interval(val);
		pr_debug("cm3623 light sensor sample interval is %d ms\n",
			cm3623_dev.als_sample_interval);
	}
	return count;
}

static ssize_t wake_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	static int i = 1;

	pr_debug("%s: client->name = %s\n", __func__, client->name);

	if (i == 10)
		i = 1;
	if (!strcmp(client->name, "cm3623_ps"))
		input_report_abs(cm3623_dev.g_ps, ABS_MISC, i++);
	else
		input_report_abs(cm3623_dev.g_als, ABS_MISC, i++);
	return count;
}

static int data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	int ret;

	if (!strcmp(client->name, "cm3623_ps")) {
		unsigned char tmp = 0;
		cm3623_get_ps(&tmp);
		ret = tmp;
	} else {
		unsigned short tmp = 0;
		cm3623_get_als(&tmp);
		ret = tmp;
	}
	return sprintf(buf, "%d\n", ret);
}

static int status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	int status;

	pr_debug("%s: client->name = %s\n", __func__, client->name);

	if (!strcmp(client->name, "cm3623_ps"))
		status = cm3623_dev.g_ps_state & PS_SD_PS_SHUTDOWN;
	else
		status = cm3623_dev.g_als_state & ALS_SD_ALS_SHUTDOWN;

	if (status)
		status = 0;
	else
		status = 1;

	return sprintf(buf, "%d\n", status);
}

static DEVICE_ATTR(active, S_IRUGO|S_IWUSR|S_IWGRP, active_show, active_set);
static DEVICE_ATTR(interval, S_IRUGO|S_IWUSR|S_IWGRP,
			interval_show, interval_set);
static DEVICE_ATTR(data, S_IRUGO, data_show, NULL);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP, NULL, wake_set);
static DEVICE_ATTR(status, S_IRUGO, status_show, NULL);

static struct attribute *cm3623_attributes[] = {
		&dev_attr_status.attr,
		&dev_attr_interval.attr,
		&dev_attr_data.attr,
		&dev_attr_active.attr,
		&dev_attr_wake.attr,
		NULL
};

static struct attribute_group cm3623_attribute_group = {
	.attrs = cm3623_attributes
};

static int cm3623_add_fs(struct device *device)
{
	return sysfs_create_group(&device->kobj, &cm3623_attribute_group);
}

static int cm3623_remove_als_fs(void)
{
	sysfs_remove_group(&cm3623_dev.g_client_als_cmd_or_msb->dev.kobj,
						&cm3623_attribute_group);
	return 0;
}

static int cm3623_remove_ps_fs(void)
{
	sysfs_remove_group(&cm3623_dev.g_client_ps_cmd_or_data->dev.kobj,
						&cm3623_attribute_group);
	return 0;
}
/*
Set the initial state of cm3623
returns negative errno, or 0 for success
*/
static int cm3623_poweron(void)
{
	unsigned char tmp;
	unsigned char haveInterrupt;
	int ret;
	int i;

	/* disable PS and ALS interrupts */
	cm3623_dev.g_ps_state = 0;
	ret = i2c_master_send(cm3623_dev.g_client_ps_cmd_or_data,
		&cm3623_dev.g_ps_state, 1);
	pr_debug("%s/%d: cm3623_dev.g_ps_state = %02X, ret = %d\n",
		__func__, __LINE__, cm3623_dev.g_ps_state, ret);
	if (ret < 0)
		return -ENOMEDIUM;

	/* clear any outstanding PS / ALS interrupts */
	for (i = 0; i < 4; i++) {
		ret = i2c_master_recv(cm3623_dev.g_client_int,
			&haveInterrupt, 1);
		pr_debug("%s: ret = %d, haveInterrupt = %d\n",
			__func__, ret, haveInterrupt);

		if (ret < 0 || haveInterrupt == 0) {
			/* error indicates no outstanding interrupts */
			ret = 0;
			break;
		}
	}

	if (ret != 0)
		return -ENOMEDIUM;

	tmp = CM3623_IR_PS_HIGH_LOW_MODE;
	ret = i2c_master_send(cm3623_dev.g_client_init_or_lsb, &tmp, 1);
	pr_debug("%s/%d: cm3623_dev.g_client_init_or_lsb = %02X, ret = %d\n",
		__func__, __LINE__, tmp, ret);
	if (ret < 0)
		return ret;

	cm3623_dev.g_als_state = ALS_WDM_WORD_MODE;
	ret = i2c_master_send(cm3623_dev.g_client_als_cmd_or_msb,
		&cm3623_dev.g_als_state, 1);
	pr_debug("%s/%d: cm3623_dev.g_als_state = %02X, ret = %d\n",
		__func__, __LINE__, cm3623_dev.g_als_state, ret);
	return ret;
}

static int cm3623_lowpower(void)
{
	int ret;

	cm3623_dev.g_ps_state |= PS_SD_PS_SHUTDOWN;
	ret = i2c_master_send(cm3623_dev.g_client_ps_cmd_or_data,
		&cm3623_dev.g_ps_state, 1);
	pr_debug("%s/%d: cm3623_dev.g_ps_state = %02X, ret = %d\n",
		__func__, __LINE__, cm3623_dev.g_ps_state, ret);
	if (ret < 0)
		return ret;

	cm3623_dev.g_als_state |= ALS_SD_ALS_SHUTDOWN;
	ret = i2c_master_send(cm3623_dev.g_client_als_cmd_or_msb,
		&cm3623_dev.g_als_state, 1);
	pr_debug("%s/%d: cm3623_dev.g_als_state = %02X, ret = %d\n",
		__func__, __LINE__, cm3623_dev.g_als_state, ret);
	return ret;
}

static void cm3623_report_ps_value(struct work_struct *work)
{
	unsigned char tmp;

	cm3623_get_ps(&tmp);
	input_event(cm3623_dev.g_ps, EV_ABS, ABS_DISTANCE, tmp);
	input_sync(cm3623_dev.g_ps);
	schedule_delayed_work(&cm3623_ps_input_work,
		msecs_to_jiffies(cm3623_dev.ps_sample_interval));
}

static void cm3623_report_als_value(struct work_struct *work)
{
	unsigned short tmp;

	cm3623_get_als(&tmp);
	input_event(cm3623_dev.g_als, EV_ABS, ABS_X, tmp);
	input_sync(cm3623_dev.g_als);
	schedule_delayed_work(&cm3623_als_input_work,
		msecs_to_jiffies(cm3623_dev.als_sample_interval));
}


static int all_devices_ready(void)
{
	return (cm3623_dev.g_client_ps_cmd_or_data != NULL &&
	    cm3623_dev.g_client_als_cmd_or_msb != NULL &&
	    cm3623_dev.g_client_init_or_lsb != NULL &&
	    cm3623_dev.g_client_int != NULL &&
	    cm3623_dev.g_client_ps_threshold != NULL);
}

static int all_devices_halt(void)
{
	return (cm3623_dev.g_client_ps_cmd_or_data == NULL &&
	    cm3623_dev.g_client_als_cmd_or_msb == NULL &&
	    cm3623_dev.g_client_init_or_lsb == NULL &&
	    cm3623_dev.g_client_int == NULL &&
	    cm3623_dev.g_client_ps_threshold == NULL);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cm3623_early_suspend(struct early_suspend *h)
{
	int ret;

	mutex_lock(&cm3623_dev.active_lock);
	atomic_set(&suspend_flag, 1);

	if (cm3623_dev.als_user_count > 0) {
		cancel_delayed_work_sync(&cm3623_als_input_work);
		cm3623_dev.g_als_state |= ALS_SD_ALS_SHUTDOWN;
		ret = i2c_master_send(cm3623_dev.g_client_als_cmd_or_msb,
			&cm3623_dev.g_als_state, 1);
		pr_debug("%s/%d: cm3623_dev.g_als_state = %02X, ret = %d\n",
			__func__, __LINE__, cm3623_dev.g_als_state, ret);
		if (ret < 0) {
			printk(KERN_WARNING "%s: cannot disable ALS cm3623\n",
				__func__);
		}
	}

	if (cm3623_dev.ps_user_count > 0) {
		cancel_delayed_work_sync(&cm3623_ps_input_work);
		cm3623_dev.g_ps_state |= PS_SD_PS_SHUTDOWN;
		ret = i2c_master_send(cm3623_dev.g_client_ps_cmd_or_data,
			&cm3623_dev.g_ps_state, 1);
		pr_debug("%s/%d: cm3623_dev.g_ps_state = %02X, ret = %d\n",
			__func__, __LINE__, cm3623_dev.g_ps_state, ret);
		if (ret < 0) {
			printk(KERN_WARNING "%s: cannot disable PS cm3623\n",
				__func__);
		}
	}

	if (cm3623_dev.pdata && (cm3623_dev.pdata)->set_power)
		(cm3623_dev.pdata)->set_power(0);

	mutex_unlock(&cm3623_dev.active_lock);
}

static void cm3623_late_resume(struct early_suspend *h)
{
	int ret;

	mutex_lock(&cm3623_dev.active_lock);

	if (cm3623_dev.pdata && (cm3623_dev.pdata)->set_power)
		(cm3623_dev.pdata)->set_power(1);

	if (cm3623_dev.ps_user_count > 0) {
		cm3623_dev.g_ps_state &= ~PS_SD_PS_SHUTDOWN;
		ret = i2c_master_send(cm3623_dev.g_client_ps_cmd_or_data,
			&cm3623_dev.g_ps_state, 1);
		pr_debug("%s/%d: cm3623_dev.g_ps_state = %02X, ret = %d\n",
			__func__, __LINE__,
			cm3623_dev.g_ps_state, ret);
		if (ret < 0) {

			printk(KERN_WARNING "%s: cannot enable cm3623\n",
				__func__);
		}
		schedule_delayed_work(&cm3623_ps_input_work,
			msecs_to_jiffies(cm3623_dev.ps_sample_interval));
	}

	if (cm3623_dev.als_user_count > 0) {
		cm3623_dev.g_als_state &= ~ALS_SD_ALS_SHUTDOWN;
		ret = i2c_master_send(cm3623_dev.g_client_als_cmd_or_msb,
			&cm3623_dev.g_als_state, 1);
		pr_debug("%s/%d: cm3623_dev.g_als_state = %02X, ret = %d\n",
			__func__, __LINE__, cm3623_dev.g_als_state, ret);
		if (ret < 0) {
			printk(KERN_WARNING
				"%s: cannot enable cm3623 ret = %d\n",
				__func__, ret);
		}
		schedule_delayed_work(&cm3623_als_input_work,
			msecs_to_jiffies(cm3623_dev.als_sample_interval));
	}

	atomic_set(&suspend_flag, 0);
	mutex_unlock(&cm3623_dev.active_lock);
}
#endif

static int cm3623_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret;

	cm3623_dev.pdata = (struct axis_sensor_platform_data *)
		(client->dev.platform_data);

	pr_debug("%s:client->name = %s\n", __func__, client->name);

	if (!strcmp(client->name, "cm3623_ps"))
		cm3623_dev.g_client_ps_cmd_or_data = client;
	else if (!strcmp(client->name, "cm3623_als_msb"))
		cm3623_dev.g_client_als_cmd_or_msb = client;
	else if (!strcmp(client->name, "cm3623_als_lsb"))
		cm3623_dev.g_client_init_or_lsb = client;
	else if (!strcmp(client->name, "cm3623_int"))
		cm3623_dev.g_client_int = client;
	else if (!strcmp(client->name, "cm3623_ps_threshold"))
		cm3623_dev.g_client_ps_threshold = client;
	else {
		printk(KERN_ERR "%s: unknown i2c_client = %p\n",
			__func__, client);
		return -ENOMEDIUM;
	}

	if (!all_devices_ready())
		return 0;

	pr_debug("%s: all_devices_ready\n", __func__);
	pr_debug("cm3623_dev.g_client_ps_cmd_or_data->addr = %02X\n",
		cm3623_dev.g_client_ps_cmd_or_data->addr);
	pr_debug("cm3623_dev.g_client_als_cmd_or_msb->addr = %02X\n",
		cm3623_dev.g_client_als_cmd_or_msb->addr);
	pr_debug("cm3623_dev.g_client_init_or_lsb->addr = %02X\n",
		cm3623_dev.g_client_init_or_lsb->addr);
	pr_debug("cm3623_dev.g_client_int->addr = %02X\n",
		cm3623_dev.g_client_int->addr);
	pr_debug("cm3623_dev.g_client_ps_threshold->addr = %02X\n",
			cm3623_dev.g_client_ps_threshold->addr);

	if (cm3623_dev.pdata && (cm3623_dev.pdata)->set_power)
			(cm3623_dev.pdata)->set_power(1);

	cm3623_dev.g_ps_state = 0;
	cm3623_dev.g_als_state = 0;

	ret = cm3623_poweron();
	if (ret < 0) {
		dev_err(&client->dev, "cm3623 poweron failure");
		return ret;
	}

	ret = cm3623_lowpower();
	if (ret < 0) {
		dev_err(&client->dev, "cm3623 low power failure\n");
		return ret;
	}

	set_als_interval(cm3623_dev.als_sample_interval);
	set_ps_interval(cm3623_dev.ps_sample_interval);

	INIT_DELAYED_WORK(&cm3623_als_input_work, cm3623_report_als_value);
	INIT_DELAYED_WORK(&cm3623_ps_input_work, cm3623_report_ps_value);
	mutex_init(&cm3623_dev.active_lock);

	ret = ps_register_device(cm3623_dev.g_client_ps_cmd_or_data);
	if (ret)
		return ret;

	ret = als_register_device(cm3623_dev.g_client_als_cmd_or_msb);
	if (ret)
		return ret;

	cm3623_add_fs(&cm3623_dev.g_ps->dev);
	cm3623_add_fs(&cm3623_dev.g_als->dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
	cm_early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	cm_early_suspend.suspend = cm3623_early_suspend;
	cm_early_suspend.resume = cm3623_late_resume;
	register_early_suspend(&cm_early_suspend);
#endif
	return 0;
}

static int cm3623_remove(struct i2c_client *client)
{
	if (!strcmp(client->name, "cm3623_ps")) {
		cm3623_remove_ps_fs();
		cancel_delayed_work_sync(&cm3623_ps_input_work);
	} else if (!strcmp(client->name, "cm3623_als_msb")) {
		cm3623_remove_als_fs();
		cancel_delayed_work_sync(&cm3623_als_input_work);
	}

	if (!strcmp(client->name, "cm3623_ps"))
		cm3623_dev.g_client_ps_cmd_or_data  = NULL;
	else if (!strcmp(client->name, "cm3623_als_msb"))
		cm3623_dev.g_client_als_cmd_or_msb = NULL;
	else if (!strcmp(client->name, "cm3623_als_lsb"))
		cm3623_dev.g_client_init_or_lsb = NULL;
	else if (!strcmp(client->name, "cm3623_int"))
		cm3623_dev.g_client_int = NULL;
	else if (!strcmp(client->name, "cm3623_ps_threshold"))
		cm3623_dev.g_client_ps_threshold = NULL;

	if (all_devices_halt()) {
		if (cm3623_dev.pdata && (cm3623_dev.pdata)->set_power)
			(cm3623_dev.pdata)->set_power(0);
	}

	return 0;
}

static const struct i2c_device_id cm3623_id[] = {
	{"cm3623_ps", 0},
	{"cm3623_als_lsb", 0},
	{"cm3623_als_msb", 0},
	{"cm3623_int", 0},
	{"cm3623_ps_threshold", 0},
	{}
};
static struct i2c_driver cm3623_driver = {
	.driver = {
		.name  = "cm3623",
		.owner = THIS_MODULE,
	},
	.probe = cm3623_probe,
	.remove = cm3623_remove,
	.id_table = cm3623_id
};

static int __init cm3623_init(void)
{
	return i2c_add_driver(&cm3623_driver);
}

static void  __exit cm3623_exit(void)
{
	i2c_del_driver(&cm3623_driver);
}

MODULE_DESCRIPTION("CAPELLA CM3623 I2C Proximity sensor"
					"with Ambient Light Sensor");
MODULE_AUTHOR("Yifan Zhang<zhangyf@marvell.com>");
MODULE_AUTHOR("Leo Yan<Leoy@marvell.com>");
MODULE_AUTHOR("Philip Rakity<prakity@marvell.com>");
MODULE_LICENSE("GPL");

module_init(cm3623_init);
module_exit(cm3623_exit);
