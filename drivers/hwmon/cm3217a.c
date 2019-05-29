/*
*  cm3217a.c - CM3217a light and proximity sensor driver
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
#include "cm3217a.h"
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

struct work_struct cm3217a_als_work;

/*global struct store all input_dev and i2c_client*/
static struct i2c_cm3217a cm3217a_dev = {

	.g_client_msb = NULL,
	.g_client_lsb = NULL,
	.g_als_status = OFF,
	.als_user_count = 0,
	.als_sample_interval = SAMPLE_INTERVAL,
};

/* Add by Marco @110308 for that doing power on/off on active attr */
static int cm3217a_poweron(void);
static int cm3217a_poweroff(void);

/*
 * register light sensor i2c cLINEt, return 0 for success, 
 * non-zero for fail
 */
static int als_register_device(struct i2c_client *client)
{
	int err;

	cm3217a_dev.g_als = input_allocate_device();
	cm3217a_dev.g_als->name       = "CM3217a Ambient Light sensor";
	cm3217a_dev.g_als->phys       = "light-sensor/input0";
	cm3217a_dev.g_als->id.bustype = BUS_I2C;
	cm3217a_dev.g_als->id.vendor  = 0;
	cm3217a_dev.g_als->dev.parent = &client->dev;

	__set_bit(EV_ABS, cm3217a_dev.g_als->evbit);
	__set_bit(ABS_PRESSURE, cm3217a_dev.g_als->absbit);
	__set_bit(EV_SYN, cm3217a_dev.g_als->evbit);	
	input_set_abs_params(cm3217a_dev.g_als, ABS_PRESSURE, 0, 65535, 0, 0);

	err = input_register_device(cm3217a_dev.g_als);
	if (err) {
		dev_err(&client->dev, "regist input driver error\n");
		input_free_device(cm3217a_dev.g_als);
		cm3217a_dev.g_als = NULL;
	}

	return err;
}

/* Begin Add by Marco @110308 for that doing power on/off on active attr */
static int als_active_show(struct device *dev,
		struct device_attribute *attr, char *buf)		
{
	printk(KERN_ERR "%s: %d\n", __func__, cm3217a_dev.active);
	return sprintf(buf, "%d\n", cm3217a_dev.active);
}

static int als_active_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int val = 0;
	
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	printk(KERN_ERR "%s: active %d\n", __func__, val);

	cm3217a_dev.active= val;	

	if( 1 == val )
		cm3217a_poweron();
	else
		cm3217a_poweroff();

	return count;
}
/* End Add by Marco @110308 */

static int als_interval_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", cm3217a_dev.als_sample_interval);
}

static int als_interval_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int val = 0;
	char msg[256];

	if(count > 256)
		count = 256;

	memcpy(msg, buf, count-1);
	msg[count-1] = '\0';

	val = (unsigned int)simple_strtoul(msg, NULL, 10);
	if(val < 100)
		val = 100;

	cm3217a_dev.als_sample_interval = val;
	dev_info(dev, "cm3217a light  sensor sample interval is %d ms\n", cm3217a_dev.als_sample_interval);

	return count;
}

/*show the status of light sensor, return value is the result*/
static int als_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (cm3217a_dev.g_als_status == ON)
		return sprintf(buf, "on, totally %d users\n", cm3217a_dev.als_user_count);
	else
		return sprintf(buf, "off\n");
}

/*Poll the light sensor*/
static void cm3217a_als_timer_func(unsigned long data)
{
	schedule_work(&cm3217a_als_work);
	mod_timer(&cm3217a_dev.timer_als, jiffies + msecs_to_jiffies(cm3217a_dev.als_sample_interval));
}

/*show the status of proximity sensor*/
static int als_status_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	unsigned char tmp = INITIAL_MSB_DATA;

	if (strcmp(buf,"on\n") == 0) {

		mutex_lock(&cm3217a_dev.als_lock);

		cm3217a_dev.als_user_count++;

		if (cm3217a_dev.als_user_count == 1) {

			tmp = CM3217a_SET_BITSLICE(tmp, ALS_SHUTDOWN, 0);
			ret = i2c_master_send(cm3217a_dev.g_client_msb, &tmp, 1);

			cm3217a_dev.g_als_status = ON;
			setup_timer(&cm3217a_dev.timer_als, cm3217a_als_timer_func, 0);	
			mod_timer(&cm3217a_dev.timer_als, jiffies + msecs_to_jiffies(cm3217a_dev.als_sample_interval));
			dev_info(dev, "The status is set to on\n");
		}

		mutex_unlock(&cm3217a_dev.als_lock);

	} else if (strcmp(buf,"off\n") == 0) {

		mutex_lock(&cm3217a_dev.als_lock);

		if (cm3217a_dev.als_user_count > 0) {
			cm3217a_dev.als_user_count--;

			if (cm3217a_dev.als_user_count == 0) {

				tmp = CM3217a_SET_BITSLICE(tmp, ALS_SHUTDOWN, 1);
				ret = i2c_master_send(cm3217a_dev.g_client_msb, &tmp, 1);

				cm3217a_dev.g_als_status = OFF;
				del_timer_sync(&cm3217a_dev.timer_als);
				dev_info(dev, "The status is set to off\n");
			}
		}

		mutex_unlock(&cm3217a_dev.als_lock);
	}else
		dev_info(dev, "unrecognized, using on/off\n");

	return count;
}

static DEVICE_ATTR(als_status, 644, als_status_show,
		als_status_set);
/* Begin Add by Marco @110308 for that adding attrs for sensor HAL */
static DEVICE_ATTR(interval, 644, als_interval_show,
		als_interval_set);
static DEVICE_ATTR(active, 644, als_active_show,
		als_active_set);
static struct attribute *cm3217a_als_attributes[] = {
		&dev_attr_als_status.attr,
		&dev_attr_interval.attr,
		&dev_attr_active.attr,
		NULL
};
/* End Add by Marco @110308 */

static struct attribute_group cm3217a_als_attribute_group = {
	.attrs = cm3217a_als_attributes
};

static int cm3217a_add_als_fs(struct device *device)
{
	return sysfs_create_group(&device->kobj, &cm3217a_als_attribute_group);
}

static int cm3217a_remove_als_fs(void)
{
	sysfs_remove_group(&cm3217a_dev.g_client_msb->dev.kobj, 
						&cm3217a_als_attribute_group);
	return 0;
}

/*
Set the initial state of cm3217a
returns negative errno, or 0 for success
*/
static int cm3217a_poweron(void)
{
	int ret = 0;
	
	/*initialize lsb*/
	unsigned char tmp = INITIAL_LSB_DATA; 
	ret = i2c_master_send(cm3217a_dev.g_client_lsb, &tmp, 1);
	/*initialize msb*/
	tmp = INITIAL_MSB_DATA;
	ret |= i2c_master_send(cm3217a_dev.g_client_msb, &tmp, 1);
	setup_timer(&cm3217a_dev.timer_als, cm3217a_als_timer_func, 0);	
	mod_timer(&cm3217a_dev.timer_als, jiffies + msecs_to_jiffies(cm3217a_dev.als_sample_interval));

	return ret;
}
static int cm3217a_poweroff(void)
{
	int ret = 0;
	unsigned char tmp = INITIAL_MSB_DATA;
		
	tmp = CM3217a_SET_BITSLICE(tmp, ALS_SHUTDOWN, 1);
	ret |= i2c_master_send(cm3217a_dev.g_client_msb, &tmp, 1);

	cm3217a_dev.g_als_status = OFF;
	del_timer_sync(&cm3217a_dev.timer_als);
	
	return ret;
}

/*Get the value for ambient light sensor*/
static int cm3217a_get_als(unsigned short *data)
{
	unsigned char m_data, l_data;
	int ret;

	ret = i2c_master_recv(cm3217a_dev.g_client_msb, &m_data, 1);
	ret |= i2c_master_recv(cm3217a_dev.g_client_lsb, &l_data, 1);
	*data = l_data | m_data<<8;

	return ret;
}
static void cm3217a_report_als_value(void)	
{
	unsigned short tmp;
	int ret;

	ret = cm3217a_get_als(&tmp);
	if (ret < 0) {
		mod_timer(&cm3217a_dev.timer_als, jiffies + msecs_to_jiffies(cm3217a_dev.als_sample_interval));
		return;
	}

#if 0
	pr_info("[cm3217A] cm3217a_report_als_value\n");
	pr_info("[cm3217A] ALS read: 0x%X \n",tmp );
#endif
	input_event(cm3217a_dev.g_als, EV_ABS, ABS_PRESSURE, tmp);
	input_sync(cm3217a_dev.g_als);
}

/*are all i2c client ready ?*/
static int all_devices_ready(void)
{	
	if(cm3217a_dev.g_client_msb != NULL 
		&& cm3217a_dev.g_client_lsb != NULL) {
		return 0;
	}
	else 
		return -1;
}

static int all_devices_halt(void)
{
	if(cm3217a_dev.g_client_msb == NULL &&
	    cm3217a_dev.g_client_lsb == NULL)
		return 1;
	else
		return 0;
}

static int cm3217a_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	unsigned char tmp;
       
	if (!strcmp(client->name,"cm3217a_als_msb")) {
		tmp = INITIAL_MSB_DATA;
		tmp = CM3217a_SET_BITSLICE(tmp, ALS_SHUTDOWN, 1);
		ret = i2c_master_send(cm3217a_dev.g_client_msb, &tmp, 1);
		del_timer_sync(&cm3217a_dev.timer_als);
	}

	if (cm3217a_dev.pdata && (cm3217a_dev.pdata)->set_power)
		(cm3217a_dev.pdata)->set_power(1);

	return 0;
}

static int cm3217a_resume(struct i2c_client *client)
{
	int ret;
	unsigned char tmp;

	if (!strcmp(client->name, "cm3217a_als_msb")) {

		if (cm3217a_dev.pdata && (cm3217a_dev.pdata)->set_power)
			(cm3217a_dev.pdata)->set_power(1);

		tmp = INITIAL_MSB_DATA;
		tmp = CM3217a_SET_BITSLICE(tmp, ALS_SHUTDOWN, 0);

		ret = i2c_master_send(cm3217a_dev.g_client_msb, &tmp, 1);
		if (cm3217a_dev.g_als_status == ON) {
			setup_timer(&cm3217a_dev.timer_als, cm3217a_als_timer_func, 0);
			mod_timer(&cm3217a_dev.timer_als, jiffies + msecs_to_jiffies(cm3217a_dev.als_sample_interval));
		}
	}

	return 0;
}


static int cm3217a_status = 1;
static ssize_t cm3217a_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = strlen(cm3217a_status);
	
	sprintf(page, "%d\n", cm3217a_status);
	return len;
}

static ssize_t cm3217a_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	int flag, ret;
	char buffer[7];
 	int cm3217a_pwr_en;
	unsigned int cm3217a_cmd;
	unsigned char tmp = INITIAL_LSB_DATA;	
        if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
	{
		printk(KERN_ERR "%s error: %s \n", __func__,messages);
		return -EFAULT;
	}
	cm3217a_cmd = (unsigned int) simple_strtoul(messages, NULL, 10);
	switch(cm3217a_cmd){
	case 0:
		pr_info("%s  power off  \n", __func__);
	        ret = cm3217a_poweroff();
		cm3217a_dev.active= 0;
		break; 
	case 1:
		pr_info("%s  power on  \n", __func__);
                ret = cm3217a_poweron();
		cm3217a_dev.active= 1;
		break;
	
	 default:
		pr_info("%s  default  \n", __func__);
		break;
	}
	return len;
}

static void create_cm3217a_proc_file(void)
{
	struct proc_dir_entry *cm3217a_proc_file =
		create_proc_entry("driver/cm3217a", 0644, NULL);
        printk(KERN_ERR "%s: n", __func__);
	if (cm3217a_proc_file) {
		printk(KERN_INFO "proc file create success!\n");
		} else
		printk(KERN_ERR "proc file create failed!\n");
	cm3217a_proc_file->read_proc = (read_proc_t *)cm3217a_read_proc;
	cm3217a_proc_file->write_proc = (write_proc_t  *)cm3217a_write_proc;

}

static int cm3217a_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	pr_info("[cm3217A] cm3217a_probe\n");
	if (!strcmp(client->name, "cm3217a_als_msb"))
		cm3217a_dev.g_client_msb = client;
	else if (!strcmp(client->name, "cm3217a_als_lsb"))
		cm3217a_dev.g_client_lsb = client;

	if (all_devices_ready() == 0) {
		cm3217a_dev.pdata = (struct cm3217a_platform_data *)(client->dev.platform_data);
		if (cm3217a_dev.pdata && (cm3217a_dev.pdata)->set_power)
			(cm3217a_dev.pdata)->set_power(1);
		
		ret = cm3217a_poweron();
		ret |= cm3217a_poweroff();/*poweron and off to test the chip*/
		if (ret < 0) {
			dev_err(&client->dev, "cm3217a chip error\n");
			return -1;
		} else {
			cm3217a_dev.g_als_status = 0;
		}

		INIT_WORK(&cm3217a_als_work, cm3217a_report_als_value);
		mutex_init(&cm3217a_dev.als_lock);
		
		ret = als_register_device(cm3217a_dev.g_client_msb);
		if (ret)
			return ret;

		/* Add by Marco @110308 for that attrs should be added on input device
           not i2c device for sensor HAL */	
		//cm3217a_add_als_fs(&cm3217a_dev.g_client_msb->dev);
		cm3217a_add_als_fs(&cm3217a_dev.g_als->dev);
		create_cm3217a_proc_file();
		
	}
	//create_cm3217a_proc_file();
	return 0;
}

static int cm3217a_remove(struct i2c_client *client)
{
        pr_info("[cm3217A] cm3217a_remove\n");

	if (!strcmp(client->name,"cm3217a_als_msb")) {
		cm3217a_remove_als_fs();
		del_timer_sync(&cm3217a_dev.timer_als);
	}

	if (all_devices_halt()) {
		if (cm3217a_dev.pdata && (cm3217a_dev.pdata)->set_power)
			(cm3217a_dev.pdata)->set_power(0);
	}

	return 0;
}
 
static const struct i2c_device_id cm3217a_id[] = {
	{"cm3217a_als_lsb", 0},
	{"cm3217a_als_msb", 0},
	{}
};
static struct i2c_driver cm3217a_driver = {
	.driver = {
		.name  = "cm3217a",
		.owner = THIS_MODULE,
	},
	.suspend = cm3217a_suspend,
	.resume =  cm3217a_resume,
	.probe = cm3217a_probe,
	.remove = cm3217a_remove,
	.id_table = cm3217a_id
};

static int __init cm3217a_init(void)
{
	return i2c_add_driver(&cm3217a_driver);
}

static void  __exit cm3217a_exit(void)
{
	i2c_del_driver(&cm3217a_driver);
}

MODULE_DESCRIPTION("CAPELLA CM3217a I2C Ambient Light Sensor");
MODULE_AUTHOR("Yifan Zhang<zhangyf@marvell.com>");
MODULE_LICENSE("GPL");

module_init(cm3217a_init);
module_exit(cm3217a_exit);
