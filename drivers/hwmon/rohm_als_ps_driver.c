/* drivers/input/touchscreen/rohm_bh1771.c
 *
 * ROHM BH1771 Light Sensor driver
 *
 * Copyright (C) 2010 ROHM SEMICONDUCTOR Co. Ltd.
 *
 * Author:  Tracy Wen <tracy-wen@rohm.com.cn>
 */
/******************************************************************************
 * MODULE     : rohm_als_driver.c
 * FUNCTION   : Light Sensor driver of BH1780
 * PROGRAMMED : sensor application development group
 * DATE(ORG)  : Jun-09-2011(Jun-09-2011)
 * REMARKS    :
 * C-FORM     : 1.00A
 * COPYRIGHT  : Copyright (C) 2011 ROHM CO.,LTD.
 *            : This software is licensed under the terms of the GNU General Public
 *            : License version 2, as published by the Free Software Foundation, and
 *            : may be copied, distributed, and modified under those terms.
 *            :
 *            : This program is distributed in the hope that it will be useful,
 *            : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *            : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *            : GNU General Public License for more details.
 * HISTORY    :
 * 1.00A Jun-09-2011  SEI   Made a new file
 *****************************************************************************/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/rohm_als_ps_driver.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>

#include <linux/rohm_bh1772_driver.h>

#define DATA_MAX	200
#define DATA_MIN	4
#define LIGHT_SENSOR "rohm_1772_als"
#define PROXIMITY_SENSOR "rohm_1772_ps"

#define LIGHT_SENSOR_REPORT 0
#define PROXIMITY_SENSOR_REPORT 1
#define DEVICE_ATTR2(_name, _mode, _show, _store) \
struct device_attribute dev_attr2_##_name = __ATTR(_name, _mode, _show, _store)

unsigned short delay_map[] = { 0, 100, 200, 500, 1000, 2000 };

typedef struct delay_map {
	unsigned char code;
	unsigned short delay;
} delay_convert_map;

static delay_convert_map als_meas_time_map[] = {
	{.code = 0x80, .delay = 0},
	{.code = 0x00, .delay = 100},
	{.code = 0x01, .delay = 200},
	{.code = 0x02, .delay = 500},
	{.code = 0x03, .delay = 1000},
	{.code = 0x04, .delay = 2000}
};

#ifdef PS_DELAY_SET
static delay_convert_map ps_meas_time_map[] = {
	{.code = 0x00, .delay = 10},
	{.code = 0x01, .delay = 20},
	{.code = 0x02, .delay = 30},
	{.code = 0x03, .delay = 50},
	{.code = 0x04, .delay = 70},
	{.code = 0x05, .delay = 100},
	{.code = 0x06, .delay = 200},
	{.code = 0x07, .delay = 500},
	{.code = 0x08, .delay = 1000},
	{.code = 0x09, .delay = 2000}
};
#endif

static void rohm_work_report(struct rohm_ls_data *ls, int data, int sensor)
{
	if (sensor == LIGHT_SENSOR_REPORT) {

		input_report_abs(ls->input_dev_als, ABS_PRESSURE, data);
		input_sync(ls->input_dev_als);

	} else if (sensor == PROXIMITY_SENSOR_REPORT) {

		input_report_abs(ls->input_dev_ps, ABS_DISTANCE, data);
		input_sync(ls->input_dev_ps);
	}
	return;
}

static void als_polling_work(struct work_struct *work)
{
	int result;
	struct rohm_ls_data *ls;
	ls = container_of((struct delayed_work *)work, struct rohm_ls_data,
			  als_work);

	result = bh1772_driver_read_illuminance(&ls->als_data, ls->client);

	rohm_work_report(ls, ls->als_data, LIGHT_SENSOR_REPORT);

	pr_debug("in %s, als_data is %d", __func__, ls->als_data);

	schedule_delayed_work(&ls->als_work, delay_to_jiffies(ls->als_poll_delay));	/* restart timer */
}

static int als_set_delay(struct rohm_ls_data *ls, unsigned long delay)
{

	int ret = 0;
	int map_count;
	int map_size = sizeof(als_meas_time_map) / sizeof(als_meas_time_map[0]);
	for (map_count = 0; map_count < map_size; map_count++) {
		if (map_count == map_size - 1)
			break;
		if ((delay < als_meas_time_map[map_count + 1].delay) &&
		    (delay >= als_meas_time_map[map_count].delay)) {
			break;
		}
	}

	if (ls->als_enable == CTL_STAND) {

		mutex_lock(&ls->sensor_lock);

		cancel_delayed_work_sync(&ls->als_work);

		ls->als_meas_time = als_meas_time_map[map_count].code;
		ls->als_poll_delay = delay;

		ret =
		    i2c_smbus_write_byte_data(ls->client, REG_ALSMEASRATE,
					      ls->als_meas_time);
		if (ret < 0)
			return ret;
		schedule_delayed_work(&ls->als_work, delay_to_jiffies(delay));

		mutex_unlock(&ls->sensor_lock);

		pr_debug("cancel and enable queue work");
		pr_info("als_meas_time is 0x%x", ls->als_meas_time);

	} else {
		ls->als_meas_time = als_meas_time_map[map_count].code;
		ls->als_poll_delay = delay;

		pr_debug("NO need to cancel and enable queue work");
		pr_info("als_meas_time is 0x%x", ls->als_meas_time);

	}

	return ret;
}

static int active_als_set(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct rohm_ls_data *ls = input_get_drvdata(input);
	int ret;

	unsigned char enabled =
	    (strcmp(buf, "1\n") == 0) ? POWER_ON : POWER_OFF;
	if (enabled)
		enabled = ls->als_enable | CTL_STAND;
	else
		enabled = ls->als_enable & (~CTL_STAND);
	ret = bh1772_driver_write_als_sens(0xFD, ls->client);
	if (ret < 0)
		return ret;
	ret = bh1772_driver_als_power_on(enabled, ls->client);
	if (ret < 0)
		return ret;
	ls->als_enable = enabled;
	return count;
}

static int active_als_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct rohm_ls_data *ls = input_get_drvdata(input);

	return sprintf(buf, "%d\n", ((ls->als_enable & 0x02) >> 1));
}

static int als_interval_set(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct rohm_ls_data *ls = input_get_drvdata(input);
	unsigned long delay;
	int ret = 0;
	ret = strict_strtoul(buf, 10, &delay);
	if (ret < 0)
		return ret;

	pr_info("%s's interval is %ld ms", input->name, delay);

	ret = als_set_delay(ls, delay);
	if (ret < 0)
		return ret;
	return count;
}

static int als_interval_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct rohm_ls_data *ls = input_get_drvdata(input);

	return sprintf(buf, "%d\n", ls->als_poll_delay);
}

static int als_data_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct rohm_ls_data *ls = input_get_drvdata(input);

	return sprintf(buf, "%d\n", ls->als_data);
}

static int active_ps_set(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct rohm_ls_data *ls = input_get_drvdata(input);
	unsigned int interrupt;
	int ret;
	int enabled = (strcmp(buf, "1\n") == 0) ? POWER_ON : POWER_OFF;

	interrupt =
	    (OUTPUT_LUTCH | MODE_PROXIMITY) & (~POLA_INACTIVEL);
	if (enabled) {
		unsigned char status;

		enabled = CTL_STAND;

		ret = bh1772_driver_ps_power_on(enabled, interrupt, ls->client);
		if (ret < 0)
			return ret;

		ret = bh1772_driver_general_read(REG_ALSPSSTATUS,
						 &status, sizeof(status),
						 ls->client);
		if (ret < 0)
			return ret;
		status = ((status >> 1) & 0x01);
		if (status == 1) {
			ls->ps_data = 2;
			rohm_work_report(ls, ls->ps_data,
					 PROXIMITY_SENSOR_REPORT);
		} else if (status == 0) {
			ls->ps_data = 10;
			rohm_work_report(ls, ls->ps_data,
					 PROXIMITY_SENSOR_REPORT);
		}
	} else {
		enabled = CTL_SATBY;
		interrupt &= PS_INT_DISABLE;
		ret = bh1772_driver_ps_power_on(enabled, interrupt, ls->client);
		if (ret < 0)
			return ret;
	}
	ls->interrupt = interrupt;
	ls->ps_enable = enabled;

	return count;
}

static int active_ps_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct rohm_ls_data *ls = input_get_drvdata(input);

	return sprintf(buf, "%d\n", ((ls->ps_enable & 0x02) >> 1));
}

static int ps_data_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct rohm_ls_data *ls = input_get_drvdata(input);

	return sprintf(buf, "%d\n", ls->ps_data);
}

#ifdef PS_DELAY_SET
static int ps_set_delay(struct rohm_ls_data *ls, unsigned long delay)
{

	int ret = 0;
	int map_count;
	int map_size = sizeof(ps_meas_time_map) / sizeof(ps_meas_time_map[0]);
	for (map_count = 0; map_count < map_size; map_count++) {
		if (map_count == map_size - 1)
			break;
		if ((map_count == 0)
		    && delay < ps_meas_time_map[map_count].delay) {
			delay = ps_meas_time_map[map_count].delay;
			pr_info("delay is too small, change to %ld", delay);
			break;
		}
		if ((delay < ps_meas_time_map[map_count + 1].delay) &&
		    (delay >= ps_meas_time_map[map_count].delay)) {
			break;
		}
	}

	ls->ps_meas_time = ps_meas_time_map[map_count].code;
	ls->ps_delay = delay;

	ret = bh1772_driver_write_ps_meas_rate(ls->ps_meas_time, ls->client);
	if (ret < 0)
		return ret;

	pr_info("in %s, ps_meas_time is 0x%x", __func__, ls->ps_meas_time);
	pr_info("ps_delay is %d", ls->ps_delay);

	return ret;
}
#endif

static int ps_interval_set(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
#ifdef PS_DELAY_SET
	struct input_dev *input = to_input_dev(dev);
	struct rohm_ls_data *ls = input_get_drvdata(input);
	unsigned long delay;
	int ret = 0;
	ret = strict_strtoul(buf, 10, &delay);
	if (ret < 0)
		return ret;

	ret = ps_set_delay(ls, delay);
	if (ret < 0)
		return ret;
#endif
	return count;
}

static int ps_interval_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct rohm_ls_data *ls = input_get_drvdata(input);

	return sprintf(buf, "%d\n", ls->ps_delay);
}

static ssize_t wake_set(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	return 0;
}

static int status_show(struct device *dev,
		       struct device_attribute *attr, char *buf)
{
	return 0;
}

static DEVICE_ATTR(active, S_IRUGO | S_IWUGO, active_als_show, active_als_set);
static DEVICE_ATTR(interval, S_IRUGO | S_IWUGO, als_interval_show,
		   als_interval_set);
static DEVICE_ATTR(data, S_IRUGO, als_data_show, NULL);
static DEVICE_ATTR(wake, S_IRUGO | S_IWUGO, NULL, wake_set);
static DEVICE_ATTR(status, S_IRUGO | S_IWUGO, status_show, NULL);

static DEVICE_ATTR2(active, S_IRUGO | S_IWUGO, active_ps_show, active_ps_set);
static DEVICE_ATTR2(interval, S_IRUGO | S_IWUGO, ps_interval_show,
		    ps_interval_set);
static DEVICE_ATTR2(data, S_IRUGO, ps_data_show, NULL);
static DEVICE_ATTR2(wake, S_IRUGO | S_IWUGO, NULL, wake_set);
static DEVICE_ATTR2(status, S_IRUGO | S_IWUGO, status_show, NULL);

static struct attribute *sysfs_als_attributes[] = {
	&dev_attr_status.attr,
	&dev_attr_interval.attr,
	&dev_attr_data.attr,
	&dev_attr_active.attr,
	&dev_attr_wake.attr,
	NULL
};

static struct attribute *sysfs_ps_attributes[] = {
	&dev_attr2_active.attr,
	&dev_attr2_interval.attr,
	&dev_attr2_wake.attr,
	&dev_attr2_data.attr,
	&dev_attr2_status.attr,
	NULL
};

static struct attribute_group sysfs_als_attribute_group = {
	.attrs = sysfs_als_attributes
};

static struct attribute_group sysfs_ps_attribute_group = {
	.attrs = sysfs_ps_attributes
};

static void check_far_work(struct work_struct *work)
{
	struct rohm_ls_data *ls =
	    container_of((struct delayed_work *)work, struct rohm_ls_data,
			 ps_delay_work);
	unsigned char status;

	bh1772_driver_general_read(REG_ALSPSSTATUS,
				   &status, sizeof(status), ls->client);

	status = ((status >> 1) & 0x01);

	if (status == 0) {
		/*bh1772_driver_read_proximity(&ls->ps_data, ls->client); */
		ls->ps_data = 10;
		rohm_work_report(ls, ls->ps_data, PROXIMITY_SENSOR_REPORT);

		pr_debug("near to far detect, in  %s", __func__);
		pr_debug("proximity = %d\n", ls->ps_data);

		__cancel_delayed_work(&ls->ps_delay_work);
	} else if ((!ls->device_suspend) && (ls->ps_enable) && (status)) {

		schedule_delayed_work(&ls->ps_delay_work,
				      delay_to_jiffies(ls->ps_delay));

		pr_debug("interrupt is still active, and status is  0x%x",
			 status);

	}
}

static void ps_work_handler(struct work_struct *work)
{
	struct rohm_ls_data *ls =
	    container_of(work, struct rohm_ls_data, ps_work);
	unsigned char status;

	bh1772_driver_general_read(REG_INTERRUPT,
				   &status, sizeof(status), ls->client);

	status = (status >> 5);

	if (status == 1) {
		/*bh1772_driver_read_proximity(&ls->ps_data, ls->client); */
		ls->ps_data = 2;
		rohm_work_report(ls, ls->ps_data, PROXIMITY_SENSOR_REPORT);

		pr_debug("proximity = %d\n", ls->ps_data);
		pr_debug("far to near detect, in  %s", __func__);

		schedule_delayed_work(&ls->ps_delay_work,
				      delay_to_jiffies(ls->ps_delay));
	}

}

static irqreturn_t rohm_ls_irq_handler(int irq, void *dev_id)
{
	struct rohm_ls_data *ls = dev_id;

	schedule_work(&ls->ps_work);

	return IRQ_HANDLED;
}

static int rohm_sensor_input_init(struct rohm_ls_data *ls)
{
	int err = 0;

	pr_debug("bh1772_input_init!\n");

	ls->input_dev_als = input_allocate_device();
	if (!ls->input_dev_als) {
		err = -ENOMEM;
		dev_err(&ls->client->dev,
			"input device allocate for als failed\n");
		goto exit;
	}
	ls->input_dev_ps = input_allocate_device();
	if (!ls->input_dev_ps) {
		err = -ENOMEM;
		dev_err(&ls->client->dev,
			"input device allocate for ps failed\n");
		goto exit_free_dev_als;
	}
	input_set_drvdata(ls->input_dev_ps, ls);
	input_set_drvdata(ls->input_dev_als, ls);

	ls->input_dev_als->id.bustype = BUS_I2C;
	input_set_capability(ls->input_dev_als, EV_ABS, ABS_MISC);
	__set_bit(EV_ABS, ls->input_dev_als->evbit);
	__set_bit(ABS_PRESSURE, ls->input_dev_als->absbit);
	input_set_abs_params(ls->input_dev_als, ABS_PRESSURE, 0, 65535, 0, 0);

	ls->input_dev_ps->id.bustype = BUS_I2C;
	input_set_capability(ls->input_dev_ps, EV_ABS, ABS_MISC);
	__set_bit(EV_ABS, ls->input_dev_ps->evbit);
	__set_bit(ABS_DISTANCE, ls->input_dev_ps->absbit);
	input_set_abs_params(ls->input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);

	ls->input_dev_als->name = LIGHT_SENSOR;
	ls->input_dev_ps->name = PROXIMITY_SENSOR;

	err = input_register_device(ls->input_dev_als);
	if (err) {
		dev_err(&ls->client->dev,
			"unable to register input polled device %s: %d\n",
			ls->input_dev_als->name, err);
		goto exit_free_dev_ps;
	}

	err = input_register_device(ls->input_dev_ps);
	if (err) {
		dev_err(&ls->client->dev,
			"unable to register ps input polled device %s: %d\n",
			ls->input_dev_ps->name, err);
		goto exit_unregister_dev_als;
	}
	return 0;

exit_unregister_dev_als:
	input_unregister_device(ls->input_dev_als);
exit_free_dev_ps:
	input_free_device(ls->input_dev_ps);
exit_free_dev_als:
	input_free_device(ls->input_dev_als);
exit:
	return err;
}

static void rohm_input_fini(struct rohm_ls_data *ls)
{
	input_unregister_device(ls->input_dev_als);
	input_free_device(ls->input_dev_als);

	input_unregister_device(ls->input_dev_ps);
	input_free_device(ls->input_dev_ps);
}

static int rohm_ls_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct rohm_ls_data *ls;
	int ret = 0;
	struct ROHM_I2C_platform_data *pdata;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "Rohm_ls_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ls = kzalloc(sizeof(*ls), GFP_KERNEL);
	if (ls == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ls->ps_work, ps_work_handler);
	INIT_DELAYED_WORK(&ls->ps_delay_work, check_far_work);
	INIT_DELAYED_WORK(&ls->als_work, als_polling_work);
	ls->client = client;
	i2c_set_clientdata(client, ls);
	pdata = client->dev.platform_data;
	if (pdata && pdata->power) {
		ls->power = pdata->power;
		ret = ls->power(1);
		if (ret < 0) {
			printk(KERN_ERR "Rohm_ls_probe power on failed\n");
			goto err_power_failed;
		}
	}
	ret = rohm_sensor_input_init(ls);
	if (ret)
		goto err_input_init;
	else {
		ret =
		    bh1772_driver_init(0x04, 0x00, 0x04, 0x80, 0x09, 0x5E, 0xFF,
				       0x00, 0xFE, 0x04, 0x00, ls->client);
		if (ret < 0) {
			printk(KERN_ERR "In %s, bh1772_driver_init call failed",
			       __func__);
			return ret;
		}
		ls->als_enable = 0;
		ls->ps_enable = 0;
		ls->als_poll_delay = 20;
		ls->ps_delay = 50;
		mutex_init(&ls->sensor_lock);
	}
	if (client->irq) {
		ret = request_irq(client->irq,
				  rohm_ls_irq_handler,
				  IRQF_TRIGGER_FALLING, client->name, ls);
		if (ret == 0)
			ls->use_irq = 1;	/*1 : interrupt mode/0 : polling mod */
		else {
			ls->use_irq = 0;	/*1 set 1 : interrupt mode/0 : polling mode */
			printk(KERN_ERR "Request IRQ Failed==>ret : %d\n", ret);
		}
	}
	ret = sysfs_create_group(&ls->input_dev_als->dev.kobj,
				 &sysfs_als_attribute_group);
	if (ret)
		goto exit_free_input;

	ret = sysfs_create_group(&ls->input_dev_ps->dev.kobj,
				 &sysfs_ps_attribute_group);
	if (ret)
		goto exit_free_input;

	pr_debug(" %s complete", __func__);

	return 0;

exit_free_input:
	rohm_input_fini(ls);
err_power_failed:
err_input_init:
	kfree(ls);
err_alloc_data_failed:
err_check_functionality_failed:
	mutex_destroy(&ls->sensor_lock);
	return ret;
}

static int rohm_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct rohm_ls_data *ls = i2c_get_clientdata(client);
	int ret = 0;
	unsigned char enabled = CTL_SATBY;

	pr_debug("in %s", __func__);

	ls->device_suspend = 1;
	if (ls->ps_enable != CTL_SATBY) {
		ret = bh1772_driver_ps_power_on(enabled, 0, ls->client);
		if (ret < 0) {
			printk(KERN_ERR "ps cannot disable");
			return ret;
		}
	}
	if ((ls->als_enable & 0x03) != CTL_SATBY) {
		enabled = ls->als_enable & (~CTL_STAND);
		ret = bh1772_driver_als_power_on(enabled, ls->client);
		if (ret < 0) {
			printk(KERN_ERR "als cannot disable");
			return ret;
		}
	}
	return 0;
}

static int rohm_resume(struct i2c_client *client)
{
	struct rohm_ls_data *ls = i2c_get_clientdata(client);
	int ret = 0;

	pr_debug("in %s", __func__);

	ls->device_suspend = 0;
	if (ls->ps_enable != CTL_SATBY)
		ret =
		    bh1772_driver_ps_power_on(ls->ps_enable, ls->interrupt,
					      ls->client);
	if (ret < 0) {
		printk(KERN_ERR "ps cannot resume");
		return ret;
	}
	if ((ls->als_enable & 0x03) != CTL_SATBY)
		ret = bh1772_driver_als_power_on(ls->als_enable, ls->client);
	if (ret < 0) {
		printk(KERN_ERR "als cannot resume");
		return ret;
	}
	return 0;
}

static int rohm_ls_remove(struct i2c_client *client)
{
	struct rohm_ls_data *ls = i2c_get_clientdata(client);

	pr_debug("%s to remove sensor", __func__);

	__cancel_delayed_work(&ls->als_work);
	input_unregister_device(ls->input_dev_als);
	input_unregister_device(ls->input_dev_ps);

	input_free_device(ls->input_dev_als);
	input_free_device(ls->input_dev_ps);

	sysfs_remove_group(&ls->input_dev_als->dev.kobj,
			   &sysfs_als_attribute_group);
	sysfs_remove_group(&ls->input_dev_als->dev.kobj,
			   &sysfs_ps_attribute_group);

	if (ls->use_irq)
		free_irq(client->irq, ls);
	kfree(ls);
	return 0;
}

static const struct i2c_device_id rohm_ls_id[] = {
	{ROHM_I2C_NAME, 0},
	{}
};

static struct i2c_driver rohm_ls_driver = {
	.driver = {
		   .name = ROHM_I2C_NAME,
		   },
	.probe = rohm_ls_probe,
	.suspend = rohm_suspend,
	.resume = rohm_resume,
	.remove = rohm_ls_remove,
	.id_table = rohm_ls_id,
};

static int __devinit rohm_ls_init(void)
{

	pr_info("rohm_ls_init\n");

	return i2c_add_driver(&rohm_ls_driver);
}

static void __exit rohm_ls_exit(void)
{

	pr_debug("rohm_ls_exit\n");

	i2c_del_driver(&rohm_ls_driver);
}

MODULE_DESCRIPTION("Rohm Ambient Lighit Sensor Driver");
MODULE_LICENSE("GPL");

module_init(rohm_ls_init);
module_exit(rohm_ls_exit);
