/*
 * linux/drivers/input/touchscreen/ft5x06_touch.c
 *
 * Touchscreen driver for ft5x06 touch controller.
 *
 * Copyright (C) 2012 Marvell International Ltd. (pengdu@marvell.com)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <linux/suspend.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <mach/gpio.h>
#include <plat/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/i2c/ft5x06_touch.h>
#include <plat/mfp.h>
#include <linux/input/mt.h>

#if 0
#define DEBUG
#endif

#define MAX_FINGER		(5)  /* ft5x06 can only support max 5 point */

#define FT5x06_LEN 31

#define POINT_PUTDOWN		(0)
#define POINT_PUTUP			(1)
#define POINT_CONTACT		(2)
#define POINT_INVALID		(3)

static u8 ft5x06_mode_cmd_sleep[2] = { 0xA5, 0x03 };

struct touch_finger {
	int pi;			/* point index */
	int ps;			/* point status */
	u16 px;			/* point x */
	u16 py;			/* point y */
};

struct ft5x06_touch {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct touch_finger finger[MAX_FINGER];
	struct work_struct work;
	const struct ft5x06_touch_platform_data *data;
};

static struct ft5x06_touch *touch;


int ft5x06_touch_read_reg(u8 reg, u8 *pval)
{
	int ret;
	int status;

	if (touch->client == NULL)
		return -1;
	ret = i2c_smbus_read_byte_data(touch->client, reg);
	if (ret >= 0) {
		*pval = ret;
		status = 0;
	} else {
		status = -EIO;
	}

	return status;
}

int ft5x06_touch_write_reg(u8 reg, u8 val)
{
	int ret;
	int status;

	if (touch->client == NULL)
		return -1;
	ret = i2c_smbus_write_byte_data(touch->client, reg, val);
	if (ret == 0)
		status = 0;
	else
		status = -EIO;

	return status;
}

static int ft5x06_touch_read(char *buf, int count)
{
	int ret;

	ret = i2c_master_recv(touch->client, (char *) buf, count);

	return ret;
}

static int ft5x06_touch_write(char *buf, int count)
{
	int ret;

	ret = i2c_master_send(touch->client, buf, count);

	return ret;
}

static inline int ft5x06_touch_read_data(struct ft5x06_touch *data)
{
	int ps, pi, i, b, ret;
	u8 buf[FT5x06_LEN];
	u16 px, py;
#ifdef DEBUG
	static u8 buflog[FT5x06_LEN * 5], *pbuf;
#endif

	memset(data->finger, 0xFF, MAX_FINGER * sizeof(struct touch_finger));

	ret = ft5x06_touch_read(buf, FT5x06_LEN);
	if (ret < 0)
		goto out;

#ifdef DEBUG
	pbuf = buflog;
	for (i = 0; i < FT5x06_LEN; ++i)
		pbuf += sprintf(pbuf, "%02x ", buf[i]);

	dev_info(&data->client->dev, "RAW DATA: %s\n", buflog);
#endif

	for (i = 0; i < MAX_FINGER; ++i) {
		b = 3 + i * 6;
		px = ((u16) (buf[b + 0] & 0x0F) << 8) | (u16) buf[b + 1];
		py = ((u16) (buf[b + 2] & 0x0F) << 8) | (u16) buf[b + 3];
		ps = ((u16) (buf[b + 0] & 0xC0) >> 6);
		pi = ((u16) (buf[b + 2] & 0xF0) >> 4);

		data->finger[i].px = px;
		data->finger[i].py = py;
		data->finger[i].ps = ps;
		data->finger[i].pi = pi;
	}
out:
	return ret;
}

static void ft5x06_touch_work(struct work_struct *work)
{
	struct i2c_client *client = touch->client;
	struct input_dev *input_dev = touch->input_dev;
	int status, i, ret;
	int pi, ps;
	u16 px, py;

	dev_dbg(&client->dev, "I'm in ft5x06 work\n");

	ret = ft5x06_touch_read_data(touch);

	for (i = 0; i < MAX_FINGER; ++i) {

#ifdef DEBUG
		dev_info(&client->dev,
			"REPP: i=%d pi=%d ps=0x%02x px=%d py=%d\n",
			i, touch->finger[i].pi, touch->finger[i].ps,
			touch->finger[i].px, touch->finger[i].py);
#endif

		ps = touch->finger[i].ps;
		if (POINT_INVALID == ps)
			continue;

		pi = touch->finger[i].pi;
		status = (POINT_PUTUP != ps);

		input_mt_slot(input_dev, pi);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, status);

		if (status) {
			px = touch->finger[i].px;
			py = touch->finger[i].py;
			input_report_abs(input_dev, ABS_MT_POSITION_X, px);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, py);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 16);
		}
	}

	input_sync(input_dev);
}

static irqreturn_t ft5x06_touch_irq_handler(int irq, void *dev_id)
{
	dev_dbg(&touch->input_dev->dev, "I'm in ft5x06_touch_irq_handler.\n");

	schedule_work(&touch->work);

	return IRQ_HANDLED;
}

#ifdef	CONFIG_PM
static int
ft5x06_touch_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int ft5x06_touch_resume(struct i2c_client *client)
{
	return 0;
}
#else
#define	ft5x06_touch_suspend		NULL
#define	ft5x06_touch_resume		NULL
#endif


static int ft5x06_touch_open(struct input_dev *idev)
{
	touch->data->power(1);
	return 0;
}

static void ft5x06_touch_close(struct input_dev *idev)
{
	touch->data->power(0);
	return;
}

#ifdef CONFIG_EARLYSUSPEND
static void ft5x06_touch_sleep_early_suspend(struct early_suspend *h)
{
	int ret, i = 0;

sleep_retry:
	ret = ft5x06_touch_write(ft5x06_mode_cmd_sleep, 2);
	if (ret < 0) {
		if (i < 50) {
			msleep(20);
			i++;
			dev_info(&touch->client->dev,
			       "ft5x06_touch can't enter sleep,retry %d\n",
			       i);
			goto sleep_retry;
		}
		dev_err(&touch->client->dev, "ft5x06_touch can't enter sleep\n");
		return;
	} else {
		dev_info(&touch->client->dev,
			"ft5x06_touch enter sleep mode ~~.\n");
	}
	touch->data->power(0);
}

static void ft5x06_touch_normal_late_resume(struct early_suspend *h)
{
	touch->data->power(1);
	msleep(20);
	if (touch->data->reset)
		touch->data->reset();

	dev_info(&touch->client->dev,
		"ft5x06_touch late resume to normal ~~.\n");
}


static struct early_suspend ft5x06_touch_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
	.suspend = ft5x06_touch_sleep_early_suspend,
	.resume = ft5x06_touch_normal_late_resume,
};
#endif

static int __devinit
ft5x06_touch_probe(struct i2c_client *client,
		   const struct i2c_device_id *id)
{
	struct input_dev *input_dev;
	int maxx, maxy;
	int ret;
	u8 reg_val;

	dev_info(&client->dev, "ft5x06_touch.c - ft5x06_touch_probe.\n");

	touch = kzalloc(sizeof(struct ft5x06_touch), GFP_KERNEL);
	if (touch == NULL)
		return -ENOMEM;

	touch->data = client->dev.platform_data;
	if (touch->data == NULL) {
		dev_err(&client->dev, "no platform data\n");
		return -EINVAL;
	}
	touch->client = client;
	touch->data->power(1);

	if (touch->data->reset)
		touch->data->reset();

	ret = ft5x06_touch_read_reg(0x00, (u8 *)&reg_val);
	if (ret < 0) {
		dev_err(&client->dev, "ft5x06 detect fail!\n");
		touch->client = NULL;
		return -ENXIO;
	} else {
		dev_info(&client->dev, "ft5x06 detect ok.\n");
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto out;
	}

	touch->input_dev = input_dev;
	maxx = touch->data->abs_x_max;
	maxy = touch->data->abs_y_max;

	dev_warn(&client->dev, "maxx: %d maxy: %d, virtual_key: %p\n",
		 maxx, maxy, touch->data->set_virtual_key);
	BUG_ON(maxx == 0 || maxy == 0);

	input_dev->name = "ft5x06-ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	touch->input_dev->open = ft5x06_touch_open;
	touch->input_dev->close = ft5x06_touch_close;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);

	input_mt_init_slots(input_dev, MAX_FINGER);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, maxx, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, maxy, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 16, 0, 0);

	if (touch->data->set_virtual_key)
		ret = touch->data->set_virtual_key(input_dev);
	BUG_ON(ret != 0);

	input_set_drvdata(input_dev, touch);

	ret = request_irq(client->irq, ft5x06_touch_irq_handler,
			  IRQF_DISABLED | IRQF_TRIGGER_FALLING,
			  "ft5x06 touch", touch);
	if (ret < 0)
		goto out_irg;

	if (ret) {
		dev_err(&client->dev,
		       "Request IRQ for Bigstream touch failed, return:%d\n",
		       ret);
		goto out_rg;
	}

	INIT_WORK(&touch->work, ft5x06_touch_work);

#ifdef CONFIG_EARLYSUSPEND
	register_early_suspend(&ft5x06_touch_early_suspend_desc);
#endif

	ret = input_register_device(touch->input_dev);
	if (ret < 0)
		goto out_irg;

	i2c_set_clientdata(client, touch);

	dev_info(&client->dev, "ft5x06 touch probe successfully!\n");
	return 0;

out_irg:
	free_irq(client->irq, touch);
out_rg:
	input_free_device(touch->input_dev);
out:
	kfree(touch);
	return ret;

}

static int ft5x06_touch_remove(struct i2c_client *client)
{
	input_unregister_device(touch->input_dev);
	unregister_early_suspend(&ft5x06_touch_early_suspend_desc);
	free_irq(client->irq, touch);
	kfree(touch);

	return 0;
}

static const struct i2c_device_id ft5x06_touch_id[] = {
	{"ft5x06_touch", 0},
	{}
};

static struct i2c_driver ft5x06_touch_driver = {
	.driver = {
		   .name = "ft5x06_touch",
		   },
	.id_table = ft5x06_touch_id,
	.probe = ft5x06_touch_probe,
	.remove = ft5x06_touch_remove,
	.suspend = ft5x06_touch_suspend,
	.resume = ft5x06_touch_resume,
};

static int __init ft5x06_touch_init(void)
{
	return i2c_add_driver(&ft5x06_touch_driver);
}

static void __exit ft5x06_touch_exit(void)
{
	i2c_del_driver(&ft5x06_touch_driver);
}

module_init(ft5x06_touch_init);
module_exit(ft5x06_touch_exit);

MODULE_AUTHOR("Peng Du<pengdu@marvell.com>");
MODULE_DESCRIPTION("Touchscreen driver for FT5x06 touch controller");
MODULE_LICENSE("GPL v2");
