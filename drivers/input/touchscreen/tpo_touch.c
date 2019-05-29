/*
 * * Copyright (C) 2009, Marvell Corporation(bin.yang@marvell.com).
 * *
 * * Author: Bin Yang <bin.yang@marvell.com>
 * *
 * * This software program is licensed subject to the GNU General Public License
 * * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 * */

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


struct tpo_ts_priv {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
	struct timer_list timer;
	struct device dev;
};

#define TPO_LEN 12
static u8 tpo_buf[TPO_LEN];
#define TPO_PEN_UP	0
#define TPO_PEN_DOWN	1
static int pen_status = TPO_PEN_UP;

int tpo_touch_read_reg(struct tpo_ts_priv *priv, u8 reg, u8 *pval)
{
	int ret;
	int status;

	if (priv->client == NULL)	/* No global client pointer? */
		return -1;
	ret = i2c_smbus_read_byte_data(priv->client, reg);
	if (ret >= 0) {
		*pval = ret;
		status = 0;
	} else {
		status = -EIO;
	}

	return status;
}

int tpo_touch_write_reg(struct tpo_ts_priv *priv, u8 reg, u8 val)
{
	int ret;
	int status;

	if (priv->client == NULL)	/* No global client pointer? */
		return -1;
	ret = i2c_smbus_write_byte_data(priv->client, reg, val);
	if (ret == 0)
		status = 0;
	else
		status = -EIO;

	return status;
}

static int tpo_touch_read(struct tpo_ts_priv *priv, char *buf, int count)
{
	int ret;

	ret = i2c_master_recv(priv->client, (char *)buf, count);

	return ret;
}

static int tpo_touch_write(struct tpo_ts_priv *priv, char *buf, int count)
{
	int ret;

	ret = i2c_master_send(priv->client, buf, count);

	return ret;
}

#define tpo_16(x) ((((u16)tpo_buf[x] & 0xff) << 8) | (u16)tpo_buf[x+1])
static void tpo_touch_work(struct work_struct *work)
{
	u16 tem_x1 = 0xffff;
	u16 tem_y1 = 0xffff;
	u8 tmp;

	struct tpo_ts_priv *priv =
		container_of(work, struct tpo_ts_priv, work);

	int ret = tpo_touch_read(priv, tpo_buf, TPO_LEN);
	if (ret < 0) {
		dev_dbg(&priv->client->dev, "%s: failed to receive data", __func__);
		return;
	}

	tmp = tpo_buf[10];

	if (tmp == 1) {
		if (pen_status == TPO_PEN_DOWN) {
			tem_x1 = tpo_16(2);
			tem_y1 = tpo_16(4);
			tem_y1 = 480 - tem_y1;
			input_report_abs(priv->input, ABS_PRESSURE, 255);
			input_report_abs(priv->input, ABS_X, tem_x1);
			input_report_abs(priv->input, ABS_Y, tem_y1);
			input_report_key(priv->input, BTN_TOUCH, 1);
			input_sync(priv->input);
		}
		pen_status = TPO_PEN_DOWN;
		mod_timer(&priv->timer, jiffies + msecs_to_jiffies(50));
	} else if (tmp == 0) {
		if (pen_status == TPO_PEN_DOWN) {
			pen_status = TPO_PEN_UP;
			input_report_abs(priv->input, ABS_PRESSURE, 0);
			input_report_key(priv->input, BTN_TOUCH, 0);
			input_sync(priv->input);
		}
	}
}

static irqreturn_t tpo_touch_irq_handler(int irq, void *dev_id)
{
	struct tpo_ts_priv *priv = dev_id;
	schedule_work(&priv->work);
	return IRQ_HANDLED;
}

#ifdef	CONFIG_PM
static int tpo_touch_suspend(struct i2c_client *client, pm_message_t state)
{
	struct tpo_ts_priv *priv = i2c_get_clientdata(client);

	/* sleep mode */
	tpo_touch_write_reg(priv, 0, 0x10);
	return 0;
}

static int tpo_touch_resume(struct i2c_client *client)
{
	struct tpo_ts_priv *priv = i2c_get_clientdata(client);

	/* normal mode */
	tpo_touch_write_reg(priv, 0, 0);
	return 0;
}
#else
#define	tpo_touch_suspend		NULL
#define	tpo_touch_resume		NULL
#endif

static int index;
static ssize_t tpo_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	u8 reg_val;

	struct tpo_ts_priv *priv =
		container_of(dev, struct tpo_ts_priv, dev);

	if ((index < 0) || (index > TPO_LEN))
		return 0;

	tpo_touch_read_reg(priv, index, &reg_val);
	dev_info(dev, "register 0x%x: 0x%x\n", index, reg_val);
	return 0;
}

static ssize_t tpo_store(struct device *dev,
					struct device_attribute *attr,
					const char *buff, size_t len)

{
	u8 reg_val;
	char vol[256];

	struct tpo_ts_priv *priv =
		container_of(dev, struct tpo_ts_priv, dev);

	if (len > 256)
		len = 256;

	if ('-' == buff[0]) {
		/* set the register index */
		memcpy(vol, buff+1, len-1);
		index = (int) simple_strtoul(vol, NULL, 16);
	} else {
		/* set the register value */
		reg_val = (int)simple_strtoul(buff, NULL, 16);
		tpo_touch_write_reg(priv, index, reg_val & 0xFF);
	}

	return len;
}

static DEVICE_ATTR(reg_show, 0444, tpo_show, NULL);
static DEVICE_ATTR(reg_store, 0664, NULL, tpo_store);

static struct attribute *tpo_attributes[] = {
	&dev_attr_reg_show.attr,
	&dev_attr_reg_store.attr,
	NULL
};

static const struct attribute_group tpo_attr_group = {
	.attrs = tpo_attributes,
};

static int tpo_touch_open(struct input_dev *idev)
{
	return 0;
}

static void tpo_touch_close(struct input_dev *idev)
{
	return;
}

static void tpo_send_event_workaround(unsigned long data)
{
	struct tpo_ts_priv *priv = (struct tpo_ts_priv *)(data);

	schedule_work(&priv->work);
}

static int __devinit tpo_touch_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	struct tpo_ts_priv *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		goto err0;
	}
	priv->client = client;
	priv->dev = client->dev;

	ret = tpo_touch_read(priv, tpo_buf, TPO_LEN);
	if (ret < 0) {
		dev_dbg(&client->dev, "tpo_touch unavailable!\n");
		priv->client = NULL;
		return -ENXIO;
	} else {
		dev_dbg(&client->dev, "tpo_touch(chip id:0x%02x) detected.\n", tpo_buf[0]);
	}

	/* register input device */
	priv->input = input_allocate_device();
	if (priv->input == NULL) {
		dev_dbg(&client->dev, "%s: failed to allocate input dev\n",
				__func__);
		return -ENOMEM;
	}

	priv->input->name = "tpo-ts";
	priv->input->phys = "tpo-ts/input0";
	priv->input->open = tpo_touch_open;
	priv->input->close = tpo_touch_close;

	__set_bit(EV_ABS, priv->input->evbit);
	__set_bit(ABS_X, priv->input->absbit);
	__set_bit(ABS_Y, priv->input->absbit);
	__set_bit(ABS_PRESSURE, priv->input->absbit);

	__set_bit(EV_SYN, priv->input->evbit);
	__set_bit(EV_KEY, priv->input->evbit);
	__set_bit(BTN_TOUCH, priv->input->keybit);

	input_set_abs_params(priv->input, ABS_X, 0, 320, 0, 0);
	input_set_abs_params(priv->input, ABS_Y, 0, 480, 0, 0);
	input_set_abs_params(priv->input, ABS_PRESSURE, 0, 255, 0, 0);

	ret = input_register_device(priv->input);
	if (ret) {
		dev_dbg(&client->dev,
				"%s: unabled to register input device, ret = %d\n",
				__func__, ret);
		return ret;
	}

	INIT_WORK(&priv->work, tpo_touch_work);

	init_timer(&priv->timer);
	priv->timer.function = tpo_send_event_workaround;
	priv->timer.data = (long)priv;

	ret = request_irq(client->irq, tpo_touch_irq_handler, IRQF_DISABLED | IRQF_TRIGGER_FALLING,
			"tpo touch", priv);

	if (ret) {
		dev_dbg(&client->dev, "Request IRQ for Bigstream touch failed, return:%d\n",
				ret);
		return ret;
	}

	sysfs_create_group(&client->dev.kobj, &tpo_attr_group);

err0:
	return 0;
}

static int tpo_touch_remove(struct i2c_client *client)
{
	struct tpo_ts_priv *priv = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &tpo_attr_group);
	input_unregister_device(priv->input);
	return 0;
}

static const struct i2c_device_id tpo_touch_id[] = {
	{ "tpo_touch", 0 },
	{ }
};

static struct i2c_driver tpo_touch_driver = {
	.driver = {
		.name	= "tpo_touch",
	},
	.id_table 	= tpo_touch_id,
	.probe		= tpo_touch_probe,
	.remove		= tpo_touch_remove,
	.suspend	= tpo_touch_suspend,
	.resume		= tpo_touch_resume,
};

static int __init tpo_touch_init(void)
{
	return i2c_add_driver(&tpo_touch_driver);
}

static void __exit tpo_touch_exit(void)
{
	i2c_del_driver(&tpo_touch_driver);
}

module_init(tpo_touch_init);
module_exit(tpo_touch_exit);

MODULE_DESCRIPTION("TPO touch Driver");
MODULE_LICENSE("GPL");

