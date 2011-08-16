/*
 * * Copyright (C) 2009, Notioni Corporation chenjian@notioni.com).
 * *
 * * Author: jeremy.chen
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
#include <linux/earlysuspend.h>
#include <linux/i2c/ft5306_touch.h>
#include <plat/mfp.h>


struct ft5306_touch {
	struct input_dev *idev;
	struct i2c_client *i2c;
	struct work_struct work;
	struct ft5306_touch_platform_data *data;
	int pen_status;
	int irq;
};
static struct ft5306_touch *touch;

#define FT5306_LEN 0x3E
static u8 ft5306_buf[FT5306_LEN];
static u8 ft5306_mode_cmd_sleep[2] = { 0xA5, 0x03 };
static u8 ft5306_cmd[2] = { 0x0, 0x0 };



#define FT5306_PEN_UP	0
#define FT5306_PEN_DOWN	1

int ft5306_touch_read_reg(u8 reg, u8 *pval)
{
	int ret;
	int status;

	if (touch->i2c == NULL)
		return -1;
	ret = i2c_smbus_read_byte_data(touch->i2c, reg);
	if (ret >= 0) {
		*pval = ret;
		status = 0;
	} else {
		status = -EIO;
	}

	return status;
}

int ft5306_touch_write_reg(u8 reg, u8 val)
{
	int ret;
	int status;

	if (touch->i2c == NULL)
		return -1;
	ret = i2c_smbus_write_byte_data(touch->i2c, reg, val);
	if (ret == 0)
		status = 0;
	else
		status = -EIO;

	return status;
}

static int ft5306_touch_read(char *buf, int count)
{
	int ret;

	ret = i2c_master_recv(touch->i2c, (char *) buf, count);

	return ret;
}

static int ft5306_touch_write(char *buf, int count)
{
	int ret;

	ret = i2c_master_send(touch->i2c, buf, count);

	return ret;
}

static int ft5306_touch_recieve_data(void)
{
	return ft5306_touch_read(ft5306_buf, FT5306_LEN);
}

static void ft5306_touch_work(struct work_struct *work)
{
	u16 tem_x1 = 0;
	u16 tem_y1 = 0;
	u16 tem_x2 = 0;
	u16 tem_y2 = 0;
	u8 tmp;
	u8 ret;

	ret = ft5306_touch_read(ft5306_buf, 31);

	tem_x1 = ((u16) (ft5306_buf[3] & 0xF) << 8) | (u16) ft5306_buf[4];
	tem_y1 = ((u16) (ft5306_buf[5] & 0xF) << 8) | (u16) ft5306_buf[6];
	tem_x2 = ((u16) (ft5306_buf[9] & 0xF) << 8) | (u16) ft5306_buf[10];
	tem_y2 =
	    ((u16) (ft5306_buf[11] & 0xF) << 8) | (u16) ft5306_buf[12];

	dev_dbg(&touch->i2c->dev,
	       "cj:ft5306_touch.c----x1:%d,y1:%d;x2:%d,y2:%d.\n", tem_x1,
	       tem_y1, tem_x2, tem_y2);
	tmp = ft5306_buf[2] & 0x07;
	if (tmp == 1) {
		/* One finger */
		dev_dbg(&touch->i2c->dev, "cj:ft5306_touch.c----One finger.\n");
		touch->pen_status = FT5306_PEN_DOWN;

		input_report_abs(touch->idev, ABS_MT_TRACKING_ID, 0);
		input_report_abs(touch->idev, ABS_PRESSURE, 255);
		input_report_abs(touch->idev, ABS_X, tem_x1);
		input_report_abs(touch->idev, ABS_Y, tem_y1);
		input_report_abs(touch->idev, ABS_MT_TOUCH_MAJOR, 16);
		input_report_abs(touch->idev, ABS_MT_POSITION_X, tem_x1);
		input_report_abs(touch->idev, ABS_MT_POSITION_Y, tem_y1);
		input_report_key(touch->idev, BTN_TOUCH, 1);
		input_mt_sync(touch->idev);
		input_sync(touch->idev);
	} else if (tmp == 0x2) {
		/* Two fingers */
		dev_dbg(&touch->i2c->dev, "ft5306_touch.c----Two finger.\n");
		touch->pen_status = FT5306_PEN_DOWN;

		input_report_abs(touch->idev, ABS_MT_TRACKING_ID, 0);
		input_report_abs(touch->idev, ABS_PRESSURE, 255);
		input_report_abs(touch->idev, ABS_X, tem_x1);
		input_report_abs(touch->idev, ABS_Y, tem_y1);
		input_report_abs(touch->idev, ABS_MT_TOUCH_MAJOR, 16);
		input_report_abs(touch->idev, ABS_MT_POSITION_X, tem_x1);
		input_report_abs(touch->idev, ABS_MT_POSITION_Y, tem_y1);
		input_report_key(touch->idev, BTN_TOUCH, 1);
		input_mt_sync(touch->idev);

		input_report_abs(touch->idev, ABS_MT_TRACKING_ID, 1);
		input_report_abs(touch->idev, ABS_PRESSURE, 255);
		input_report_abs(touch->idev, ABS_X, tem_x2);
		input_report_abs(touch->idev, ABS_Y, tem_y2);
		input_report_abs(touch->idev, ABS_MT_TOUCH_MAJOR, 16);
		input_report_abs(touch->idev, ABS_MT_POSITION_X, tem_x2);
		input_report_abs(touch->idev, ABS_MT_POSITION_Y, tem_y2);
		input_report_key(touch->idev, BTN_2, 1);
		input_mt_sync(touch->idev);

		input_sync(touch->idev);
	} else if (tmp == 0) {
		/* No finger */
		dev_dbg(&touch->i2c->dev, "cj:ft5306_touch.c----No finger.\n");
		touch->pen_status = FT5306_PEN_UP;

		input_report_abs(touch->idev, ABS_PRESSURE, 0);
		input_report_key(touch->idev, BTN_TOUCH, 0);
		input_report_key(touch->idev, BTN_2, 0);
		input_report_abs(touch->idev, ABS_MT_TOUCH_MAJOR, 0);
		input_sync(touch->idev);
	}
}

static irqreturn_t ft5306_touch_irq_handler(int irq, void *dev_id)
{
	dev_dbg(&touch->i2c->dev, "ft5306_touch.c----ft5306_touch_irq_handler.\n");

	schedule_work(&touch->work);

	return IRQ_HANDLED;
}

#ifdef	CONFIG_PM
static int
ft5306_touch_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int ft5306_touch_resume(struct i2c_client *client)
{
	return 0;
}
#else
#define	ft5306_touch_suspend		NULL
#define	ft5306_touch_resume		NULL
#endif

static int index;
static ssize_t ft5306_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 reg_val;

	if ((index < 0) || (index > FT5306_LEN))
		return 0;

	ft5306_touch_read_reg(index, (u8 *)&reg_val);
	dev_info(dev, "register 0x%x: 0x%x\n", index, reg_val);
	return 0;
}

static ssize_t ft5306_reg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t len)

{
	int ret;
	char vol[256] = { 0 };
	u32 reg = 0, val = 0;
	int i;

	if (len > 256)
		len = 256;

	if ('w' == buff[0]) {
		memcpy(vol, buff + 2, 4);
		reg = (int) simple_strtoul(vol, NULL, 16);
		memcpy(vol, buff + 7, 4);
		val = (int) simple_strtoul(vol, NULL, 16);
		ft5306_cmd[0] = reg;
		ft5306_cmd[1] = val;
		ret = ft5306_touch_write(ft5306_cmd, 2);
		dev_info(dev, "write! reg:0x%x, val:0x%x\n", reg, val);

	} else if ('r' == buff[0]) {
		memcpy(vol, buff + 2, 4);
		reg = (int) simple_strtoul(vol, NULL, 16);
		ret = ft5306_touch_read_reg(reg, (u8 *)&val);
		dev_info(dev, "Read! reg:0x%x, val:0x%x\n", reg, val);

	} else if ('d' == buff[0]) {
		for (i = 0x00; i <= 0x3E; i++) {
			reg = i;
			ft5306_touch_read_reg(reg, (u8 *)&val);
			msleep(2);
			dev_info(dev, "Display! reg:0x%x, val:0x%x\n",
			       reg, val);
		}
	}
	return len;
}


static DEVICE_ATTR(reg_show, 0444, ft5306_reg_show, NULL);
static DEVICE_ATTR(reg_store, 0664, NULL, ft5306_reg_store);

static struct attribute *ft5306_attributes[] = {
	&dev_attr_reg_show.attr,
	&dev_attr_reg_store.attr,
	NULL
};

static const struct attribute_group ft5306_attr_group = {
	.attrs = ft5306_attributes,
};

static void ft5306_reset(void)
{
	if (gpio_request(MFP_PIN_GPIO46, "ft5306_reset")) {
		dev_dbg(&touch->i2c->dev, "Failed to request GPIO for ft5306_reset pin!\n");
		goto out;
	}
	gpio_direction_output(MFP_PIN_GPIO46, 0);
	mdelay(5);
	gpio_direction_output(MFP_PIN_GPIO46, 1);
	dev_dbg(&touch->i2c->dev, "ft5306_touch reset successful.\n");
	gpio_free(MFP_PIN_GPIO46);
out:
	return;
}

static int ft5306_touch_open(struct input_dev *idev)
{
	touch->data->power(1);
	return 0;
}

static void ft5306_touch_close(struct input_dev *idev)
{
	touch->data->power(0);
	return;
}

#ifdef CONFIG_EARLYSUSPEND
static void ft5306_touch_sleep_early_suspend(struct early_suspend *h)
{
	int ret, i = 0;

sleep_retry:
	ret = ft5306_touch_write(ft5306_mode_cmd_sleep, 2);
	if (ret < 0) {
		if (i < 50) {
			msleep(5);
			i++;
			dev_dbg(&touch->i2c->dev,
			       "ft5306_touch can't enter sleep,retry %d\n",
			       i);
			goto sleep_retry;
		}
		dev_dbg(&touch->i2c->dev, "ft5306_touch can't enter sleep\n");
		return;
	} else {
		dev_dbg(&touch->i2c->dev, "ft5306_touch enter sleep mode.\n");
	}
	touch->data->power(0);
}

static void ft5306_touch_normal_late_resume(struct early_suspend *h)
{
	touch->data->power(1);
	msleep(10);
	ft5306_reset();
}


static struct early_suspend ft5306_touch_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
	.suspend = ft5306_touch_sleep_early_suspend,
	.resume = ft5306_touch_normal_late_resume,
};
#endif

static int __devinit
ft5306_touch_probe(struct i2c_client *client,
		   const struct i2c_device_id *id)
{
	int ret;
	u8 reg_val;

	dev_dbg(&client->dev, "ft5306_touch.c----ft5306_touch_probe.\n");

	touch = kzalloc(sizeof(struct ft5306_touch), GFP_KERNEL);
	if (touch == NULL)
		return -ENOMEM;

	touch->data = client->dev.platform_data;
	if (touch->data == NULL) {
		dev_dbg(&client->dev, "no platform data\n");
		return -EINVAL;
	}
	touch->i2c = client;
	touch->irq = client->irq;
	touch->pen_status = FT5306_PEN_UP;
	touch->data->power(1);
	ret = ft5306_touch_read_reg(0x00, (u8 *)&reg_val);
	if (ret < 0) {
		dev_dbg(&client->dev, "ft5306 detect fail!\n");
		touch->i2c = NULL;
		return -ENXIO;
	} else {
		dev_dbg(&client->dev, "ft5306 reset.\n");
	}

	ft5306_reset();

	/* register input device */
	touch->idev = input_allocate_device();
	if (touch->idev == NULL) {
		dev_dbg(&client->dev, "%s: failed to allocate input dev\n",
		       __func__);
		ret = -ENOMEM;
		goto out;
	}

	touch->idev->name = "ft5306-ts";
	touch->idev->phys = "ft5306-ts/input0";
	touch->idev->open = ft5306_touch_open;
	touch->idev->close = ft5306_touch_close;

	__set_bit(EV_ABS, touch->idev->evbit);
	__set_bit(ABS_X, touch->idev->absbit);
	__set_bit(ABS_Y, touch->idev->absbit);
	__set_bit(ABS_MT_POSITION_X, touch->idev->absbit);
	__set_bit(ABS_MT_POSITION_Y, touch->idev->absbit);
	__set_bit(ABS_PRESSURE, touch->idev->absbit);

	__set_bit(EV_SYN, touch->idev->evbit);
	__set_bit(EV_KEY, touch->idev->evbit);
	__set_bit(BTN_TOUCH, touch->idev->keybit);
	__set_bit(BTN_2, touch->idev->keybit);

	input_set_abs_params(touch->idev, ABS_MT_TOUCH_MAJOR, 0, 16, 0, 0);
	input_set_abs_params(touch->idev, ABS_X, 0, 480, 0, 0);
	input_set_abs_params(touch->idev, ABS_Y, 0, 800, 0, 0);
	input_set_abs_params(touch->idev, ABS_PRESSURE, 0, 255, 0, 0);

	/*   muti touch */
	input_set_abs_params(touch->idev, ABS_MT_POSITION_X, 0, 480, 0, 0);
	input_set_abs_params(touch->idev, ABS_MT_POSITION_Y, 0, 800, 0, 0);

	ret = input_register_device(touch->idev);
	if (ret) {
		dev_dbg(&client->dev,
		       "%s: unabled to register input device, ret = %d\n",
		       __func__, ret);
		goto out_rg;
	}

	ret = request_irq(touch->irq, ft5306_touch_irq_handler,
			  IRQF_DISABLED | IRQF_TRIGGER_FALLING,
			  "ft5306 touch", touch);
	if (ret < 0)
		goto out_irg;

	if (ret) {
		dev_dbg(&client->dev,
		       "Request IRQ for Bigstream touch failed, return:%d\n",
		       ret);
		goto out_rg;
	}
	INIT_WORK(&touch->work, ft5306_touch_work);
	register_early_suspend(&ft5306_touch_early_suspend_desc);

	ret = sysfs_create_group(&client->dev.kobj, &ft5306_attr_group);
	if (ret)
		goto out_irg;

	return 0;

out_irg:
	free_irq(touch->irq, touch);
out_rg:
	input_free_device(touch->idev);
out:
	kfree(touch);
	return ret;

}

static int ft5306_touch_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &ft5306_attr_group);

	input_unregister_device(touch->idev);
	return 0;
}

static const struct i2c_device_id ft5306_touch_id[] = {
	{"ft5306_touch", 0},
	{}
};

static struct i2c_driver ft5306_touch_driver = {
	.driver = {
		   .name = "ft5306_touch",
		   },
	.id_table = ft5306_touch_id,
	.probe = ft5306_touch_probe,
	.remove = ft5306_touch_remove,
	.suspend = ft5306_touch_suspend,
	.resume = ft5306_touch_resume,
};

static int __init ft5306_touch_init(void)
{
	return i2c_add_driver(&ft5306_touch_driver);
}

static void __exit ft5306_touch_exit(void)
{
	unregister_early_suspend(&ft5306_touch_early_suspend_desc);
	i2c_del_driver(&ft5306_touch_driver);
}


module_init(ft5306_touch_init);
module_exit(ft5306_touch_exit);

MODULE_DESCRIPTION("ft5306 touch Driver");
MODULE_LICENSE("GPL");
