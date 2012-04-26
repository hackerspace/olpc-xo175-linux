/*
 *  linux/drivers/input/touchscreen/SSD2531_touch.c
 *
 *  touch screen driver for Solomon systech SSD2531
 *
 *  Copyright (C) 2006, Marvell Corporation (fengwei.yin@Marvell.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>
#include <linux/proc_fs.h>
#include <linux/earlysuspend.h>

#include <asm/irq.h>
#include <mach/hardware.h>

#include <mach/regs-ost.h>
#include <mach/irqs.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <plat/mfp.h>
#include <linux/gpio.h>
#include <linux/SSD2531_touch.h>

#define TOUCHSCREEN_X_AXIS_MIN				(0)
#define TOUCHSCREEN_X_AXIS_MAX				(479)
#define TOUCHSCREEN_Y_AXIS_MIN				(0)
#define TOUCHSCREEN_Y_AXIS_MAX				(800)
#define TOUCHSCREEN_Z_AXIS_MIN				(0)
#define TOUCHSCREEN_Z_AXIS_MAX				(0x0f)
#define TOUCHSCREEN_WIDTH_MIN				(0)
#define TOUCHSCREEN_WIDTH_MAX				(15)
#define TOUCHSCREEN_ID_MIN					(0)
#define TOUCHSCREEN_ID_MAX					(3)
#define MAX_NUM_OF_POINTS					(4)
#define POINT_DATA_SIZE						(4)
#define IS_FINGER_DOWN(i, status)			((0x01<<i)&(status))

struct ssd2531_command_entry {
	u8 reg;			/*register to write */
	u8 is_word_value;	/*write the data to i2c as word */
	u16 value;		/*value to write */
	u32 delay;		/*delay in us before the next command */
} __attribute__ ((__packed__));

struct finger_info {
	u16 x;
	u16 y;
	u16 weight;
	u16 f_id;
};

struct ssd2531_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int prev_finger_bitmask;
	struct finger_info prev_finger_info[MAX_NUM_OF_POINTS];
	int btn_info[MAX_NUM_OF_POINTS];
	int b_is_suspended;
	int mfp_gpio_pin_int;
	int mfp_gpio_pin_reset;
	struct delayed_work *work;
};

struct ssd2531_ts_data *g_p_ssd2531_data;
struct ssd2531_platform_data *touch_data;
/*////////////////////////////////////////////////////////////////////////*/
static int ssd2531_ts_reset(void);
static int ssd2531_ts_init(void);
static void ssd2531_ts_crl_all_events(void);

#if defined CONFIG_PROC_FS
#define SSD2531_PROC_FILE_PATH	"driver/ssd2531_ts"

static int
ssd2531_ts_proc_write(struct file *file, const char __user * buffer,
		      unsigned long count, void *data)
{
	static char kbuf[1024];

	if (count >= 1024)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	if ('?' == kbuf[0]) {
		printk(KERN_INFO
		       "\nHowTo: echo [clr|rst] > /proc/driver/ssd2531_ts\n");
		printk(KERN_INFO
		       " clr -\tclear pending events\n rst -\treset touch\n");
	} else if (strncmp(kbuf, "rst", 3) == 0) {
		printk(KERN_INFO "\nrunning reset\n");
		ssd2531_ts_reset();
		ssd2531_ts_init();
	} else if (strncmp(kbuf, "clr", 3) == 0) {
		printk(KERN_INFO "\nclearing all pending events\n");
		ssd2531_ts_crl_all_events();
	} else
		printk(KERN_ERR "unknown command\n");
	return count;
}

static void ssd2531_create_proc_entry(void)
{
	struct proc_dir_entry *proc_entry;
	proc_entry = create_proc_entry(SSD2531_PROC_FILE_PATH, 0, NULL);
	if (proc_entry)
		proc_entry->write_proc = ssd2531_ts_proc_write;
	else
		dev_err(&(g_p_ssd2531_data->input_dev->dev),
			"ssd2531: failed to create proc entry");
}

static void ssd2531_remove_proc_entry(void)
{
	remove_proc_entry(SSD2531_PROC_FILE_PATH, NULL);
}
#else
static void ssd2531_create_proc_entry(void)
{
}

static void ssd2531_remove_proc_entry(void)
{
}
#endif
/*////////////////////////////////////////////////////////////////////////*/

/**************************************************************************/
/*SSD2531 i2c driver*/
/*////////////////////////////////////////////////////////////////////////*/
/*/ need to move to h file touch reg discription*/
#define SSD2531_BIT_0						((0x1)<<0)
#define SSD2531_BIT_1						((0x1)<<1)
#define SSD2531_BIT_2						((0x1)<<2)
#define SSD2531_BIT_3						((0x1)<<3)
#define SSD2531_BIT_4						((0x1)<<4)
#define SSD2531_BIT_5						((0x1)<<5)
#define SSD2531_BIT_6						((0x1)<<6)
#define SSD2531_BIT_7						((0x1)<<7)

#define SSD2531_SYSTEM_ENABLE_REG				0x23
#define SSD2531_SYSTEM_DISABLE_REG				0x24
#define SSD2531_EVENT_STATUS_REG				0x79
#define SSD2531_STATUS_FINGER_1_DETECTED_BIT	SSD2531_BIT_0
#define SSD2531_STATUS_FINGER_2_DETECTED_BIT	SSD2531_BIT_1
#define SSD2531_STATUS_FINGER_3_DETECTED_BIT	SSD2531_BIT_2
#define SSD2531_STATUS_FINGER_4_DETECTED_BIT	SSD2531_BIT_3
#define SSD2531_STATUS_FIFO_NOT_EMPTY_BIT		SSD2531_BIT_4
#define SSD2531_STATUS_FIFO_OVERFLOW_EMPTY_BIT	SSD2531_BIT_5
#define SSD2531_STATUS_LARGE_OBJECT_BIT			SSD2531_BIT_6
#define SSD2531_EVENT_MASK_REG					0x7A
#define SSD2531_IRQ_MASK_REG					0x7B

#define SSD2531_FINGER_1_REG					0x7C
#define SSD2531_FINGER_2_REG					0x7D
#define SSD2531_FINGER_3_REG					0x7E
#define SSD2531_FINGER_4_REG					0x7F

#define SSD2531_EVENT_STACK_REG					0x80
#define SSD2531_EVENT_STACK_CLEAR_REG			0x81

/*////////////////////////////////////////////////////////////////////////*/
static int ssd2531_i2c_read_byte(u8 reg)
{
	return i2c_smbus_read_byte_data(g_p_ssd2531_data->client, reg);
}

static int ssd2531_i2c_read_word(u8 reg)
{
	return i2c_smbus_read_word_data(g_p_ssd2531_data->client, reg);
}

static int ssd2531_i2c_read_burst(u8 reg, u8 *buff, int sz)
{
	int rc;
	rc = i2c_smbus_read_i2c_block_data(g_p_ssd2531_data->client,
					   reg, sz, buff);
	if (rc != sz)
		printk(KERN_ERR
		       "SSD2531 read burst failed to read all bytes\n");
	return rc;

}

static int ssd2531_i2c_write(u8 reg, u16 val)
{
	int ret;
	ret = i2c_smbus_write_byte_data(g_p_ssd2531_data->client, reg, val);
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
		"ssd2531_i2c_w(reg=0x%x,val=0x%x)\n", reg, val);
	if (ret < 0)
		dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
			"failed to write to reg 0x%x, rc %d", reg, ret);
	return ret;
}

static int ssd2531_i2c_write_word(u8 reg, u16 val)
{
	return i2c_smbus_write_word_data(g_p_ssd2531_data->client, reg, val);
}

int ssd2531_ts_wakeup(int enable)
{
	u8 reg;
	int ret = 0;
	printk(KERN_NOTICE "ssd2531_ts_wakeup");
	reg =
	    ((enable ==
	      1) ? SSD2531_SYSTEM_ENABLE_REG : SSD2531_SYSTEM_DISABLE_REG);
	if (enable == 1) {
		if (touch_data->set_power) {
			ret = touch_data->set_power(1);
			if (ret)
				return ret;
		}
		/*Enable/disable operation will cause i2c no response
		  So, don't checke the return value here.*/
		ssd2531_i2c_write(reg, 0x00);
	} else {
		ssd2531_i2c_write(reg, 0x00);
		if (touch_data->set_power)
			return touch_data->set_power(0);
	}
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static int ssd2531_ts_resume(struct platform_device *pdev);
static int ssd2531_ts_suspend(struct platform_device *pdev, pm_message_t state);

static void ssd2531_ts_early_suspend(struct early_suspend *h)
{
	pm_message_t t;
	t.event = 0;
	if (ssd2531_ts_suspend != NULL)
		ssd2531_ts_suspend(NULL, t);
}

static void ssd2531_ts_late_resume(struct early_suspend *h)
{
	if (ssd2531_ts_resume != NULL)
		ssd2531_ts_resume(NULL);
}

static struct early_suspend ssd2531_ts_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = ssd2531_ts_early_suspend,
	.resume = ssd2531_ts_late_resume,
};

static int ssd2531_ts_resume(struct platform_device *pdev)
{
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "++ssd2531_ts_resume\n");
	if (g_p_ssd2531_data->b_is_suspended == 1) {
		ssd2531_ts_reset();
		ssd2531_ts_init();
		ssd2531_ts_crl_all_events();
	}
	g_p_ssd2531_data->b_is_suspended = 0;
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "--ssd2531_ts_resume\n");
	return 0;
}

static int ssd2531_ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "++ssd2531_ts_suspend\n");
	if (g_p_ssd2531_data->b_is_suspended == 0) {
		del_timer_sync(&(g_p_ssd2531_data->work->timer));
		/*delete timer and wait for timer task to end
		   need to wait to prevent i2c issues and extra wakeups */
		ssd2531_ts_wakeup(0);
	}
	g_p_ssd2531_data->b_is_suspended = 1;

	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "--ssd2531_ts_suspend\n");
	return 0;
}
#endif

static void ssd2531_timer_work_func(struct work_struct *work)
{
	int status;
	do {
		status = ssd2531_i2c_read_byte(SSD2531_EVENT_STATUS_REG);
		if (status < 0) {
			dev_err(&(g_p_ssd2531_data->input_dev->dev),
				"failed to read SSD2531 data via i2c\n");
			return;
		}
		if (status != 0) {
			dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
				"calling clear all, stat %x\n", status);
			ssd2531_ts_crl_all_events();
		}
	} while (status != 0);

	input_mt_sync(g_p_ssd2531_data->input_dev);
	input_sync(g_p_ssd2531_data->input_dev);
}

static void ssd2531_ts_work_func(struct work_struct *work)
{
	uint8_t read_buf[POINT_DATA_SIZE];	/*each point has 4 byte data */
	struct finger_info info;
	int i;
	int status;
	int reg;
	int send_press;
	int pressed_finger = -1;

	/*printk(KERN_NOTICE"ssd2531_ts_work_func ++\n"); */
	/*read number of point down */
	status = ssd2531_i2c_read_byte(SSD2531_EVENT_STATUS_REG);
	if (status < 0) {
		printk(KERN_ERR "failed to read SSD2531 dava via i2c\n");
		return;
	}
	for (i = 0; i < MAX_NUM_OF_POINTS; ++i) {
		reg = SSD2531_FINGER_1_REG + i;
		send_press = 0;
		info.f_id = i;

		if (IS_FINGER_DOWN(i, status)) {
			ssd2531_i2c_read_burst(reg, read_buf, POINT_DATA_SIZE);
			info.x = read_buf[0] | ((read_buf[2] & 0xF0) << 4);
			info.y = read_buf[1] | ((read_buf[2] & 0x0F) << 8);
			info.weight = (read_buf[3] & 0xF0) >> 4;
			if (info.weight == 0)
				info.weight++;
			if (info.x > TOUCHSCREEN_X_AXIS_MAX ||
			    info.y > TOUCHSCREEN_Y_AXIS_MAX) {
				dev_warn(&(g_p_ssd2531_data->input_dev->dev),
					 "received index out of range value\n");
				/*this is a bit ugly but will work
				   this will report the same point and prev
				   successfull read. */
				info.x =
				    g_p_ssd2531_data->prev_finger_info[i].x;
				info.y =
				    g_p_ssd2531_data->prev_finger_info[i].y;
				info.weight =
				    g_p_ssd2531_data->
				    prev_finger_info[i].weight;
			}
			/*update prev location array.
			   may need to move this in case we use averaging */
			g_p_ssd2531_data->prev_finger_info[i].x = info.x;
			g_p_ssd2531_data->prev_finger_info[i].y = info.y;
			g_p_ssd2531_data->prev_finger_info[i].weight =
			    info.weight;

			dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
				"finger %d x-%x, y-%x, weight-%x\n",
				info.f_id, info.x, info.y, info.weight);
			send_press = 1;
			if (pressed_finger < 0)
				pressed_finger = i;
		} else {
			if (IS_FINGER_DOWN
			    (i, g_p_ssd2531_data->prev_finger_bitmask)) {
				/*was down and now not -> finger up event */
				dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
					"finger %d up\n", i);
				info.x =
				    g_p_ssd2531_data->prev_finger_info[i].x;
				info.y =
				    g_p_ssd2531_data->prev_finger_info[i].y;
				info.weight = 0;	/*0 indicated release */
				send_press = 1;
				/*update prev location array.
				   may need to move this in case
				   we need to use averaging */
				g_p_ssd2531_data->prev_finger_info[i].x = 0;
				g_p_ssd2531_data->prev_finger_info[i].y = 0;
				g_p_ssd2531_data->prev_finger_info[i].weight =
				    0;
			}
		}
		if (send_press) {
			input_report_abs(g_p_ssd2531_data->input_dev,
					 ABS_MT_POSITION_X, info.x);
			input_report_abs(g_p_ssd2531_data->input_dev,
					 ABS_MT_POSITION_Y, info.y);
			input_report_abs(g_p_ssd2531_data->input_dev,
					 ABS_MT_PRESSURE, info.weight);
			input_report_abs(g_p_ssd2531_data->input_dev,
					 ABS_MT_TRACKING_ID, info.f_id);
			input_mt_sync(g_p_ssd2531_data->input_dev);
		}
	}
	if (pressed_finger < 0) {	/*all fingers are up */
		input_mt_sync(g_p_ssd2531_data->input_dev);
		input_sync(g_p_ssd2531_data->input_dev);
	} else {
		input_report_abs(g_p_ssd2531_data->input_dev, ABS_X,
				 g_p_ssd2531_data->prev_finger_info
				 [pressed_finger].x);
		input_report_abs(g_p_ssd2531_data->input_dev, ABS_Y,
				 g_p_ssd2531_data->prev_finger_info
				 [pressed_finger].y);
		input_report_abs(g_p_ssd2531_data->input_dev, ABS_PRESSURE,
				 g_p_ssd2531_data->prev_finger_info
				 [pressed_finger].weight);
		if (g_p_ssd2531_data->prev_finger_bitmask == 0) {
			input_report_key(g_p_ssd2531_data->input_dev,
					 g_p_ssd2531_data->btn_info
					 [pressed_finger], 1);
			dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
				"send press %d-(%d,%d)\n", pressed_finger,
				g_p_ssd2531_data->prev_finger_info
				[pressed_finger].x,
				g_p_ssd2531_data->prev_finger_info
				[pressed_finger].y);
		}
		input_sync(g_p_ssd2531_data->input_dev);
	}
	g_p_ssd2531_data->prev_finger_bitmask = status & 0x0F;
	if (status & (SSD2531_STATUS_FIFO_OVERFLOW_EMPTY_BIT |
		      SSD2531_STATUS_FIFO_NOT_EMPTY_BIT))
		ssd2531_ts_crl_all_events();
	/*set a timer for some time to check for key up event */
	mod_timer(&g_p_ssd2531_data->work->timer, jiffies + 4);
}

static void ssd2531_timer_handler(unsigned long data)
{
	/*key is reported as down and no irq for some time */
	if (g_p_ssd2531_data->prev_finger_bitmask != 0) {
		dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
			"ssd2531: timer timeout and key is down\n");
		schedule_work(&g_p_ssd2531_data->work->work);
	}
}

static int ssd2531_ts_reset(void)
{
	int ret = 0;
	unsigned rst_gpio;
	unsigned inr_gpio;
	mfp_cfg_t config[] = {
		g_p_ssd2531_data->mfp_gpio_pin_int | MFP_LPM_EDGE_FALL,
	};

	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "++ssd2531_ts_reset\n");

	rst_gpio = mfp_to_gpio(g_p_ssd2531_data->mfp_gpio_pin_reset);
	inr_gpio = mfp_to_gpio(g_p_ssd2531_data->mfp_gpio_pin_int);
	if (gpio_request(rst_gpio, "ssd2531_reset")) {
		printk(KERN_ERR "Request GPIO failed," "gpio: %d\n", rst_gpio);
		return -EIO;
	}
	if (gpio_request(inr_gpio, "ssd2531_interrupt")) {
		printk(KERN_ERR "Request GPIO failed," "gpio: %d\n", inr_gpio);
		gpio_free(rst_gpio);
		return -EIO;
	}
	gpio_direction_input(inr_gpio);	/*set as input */
	/*set as output */
	gpio_direction_output(rst_gpio, 1);
	udelay(30);
	gpio_direction_output(rst_gpio, 0);
	udelay(20);
	gpio_direction_output(rst_gpio, 1);

	mfp_config(config, ARRAY_SIZE(config));
	gpio_free(rst_gpio);
	gpio_free(inr_gpio);
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "--ssd2531_ts_reset\n");
	udelay(10);
	return ret;
}

static struct ssd2531_command_entry init_sequence[] = {
	/*reg, is Word, value, delay after write */
	{0x2B, 0, 0x03, 0},	/*Enable DSP clock */
	{0xD9, 0, 0x01, 0},
	{0x2C, 0, 0x02, 0},	/* Set median filter to 1 tap */
	{0x5A, 0, 0x00, 0},	/*Set maximum miss frame to 0 */
	{0x3D, 0, 0x01, 0},
	{0xD8, 0, 0x03, 0},	/*Set sampling delay */
	{0xD4, 0, 0x00, 0},
	{0x06, 0, 0x1F, 0},	/* Set drive line no. */
	{0x07, 0, 0x06, 0},	/* Set sense line no. = 12 */
	{0x08, 0, 0x00, 0},
	{0x09, 0, 0x01, 0},
	{0x0A, 0, 0x02, 0},
	{0x0B, 0, 0x03, 0},
	{0x0C, 0, 0x04, 0},
	{0x0D, 0, 0x05, 0},
	{0x0E, 0, 0x06, 0},
	{0x0F, 0, 0x07, 0},
	{0x10, 0, 0x08, 0},
	{0x11, 0, 0x09, 0},
	{0x12, 0, 0x0A, 0},
	{0x13, 0, 0x0B, 0},
	{0x14, 0, 0x0C, 0},
	{0x15, 0, 0x0D, 0},
	{0x16, 0, 0x0E, 0},
	{0x17, 0, 0x0F, 0},
	{0x18, 0, 0x10, 0},
	{0x19, 0, 0x11, 0},
	{0x1A, 0, 0x12, 0},
	{0x1B, 0, 0x13, 0},
	{0x1C, 0, 0x14, 0},
	{0x2A, 0, 0x03, 0},	/* Set sub-frame */
	{0x8D, 0, 0x01, 0},	/*not exsit in spec */
	{0x8E, 0, 0x02, 0},	/*not exsit in spec */
	{0x94, 1, 0x0000, 0},	/*not exsit in spec */
	{0x8D, 0, 0x00, 0},	/*not exsit in spec */
	{0x25, 0, 0x02, 0},	/* Set scan mode */
	{0xC1, 0, 0x02, 0},	/*booster control */
	{0xD5, 0, 0x0F, 1000},	/*driving voltage */
	{0x38, 0, 0x00, 0},
	{0x33, 0, 0x01, 0},	/*min finger area */
	{0x34, 0, 0x60, 0},	/*min finger level */
	{0x35, 1, 0x1000, 0},	/*min finger weight */
	{0x36, 0, 0x1E, 0},	/*max finger area */
	{0x37, 0, 0x03, 0},	/*control depth of .. */
	{0x39, 0, 0x00, 0},	/*select CG caculation method */
	{0x56, 0, 0x01, 0},	/*smooth finger's output coordinate */
	{0x51, 1, 0x0000, 0},	/*single click time - change to 0
				   we do not want click event */
	{0x52, 1, 0xFF02, 0},	/*double click time */
	{0x53, 0, 0x08, 0},	/*CG TOLENCE */
	{0x54, 0, 0x14, 0},	/*x tracking tolence */
	{0x55, 0, 0x14, 0},	/*y tracking tolence */
	{0x65, 0, 0x02, 0},	/*Added by Marvell to invert X axis */
	/*set scaling to fit 480*800 resolution */
	/*default resolution is 352*640 (# of lines * 32) */
	{0x66, 0, 0x57, 0},	/*need to change X from 352 to 480 ->
				   factor is 1.363636,
				   closest value is 1.359375.
				   this gives res of 478 */
	{0x67, 0, 0x50, 0},	/*need to change Y from 640 to 800 ->
				   factor is 1.25 */
	{0xA2, 0, 0x00, 0},	/*reset init reference procedure */
	{0xFF, 0xFF, 0xFF, 0xFF}	/*last entry */
};

static int ssd2531_ts_init(void)
{
	int ret;
	u8 reg;
	u16 value;
	int i = 0;

	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "ssd2531_ts_init ++\n");
	/*Below commands must send first in order wake up the IC */
	ret = ssd2531_ts_wakeup(1);	/*Exit sleep mode */
	if (ret < 0) {
		dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
			"error writing to touch I2C, exiting");
		return -EIO;
	}
	udelay(500);

	do {
		reg = init_sequence[i].reg;
		value = init_sequence[i].value;
		if (init_sequence[i].is_word_value != 0)
			ssd2531_i2c_write_word(reg, value);
		else
			ssd2531_i2c_write(reg, value);
		if (init_sequence[i].delay > 0)
			udelay(init_sequence[i].delay);
		i++;
	} while (init_sequence[i].is_word_value != 0xFF);

	reg = 0x02;
	value = ssd2531_i2c_read_word(reg);
	value = ((value & 0xFF) << 8) | ((value & 0xFF00) >> 8);
	printk(KERN_INFO "ssd2531 chip Id is 0x%x\n", value);

	dev_dbg(&(g_p_ssd2531_data->input_dev->dev), "ssd2531_ts_init --\n");

	return 0;
}

static void ssd2531_ts_crl_all_events(void)
{
	u8 reg = SSD2531_EVENT_STACK_REG;
	uint8_t read_buf[POINT_DATA_SIZE];
	int value;

	dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
		"ssd2531_ts_crl_all_events ++\n");
	while (1) {
		ssd2531_i2c_read_burst(reg, read_buf, POINT_DATA_SIZE);
		if (read_buf[0] != 0) {
			dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
				"found event # %d in FIFO. flag = %d, x-%d, y-%d\n",
				(read_buf[0] & 0x0F),
				((read_buf[0] & 0xF0) >> 4),
				(((read_buf[3] & 0xF0) << 4) | read_buf[1]),
				(((read_buf[3] & 0x0F) << 8) | read_buf[2]));
		}
		value = ssd2531_i2c_read_byte(SSD2531_EVENT_STATUS_REG);
		if ((value < 0) ||
		    (!(value & (SSD2531_STATUS_FIFO_OVERFLOW_EMPTY_BIT |
				SSD2531_STATUS_FIFO_NOT_EMPTY_BIT))))
			break;
	}

	ssd2531_i2c_read_byte(SSD2531_EVENT_STACK_CLEAR_REG);
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
		"ssd2531_ts_crl_all_events --\n");
}

static int ssd2531_ts_interrupt_init(void)
{
	u8 reg;
	u16 value;

	dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
		"++ssd2531_ts_interrupt_init\n");
	reg = SSD2531_IRQ_MASK_REG;
	value = 0xE0;
	ssd2531_i2c_write(reg, value);	/*disable interrupts */

	/*mask all events except finger remove event,
	   since we are using only interrupt
	   and status register for all others */
	reg = SSD2531_EVENT_MASK_REG;
	value = 0xBFBF;
	ssd2531_i2c_write_word(reg, value);

	/*need to add GPIO settings if needed */
	dev_dbg(&(g_p_ssd2531_data->input_dev->dev),
		"--ssd2531_ts_interrupt_init\n");
	return 0;
}

static irqreturn_t ssd2531_ts_irq_handler(int irq, void *dev_id)
{
	del_timer(&(g_p_ssd2531_data->work->timer));
	ssd2531_ts_work_func(NULL);
	return IRQ_HANDLED;
}

static int __devinit
ssd2531_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int *mfp_pins;
	unsigned long flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;

	printk(KERN_NOTICE "++ssd2531_i2c_probe\n");

	if (g_p_ssd2531_data != NULL) {
		printk(KERN_ERR
		       "SSD2531_i2c_probe: call with device not NULL!!");
		return -ENOMEM;
	}

	touch_data = ((struct ssd2531_platform_data *)(client->dev.platform_data));
	mfp_pins = touch_data->pin_data;
	if (mfp_pins == NULL)
		return -EINVAL;

	g_p_ssd2531_data = kzalloc(sizeof(struct ssd2531_ts_data), GFP_KERNEL);
	if (!g_p_ssd2531_data)
		return -ENOMEM;

	g_p_ssd2531_data->mfp_gpio_pin_int = mfp_pins[0];
	g_p_ssd2531_data->mfp_gpio_pin_reset = mfp_pins[1];

	g_p_ssd2531_data->prev_finger_bitmask = 0;
	g_p_ssd2531_data->b_is_suspended = 0;
	g_p_ssd2531_data->btn_info[0] = BTN_TOUCH;
	g_p_ssd2531_data->btn_info[1] = BTN_2;
	g_p_ssd2531_data->btn_info[2] = BTN_3;
	g_p_ssd2531_data->btn_info[3] = BTN_4;

	g_p_ssd2531_data->client = client;

	i2c_set_clientdata(client, g_p_ssd2531_data);

	ret = ssd2531_ts_reset();
	if (ret < 0) {
		printk(KERN_ERR "failed to reset ssd2531 touch screen\n");
		goto err_no_dev;
	}
	ret = ssd2531_ts_init();
	if (ret < 0) {
		printk(KERN_ERR "failed to init ssd2531 touch screen\n");
		goto err_no_dev;
	}

	ret = ssd2531_ts_interrupt_init();
	if (ret < 0) {
		printk(KERN_ERR "failed to init ssd2531 interrupt\n");
		goto err_no_dev;
	}
	g_p_ssd2531_data->input_dev = input_allocate_device();
	if (g_p_ssd2531_data->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "failed to allocate input device\n");
		goto err_dev_alloc;
	}
	g_p_ssd2531_data->input_dev->name = "ssd2531-touch";
	set_bit(EV_SYN, g_p_ssd2531_data->input_dev->evbit);
	set_bit(EV_KEY, g_p_ssd2531_data->input_dev->evbit);
	set_bit(BTN_TOUCH, g_p_ssd2531_data->input_dev->keybit);
	set_bit(EV_ABS, g_p_ssd2531_data->input_dev->evbit);

	input_set_capability(g_p_ssd2531_data->input_dev, EV_KEY, BTN_TOUCH);
	input_set_capability(g_p_ssd2531_data->input_dev, EV_KEY, BTN_2);
	input_set_capability(g_p_ssd2531_data->input_dev, EV_KEY, BTN_3);
	input_set_capability(g_p_ssd2531_data->input_dev, EV_KEY, BTN_4);

	input_set_abs_params(g_p_ssd2531_data->input_dev,
			     ABS_X, TOUCHSCREEN_X_AXIS_MIN,
			     TOUCHSCREEN_X_AXIS_MAX, 0, 0);
	input_set_abs_params(g_p_ssd2531_data->input_dev,
			     ABS_Y, TOUCHSCREEN_Y_AXIS_MIN,
			     TOUCHSCREEN_Y_AXIS_MAX, 0, 0);
	input_set_abs_params(g_p_ssd2531_data->input_dev,
			     ABS_PRESSURE, TOUCHSCREEN_Z_AXIS_MIN,
			     TOUCHSCREEN_Z_AXIS_MAX, 0, 0);

	input_set_abs_params(g_p_ssd2531_data->input_dev,
			     ABS_MT_POSITION_X, TOUCHSCREEN_X_AXIS_MIN,
			     TOUCHSCREEN_X_AXIS_MAX, 0, 0);
	input_set_abs_params(g_p_ssd2531_data->input_dev,
			     ABS_MT_POSITION_Y, TOUCHSCREEN_Y_AXIS_MIN,
			     TOUCHSCREEN_Y_AXIS_MAX, 0, 0);
	input_set_abs_params(g_p_ssd2531_data->input_dev,
			     ABS_MT_PRESSURE, TOUCHSCREEN_Z_AXIS_MIN,
			     TOUCHSCREEN_Z_AXIS_MAX, 0, 0);
	input_set_abs_params(g_p_ssd2531_data->input_dev,
			     ABS_MT_TRACKING_ID, TOUCHSCREEN_ID_MIN,
			     TOUCHSCREEN_ID_MAX, 0, 0);

	ret = input_register_device(g_p_ssd2531_data->input_dev);
	if (ret) {
		printk(KERN_ERR
		       "ssd2531 probe: Unable to register input device\n");
		ret = -EXDEV;
		goto err_register_dev;
	}
	g_p_ssd2531_data->work = kzalloc(sizeof(struct delayed_work),
					 GFP_KERNEL);
	if (!g_p_ssd2531_data->work) {
		ret = -ENOMEM;
		input_unregister_device(g_p_ssd2531_data->input_dev);
		goto err_register_dev;
	}

	INIT_DELAYED_WORK(g_p_ssd2531_data->work, ssd2531_timer_work_func);
	g_p_ssd2531_data->work->timer.function = ssd2531_timer_handler;
	g_p_ssd2531_data->work->timer.data = (long)g_p_ssd2531_data;

	/*register irq */
	g_p_ssd2531_data->client->irq =
	    IRQ_GPIO(mfp_to_gpio(g_p_ssd2531_data->mfp_gpio_pin_int));
	printk(KERN_NOTICE "received IRQ # %d\n",
	       g_p_ssd2531_data->client->irq);

	ret = request_threaded_irq(g_p_ssd2531_data->client->irq,
				   NULL,
				   ssd2531_ts_irq_handler,
				   flags, "ssd2531_touch", g_p_ssd2531_data);
	if (ret) {
		printk(KERN_ERR "ssd2531 probe: failed to register to IRQ\n");
		goto err_irq_failed;
	}
	ssd2531_ts_crl_all_events();

	ssd2531_create_proc_entry();

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&ssd2531_ts_early_suspend_desc);
#endif

	printk(KERN_NOTICE "--ssd2531_i2c_probe - exit OK\n");

	return 0;
err_irq_failed:
	cancel_delayed_work_sync(g_p_ssd2531_data->work);
	kfree(g_p_ssd2531_data->work);
	input_unregister_device(g_p_ssd2531_data->input_dev);
err_register_dev:
	input_free_device(g_p_ssd2531_data->input_dev);
	g_p_ssd2531_data->input_dev = NULL;
err_dev_alloc:
	/*ssd2531_deinit_ts();
	   release IRQ? */
err_no_dev:
	if (g_p_ssd2531_data != NULL)
		kfree(g_p_ssd2531_data);
	g_p_ssd2531_data = NULL;
	printk(KERN_NOTICE "--ssd2531_i2c_probe - exit ERROR\n");
	return ret;
}

static int __devexit ssd2531_i2c_remove(struct i2c_client *client)
{
	del_timer_sync(&(g_p_ssd2531_data->work->timer));
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ssd2531_ts_early_suspend_desc);
#endif
	cancel_delayed_work_sync(g_p_ssd2531_data->work);

	ssd2531_remove_proc_entry();
	free_irq(client->irq, g_p_ssd2531_data);
	input_unregister_device(g_p_ssd2531_data->input_dev);
	input_free_device(g_p_ssd2531_data->input_dev);
	kfree(g_p_ssd2531_data->work);
	kfree(g_p_ssd2531_data);
	g_p_ssd2531_data = NULL;
	return 0;
}

static const struct i2c_device_id SSD2531_i2c_id[] = {
	{"ssd2531_ts", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, SSD2531_i2c_id);

static struct i2c_driver SSD2531_i2c_driver = {
	.driver = {
		   .name = "ssd2531",
		   .owner = THIS_MODULE,
		   },
	.probe = ssd2531_i2c_probe,
	.remove = __devexit_p(ssd2531_i2c_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = ssd2531_ts_suspend,
	.resume = ssd2531_ts_resume,
#endif
	.id_table = SSD2531_i2c_id,
};

static int __init ssd2531_i2c_init(void)
{
	return i2c_add_driver(&SSD2531_i2c_driver);
}

module_init(ssd2531_i2c_init);

static void __exit ssd2531_i2c_exit(void)
{
	i2c_del_driver(&SSD2531_i2c_driver);
}

module_exit(ssd2531_i2c_exit);

MODULE_AUTHOR("Chen Reichbach<creichba@marvell.com>");
MODULE_DESCRIPTION("ssd2531 touch screen driver");
MODULE_LICENSE("GPL");
