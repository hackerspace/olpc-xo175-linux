/********************************************************************/
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
/********************************************************************/
/* 1. SSL_DEBUG: print debug information of this driver */
/* 2. CAL_THRESHOLD: auto-calibration threshold, it should be negative */
/* 3. CAL_INTERVAL: idle time between auto-calibration checking	*/
/* 4. CAL_COUNT: auto-calibration will check condition for CAL_COUNT seconds */
/* 5. TOUCH_INT_PIN and TOUCH_RES_PIN: GPIO for IRQ and RESET signal */
/* 6. HAS_BUTTON: if there is any button */
/* 7. SIMULATED_BUTTON: if buttons are simulated by touch area */
/* 8. FINGER_USED: define finger count in system */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/input/mt.h>
#include <linux/SSD2531_touch.h>
/****** output debug information ******/
/*#define SSL_DEBUG*/

#ifdef SSL_DEBUG
#define SSL_PRINT(x...)		pr_info("SSD253X:"x)
#else
#define SSL_PRINT(x...)
#endif

/****** auto-calibration conditions (only applied to SSD2531) ******/
/****** define auto-calibration threshold ******/
#define CAL_THRESHOLD -100
/****** define auto-calibration checking interval ******/
#define CAL_INTERVAL	200	/*in ms*/
/****** define auto-calibration checking times ******/
#define CAL_COUNT		5	/*heck 5 seconds after boot-up/sleep out*/
/****** define the number of finger that used to report coodinates ******/
#define FINGER_USED		10

/****** define your RESET and IRQ input here ******/
static int TOUCH_INT_PIN;
static int TOUCH_RES_PIN;

/****** define your button codes ******/
/*#define HAS_BUTTON*/
/*#define SIMULATED_BUTTON*/

#define BUTTON0	59	/*menu*/
#define BUTTON1	158	/*back*/
#define BUTTON2	217	/*search*/
#define BUTTON3	102	/*home*/

typedef struct {
	unsigned long x;	/*center x of button*/
	unsigned long y;	/*center y of button*/
	unsigned long dx;	/*x range (x-dx)~(x+dx)*/
	unsigned long dy;	/*y range (y-dy)~(y+dy)*/
	unsigned long code;	/*key code for simulated key*/
} SKey_Info, *pSKey_Info;

/****** define simulated button information ******/
#ifdef SIMULATED_BUTTON
SKey_Info SKeys[] = {
{45, 530, 20, 16, BUTTON0},
{160, 530, 20, 16, BUTTON3},
{275, 530, 20, 16, BUTTON1},
};
#endif


#define REG_DEVICE_ID		    0x02
#define FINGER_STATUS           0x79
#define FINGER_DATA             0x7C
#define BUTTON_RAW				0xB5
#define BUTTON_STATUS		0xB9

struct ssd2531_platform_data *touch_data;

enum CMD_TYPE {CMD_DELAY = -1, CMD_ONLY = 0, CMD_1B = 1, CMD_2B = 2};
typedef struct {
	enum CMD_TYPE type;
	unsigned int reg;
	unsigned char data1;
	unsigned char data2;
} Reg_Item, *pReg_Item;

/****** The following block is TP configuration, please get it from your TP vendor ******/
/****** TP configuration starts here ******/
#define LCD_RANGE_X 480
#define LCD_RANGE_Y 800
Reg_Item ssd2531_Init[] = {
	{CMD_ONLY, 0x23, 0x00},
	{CMD_DELAY, 10},
	{CMD_1B, 0x2B, 0x02},
	{CMD_DELAY, 10},
	{CMD_1B, 0xD4, 0x00},
	{CMD_1B, 0x06, 0x0F},
	{CMD_1B, 0x07, 0x06},
	{CMD_1B, 0x08, 0x00},
	{CMD_1B, 0x09, 0x01},
	{CMD_1B, 0x0A, 0x02},
	{CMD_1B, 0x0B, 0x03},
	{CMD_1B, 0x0C, 0x04},
	{CMD_1B, 0x0D, 0x05},
	{CMD_1B, 0x0E, 0x06},
	{CMD_1B, 0x0F, 0x07},
	{CMD_1B, 0x10, 0x08},
	{CMD_1B, 0x11, 0x09},
	{CMD_1B, 0x12, 0x0a},
	{CMD_1B, 0x13, 0x0b},
	{CMD_1B, 0x14, 0x0c},
	{CMD_1B, 0x15, 0x0d},
	{CMD_1B, 0x16, 0x0e},
	{CMD_1B, 0x17, 0x0f},
	{CMD_1B, 0x18, 0x10},
	{CMD_1B, 0x19, 0x11},
	{CMD_1B, 0x1A, 0x12},
	{CMD_1B, 0x1B, 0x13},
	{CMD_1B, 0x1C, 0x14},
	{CMD_1B, 0x27, 0x07},
	{CMD_1B, 0x28, 0x0F},
	{CMD_1B, 0x29, 0x03},
	{CMD_1B, 0x2A, 0x03},
	{CMD_1B, 0x2C, 0x02},
	{CMD_1B, 0x33, 0x01},
	{CMD_1B, 0x34, 0x50},
	{CMD_2B, 0x35, 0x00, 0x50},
	{CMD_1B, 0x36, 0x20},
	{CMD_1B, 0x37, 0x03},
	{CMD_1B, 0x38, 0x00},
	{CMD_1B, 0x39, 0x00},
	{CMD_1B, 0x3D, 0x01},
	{CMD_1B, 0x53, 0x08},
	{CMD_1B, 0x54, 0x30},
	{CMD_1B, 0x55, 0x30},
	{CMD_1B, 0x56, 0x01},
	{CMD_1B, 0x58, 0x01},
	{CMD_1B, 0x59, 0x01},
	{CMD_1B, 0x5A, 0x00},
	{CMD_1B, 0x5B, 0x08},
	{CMD_1B, 0x65, 0x02},
	{CMD_1B, 0x66, 0x57},
	{CMD_1B, 0x67, 0x50},
	{CMD_2B, 0x7A, 0xFF, 0xFF},
	{CMD_1B, 0x7B, 0xb0},
	{CMD_1B, 0xC1, 0x02},
	{CMD_1B, 0xD5, 0x0F},
	{CMD_1B, 0xD9, 0x01},
	{CMD_1B, 0x25, 0x02},
};

Reg_Item ssd2533_Init[] = {	/* for SSD2532 or SSD2533 */
	{CMD_ONLY, 0x04},
	{CMD_DELAY, 30},
	{CMD_1B, 0x06, 0x0C},
	{CMD_1B, 0x07, 0x08},
	{CMD_2B, 0x08, 0x00, 0x05},
	{CMD_2B, 0x09, 0x00, 0x06},
	{CMD_2B, 0x0A, 0x00, 0x07},
	{CMD_2B, 0x0B, 0x00, 0x08},
	{CMD_2B, 0x0C, 0x00, 0x09},
	{CMD_2B, 0x0D, 0x00, 0x0A},
	{CMD_2B, 0x0E, 0x00, 0x0B},
	{CMD_2B, 0x0F, 0x00, 0x0C},
	{CMD_2B, 0x10, 0x00, 0x0D},
	{CMD_2B, 0x11, 0x00, 0x0E},
	{CMD_2B, 0x12, 0x00, 0x0F},
	{CMD_2B, 0x13, 0x00, 0x10},
	{CMD_2B, 0x14, 0x00, 0x11},
	{CMD_2B, 0x15, 0x00, 0x12},
	{CMD_2B, 0x16, 0x00, 0x13},
	{CMD_2B, 0x17, 0x00, 0x14},
	{CMD_2B, 0x18, 0x00, 0x15},
	{CMD_2B, 0x19, 0x00, 0x16},
	{CMD_2B, 0x1A, 0x00, 0x00},
	{CMD_2B, 0x1B, 0x00, 0x01},
	{CMD_2B, 0x1C, 0x00, 0x02},
	{CMD_2B, 0x1D, 0x00, 0x03},
	{CMD_2B, 0x1E, 0x00, 0x04},
	{CMD_1B, 0x1F, 0x00},
	{CMD_1B, 0x28, 0x20},
	{CMD_1B, 0x2A, 0x07},
	{CMD_1B, 0x2C, 0x01},
	{CMD_1B, 0x2E, 0x0C},
	{CMD_1B, 0x2F, 0x01},
	{CMD_1B, 0x30, 0x01},
	{CMD_1B, 0x31, 0x07},
	{CMD_1B, 0xD5, 0x03},
	{CMD_1B, 0xD7, 0x04},
	{CMD_1B, 0xD8, 0x04},
	{CMD_1B, 0xDB, 0x02},
	{CMD_2B, 0x33, 0x00, 0x01},
	{CMD_2B, 0x34, 0x00, 0x60},
	{CMD_2B, 0x35, 0x00, 0x20},
	{CMD_2B, 0x36, 0x00, 0x20},
	{CMD_1B, 0x37, 0x02},
	{CMD_1B, 0x39, 0x02},
	{CMD_1B, 0x3A, 0x00},
	{CMD_1B, 0x3D, 0x01},
	{CMD_1B, 0x40, 0xA0},
	{CMD_1B, 0x44, 0x01},
	{CMD_1B, 0x53, 0x10},
	{CMD_2B, 0x54, 0x00, 0xC0},
	{CMD_2B, 0x55, 0x00, 0xC0},
	{CMD_1B, 0x56, 0x03},
	{CMD_1B, 0x58, 0x00},
	{CMD_1B, 0x59, 0x01},
	{CMD_1B, 0x5A, 0x00},
	{CMD_1B, 0x5B, 0x10},
	{CMD_1B, 0x65, 0x05},
	{CMD_2B, 0x66, 0xFF, 0xF0},
	{CMD_2B, 0x67, 0xFF, 0xF0},
	{CMD_2B, 0x7A, 0xFF, 0xFF},
	{CMD_2B, 0x7B, 0xC0, 0x0B},
	{CMD_1B, 0x8A, 0x0A},
	{CMD_1B, 0xAC, 0x00},
	{CMD_1B, 0xAB, 0x64},
	{CMD_1B, 0xAD, 0x02},
	{CMD_1B, 0xAE, 0x03},
	{CMD_1B, 0xAF, 0x50},
	{CMD_1B, 0xB0, 0x00},
	{CMD_1B, 0xBB, 0x05},
	{CMD_1B, 0xBC, 0x01},
	{CMD_1B, 0x25, 0x0A},
};
/****** TP configuration ends here ******/
/****** The above block is TP configuration, please get it from your TP vendor ******/

static int ssd253x_ts_open(struct input_dev *dev);
static void ssd253x_ts_close(struct input_dev *dev);
static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id);
static enum hrtimer_restart ssd253x_ts_timer(struct hrtimer *timer);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ssd253x_ts_early_suspend(struct early_suspend *h);
static void ssd253x_ts_late_resume(struct early_suspend *h);
#else
static int ssd253x_ts_suspend(struct i2c_client *client, pm_message_t mesg);
static int ssd253x_ts_resume(struct i2c_client *client);
#endif

static struct workqueue_struct *ssd253x_wq;

struct ssl_ts_priv {
	struct i2c_client *client;
	struct input_dev *input;
	struct hrtimer timer;
	struct work_struct  ssl_work;
	u32 fingers;
	u32 buttons;
	int cal_counter;
	int cal_tick;
	int irq;
	int prev_delta;
	int prev_x[10];
	int prev_y[10];
	int est_en[10];
	int est_x[10];
	int est_y[10];
	u8	keys[4];
	u8	keyindex[4];
	u32 fingerbits;
	unsigned long touchtime[10];
	unsigned long timemark;
	u32	tp_id;
	unsigned long cal_end_time;
	int finger_count;
	int working_mode;
	int drives;
	int senses;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	u8	r25h;
	u8	bsleep;
	u8	binitialrun;
	u8	bcalibrating;
	u8	rawout;
	u8      inited;
};

/****** for qualcomm platform only  *****************************
static unsigned ts_gpio_table[]={

	GPIO_CFG(TOUCH_RES_PIN,0,GPIO_OUTPUT,GPIO_PULL_UP,GPIO_2MA),
	GPIO_CFG(TOUCH_INT_PIN,0,GPIO_INPUT,GPIO_PULL_UP,GPIO_2MA),
};
****************************************************************/

#define PROCFS_NAME "driver/ssd253x-mode"
static struct proc_dir_entry *TP_PROC_FILE;
extern struct proc_dir_entry proc_root;

/* Read data from register
* return 0, if success
* return negative errno, if error occur */
static int ssd253x_read_reg(
struct i2c_client *client,
u8 reg_index, /* start reg_index */
u8 *rbuff, /* buffer to restore data be read */
u8 rbuff_len, /* the size of rbuff */
u8 length) /* how many bytes will be read */
{
	int ret = 0;
	u8 idx[2];
	struct i2c_msg msgs[2];
	/* send reg index */
	idx[0] = reg_index;
	idx[1] = 0x0;
	/* msg[0]: write reg command */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0; /* i2c write */
	msgs[0].len = 1;
	msgs[0].buf = idx;
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD; /* i2c read */
	msgs[1].len = length;
	msgs[1].buf = rbuff;
	if (length > rbuff_len)
		return -EINVAL;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0) {
		SSL_PRINT("Error in %s, reg is 0x%02X.\n", __func__, reg_index);
		return -EIO;
	}
	return 0;
}


/* write command & parameters
* return 0, if success
* return negative errno, if error occur */
static int ssd253x_write_cmd(
struct i2c_client *client,
u8 command,	/* start reg_index */
u8 *wbuff,		/* buffer to write */
u8 length)		/* how many byte will be write */
{
	int ret = 0;
	struct i2c_msg msg;
	u8 buf[128];
	buf[0] = command;

	if (length > 126)
		return -1;

	memcpy(&buf[1], wbuff, length);
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = length+1;
	msg.buf = buf;

	/* step1. send reg index */
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		SSL_PRINT("Error in %s, reg is 0x%02X. len=%d\n", __func__, command, (int)length);
		return -EIO;
	}

	return 0;
}

static int ssd253x_read_id(struct i2c_client *client)
{
	u8 buf[2];
	u16 ic_id;
	buf[0] = 0;
	ssd253x_write_cmd(client, 0x04, buf, 0);
	mdelay(10);
	ssd253x_read_reg(client, 0x02, &buf[0], 2, 2);
	ic_id = buf[0]<<8|buf[1];
	if (ic_id == 0x2533)
		return ic_id;
	buf[0] = 0x0;
	ssd253x_write_cmd(client, 0x23, buf, 0);
	mdelay(10);
	buf[0] = 0x02;
	ssd253x_write_cmd(client, 0x2b, buf, 1);
	mdelay(10);
	ssd253x_read_reg(client, 0x02, &buf[0], 2, 2);
	ic_id = buf[0]<<8|buf[1];
	if (ic_id == 0x2531)
		return ic_id;
	else
		return 0;

}


int deviceInit(struct ssl_ts_priv *ssl_priv)
{
	int err = 0;
	int index;
	pReg_Item pReg;
	int InitSize;
	u8 data_buf[2];
	struct i2c_client *client = ssl_priv->client;

	gpio_set_value(TOUCH_RES_PIN, 0);
	mdelay(10);
	gpio_set_value(TOUCH_RES_PIN, 1);
	mdelay(10);
	if (ssl_priv->tp_id == 0x2533) {
		pReg = ssd2533_Init;
		InitSize = sizeof(ssd2533_Init) / sizeof(Reg_Item);
	} else if (ssl_priv->tp_id == 0x2531) {
		pReg = ssd2531_Init;
		InitSize = sizeof(ssd2531_Init) / sizeof(Reg_Item);
	} else
		return -1;

	for (index = 0; index < InitSize; index++) {
		switch (pReg->type) {
		case CMD_DELAY:
			mdelay(pReg->reg);
			break;
		default:
			{
				if (pReg->reg == 0x25)
					ssl_priv->r25h = pReg->data1;
				else if (pReg->reg == 0x06) {
					if (ssl_priv->tp_id == 0x2533)
						ssl_priv->drives = pReg->data1 + 1;
					else
						ssl_priv->drives = pReg->data1 + 6;
				} else if (pReg->reg == 0x07) {
					if (ssl_priv->tp_id == 0x2533)
						ssl_priv->senses = pReg->data1+1;
					else
						ssl_priv->senses = pReg->data1+6;
				}
				data_buf[0] = pReg->data1;
				data_buf[1] = pReg->data2;
				if (pReg->reg != 0x25) {
					err = ssd253x_write_cmd(client, pReg->reg, &data_buf[0], (int)(pReg->type));
					/*mdelay(1);*/
				}
				break;
			}
		}
		pReg++;
	}
	ssl_priv->inited = 1;
	return 0;
}

void StartWork(struct ssl_ts_priv *ssl_priv)
{
	u8 buf[2];
	buf[0] = ssl_priv->r25h;
	ssd253x_write_cmd(ssl_priv->client, 0x25, buf, 1);
	ssl_priv->cal_tick = 1-(500/CAL_INTERVAL);
	ssl_priv->fingers = 0;
	ssl_priv->buttons = 0;
	ssl_priv->fingerbits = 0;
	if (ssl_priv->tp_id == 0x2531)
		ssl_priv->binitialrun = 1;
	ssl_priv->cal_counter = 0;
	ssl_priv->cal_end_time = jiffies + HZ*CAL_COUNT;
	hrtimer_start(&ssl_priv->timer, ktime_set(0, 300 * 1000000), HRTIMER_MODE_REL); /*check status after 300ms*/
	ssl_priv->bsleep = 0;
}

static int ssd253x_Proc_Read(char *buffer, char **buffer_localation, off_t offset, int buffer_length, int * eof, void *data)
{
	struct ssl_ts_priv *ssl_priv = (struct ssl_ts_priv *)data;
	buffer[0] = ssl_priv->working_mode + '0';
	buffer[1] = '\n';
	buffer[2] = '\0';
	return 2;
}
static void touch_power(int en)
{
	if (en) {
			if (touch_data->set_power)
				touch_data->set_power(1);
	} else {
			if (touch_data->set_power)
				touch_data->set_power(0);
	}
}
void deviceWakeUp(struct ssl_ts_priv *ssl_priv)
{
	deviceInit(ssl_priv);
	StartWork(ssl_priv);
}

void deviceSleep(struct ssl_ts_priv *ssl_priv)
{
	ssl_priv->bsleep = 1;
	hrtimer_cancel(&ssl_priv->timer);
	mdelay(30);
	gpio_set_value(TOUCH_RES_PIN, 0);
	mdelay(10);
	gpio_set_value(TOUCH_RES_PIN, 1);

	ssl_priv->inited = 0;

	touch_power(0);
}
static int ssd253x_Proc_Write(struct file *filp, const char *buffer, unsigned long count, void *data)
{
	char lbuf[4] = {0, 0, 0, 0};
	char cmdbuf[256];
	int reg, len, value;
	struct ssl_ts_priv *ssl_priv = (struct ssl_ts_priv *)data;
	if (count == 0)
		return -EFAULT;
	if (copy_from_user(lbuf, buffer, 1))
		return -EFAULT;
	lbuf[0] -= '0';
	switch (lbuf[0]) {
	case 0:
		printk(KERN_INFO "SSD253X: Switch to normal mode.\n");
		break;
	case 1:
		printk(KERN_INFO "SSD253X: Switch to raw data mode.\n");
		break;
	case 2:
		printk(KERN_INFO "SSD253X: Switch to delta data mode.\n");
		break;
	case 3:
		printk(KERN_INFO "SSD253X: Sleep In.\n");
		deviceSleep(ssl_priv);
		break;
	case 4:
		printk(KERN_INFO "SSD253X: Sleep Out.\n");
		deviceWakeUp(ssl_priv);
		break;
	case 5:
		if (count >= sizeof(cmdbuf))
			len = 254;
		else
			len = count - 2;
		if (len <= 0)
			break;
		if (!copy_from_user(cmdbuf, &buffer[2], len)) {
			sscanf(cmdbuf, "%x,%x,%x", &reg, &len, &value);
			printk(KERN_INFO "SSD253X: Set R%02Xh=0x%X.\n", reg, value);
			if (len == 1) {
				cmdbuf[0] = value&0xFF;
			} else if (len == 2) {
				cmdbuf[0] = (value >> 8) & 0xFF;
				cmdbuf[1] = value & 0xFF;
			} else
				len = 0;
			ssd253x_write_cmd(ssl_priv->client, reg&0xFF, cmdbuf, len);
		}
		break;
	case 6:
		ssl_priv->rawout = 1;
		break;
	case 7:
		ssl_priv->rawout = 0;
		break;
	case 8:
		if (count >= sizeof(cmdbuf))
			len = 254;
		else
			len = count - 2;
		if (len <= 0)
			break;
		if (!copy_from_user(cmdbuf, &buffer[2], len)) {
			sscanf(cmdbuf, "%x,%x", &reg, &len);
			ssd253x_read_reg(ssl_priv->client, reg & 0xFF, cmdbuf, len, len);
			if (len == 1) {
				printk(KERN_INFO "SSD253X: R%02Xh=0x%02X.\n", reg, cmdbuf[0]);
			} else if (len == 2) {
				printk(KERN_INFO "SSD253X: R%02Xh=0x%02X,0x%02X.\n",
					reg, cmdbuf[0], cmdbuf[1]);
			} else if (len == 4) {
				printk(KERN_INFO "SSD253X: R%02Xh=0x%02X,0x%02X,0x%02X,0x%02X.\n",
					reg, cmdbuf[0], cmdbuf[1], cmdbuf[2], cmdbuf[3]);
			} else if (len == 5) {
				printk(KERN_INFO "SSD253X: R%02Xh=0x%02X,0x%02X,0x%02X,0x%02X,0x%02X.\n",
					reg, cmdbuf[0], cmdbuf[1], cmdbuf[2], cmdbuf[3], cmdbuf[4]);
			}
		}
		break;
	}
	if ((lbuf[0] >= 0) && (lbuf[0] <= 2)) {
		if (ssl_priv->working_mode != lbuf[0]) {
			ssl_priv->working_mode = lbuf[0];
			hrtimer_start(&ssl_priv->timer, ktime_set(0,
					1000000000), HRTIMER_MODE_REL);
			/*check again after 1s */
		}
	}
	return count;
}

static void ssd253x_raw(struct ssl_ts_priv *ssl_priv)
{
	long max_raw = 0, min_raw = 4095, avg_raw = 0, temp;
	int i, j;
	u8 reg_buff[4] = {0};
	volatile unsigned long time0 = jiffies;
#if (defined(HAS_BUTTON) && !defined(SIMULATED_BUTTON))
	printk(KERN_INFO "Buttons: ");
	for (i = 0; i < 4; i++) {
		ssd253x_read_reg(ssl_priv->client, BUTTON_RAW + i, &reg_buff[0], 2, 2);
		temp = reg_buff[1] + (reg_buff[0] << 8);
		printk("[%d]=%ld ", i, temp);
	}
	printk("\n");
#endif
	if (ssl_priv->tp_id == 0x2533) {
		reg_buff[0] = 0;
		reg_buff[1] = 1;
		ssd253x_write_cmd(ssl_priv->client, 0x93, reg_buff, 2);
	} else {
		reg_buff[0] = 0;
		ssd253x_write_cmd(ssl_priv->client, 0x93, reg_buff, 1);
	}
	mdelay(30);
	reg_buff[0] = 0;
	ssd253x_write_cmd(ssl_priv->client, 0x8E, reg_buff, 1);
	reg_buff[0] = 0;
	ssd253x_write_cmd(ssl_priv->client, 0x8F, reg_buff, 1);
	reg_buff[0] = 0;
	ssd253x_write_cmd(ssl_priv->client, 0x90, reg_buff, 1);
	mdelay(10);
	for (i = 0; i < ssl_priv->drives; i++) {
		for (j = 0; j < ssl_priv->senses; j++) {
			ssd253x_read_reg(ssl_priv->client, 0x92, &reg_buff[0], 2, 2);
			temp = reg_buff[1] + ((reg_buff[0] & 0x0F) << 8);
			if (ssl_priv->rawout)
				printk(KERN_INFO "%4ld ", temp);
			avg_raw += temp;
			if (max_raw < temp)
				max_raw = temp;
			if (min_raw > temp)
				min_raw = temp;
		}
		if (ssl_priv->rawout)
			printk("\n");
	}
	avg_raw /= ssl_priv->drives * ssl_priv->senses;
	printk(KERN_INFO "Raw (read in %u ms): max=%ld, min=%ld, avg=%ld\n",
		jiffies_to_msecs(jiffies-time0), max_raw, min_raw, avg_raw);
}

static int ssd253x_delta(struct ssl_ts_priv *ssl_priv, int bout)
{
	int max_delta = -256, min_delta = 4095, temp;
	int i, j;
	u8 reg_buff[4] = {0};
	volatile unsigned long time0 = jiffies;
	if (ssl_priv->tp_id == 0x2533) {
		reg_buff[0] = 0;
		reg_buff[1] = 5;
		ssd253x_write_cmd(ssl_priv->client, 0x93, reg_buff, 2);
		mdelay(30);
	} else {
		reg_buff[0] = 1;
		ssd253x_write_cmd(ssl_priv->client, 0x8D, reg_buff, 1);
		}
	reg_buff[0] = 0;
	ssd253x_write_cmd(ssl_priv->client, 0x8E, reg_buff, 1);
	reg_buff[0] = 0;
	ssd253x_write_cmd(ssl_priv->client, 0x8F, reg_buff, 1);
	reg_buff[0] = 0;
	ssd253x_write_cmd(ssl_priv->client, 0x90, reg_buff, 1);
	for (i = 0; i < ssl_priv->drives; i++) {
		for (j = 0; j < ssl_priv->senses; j++) {
			ssd253x_read_reg(ssl_priv->client, 0x92, &reg_buff[0], 2, 2);
			if (ssl_priv->tp_id == 0x2533) {
				temp = reg_buff[1] + ((reg_buff[0] & 0x03) << 8);
				if (temp > 511)
					temp -= 1024;
			} else {
				temp = reg_buff[1]+((reg_buff[0]&0x01)<<8);
				if (temp > 255)
					temp -= 512;
			}
			if (ssl_priv->rawout)
				printk(KERN_INFO "%4d ", temp);
			if (max_delta < temp)
				max_delta = temp;
			if (min_delta > temp)
				min_delta = temp;
		}
		if (ssl_priv->rawout)
			printk("\n");
	}
	if (bout)
		printk(KERN_INFO "Delta (read in %u ms): max=%d, min=%d\n",
			jiffies_to_msecs(jiffies-time0), max_delta, min_delta);
	if (ssl_priv->tp_id == 0x2531) {
		reg_buff[0] = 0;
		ssd253x_write_cmd(ssl_priv->client, 0x8D, reg_buff, 1);
	}
	if (min_delta < CAL_THRESHOLD)
		return 1;
	else
		return 0;
}

static void ssd253x_finger_down(struct input_dev *input, int id, int px, int py, int pressure)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	input_mt_slot(input, id);
	input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
	input_report_abs(input, ABS_MT_POSITION_X, px);
	input_report_abs(input, ABS_MT_POSITION_Y, py);
	input_report_abs(input, ABS_MT_PRESSURE, pressure);
#else
	input_report_abs(input, ABS_MT_TRACKING_ID, id);
	input_report_abs(input, ABS_MT_POSITION_X, px);
	input_report_abs(input, ABS_MT_POSITION_Y, py);
	input_report_abs(input, ABS_MT_PRESSURE, pressure);
	input_mt_sync(input);
#endif
}

static void ssd253x_finger_up(struct input_dev *input, int id)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	input_mt_slot(input, id);
	input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
#else
	input_report_abs(input, ABS_MT_PRESSURE, 0);
	input_report_abs(input, ABS_MT_TRACKING_ID, id);
	input_mt_sync(input);
#endif
}

#define X_COOR (reg_buff[0] + (((u16)reg_buff[2] & 0xF0) << 4))
#define Y_COOR (reg_buff[1] + (((u16)reg_buff[2] & 0x0F) << 8))
#define PRESS_PRESSURE (1+(reg_buff[3] >> 4) & 0x0F)
#define PRESS_SPEED  (reg_buff[3]&0x0F)
#define SPEED_EST_TH	3
#define SPEED_EST_RATE	3
#define DEFORMATION_DETECT	2	/* deformation detection interval */
#define LCD_EDGE_DEVIATION	8
static void ssd253x_ts_work(struct work_struct *work)
{
	static int wmode;
	static int oldx[10], oldy[10];
	int ret = 0;
	int i, smode;
#if (defined(HAS_BUTTON) && defined(SIMULATED_BUTTON))
	int j;
#endif
	u8 reg_buff[4] = { 0 };
	u32 finger_flag = 0;
	u32 old_flag = 0;
	u32 button_flag = 0;
	u32 bitmask;
	int px, py;
	struct ssl_ts_priv *ssl_priv = container_of(work, struct ssl_ts_priv, ssl_work);
	SSL_PRINT("Run into %s.\n", __func__);
	wmode = 0;
	if (ssl_priv->bsleep)
		return;
	smode = ssl_priv->working_mode;
	if (wmode != smode) {
		if (ssl_priv->tp_id == 0x2533) {
			if (smode == 0) {
				reg_buff[0] = 0;
				ssd253x_write_cmd(ssl_priv->client, 0x8D, reg_buff, 1);
				mdelay(50);
				reg_buff[0] = ssl_priv->r25h;
				ssd253x_write_cmd(ssl_priv->client, 0x25, reg_buff, 1);
				mdelay(300);
			} else {
				reg_buff[0] = 0;
				ssd253x_write_cmd(ssl_priv->client, 0x25, reg_buff, 1);
				mdelay(100);
				reg_buff[0] = 1;
				ssd253x_write_cmd(ssl_priv->client, 0x8D, reg_buff, 1);
				mdelay(50);
				reg_buff[0] = 1;
				ssd253x_write_cmd(ssl_priv->client, 0xC2, reg_buff, 1);
				reg_buff[0] = 3;
				ssd253x_write_cmd(ssl_priv->client, 0xC3, reg_buff, 1);
			}
		} else {
			if (smode == 1)/*raw*/
				reg_buff[0] = 1;
			else
				reg_buff[0] = 0;
			ssd253x_write_cmd(ssl_priv->client, 0x8D, reg_buff, 1);
			mdelay(50);
		}
		wmode = smode;
	}
	if (smode != 0) {
		if (smode == 1)	/*raw*/
			ssd253x_raw(ssl_priv);
		else	/*delta */
			ssd253x_delta(ssl_priv, 1);
		hrtimer_start(&ssl_priv->timer, ktime_set(0, 100000000), HRTIMER_MODE_REL);
		return;
	}

	if (ssl_priv->tp_id == 0x2533) {
		ret = ssd253x_read_reg(ssl_priv->client, FINGER_STATUS, &reg_buff[0], 2, 2);
		finger_flag = reg_buff[0]<<8;
		finger_flag += reg_buff[1];
		if (ssl_priv->rawout)
			printk(KERN_INFO "R79h:0x%04X.\n", finger_flag);
		SSL_PRINT("R79h:0x%04X.\n", finger_flag);
	} else if (ssl_priv->tp_id == 0x2531) {
		ret = ssd253x_read_reg(ssl_priv->client, FINGER_STATUS, &reg_buff[0], 1, 1);
		finger_flag = ((reg_buff[0] & 0xF0) >> 4) | ((reg_buff[0] & 0x0F) << 4);
		if (ssl_priv->rawout)
			printk(KERN_INFO "R79h:0x%02X.\n", finger_flag);
		SSL_PRINT("R79h:0x%02X.\n", finger_flag);
	} else {
		SSL_PRINT("Not IRQ from SSL CTP device!\n");
		input_sync(ssl_priv->input);
		}
	if (ret) {
		finger_flag = 0;
		/*I2C Device fail*/
	} else {
		for (i = 0; i < ssl_priv->finger_count; i++) {
			bitmask = 0x10 << i;
			if (finger_flag & bitmask) { /*finger exists*/
				ret = ssd253x_read_reg(ssl_priv->client, FINGER_DATA + i, reg_buff, 4, 4);
				if (ret) {
					/*I2C device fail*/
				} else {
					px = X_COOR;
					py = Y_COOR;
					if ((px != 0xFFF) && (py != 0xFFF)) {
						ssl_priv->prev_x[i] = px;
						ssl_priv->prev_y[i] = py;
					}
					if (finger_flag&0x04)	/*don't report finger if there is large object*/
						continue;
					SSL_PRINT("SSD253X: Finger%d at x=%d, y=%d.\n", i, ssl_priv->prev_x[i], ssl_priv->prev_y[i]);
#if (defined(HAS_BUTTON) && defined(SIMULATED_BUTTON))
					for (j = 0; j < sizeof(SKeys)/sizeof(SKey_Info); j++) {
						if (ssl_priv->prev_x[i] > (SKeys[j].x - SKeys[j].dx) &&
						   ssl_priv->prev_x[i] < (SKeys[j].x + SKeys[j].dx) &&
						   ssl_priv->prev_y[i] > (SKeys[j].y - SKeys[j].dy) &&
						   ssl_priv->prev_y[i] < (SKeys[j].y + SKeys[j].dy)) {	/*simulated button down*/
							SSL_PRINT("Finger%d at:(%d,%d), report as button%d down\n",
									i, ssl_priv->prev_x[i], ssl_priv->prev_y[i], j);
							if (i < FINGER_USED)
								input_report_key(ssl_priv->input, ssl_priv->keys[j], 1);
							ssl_priv->buttons |= (1 << j);
							ssl_priv->keyindex[j] = i;
							break;
						} else {	/*finger is not in button area*/
							if ((ssl_priv->buttons) & (1 << j)) {/*but the button is down previously*/
								if (ssl_priv->keyindex[j] == i) {
									/*finger is the one caused button down and now is out of button area, report as button up*/
									SSL_PRINT("Finger%d at:(%d,%d), moves out of button area, report as button%d up\n",
											i, ssl_priv->prev_x[i], ssl_priv->prev_y[i], j);
									if (i < FINGER_USED)
										input_report_key(ssl_priv->input, ssl_priv->keys[j], 0);
									ssl_priv->buttons &= ~(1<<j);
								}
							}
						}
					}
					if (ssl_priv->prev_x[i] >= (LCD_EDGE_DEVIATION+LCD_RANGE_X) ||
						ssl_priv->prev_y[i] >= (LCD_EDGE_DEVIATION+LCD_RANGE_Y)) {
						/*finger out of LCD range, regard as leave*/
						if (ssl_priv->fingerbits & (1 << i)) {
							SSL_PRINT("Finger%d at:(%d,%d), out of LCD range, report as finger leave\n",
									i, ssl_priv->prev_x[i], ssl_priv->prev_y[i]);
							if (i < FINGER_USED)
								ssd253x_finger_up(ssl_priv->input,	i);
							ssl_priv->fingerbits &= ~(1 << i);
						}
						continue;
					}
#endif
					if (ssl_priv->prev_x[i] >= LCD_RANGE_X)
						ssl_priv->prev_x[i] = LCD_RANGE_X - 1;
					if (ssl_priv->prev_y[i] >= LCD_RANGE_Y)
						ssl_priv->prev_y[i] = LCD_RANGE_Y - 1;
					if (ssl_priv->prev_x[i] == 0)
						ssl_priv->prev_x[i] = 1;
					if (ssl_priv->prev_y[i] == 0)
						ssl_priv->prev_y[i] = 1;
					if (ssl_priv->fingerbits & (1 << i)) {
						SSL_PRINT("Finger%d move:(%d,%d)\n", i, ssl_priv->prev_x[i], ssl_priv->prev_y[i]);
						px = abs(ssl_priv->prev_x[i] - oldx[i]) + abs(ssl_priv->prev_y[i] - oldy[i]);
						py = jiffies_to_msecs(jiffies - ssl_priv->touchtime[i]);
						if (py == 0)
							py = 10;
						px /= py;
						if (px > SPEED_EST_TH) {/*trigger moving estimation*/
							ssl_priv->est_x[i] = ssl_priv->prev_x[i] + SPEED_EST_RATE * (ssl_priv->prev_x[i] - oldx[i]);
							if (ssl_priv->est_x[i] >= LCD_RANGE_X)
								ssl_priv->est_x[i] = LCD_RANGE_X - 1;
							else if (ssl_priv->est_x[i] < 0)
								 ssl_priv->est_x[i] = 0;
							if (ssl_priv->prev_x[i] == oldx[i])
								ssl_priv->est_y[i] = ssl_priv->prev_y[i] + SPEED_EST_RATE * (ssl_priv->prev_y[i]-oldy[i]);
							else
								ssl_priv->est_y[i] = ssl_priv->prev_y[i] + (ssl_priv->est_x[i] - ssl_priv->prev_x[i])
											* (ssl_priv->prev_y[i]-oldy[i]) / (ssl_priv->prev_x[i]-oldx[i]);
							if (ssl_priv->est_y[i] >= LCD_RANGE_Y)
								ssl_priv->est_y[i] = LCD_RANGE_Y - 1;
							else if (ssl_priv->est_y[i] < 0)
								ssl_priv->est_y[i] = 0;
							if (ssl_priv->prev_y[i] == oldy[i])
								ssl_priv->est_x[i] = ssl_priv->prev_x[i] + SPEED_EST_RATE * (ssl_priv->prev_x[i] - oldx[i]);
							else
								ssl_priv->est_x[i] = ssl_priv->prev_x[i]+(ssl_priv->est_y[i]-ssl_priv->prev_y[i])
											* (ssl_priv->prev_x[i] - oldx[i]) / (ssl_priv->prev_y[i] - oldy[i]);
							ssl_priv->est_en[i] = 1;
						} else {
								ssl_priv->est_en[i] = 0;
							}
						/*finger[i] move*/
					} else {
						SSL_PRINT("Finger%d enter:(%d,%d)\n", i, ssl_priv->prev_x[i], ssl_priv->prev_y[i]);
						ssl_priv->fingerbits |= (1 << i);
						/*finger[i] enter*/
						}
					/*report finger[i] coordinate here*/
					ssl_priv->touchtime[i] = jiffies;
					oldx[i] = ssl_priv->prev_x[i];
					oldy[i] = ssl_priv->prev_y[i];
					if (i < FINGER_USED)
						ssd253x_finger_down(ssl_priv->input, i, ssl_priv->prev_x[i],
									ssl_priv->prev_y[i], PRESS_PRESSURE);

				}
			} else {
				if (ssl_priv->fingers & bitmask) {
					/*finger[i] leave*/
#if (defined(HAS_BUTTON) && defined(SIMULATED_BUTTON))
					if (ssl_priv->prev_x[i] >= LCD_RANGE_X || ssl_priv->prev_y[i] >= LCD_RANGE_Y) {
						for (j = 0; j < sizeof(SKeys)/sizeof(SKey_Info); j++) {
							if (ssl_priv->prev_x[i] > (SKeys[j].x-SKeys[j].dx) &&
							   ssl_priv->prev_x[i] < (SKeys[j].x+SKeys[j].dx) &&
							   ssl_priv->prev_y[i] > (SKeys[j].y-SKeys[j].dy) &&
							   ssl_priv->prev_y[i] < (SKeys[j].y+SKeys[j].dy)) {/*simulated button up*/
								SSL_PRINT("Finger%d leave at:(%d,%d), report as button%d up\n",
									i, ssl_priv->prev_x[i], ssl_priv->prev_y[i], j);
								if (i < FINGER_USED)
									input_report_key(ssl_priv->input, ssl_priv->keys[j], 0);
								ssl_priv->buttons &= ~(1<<j);
								break;
							}
						}
						continue;
					}
#endif
					if (ssl_priv->est_en[i]) {
						finger_flag |= bitmask;
						ssl_priv->est_en[i] = 0;
						if (i < FINGER_USED) {
							SSL_PRINT("SSD253X: real leave coordinate (%d,%d), estimated leave coordinate (%d,%d).\n",\
									ssl_priv->prev_x[i], ssl_priv->prev_y[i], ssl_priv->est_x[i],
									ssl_priv->est_y[i]);
							ssl_priv->prev_x[i] = ssl_priv->est_x[i];
							ssl_priv->prev_y[i] = ssl_priv->est_y[i];
							ssd253x_finger_down(ssl_priv->input, i,
										ssl_priv->prev_x[i], ssl_priv->prev_y[i], 1);
						}
					} else {
						SSL_PRINT("Finger%d leave:(%d,%d)\n", i, ssl_priv->prev_x[i], ssl_priv->prev_y[i]);
						if (i < FINGER_USED)
							ssd253x_finger_up(ssl_priv->input, i);
						ssl_priv->fingerbits &= ~(1 << i);
					}
				}
			}
		}
		old_flag = ssl_priv->fingers;
		ssl_priv->fingers = finger_flag;
	}
#if (defined(HAS_BUTTON) && !defined(SIMULATED_BUTTON))
	ret = ssd253x_read_reg(ssl_priv->client, BUTTON_STATUS, &reg_buff[0], 1, 1);
	if (ret) {
		/*I2C Device fail*/
		button_flag = 0;
	} else {
		button_flag = reg_buff[0];
		SSL_PRINT("RB9h:0x%02X.\n", button_flag);
		for (i = 0; i < 4; i++) {
			bitmask = 1 << i;
			if (button_flag & bitmask) { /*button[i] on*/
				if (ssl_priv->buttons & bitmask) {	/*button[i] still on*/
					/*repeat button[i]*/
					SSL_PRINT("Button%d repeat.\n", i);
				} else {
						/*button[i] down*/
						SSL_PRINT("Button%d down.\n", i);
						input_report_key(ssl_priv->input, ssl_priv->keys[i], 1);
					}
				} else if (ssl_priv->buttons & bitmask) {
							/*button[i] up*/
							SSL_PRINT("Button%d up.\n", i);
							input_report_key(ssl_priv->input, ssl_priv->keys[i], 0);
						}
		}
		ssl_priv->buttons = button_flag;
	}
#endif
	input_sync(ssl_priv->input);
	if (finger_flag) {
		if (old_flag == 0) {
			if (ssl_priv->binitialrun == 1) {/*the first time finger on after sleep out*/
				ssl_priv->binitialrun = 2;
				ssl_priv->timemark = jiffies - (HZ * DEFORMATION_DETECT) - 1;
			} else
				ssl_priv->timemark = jiffies;

		}
		if ((ssl_priv->tp_id == 0x2531) && (time_after(jiffies, ssl_priv->timemark
			 + (HZ*DEFORMATION_DETECT))) && (ssl_priv->prev_delta ||
				time_before(jiffies, ssl_priv->cal_end_time))) {

			ssl_priv->timemark = jiffies;
			ssl_priv->bcalibrating = 1;
			ssl_priv->cal_counter++;
			ssl_priv->prev_delta = ssd253x_delta(ssl_priv, 0);
			if (ssl_priv->prev_delta) {	/*oops, there is abnormal finger hole*/
				printk(KERN_INFO "SSD253X: Lens deformation detected, now calibrate!\n");
				reg_buff[0] = 0x01;
				ssd253x_write_cmd(ssl_priv->client, 0xA2, reg_buff, 1);
				ssl_priv->cal_tick = -1;
				hrtimer_start(&ssl_priv->timer, ktime_set(0, 300*1000000),
						HRTIMER_MODE_REL); /*check again after 300ms*/
				return;
			} else
				ssl_priv->bcalibrating = 0;
		}
	}
	if (button_flag == 0 && finger_flag == 0) {
	/*no finger or button on, so no need to monitor status*/
		SSL_PRINT("No finger or button on.\n");
		if (ssl_priv->cal_tick >= 1) {
			ssl_priv->cal_tick = 0;
			ret = ssd253x_read_reg(ssl_priv->client,
				0x26, reg_buff, 1, 1);
				if ((ret != 0) || (reg_buff[0] == 0) ||
				(ssl_priv->tp_id == 0)) {/*oops, the TP was reset, need re-init*/
				printk(KERN_INFO "SSD253X: TP was reset, now restart it!\n");
				deviceWakeUp(ssl_priv);
				return;
		} else {
				if (ssl_priv->binitialrun) {
					if (ssl_priv->prev_delta || time_before
						(jiffies, ssl_priv->cal_end_time)) {/*only check when bootup/sleepout*/
						if (ssl_priv->tp_id == 0x2531) {	/*only SSD2531 needs auto-calibration*/
							ssl_priv->cal_counter++;
							ssl_priv->bcalibrating = 1;
							ssl_priv->prev_delta = ssd253x_delta(ssl_priv, 0);
							if (ssl_priv->prev_delta) {
							/*oops, there is abnormal finger hole*/
								printk(KERN_INFO "SSD253X: Finger hole detected, now calibrate!\n");
								reg_buff[0] = 0x01;
								ssd253x_write_cmd(ssl_priv->client, 0xA2, reg_buff, 1);
								ssl_priv->cal_tick = -1;
								SSL_PRINT("Calibrate! ssl_priv->cal_counter=%d.\n",
								ssl_priv->cal_counter);
							} else
								ssl_priv->bcalibrating = 0;
					}
				} else {
						printk(KERN_INFO "SSD253X: Totally checked finger hole for %d times.\n",
							ssl_priv->cal_counter);
						ssl_priv->binitialrun = 0;
					}
				}
			}
		}
		ssl_priv->cal_tick++;
		if (ssl_priv->bcalibrating == 1) {
			hrtimer_start(&ssl_priv->timer, ktime_set(0, 300 * 1000000),
					HRTIMER_MODE_REL); /*check again after 300ms*/
			return;
		}
		if (ssl_priv->binitialrun == 1)
			hrtimer_start(&ssl_priv->timer,
					ktime_set(0, CAL_INTERVAL * 1000000),
					HRTIMER_MODE_REL); /*check again after 30ms*/
		else if (ssl_priv->binitialrun == 2)
			hrtimer_start(&ssl_priv->timer,
					ktime_set(0, 500000000),
					HRTIMER_MODE_REL); /*check again after 500ms*/
	} else {
		ssl_priv->cal_tick = 0;
		hrtimer_start(&ssl_priv->timer,
				ktime_set(0, 16600000),
				HRTIMER_MODE_REL); /*polling for finger up after 16.6ms.*/
	}
}


static int ssd253x_ts_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
	struct ssl_ts_priv *ssl_priv;
	struct input_dev *ssl_input;
	int error;
	int *mfp_pins;
#if (defined(HAS_BUTTON) && defined(SIMULATED_BUTTON))
	int i;
#endif

	SSL_PRINT("Run into %s.\n", __func__);
/****** only for qualcomm platform
	gpio_tlmm_config(ts_gpio_table[0],GPIO_ENABLE);
	gpio_tlmm_config(ts_gpio_table[1],GPIO_ENABLE);
******/
	touch_data = ((struct ssd2531_platform_data *)(client->dev.platform_data));
	mfp_pins = touch_data->pin_data;
	TOUCH_RES_PIN = mfp_pins[1];
	TOUCH_INT_PIN = mfp_pins[0];
	touch_power(1);
	if (gpio_request(TOUCH_RES_PIN, "TOUCH_RES"))
		return -ENODEV;

	gpio_direction_output(TOUCH_RES_PIN, 1);
	mdelay(10);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		error = -ENODEV;
		goto	err0;
	}
	ssl_priv = kzalloc(sizeof(*ssl_priv), GFP_KERNEL);
	if (!ssl_priv) {
		error = -ENODEV;
		goto	err0;
	}
	dev_set_drvdata(&client->dev, ssl_priv);
	gpio_set_value(TOUCH_RES_PIN, 0);
	mdelay(10);
	gpio_set_value(TOUCH_RES_PIN, 1);
	mdelay(10);
	ssl_priv->tp_id = ssd253x_read_id(client);
	if (ssl_priv->tp_id == 0x2533) {
		printk(KERN_INFO "Found SSL CTP device(SSD2533) at address 0x%02X.\n",
			client->addr);
		ssl_priv->finger_count = 10;
	} else if (ssl_priv->tp_id == 0x2531) {
		printk(KERN_INFO "Found SSL CTP device(SSD2531) at address 0x%02X.\n", client->addr);
		ssl_priv->finger_count = 4;
	} else {
		ssl_priv->tp_id = 0;
		printk(KERN_INFO "No SSL CTP device found at address 0x%02X.\n", client->addr);
		goto	err1;
	}

	ssl_priv->client = client;
	deviceInit(ssl_priv);

	ssl_input = input_allocate_device();
	if (!ssl_input) {
		error = -ENODEV;
		goto	err1;
	}
	set_bit(EV_KEY, ssl_input->evbit);
	set_bit(EV_ABS, ssl_input->evbit);
	set_bit(EV_SYN, ssl_input->evbit);
#ifdef HAS_BUTTON
#ifdef SIMULATED_BUTTON
	for (i = 0; i < sizeof(SKeys)/sizeof(SKey_Info); i++) {
		ssl_priv->keys[i] = SKeys[i].code;
		input_set_capability(ssl_input, EV_KEY, ssl_priv->keys[i]);
	}
#else
	ssl_priv->keys[0] = BUTTON0;
	ssl_priv->keys[1] = BUTTON1;
	ssl_priv->keys[2] = BUTTON2;
	ssl_priv->keys[3] = BUTTON3;
	for (i = 0; i < 4; i++)
		input_set_capability(ssl_input, EV_KEY, ssl_priv->keys[i]);

#endif
#endif
	set_bit(ABS_MT_POSITION_X, ssl_input->absbit);
	set_bit(ABS_MT_POSITION_Y, ssl_input->absbit);
	set_bit(ABS_MT_PRESSURE, ssl_input->absbit);
	input_set_abs_params(ssl_input, ABS_MT_PRESSURE, 0, 16, 0, 0);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	set_bit(MT_TOOL_FINGER, ssl_input->keybit);
	set_bit(INPUT_PROP_DIRECT, ssl_input->propbit);
	input_mt_init_slots(ssl_input, ssl_priv->finger_count);
	input_set_abs_params(ssl_input, ABS_MT_POSITION_X,
				0, LCD_RANGE_X - 1, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_POSITION_Y,
				0, LCD_RANGE_Y - 1, 0, 0);
#else
	set_bit(ABS_MT_TRACKING_ID, ssl_input->absbit);
	input_set_abs_params(ssl_input, ABS_MT_TRACKING_ID,
				0, ssl_priv->finger_count, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_POSITION_X,
				0, LCD_RANGE_X, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_POSITION_Y,
				0, LCD_RANGE_Y, 0, 0);
#endif

	ssl_input->name = client->name;
	ssl_input->id.bustype = BUS_I2C;
	ssl_input->dev.parent = &client->dev;
	ssl_input->open = ssd253x_ts_open;
	ssl_input->close = ssd253x_ts_close;

	input_set_drvdata(ssl_input, ssl_priv);

	ssl_priv->input = ssl_input;

	INIT_WORK(&ssl_priv->ssl_work, ssd253x_ts_work);

	ssl_priv->irq = gpio_to_irq(TOUCH_INT_PIN);
	if (ssl_priv->irq < 0) {
		SSL_PRINT("Can't get irq number from GPIO.\n");
		error = -ENODEV;
		goto err2;
	}
	error = gpio_request(TOUCH_INT_PIN, "TOUCH INT");
	if (error < 0) {
		SSL_PRINT("GPIO request for INT fail.\n");
		error = -ENODEV;
		goto err2;
	}
	gpio_direction_input(TOUCH_INT_PIN);
	error = request_irq(ssl_priv->irq, ssd253x_ts_isr,
		IRQF_TRIGGER_FALLING, client->name, ssl_priv);
	if (error < 0) {
		SSL_PRINT("IRQ request fail.\n");
		error = -ENODEV;
		goto err3;
	}
	disable_irq_nosync(ssl_priv->irq);
	hrtimer_init(&ssl_priv->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ssl_priv->timer.function = ssd253x_ts_timer;
	error = input_register_device(ssl_input);
	if (error) {
		error = -ENODEV;
		goto	err4;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ssl_priv->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ssl_priv->early_suspend.suspend = ssd253x_ts_early_suspend;
	ssl_priv->early_suspend.resume = ssd253x_ts_late_resume;
	register_early_suspend(&ssl_priv->early_suspend);
#endif
	TP_PROC_FILE = create_proc_entry(PROCFS_NAME, 0644, NULL);
	if (TP_PROC_FILE == NULL) {
		remove_proc_entry(PROCFS_NAME, &proc_root);
		printk(KERN_INFO "Error:could not initialize %s.\n", PROCFS_NAME);
	} else {
		TP_PROC_FILE->read_proc = ssd253x_Proc_Read;
		TP_PROC_FILE->write_proc = ssd253x_Proc_Write;
		TP_PROC_FILE->mode = S_IFREG | S_IRUGO;
		TP_PROC_FILE->uid = 0;
		TP_PROC_FILE->gid = 0;
		TP_PROC_FILE->data = ssl_priv;
		TP_PROC_FILE->size = 37;
	}

	touch_power(0);
	return 0;
err4:
	free_irq(ssl_priv->irq, ssl_priv);
err3:
	gpio_free(TOUCH_INT_PIN);
err2:
	input_free_device(ssl_input);
err1:
	kfree(ssl_priv);
	dev_set_drvdata(&client->dev, NULL);
err0:
	gpio_free(TOUCH_RES_PIN);
	return error;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ssd253x_ts_early_suspend(struct early_suspend *h)
{
	struct ssl_ts_priv *ssl_priv = container_of(h, struct ssl_ts_priv, early_suspend);
	SSL_PRINT("Run into %s.\n", __func__);
	deviceSleep(ssl_priv);
}

static void ssd253x_ts_late_resume(struct early_suspend *h)
{
	struct ssl_ts_priv *ssl_priv = container_of(h, struct ssl_ts_priv, early_suspend);
	SSL_PRINT("Run into %s.\n", __func__);
	touch_power(1);
	deviceWakeUp(ssl_priv);
}
#else
static int ssd253x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);
	SSL_PRINT("Run into %s.\n", __func__);
	deviceSleep(ssl_priv);
	return 0;
}
static int ssd253x_ts_resume(struct i2c_client *client)
{
	SSL_PRINT("Run into %s.\n", __func__);
	touch_power(1);
	deviceWakeUp(ssl_priv);
	return 0;
}
#endif

static int ssd253x_ts_open(struct input_dev *dev)
{
	struct ssl_ts_priv *ssl_priv = input_get_drvdata(dev);
	SSL_PRINT("Run into %s.\n", __func__);
	touch_power(1);

	if (ssl_priv->inited == 0)
		deviceInit(ssl_priv);
	StartWork(ssl_priv);
	enable_irq(ssl_priv->irq);	/*enable irq for next interrupt*/
	return 0;
}
static void ssd253x_ts_close(struct input_dev *dev)
{
	struct ssl_ts_priv *ssl_priv = input_get_drvdata(dev);
	SSL_PRINT("Run into %s.\n", __func__);
	deviceSleep(ssl_priv);
	disable_irq_nosync(ssl_priv->irq);
}

static int ssd253x_ts_remove(struct i2c_client *client)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);
	SSL_PRINT("Run into %s.\n", __func__);
	free_irq(ssl_priv->irq, ssl_priv);
	gpio_free(TOUCH_INT_PIN);
	gpio_free(TOUCH_RES_PIN);
	input_unregister_device(ssl_priv->input);
	input_free_device(ssl_priv->input);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ssl_priv->early_suspend);
#endif
	remove_proc_entry(PROCFS_NAME, &proc_root);
	kfree(ssl_priv);
	dev_set_drvdata(&client->dev, NULL);
	return 0;
}
static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id)
{
	struct ssl_ts_priv *ssl_priv = dev_id;
	if (ssl_priv->bcalibrating == 1)
		return IRQ_HANDLED;
	hrtimer_cancel(&ssl_priv->timer);
	SSL_PRINT("Run into %s.\n", __func__);
	queue_work(ssd253x_wq, &ssl_priv->ssl_work);
	return IRQ_HANDLED;
}
static enum hrtimer_restart ssd253x_ts_timer(struct hrtimer *timer)
{
	struct ssl_ts_priv *ssl_priv = container_of(timer, struct ssl_ts_priv, timer);
	SSL_PRINT("Run into %s.\n", __func__);
	if (ssl_priv->bcalibrating == 1)
		ssl_priv->bcalibrating = 0;
	queue_work(ssd253x_wq, &ssl_priv->ssl_work);
	return HRTIMER_NORESTART;
}
static const struct i2c_device_id ssd253x_ts_id[] = {
	{ "ssd2531_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ssd253x_ts_id);

static struct i2c_driver ssd253x_ts_driver = {
	.driver = {
		.name = "ssd2531_ts",
	},
	.probe = ssd253x_ts_probe,
	.remove = ssd253x_ts_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend = NULL,
	.resume = NULL,
#else
	.suspend = ssd253x_ts_suspend,
	.resume = ssd253x_ts_resume,
#endif
	.id_table = ssd253x_ts_id,
};

static char banner[] __initdata = KERN_INFO
	"SSL Touchscreen driver, (c) 2011 Solomon Systech Ltd.\n";
static int __init ssd253x_ts_init(void)
{
	int ret;

	printk(banner);
	ssd253x_wq = create_singlethread_workqueue("ssd253x_wq");
	if (!ssd253x_wq)
		return -ENOMEM;

	ret = i2c_add_driver(&ssd253x_ts_driver);
	return ret;
}

static void __exit ssd253x_ts_exit(void)
{
	i2c_del_driver(&ssd253x_ts_driver);
	if (ssd253x_wq)
		destroy_workqueue(ssd253x_wq);
}

module_init(ssd253x_ts_init);
module_exit(ssd253x_ts_exit);

MODULE_AUTHOR("Solomon Systech Ltd - Rover Shen");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SSD2531/SSD2532/SSD2533 Touchscreen Driver, updated at 21:03, April 21, 2012");
