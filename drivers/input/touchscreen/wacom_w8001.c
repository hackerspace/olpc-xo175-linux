/*
 * Wacom W8001 penabled serial touchscreen driver
 *
 * Copyright (c) 2008 Jaya Kumar
 * Copyright (c) 2010 Red Hat, Inc.
 * Copyright (c) 2010 Ping Cheng, Wacom. <pingc@wacom.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 * Layout based on Elo serial touchscreen driver by Vojtech Pavlik
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/gpio.h> //T-Lite, for gpio
#include <mach/mfp-mmp2.h> //T-Lite, for gpio pin definitions
#include <mach/wistron.h>

#ifdef CONFIG_PROC_FS //T-Lite, add sysfs attributes to show w8001 settings, 20110810
#include <linux/proc_fs.h> //T-Lite, for proc_dir_entry
static u16 g_digitizer_fw_ver = 0; /*Added for obtaining FW version, T-Lite*/
static struct proc_dir_entry *g_digitizer_fw_ver_proc_file = NULL;
#endif //CONFIG_PROC_FS //T-Lite

#ifdef CONFIG_HAS_EARLYSUSPEND //T-Lite, for suspend/resume
	static struct early_suspend wacom_w8001_early_suspend;
	//T-Lite, a global flag used to keep the status if early_suspend is registered
	static bool g_w8001_early_suspend_registered = false;
#endif //CONFIG_HAS_EARLYSUSPEND //T-Lite

#define DRIVER_DESC	"Wacom W8001 serial touchscreen driver"

#define WACOM_DEBUG 0

MODULE_AUTHOR("Jaya Kumar <jayakumar.lkml@gmail.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

#define W8001_MAX_LENGTH	11
#define W8001_LEAD_MASK		0x80
#define W8001_LEAD_BYTE		0x80
#define W8001_TAB_MASK		0x40
#define W8001_TAB_BYTE		0x40
#define W8001_F2_MASK           0x24

/* set in first byte of touch data packets */
#define W8001_TOUCH_MASK	(0x10 | W8001_LEAD_MASK)
#define W8001_TOUCH_BYTE	(0x10 | W8001_LEAD_BYTE)

#define W8001_QUERY_PACKET	0x20

#define W8001_CMD_STOP		'0'
#define W8001_CMD_START		'1'
#define W8001_CMD_QUERY		'*'
#define W8001_CMD_TOUCHQUERY	'%'

/* length of data packets in bytes, depends on device. */
#define W8001_PKTLEN_TOUCH93	5
#define W8001_PKTLEN_TOUCH9A	7
#define W8001_PKTLEN_TPCPEN	9
#define W8001_PKTLEN_TPCCTL	11	/* control packet */
#define W8001_PKTLEN_TOUCH2FG	13

#define MAX_TRACKING_ID		0xFF	/* arbitrarily chosen */

struct w8001_coord {
	u8 rdy;
	u8 tsw;
	u8 f1;
	u8 f2;
	u16 x;
	u16 y;
	u16 pen_pressure;
	u8 tilt_x;
	u8 tilt_y;
  u16 fw; /*Added for obtaining FW version*/
};

/* touch query reply packet */
struct w8001_touch_query {
	u8 panel_res;
	u8 capacity_res;
	u8 sensor_id;
	u16 x;
	u16 y;
};

/*
 * Per-touchscreen data.
 */

struct w8001 {
	struct input_dev *dev;
	struct serio *serio;
	struct completion cmd_done;
	int id;
	int idx;
	unsigned char response_type;
	unsigned char response[W8001_MAX_LENGTH];
	unsigned char data[W8001_MAX_LENGTH];
	char phys[32];
	int type;
	unsigned int pktlen;
	int trkid[2];
	int trkid_idx;
	bool pen_in_prox;
	bool has_touch;
	int max_touch_x;
	int max_touch_y;
	int max_pen_x;
	int max_pen_y;
	int num_fingers;
	bool finger1_laststate;
	bool finger2_laststate;
};

struct sPoint 
{
	int x;
	int y;
};

struct sRect 
{
  int originX;
  int originY;
  int extentX;
  int extentY;
};

typedef enum
  {
	STATE_NORMAL,
	STATE_QUERY,
	STATE_GETCAL,
	STATE_POINTS
  } NODE_STATE;

typedef enum
  {
    DEG_0,
    DEG_90,
    DEG_180,
    DEG_270
  }ROTATION;

struct sPoint 	gSensorDims = {20470, 15400};
NODE_STATE      gNodeState = STATE_NORMAL;
struct sPoint 	gCacheCoords = {0xFFFFFFFF, 0xFFFFFFFF};
/*gCalData[0-3]: 0 = 0deg, 1 = 90deg, 2 = 180deg, 3 = 270deg*/
struct sRect 	gCalData[4] = {{0, 0, 20470, 15400}, {0, 0, 20470, 15400}, {0, 0, 20470, 15400}, {0, 0, 20470, 15400}};
char gRotation = DEG_0;
char*		w8001_buffer;
bool 		bCalibrationSet = false;

static void parse_data(u8 *data, struct w8001_coord *coord)
{
	memset(coord, 0, sizeof(*coord));

	coord->rdy = data[0] & 0x20;
	coord->tsw = data[0] & 0x01;
	coord->f1 = data[0] & 0x02;
	coord->f2 = data[0] & 0x04;

	coord->x = (data[1] & 0x7F) << 9;
	coord->x |= (data[2] & 0x7F) << 2;
	coord->x |= (data[6] & 0x60) >> 5;

	coord->y = (data[3] & 0x7F) << 9;
	coord->y |= (data[4] & 0x7F) << 2;
	coord->y |= (data[6] & 0x18) >> 3;

	coord->pen_pressure = data[5] & 0x7F;
	coord->pen_pressure |= (data[6] & 0x07) << 7 ;

	coord->tilt_x = data[7] & 0x7F;
	coord->tilt_y = data[8] & 0x7F;

	coord->fw = (data[9]<<8 | data[10]); /*Added for obtaining FW version*/
}

static void report_pen_events(struct w8001 *w8001, struct w8001_coord *coord)
{
	struct input_dev *dev = w8001->dev;
	int i, temp_coord;
	u16 temp_x, temp_y;
		
	// calibrate pen coords	
	if (bCalibrationSet && gNodeState == STATE_NORMAL) {
			/*X-Coordination with calibration value*/
			temp_coord = coord->x * gCalData[gRotation].extentX/w8001->max_pen_x + gCalData[gRotation].originX; 

			/*2012/07/27*/
			/*Check if obtained coordinations don't exceeds thier maximum or get negative numbers*/
			if (temp_coord < 0)
				coord->x = 0;
			else if (temp_coord > w8001->max_pen_x)
				coord->x = w8001->max_pen_x;
			else
				coord->x = temp_coord;
		
			/*Y-Coordination with calibration value*/
			temp_coord = coord->y * gCalData[gRotation].extentY/w8001->max_pen_y + gCalData[gRotation].originY;

			/*Check if obtained coordinations don't exceeds thier maximum or get negative numbers*/
			if (temp_coord < 0)
				coord->y = 0;
			else if (temp_coord > w8001->max_pen_y)
				coord->y = w8001->max_pen_y;
			else
				coord->y = temp_coord;
			/*2012/07/27*/

#if WACOM_DEBUG
		printk("%s Roatation*%d X:%d Y:%d \n", gRotation, coord->x, coord->y);
#endif
	}

	//	printk("%s x:%d y:%d \n", __func__, coord->x, coord->y);
	
	/*Check if obtained coordinations don't exceeds thier maximum or get negative numbers*/
	/*2012/07/27: Below commented out*/
	//if(coord->x < 0) coord->x = 0;
	//if(coord->y < 0) coord->y = 0;
	//if(coord->x > w8001->max_pen_x) coord->x = w8001->max_pen_x;
	//if(coord->y > w8001->max_pen_y) coord->y = w8001->max_pen_y; 

	input_report_abs(dev, ABS_X, coord->x);
	input_report_abs(dev, ABS_Y, coord->y);
	input_report_abs(dev, ABS_PRESSURE, coord->pen_pressure);
	input_report_key(dev, BTN_TOOL_PEN, coord->tsw);
	input_report_key(dev, BTN_STYLUS, coord->f1);
#if 1	/*Wacom 2012/05/22*/
	input_report_key(dev, BTN_TOUCH, 1);
#else
	if(coord->tsw || coord->f1)
	  input_report_key(dev, BTN_TOUCH, 1);
	else
	  input_report_key(dev, BTN_TOUCH, 0);
#endif	/*Wacom 2012/05/22*/
	input_sync(dev);
}

static irqreturn_t w8001_interrupt(struct serio *serio,
				   unsigned char data, unsigned int flags)
{
	struct w8001 *w8001 = serio_get_drvdata(serio);
	struct input_dev *dev = w8001->dev;
	struct w8001_coord coord;
	unsigned char tmp;

	w8001->data[w8001->idx] = data;
	switch (w8001->idx++) {
	case 0:
		if ((data & W8001_LEAD_MASK) != W8001_LEAD_BYTE) {
			pr_debug("w8001: unsynchronized data: 0x%02x\n", data);
			w8001->idx = 0;
		}
		break;

	/* Pen coordinates packet */
	case W8001_PKTLEN_TPCPEN - 1:
		tmp = w8001->data[0] & W8001_TAB_MASK;
		if (unlikely(tmp == W8001_TAB_BYTE))
			break;

		tmp = (w8001->data[0] & W8001_TOUCH_BYTE);
		if (tmp == W8001_TOUCH_BYTE)
			break;

		w8001->idx = 0;

		parse_data(w8001->data, &coord);
		report_pen_events(w8001, &coord);
		w8001->pen_in_prox = coord.rdy ? true : false;
		break;

	/* control packet */
	case W8001_PKTLEN_TPCCTL - 1:
		tmp = (w8001->data[0] & W8001_TOUCH_MASK);
		if (tmp == W8001_TOUCH_BYTE)
			break;

		w8001->idx = 0;
		memcpy(w8001->response, w8001->data, W8001_MAX_LENGTH);
		w8001->response_type = W8001_QUERY_PACKET;
		complete(&w8001->cmd_done);
		break;
	}

	return IRQ_HANDLED;
}

static int w8001_command(struct w8001 *w8001, unsigned char command,
			 bool wait_response)
{
	int rc;

	w8001->response_type = 0;
	init_completion(&w8001->cmd_done);

	rc = serio_write(w8001->serio, command);
	if (rc == 0 && wait_response) {

		wait_for_completion_timeout(&w8001->cmd_done, HZ);
		if (w8001->response_type != W8001_QUERY_PACKET)
			rc = -EIO;
	}

	return rc;
}

static int w8001_setup(struct w8001 *w8001)
{
	struct input_dev *dev = w8001->dev;
	struct w8001_coord coord;
	int error;

	error = w8001_command(w8001, W8001_CMD_STOP, false);
	if (error)
		return error;

	mdelay(150);	/* wait 250ms before querying the device */
	
	/* penabled? */
	error = w8001_command(w8001, W8001_CMD_QUERY, true);
	if (!error) {
	  dev->keybit[BIT_WORD(BTN_TOOL_PEN)] |= BIT_MASK(BTN_TOOL_PEN);
	  dev->keybit[BIT_WORD(BTN_TOOL_RUBBER)] |= BIT_MASK(BTN_TOOL_RUBBER);
	  dev->keybit[BIT_WORD(BTN_STYLUS)] |= BIT_MASK(BTN_STYLUS);
	  dev->keybit[BIT_WORD(BTN_STYLUS2)] |= BIT_MASK(BTN_STYLUS2);

	  parse_data(w8001->response, &coord);

#ifdef CONFIG_PROC_FS //T-Lite, store firmware version as a global data, 20110810
	  g_digitizer_fw_ver = coord.fw;
	  printk("%s: x_max:%d y_max:%d fw_ver:0x%x \n", __func__, w8001->max_pen_x, w8001->max_pen_y, g_digitizer_fw_ver); /*Added for obtaining FW version*/
#endif //T-Lite

	  gSensorDims.x = w8001->max_pen_x = coord.x;
	  gSensorDims.y = w8001->max_pen_y = coord.y;
	  printk("x_max:%d y_max:%d fw:%x \n", w8001->max_pen_x, w8001->max_pen_y, coord.fw);

	  input_set_abs_params(dev, ABS_X, 0, w8001->max_pen_x, 0, 0);
	  input_set_abs_params(dev, ABS_Y, 0, w8001->max_pen_y, 0, 0);
	  input_set_abs_params(dev, ABS_PRESSURE, 0, coord.pen_pressure, 0, 0);
	  if (coord.tilt_x && coord.tilt_y)
	    {
	      input_set_abs_params(dev, ABS_TILT_X, 0, coord.tilt_x, 0, 0);
	      input_set_abs_params(dev, ABS_TILT_Y, 0, coord.tilt_y, 0, 0);
	    }
	}

	return w8001_command(w8001, W8001_CMD_START, false);
}

#ifdef CONFIG_PROC_FS //T-Lite, add sysfs attributes to show w8001 settings, 20110810
static ssize_t
digitizer_fetch_firmware_version(char *page, char **start, off_t off, int count, int *eof, void *data)
{
        int len = 0;
        
        //printk(KERN_ERR "%s: digitizer_fw_ver = 0x%x.\r\n", __func__, g_digitizer_fw_ver);                        
        len = sprintf(page, "0x%x\n", g_digitizer_fw_ver); 
        
        return len;
}

static void 
w8001_create_digitizer_proc_file(void)
{
        struct proc_dir_entry *digitizer_fw_ver_proc_file = NULL;
        //printk(KERN_ERR "%s()\r\n", __func__);
        
        digitizer_fw_ver_proc_file = create_proc_entry(PROJ_PROC_DIGITIZER_VER, S_IRUGO, NULL);
        //printk(KERN_ERR "%s: digitizer_fw_ver_proc_file = %p.\r\n", __func__, digitizer_fw_ver_proc_file);
        if(!digitizer_fw_ver_proc_file)
        {
                printk(KERN_ERR "%s: fail to create proc file for digitizer firmware version!\r\n", __func__);
                return;
        }
        
        g_digitizer_fw_ver_proc_file = digitizer_fw_ver_proc_file;
        digitizer_fw_ver_proc_file->read_proc = (read_proc_t *)digitizer_fetch_firmware_version;
        
        printk(KERN_ERR "%s: Create firmware version proc entry done.\r\n", __func__);
        return;       
}

static void
w8001_remove_digitizer_proc_file(void)
{
       // printk(KERN_ERR "%s()\r\n", __func__);
        remove_proc_entry("driver/digitizer_fw_ver", g_digitizer_fw_ver_proc_file);
}
#endif //T-Lite

#ifdef CONFIG_HAS_EARLYSUSPEND //T-Lite, for suspend/resume
static void 
w8001_set_enable(int enable)
{
	int gpio = 0, 
	    gpio_value = 0,  
	    ret = 0;    
	//printk(KERN_ERR "%s: set enable %d.\r\n", __func__, enable);
	
	/* DIGITIZER_SLP */
	gpio = mfp_to_gpio(GPIO91_GPIO);
	ret = gpio_request(gpio, "digitizer_slp");
	if(ret)
	{
		printk(KERN_INFO "%s: request gpio %d failed! error: %d.\r\n", __func__, gpio, ret);
		return;
	}
	
	if(enable) /* Enable the device: set slp low */
	{
		//T-Lite, Not to pull high again if status has been high, Paul @ 20110531
		gpio_value = gpio_get_value(gpio);
		if(gpio_value)
		{
			printk("8001 enable- pull LOW\n");
//			printk(KERN_INFO "%s: set gpio %d output %d.\r\n", __func__, gpio, 0);
			gpio_direction_output(gpio, 0);
		}
		else
		{
//			printk(KERN_INFO "%s: set gpio %d value is %d!\r\n", __func__, gpio, gpio_value);
			printk("8001 enable- keep LOW\n");
		}
	}
	else /* Disable the device: set slp high */
	{
//		printk(KERN_INFO "%s: set gpio %d output %d.\r\n", __func__, gpio, 1);
//		gpio_direction_output(gpio, 1);
		printk("8001 disable- set as input\n");
		gpio_direction_input(gpio);
//		printk(KERN_INFO "%s: set gpio %d input .\r\n", __func__, gpio);
	}
	mdelay(10);
	
	gpio_free(gpio);
	return;
}

static void 
w8001_early_suspend(struct early_suspend *handler)
{
#if 0
	int gpio, ret; 
	printk("%s()++\r\n", __func__);
	w8001_set_enable(0);
	printk("%s()--\r\n", __func__);

	gpio = mfp_to_gpio(GPIO65_GPIO);
	ret = gpio_request(gpio, "digitizer_uart4");
	gpio_direction_output(gpio, 0);
	gpio_free(gpio);

	gpio = mfp_to_gpio(GPIO66_GPIO);
	ret = gpio_request(gpio, "digitizer_uart2");
	gpio_direction_output(gpio, 0);
	gpio_free(gpio);
#endif
	return;
}

static void 
w8001_late_resume(struct early_suspend *handler)
{
#if 0
	mfp_cfg_t mfp_gpio65_uart4 = GPIO65_UART4_RXD;
	mfp_cfg_t mfp_gpio66_uart4 = GPIO66_UART4_TXD;

	mfp_config(&mfp_gpio65_uart4, 1);
	mfp_config(&mfp_gpio66_uart4, 1);
	
	printk("%s()++\r\n", __func__);
	w8001_set_enable(1);
	printk("%s()--\r\n", __func__);
#endif
	return;
}
#endif //CONFIG_HAS_EARLYSUSPEND //T-Lite


/*
 * w8001_disconnect() is the opposite of w8001_connect()
 */

static void w8001_disconnect(struct serio *serio)
{
	struct w8001 *w8001 = serio_get_drvdata(serio);

	printk("%s()++\r\n", __func__);
	w8001_set_enable(0);
	printk("%s()--\r\n", __func__);

	input_get_device(w8001->dev);
	input_unregister_device(w8001->dev);
	serio_close(serio);
	serio_set_drvdata(serio, NULL);
	input_put_device(w8001->dev);
	kfree(w8001);
}

/*
 * w8001_connect() is the routine that is called when someone adds a
 * new serio device that supports the w8001 protocol and registers it as
 * an input device.
 */
int testVal = 0;
static int w8001_connect(struct serio *serio, struct serio_driver *drv)
{
	struct w8001 *w8001;
	struct input_dev *input_dev;
	int err;

#if 1 //T-Lite, activate device if it's in sleep mode
	printk("%s()++\r\n", __func__);
	w8001_set_enable(1);
	printk("%s()--\r\n", __func__);
#endif //T-Lite
	w8001 = kzalloc(sizeof(struct w8001), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!w8001 || !input_dev) {
		err = -ENOMEM;
		goto fail1;
	}

	w8001->serio = serio;
	w8001->id = serio->id.id;
	w8001->dev = input_dev;
	w8001->trkid[0] = w8001->trkid[1] = -1;
	init_completion(&w8001->cmd_done);
	snprintf(w8001->phys, sizeof(w8001->phys), "%s/input0", serio->phys);

#if 1 //T-Lite, for HC touch screen IDC (Input Device Calibration), 20110818
        input_dev->name = "Wacom_W8001_Penabled_Serial_TouchScreen";
#else
	input_dev->name = "Wacom W8001 Penabled Serial TouchScreen";
#endif //T-Lite
	input_dev->phys = w8001->phys;
	input_dev->id.bustype = BUS_RS232;
	input_dev->id.vendor = SERIO_W8001;
	input_dev->id.product = w8001->id;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &serio->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	serio_set_drvdata(serio, w8001);
	err = serio_open(serio, drv);
	if (err)
		goto fail2;

	err = w8001_setup(w8001);
	if (err)
		goto fail3;

	err = input_register_device(w8001->dev);
	if (err)
		goto fail3;

	//[delete-s mms] sample code
	//#ifdef CONFIG_PROC_FS //T-Lite, add sysfs attributes to show w8001 settings, 20110810
	//        if(!g_digitizer_fw_ver_proc_file)
	//                w8001_create_digitizer_proc_file();

	//#endif //T-Lite
	//[delete-e mms]

#ifdef CONFIG_HAS_EARLYSUSPEND //T-Lite, for suspend/resume
	if(!g_w8001_early_suspend_registered)
	{
		wacom_w8001_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
		wacom_w8001_early_suspend.suspend = w8001_early_suspend;
		wacom_w8001_early_suspend.resume  = w8001_late_resume;
		register_early_suspend(&wacom_w8001_early_suspend);
		g_w8001_early_suspend_registered = true;
		printk("%s: Register early_suspend done\n", __func__);
	}
#endif //CONFIG_HAS_EARLYSUSPEND //T-Lite

	testVal = 66;
	return 0;

fail3:
	serio_close(serio);
fail2:
	serio_set_drvdata(serio, NULL);
fail1:
	input_free_device(input_dev);
	kfree(w8001);
	return err;
}



int w8001_open(struct inode *inode, struct file *filp)
{
	return 0;
}

int w8001_release(struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t w8001_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	int temp;

	switch(gNodeState)
		{
		case STATE_NORMAL:
			printk("%s: reads STATE_NORMAL \n", __func__);
			break;
			
		case STATE_QUERY:
			
			copy_to_user(buf, (char*)&gSensorDims, 8);
			printk("%s: reads STATE_QUERY max_x:%d max_y:%d\n", __func__, gSensorDims.x, gSensorDims.y);
			return 8;
			
		case STATE_GETCAL:
			copy_to_user(buf, (char*)&gCalData, 16);
			printk("%s: reads STATE_GETCAL \n", __func__);
			return 16;

		case STATE_POINTS:
			copy_to_user(buf, (char*)&gCacheCoords, 8);
			printk("%s: read x:%d y:%d \n", __func__, gCacheCoords.x, gCacheCoords.y);
			return 8;

		default:
			printk("%s: reads default \n", __func__);
			break;
		}

	return 0;

}

ssize_t w8001_write(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
  if(count == 1)
    {
      char cmd = buf[0];
      int temp;
      switch(cmd)
	{
	case '?':
	  gNodeState = STATE_QUERY;
	  printk("%s Set STATE_QUERY \n", __func__);
	  break;
	  
	case '%':
	  gNodeState = STATE_GETCAL;
	  printk("%s Set STATE_GETCAL \n", __func__);
	  break;
	  
	case '*':
	  gNodeState = STATE_POINTS;
	  printk("%s Set STATE_POINTS \n", __func__);
	  break;
	  
	case '0':
	  gNodeState = STATE_NORMAL;
	  printk("%s Set STATE_NORMAL \n", __func__);
	  break;
	  
	  /*Rotation Notice*/
	  /*gSensor* coordination "if" statement has to be switched when a default orientation*/
	  /*is "Landscape". gSensor* must be used only to be reported to PenCalibration.apk*/
	case 'O':
	  gRotation = DEG_0;
	  if (gSensorDims.x > gSensorDims.y) {
		  temp = gSensorDims.x;
		  gSensorDims.x = gSensorDims.y;
		  gSensorDims.y = temp;
	  }
	  printk("%s 0degree set max_x:%d max_y:%d\n", __func__, gSensorDims.x, gSensorDims.y);
	  break;

	case '1':
	  gRotation = DEG_90;
	  if (gSensorDims.y > gSensorDims.x) {
		  temp = gSensorDims.x;
		  gSensorDims.x = gSensorDims.y;
		  gSensorDims.y = temp;
	  }
	  printk("%s 90 degree set max_x:%d max_y:%d\n", __func__, gSensorDims.x, gSensorDims.y);
	  break;

	case '2':
	  gRotation = DEG_180;
	  if (gSensorDims.x > gSensorDims.y) {
		  temp = gSensorDims.x;
		  gSensorDims.x = gSensorDims.y;
		  gSensorDims.y = temp;
	  }
	  printk("%s 180 degree set max_x:%d max_y:%d\n", __func__, gSensorDims.x, gSensorDims.y);
	  break;

	case '3':
	  gRotation = DEG_270;
	  if (gSensorDims.y > gSensorDims.x) {
		  temp = gSensorDims.x;
		  gSensorDims.x = gSensorDims.y;
		  gSensorDims.y = temp;
	  }
	  printk("%s 270 degree set max_x:%d max_y:%d \n", __func__, gSensorDims.x, gSensorDims.y);
	  break;
	}
    }
  else if (count == 16)
    {
      // import calibration points here
      struct sRect *tmp = (struct sRect*)buf;
      
      switch(gRotation) {
      /*This coordination switch must be reversed when a default screen orientation is "Landscape"*/
      case DEG_0:
      case DEG_180:
	      gCalData[gRotation].originX = tmp->originY;
	      gCalData[gRotation].originY = tmp->originX;
	      gCalData[gRotation].extentX = tmp->extentY;
	      gCalData[gRotation].extentY = tmp->extentX;
	      break;

      case DEG_90:
      case DEG_270:
	      gCalData[gRotation].originX = tmp->originX;
	      gCalData[gRotation].originY = tmp->originY;
	      gCalData[gRotation].extentX = tmp->extentX;
	      gCalData[gRotation].extentY = tmp->extentY;
	      break;
      }

      bCalibrationSet = true;
      printk("-------------------------------------------------------\n");
      printk("Calibration Data Set[%d]: %d, %d, %d, %d\n", gRotation, gCalData[gRotation].originX, gCalData[gRotation].originY, gCalData[gRotation].extentX, gCalData[gRotation].extentY);
      printk("-------------------------------------------------------\n");
    }

	return count;
}


static struct serio_device_id w8001_serio_ids[] = {
	{
		.type	= SERIO_RS232,
		.proto	= SERIO_W8001,
		.id	= SERIO_ANY,
		.extra	= SERIO_ANY,
	},
	{ 0 }
};

MODULE_DEVICE_TABLE(serio, w8001_serio_ids);

struct file_operations w8001_fops = {
        read: w8001_read,
        write: w8001_write,
        open: w8001_open,
        release: w8001_release
};

static struct serio_driver w8001_drv = {
	.driver		= {
		.name	= "w8001",
	},
	.description	= DRIVER_DESC,
	.id_table	= w8001_serio_ids,
	.interrupt	= w8001_interrupt,
	.connect	= w8001_connect,
	.disconnect	= w8001_disconnect,
};

static int __init w8001_init(void)
{
	int result = 0;

	result = register_chrdev(66, "w8001", &w8001_fops);
	if(result < 0)
	{
		printk("<1> w8001 cannot obtain major number %d\n", 66);
		return result;
	}
	
	w8001_buffer = kmalloc(1, GFP_KERNEL);
	if(!w8001_buffer) 
	{
		return -ENOMEM;
	}

//[add-s mms] sample code
#ifdef CONFIG_PROC_FS //T-Lite, add sysfs attributes to show w8001 settings, 20110810
	if(!g_digitizer_fw_ver_proc_file)
                w8001_create_digitizer_proc_file();
#endif //T-Lite
//[add-e mms]

	memset(w8001_buffer, 0, 1);
	
	return serio_register_driver(&w8001_drv);
}

static void __exit w8001_exit(void)
{
#ifdef CONFIG_PROC_FS //T-Lite, add sysfs attributes to show w8001 settings, 20110810
        if(g_digitizer_fw_ver_proc_file)
                w8001_remove_digitizer_proc_file();
#endif //T-Lite

#ifdef CONFIG_HAS_EARLYSUSPEND //T-Lite, unregister suspend/resume in driver exit
        if(g_w8001_early_suspend_registered)
	{
		unregister_early_suspend(&wacom_w8001_early_suspend);
		printk("%s: Unregister early_suspend done\n", __func__);
	}
#endif //CONFIG_HAS_EARLYSUSPEND //T-Lite

	unregister_chrdev(66, "w8001");

	if(w8001_buffer)
	{
		kfree(w8001_buffer);
	}

	serio_unregister_driver(&w8001_drv);
}

module_init(w8001_init);
module_exit(w8001_exit);

