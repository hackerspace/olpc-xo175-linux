/*
 * LED/flash driver for Rohm bd7704mux chip
 *
 * Copyright (C) 2012 Marvell Internation Ltd.
 *
 * Bin Zhou <zhoub@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/videodev2.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <linux/bd7704.h>

#define BD7704_DRV_NAME "bd7704-ledflash"
static struct bd7704 *g_bd7704 = NULL;

static void bd7704_gen_pulse(unsigned int h_time, unsigned int l_time)
{
	struct bd7704_platform_data *pdata = g_bd7704->pdata;

	/* generate a low-high pulse on upic control pin */
	if (pdata->upic_control) {
		/* delay 80% of max. time after pull high/low */
		pdata->upic_control(0);
		udelay(l_time*4/5);
		pdata->upic_control(1);
		udelay(h_time*4/5);
	}
}

static void bd7704_control_start(void)
{
	/* make register access available: high for TMAX_ACCESS_READY */
	struct bd7704_platform_data *pdata = g_bd7704->pdata;

	if (pdata->upic_control) {
		pdata->upic_control(1);
		udelay(TMAX_ACCESS_READY);
	}
}

static void bd7704_control_end(void)
{
	/* pull low for long time enough to power off the chip  */
	struct bd7704_platform_data *pdata = g_bd7704->pdata;

	if (pdata->upic_control) {
		pdata->upic_control(0);
	}
}

static void bd7704_set_register(unsigned char address, unsigned char data)
{
	unsigned char i;
	unsigned long flags;

	local_irq_save(flags);
	/* set register address */
	/* NOT pull low for the last edge, but delay for register latch time */
	for(i = 0; i < (address - 1); i ++)
		bd7704_gen_pulse(TMAX_EN_TIME, TMAX_EN_TIME);
	bd7704_gen_pulse(TMAX_LATCH_TIME, TMAX_EN_TIME);

	/* set register data: [number of edges]=data+12 */
	/* NOT pull low for the last edge, but delay for register latch time */
	for(i = 0; i < (data + 12 -1); i ++)
		bd7704_gen_pulse(TMAX_EN_TIME, TMAX_EN_TIME);
	bd7704_gen_pulse(TMAX_LATCH_TIME, TMAX_EN_TIME);

	local_irq_restore(flags);
}

static int bd7704_output_off(void)
{
	bd7704_control_end();
	return 0;
}

static int bd7704_enable_torch_mode(void)
{
	bd7704_control_start();
	bd7704_set_register(REG_TORCH_RATIO, DATA_TORCH_CURRENT_FULL);
	bd7704_set_register(REG_MODE_SET, DATA_TORCH_ON_BOTH);
	return 0;
}

static int bd7704_enable_flash_mode(void)
{
	bd7704_control_start();
	bd7704_set_register(REG_FLASH_TIMER, DATA_FLASH_TIMER_1000MS);
	bd7704_set_register(REG_MODE_SET, DATA_FLASH_ON);
	return 0;
}

/* this function is called from none-v4l2 interface */
static int bd7704_set_output_mode(char mode)
{
	int ret = 0;
	struct bd7704_platform_data *pdata = g_bd7704->pdata;

	switch(mode) {
	case BD7704_CMD_MODE_STANDBY:
		printk(KERN_INFO "bd7704: output off\n");
		ret = bd7704_output_off();
		/* nothing controls flash currently */
		pdata->current_control = 0;
		break;
	case BD7704_CMD_MODE_TORCH:
		printk(KERN_INFO "bd7704: enable torch mode\n");
		ret = bd7704_enable_torch_mode();
		if (ret >=0 )
			pdata->current_control = BD7704_CMD_BUTTON_CONTROL;
		break;
	case BD7704_CMD_MODE_FLASH:
		printk(KERN_INFO "bd7704: enable flash mode\n");
		ret = bd7704_enable_flash_mode();
		if (ret >=0 )
			pdata->current_control = BD7704_CMD_BUTTON_CONTROL;
		break;
	default:
		printk(KERN_ERR "bd7704: unknown request\n");
		ret = -EPERM;
		break;
	}

	return ret;
}

/*
 * bd7704_v4l2_flash_if - callback for v4l2 set/get ctrl from sensor driver
 * @vid_ctrl: v4l2 control structure
 * @op: 0 for get, 1 for set
 *
 * Return 0 on success, or a negative error code
 */
int bd7704_v4l2_flash_if(void *vid_ctrl, bool op)
{
	int ret;
	struct bd7704_platform_data *pdata = g_bd7704->pdata;
	struct v4l2_control *ctrl = (struct v4l2_control *)vid_ctrl;;

	if (!ctrl)
		return -EPERM;

	if (op) {
		switch (ctrl->id) {
		case V4L2_CID_FLASH_LED_MODE:
			if (pdata->default_control == BD7704_CMD_BUTTON_CONTROL
				&& pdata->current_control == BD7704_CMD_BUTTON_CONTROL) {
				return -EBUSY;
			}
			switch (ctrl->value) {
			case V4L2_FLASH_LED_MODE_NONE:
				printk(KERN_INFO "bd7704-v4l2: output off\n");
				mutex_lock(&g_bd7704->lock);
				ret = bd7704_output_off();
				/* nothing controls flash currently */
				pdata->current_control = 0;
				mutex_unlock(&g_bd7704->lock);
				return ret;
			case V4L2_FLASH_LED_MODE_FLASH:
				printk(KERN_INFO "bd7704-v4l2: enable flash mode\n");
				mutex_lock(&g_bd7704->lock);
				ret = bd7704_enable_flash_mode();
				if (ret >=0 )
					pdata->current_control = BD7704_CMD_SENSOR_CONTROL;
				mutex_unlock(&g_bd7704->lock);
				return ret;
			case V4L2_FLASH_LED_MODE_TORCH:
				printk(KERN_INFO "bd7704-v4l2: enable torch mode\n");
				mutex_lock(&g_bd7704->lock);
				ret = bd7704_enable_torch_mode();
				if (ret >=0 )
					pdata->current_control = BD7704_CMD_SENSOR_CONTROL;
				mutex_unlock(&g_bd7704->lock);
				return ret;
			default:
				return -EPERM;
			}
		/* ctrl below not support now */
		case V4L2_CID_FLASH_STROBE_SOURCE:
		case V4L2_CID_FLASH_STROBE:
		case V4L2_CID_FLASH_STROBE_STOP:
		case V4L2_CID_FLASH_TIMEOUT:
		case V4L2_CID_FLASH_INTENSITY:
		case V4L2_CID_FLASH_TORCH_INTENSITY:
		default:
			return -EINVAL;
		}
	} else {
		switch (ctrl->id) {
		/* ctrl below not support now */
		case V4L2_CID_FLASH_FAULT:
		case V4L2_CID_FLASH_STROBE_STATUS:
		default:
			return -EINVAL;
		}
	}

	return 0;
}
EXPORT_SYMBOL(bd7704_v4l2_flash_if);

#ifdef	CONFIG_PROC_FS
#define	bd7704_PROC_FILE	"driver/bd7704"
static struct proc_dir_entry *bd7704_proc_file;

static ssize_t bd7704_proc_read(struct file *filp,
				 char *buffer, size_t count, loff_t *offset)
{
	return 0;
}

static ssize_t bd7704_proc_write(struct file *filp,
				  const char *buff, size_t len, loff_t *off)
{
	int ret = 0;
	struct bd7704_platform_data *pdata = g_bd7704->pdata;
	char messages[2];

	if (len > 2)
		len = 2;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	switch (messages[0]) {
		case BD7704_CMD_MODE_STANDBY:
		case BD7704_CMD_MODE_TORCH:
		case BD7704_CMD_MODE_FLASH:
			if (pdata->default_control == BD7704_CMD_SENSOR_CONTROL
				&& pdata->current_control == BD7704_CMD_SENSOR_CONTROL) {
				return -EBUSY;
			}
			mutex_lock(&g_bd7704->lock);
			ret = bd7704_set_output_mode(messages[0]);
			mutex_unlock(&g_bd7704->lock);
			if (ret < 0)
				return ret;
			break;
		case BD7704_CMD_SENSOR_CONTROL:
		case BD7704_CMD_BUTTON_CONTROL:
			mutex_lock(&g_bd7704->lock);
			pdata->default_control = messages[0];
			mutex_unlock(&g_bd7704->lock);
			break;
		case BD7704_CMD_QUERY:
			printk(KERN_INFO "turn off LED      : echo 0 > bd7704\n");
			printk(KERN_INFO "torch mode on     : echo 1 > bd7704\n");
			printk(KERN_INFO "flash mode on(N/A): echo 2 > bd7704\n");
			printk(KERN_INFO "set sensor control: echo 3 > bd7704\n");
			printk(KERN_INFO "set button control: echo 4 > bd7704\n");
			printk(KERN_INFO "cmd query         : echo 5 > bd7704\n");
			printk(KERN_INFO "default control   : 0x%2x\n", pdata->default_control);
			printk(KERN_INFO "current control   : 0x%2x\n", pdata->current_control);
			break;
		default:
			return -EPERM;
	}

	return len;
}

static struct file_operations bd7704_proc_ops = {
	.read = bd7704_proc_read,
	.write = bd7704_proc_write,
};

static int bd7704_create_proc_file(void)
{
	if (bd7704_proc_file != NULL)
		return 0;

	bd7704_proc_file = create_proc_entry(bd7704_PROC_FILE, 0644, NULL);
	if (bd7704_proc_file) {
		bd7704_proc_file->proc_fops = &bd7704_proc_ops;
	} else {
		printk(KERN_ERR "bd7704: failed to create proc file\n");
		return -EFAULT;
	}

	return 0;
}

static void bd7704_remove_proc_file(void)
{
	if (bd7704_proc_file == NULL)
		return;

	remove_proc_entry(bd7704_PROC_FILE, NULL);
	bd7704_proc_file = NULL;
}
#endif /*CONFIG_PROC_FS */

static int bd7704_probe(struct platform_device *pdev)
{
	int ret;
	struct bd7704_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata) {
		printk(KERN_ERR "bd7704: missing platform data\n");
		return -EINVAL;
	}

	g_bd7704 = kzalloc(sizeof(struct bd7704), GFP_KERNEL);
	if (!g_bd7704) {
		printk(KERN_ERR "bd7704: failed to allocate bd7704 struct\n");
		return -ENOMEM;
	}

	g_bd7704->pdata = pdata;
	mutex_init(&g_bd7704->lock);

#ifdef	CONFIG_PROC_FS
	ret = bd7704_create_proc_file();
	if (ret < 0) {
		kfree(g_bd7704);
		return ret;
	}
#endif
	return 0;
}

static int bd7704_remove(struct platform_device *pdev)
{
#ifdef	CONFIG_PROC_FS
	bd7704_remove_proc_file();
#endif

	if (g_bd7704) {
		kfree(g_bd7704);
		g_bd7704 = NULL;
	}

	return 0;
}

static struct platform_driver bd7704_driver = {
	.driver = {
		.name = BD7704_DRV_NAME
	},
	.probe	= bd7704_probe,
	.remove	= bd7704_remove,

};

static int __init bd7704_mod_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&bd7704_driver);

	return ret;
}

module_init(bd7704_mod_init);

static void __exit bd7704_mode_exit(void)
{
	platform_driver_unregister(&bd7704_driver);
}

module_exit(bd7704_mode_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bin Zhou <zhoub@marvell.com>");
MODULE_DESCRIPTION("BD7704 LED/flash driver");

