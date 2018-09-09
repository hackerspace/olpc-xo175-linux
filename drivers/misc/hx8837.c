/*
 * Mainly by David Woodhouse, somewhat modified by Jordan Crouse
 *
 * Copyright © 2006-2007  Red Hat, Inc.
 * Copyright © 2006-2007  Advanced Micro Devices, Inc.
 * Copyright © 2009       VIA Technology, Inc.
 * Copyright (c) 2010-2011  Andres Salomon <dilinger@queued.net>
 *
 * This program is free software.  You can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General Public
 * License as published by the Free Software Foundation.
 */

#define OLPC_GPIO_DCON_STAT0	100
#define OLPC_GPIO_DCON_STAT1	101
#define OLPC_GPIO_DCON_IRQ	124
#define OLPC_GPIO_DCON_LOAD     142




#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

//#include <linux/cs5535.h>
#include <linux/gpio.h>
#include <linux/delay.h>
//#include "olpc_dcon.h"

#include <linux/wait.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/console.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/backlight.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/reboot.h>
#include <linux/olpc-ec.h>
//#include <asm/tsc.h>
//#include <asm/olpc.h>

//#include "olpc_dcon.h"





/* DCON registers */

#define DCON_REG_ID		 0
#define DCON_REG_MODE		 1

#define MODE_PASSTHRU	BIT(0)
#define MODE_SLEEP	BIT(1)
#define MODE_SLEEP_AUTO	BIT(2)
#define MODE_BL_ENABLE	BIT(3)
#define MODE_BLANK	BIT(4)
#define MODE_CSWIZZLE	BIT(5)
#define MODE_COL_AA	BIT(6)
#define MODE_MONO_LUMA	BIT(7)
#define MODE_SCAN_INT	BIT(8)
#define MODE_CLOCKDIV	BIT(9)
#define MODE_DEBUG	BIT(14)
#define MODE_SELFTEST	BIT(15)

#define DCON_REG_HRES		0x2
#define DCON_REG_HTOTAL		0x3
#define DCON_REG_HSYNC_WIDTH	0x4
#define DCON_REG_VRES		0x5
#define DCON_REG_VTOTAL		0x6
#define DCON_REG_VSYNC_WIDTH	0x7
#define DCON_REG_TIMEOUT	0x8
#define DCON_REG_SCAN_INT	0x9
#define DCON_REG_BRIGHT		0xa
#define DCON_REG_MEM_OPT_A	0x41
#define DCON_REG_MEM_OPT_B	0x42

/* Load Delay Locked Loop (DLL) settings for clock delay */
#define MEM_DLL_CLOCK_DELAY	BIT(0)
/* Memory controller power down function */
#define MEM_POWER_DOWN		BIT(8)
/* Memory controller software reset */
#define MEM_SOFT_RESET		BIT(0)

/* Status values */

#define DCONSTAT_SCANINT	0
#define DCONSTAT_SCANINT_DCON	1
#define DCONSTAT_DISPLAYLOAD	2
#define DCONSTAT_MISSED		3

/* Source values */

#define DCON_SOURCE_DCON        0
#define DCON_SOURCE_CPU         1

/* Interrupt */
#define DCON_IRQ                6






struct dcon_priv {
	struct i2c_client *client;
	struct fb_info *fbinfo;
	struct backlight_device *bl_dev;

	wait_queue_head_t waitq;
	struct work_struct switch_source;
	struct notifier_block reboot_nb;

	/* Shadow register for the DCON_REG_MODE register */
	u8 disp_mode;

	/* The current backlight value - this saves us some smbus traffic */
	u8 bl_val;

	/* Current source, initialized at probe time */
	int curr_src;

	/* Desired source */
	int pending_src;

	/* Variables used during switches */
	bool switched;
	ktime_t irq_time;
	ktime_t load_time;

	/* Current output type; true == mono, false == color */
	bool mono;
	bool asleep;
	/* This get set while controlling fb blank state from the driver */
	bool ignore_fb_events;
};




irqreturn_t dcon_interrupt(int irq, void *id);













static int dcon_init_xo_1_75(struct dcon_priv *dcon)
{

	if (gpio_request(OLPC_GPIO_DCON_STAT0, "OLPC-DCON")) {
		printk(KERN_ERR "olpc-dcon: failed to request STAT0 GPIO\n");
		return -EIO;
	}
	if (gpio_request(OLPC_GPIO_DCON_STAT1, "OLPC-DCON")) {
		printk(KERN_ERR "olpc-dcon: failed to request STAT1 GPIO\n");
		goto err_gp_stat1;
	}
	if (gpio_request(OLPC_GPIO_DCON_IRQ, "OLPC-DCON")) {
		printk(KERN_ERR "olpc-dcon: failed to request IRQ GPIO\n");
		goto err_gp_irq;
	}
	if (gpio_request(OLPC_GPIO_DCON_LOAD, "OLPC-DCON")) {
		printk(KERN_ERR "olpc-dcon: failed to request LOAD GPIO\n");
		goto err_gp_load;
	}


	/*
	 * Determine the current state by reading the GPIO bit; earlier
	 * stages of the boot process have established the state.
	 */
	dcon->curr_src = gpio_get_value(OLPC_GPIO_DCON_LOAD)
		? DCON_SOURCE_CPU
		: DCON_SOURCE_DCON;
	dcon->pending_src = dcon->curr_src;

	/* Set the directions for the GPIO pins */
	gpio_direction_input(OLPC_GPIO_DCON_STAT0);
	gpio_direction_input(OLPC_GPIO_DCON_STAT1);
	gpio_direction_input(OLPC_GPIO_DCON_IRQ);
	gpio_direction_output(OLPC_GPIO_DCON_LOAD,
			dcon->curr_src == DCON_SOURCE_CPU
			);

	/* Register the interrupt handler */
	if (request_irq(gpio_to_irq(OLPC_GPIO_DCON_IRQ), &dcon_interrupt,
		IRQF_TRIGGER_FALLING, "DCON", dcon)) {
		printk(KERN_ERR "olpc-dcon: failed to request DCON's irq\n");
		goto err_req_irq;
	}


	return 0;

err_req_irq:
	gpio_free(OLPC_GPIO_DCON_LOAD);
err_gp_load:
	gpio_free(OLPC_GPIO_DCON_IRQ);
err_gp_irq:
	gpio_free(OLPC_GPIO_DCON_STAT1);
err_gp_stat1:
	gpio_free(OLPC_GPIO_DCON_STAT0);
	return -EIO;
}

/* it would be nice if we could fetch these from olpc_xo_1_75_dcon_i2c_data... */
#define GPIO_SMBCLK	161
#define GPIO_SMBDATA	110

static void dcon_wiggle_xo_1_75(void)
{
	int x;

	/*
	 * According to HiMax, when powering the DCON up we should hold
	 * SMB_DATA high for 8 SMB_CLK cycles.  This will force the DCON
	 * state machine to reset to a (sane) initial state.  Mitch Bradley
	 * did some testing and discovered that holding for 16 SMB_CLK cycles
	 * worked a lot more reliably, so that's what we do here.
	 */
	gpio_set_value(GPIO_SMBDATA, 1);	/* should be high the entire time */

	for (x = 0; x < 16; x++) {
		udelay(5);
		gpio_set_value(GPIO_SMBCLK, 0);
		udelay(5);
		gpio_set_value(GPIO_SMBCLK, 1);
	}
	udelay(5);
}

static void dcon_set_dconload_1_75(int val)
{
	gpio_set_value(OLPC_GPIO_DCON_LOAD, val);
}

static u8 dcon_read_status_xo_1_75(void)
{
	u8 status;

	status = !!gpio_get_value(OLPC_GPIO_DCON_STAT0);
	status |= !!gpio_get_value(OLPC_GPIO_DCON_STAT1) << 1;

	return status;
}



#if 0
void dcon_deinit_xo_1_75(struct dcon_priv *dcon)
{
	free_irq(gpio_to_irq(OLPC_GPIO_DCON_IRQ), dcon);
}
#endif












/* Module definitions */

static ushort resumeline = 898;
module_param(resumeline, ushort, 0444);

/* I2C structures */

/* Platform devices */
static struct platform_device *dcon_device;

static unsigned short normal_i2c[] = { 0x0d, I2C_CLIENT_END };

static s32 dcon_write(struct dcon_priv *dcon, u8 reg, u16 val)
{
	return i2c_smbus_write_word_data(dcon->client, reg, val);
}

static s32 dcon_read(struct dcon_priv *dcon, u8 reg)
{
	return i2c_smbus_read_word_data(dcon->client, reg);
}

/* ===== API functions - these are called by a variety of users ==== */

static int dcon_hw_init(struct dcon_priv *dcon, int is_init)
{
	u16 ver;
	int rc = 0;

	ver = dcon_read(dcon, DCON_REG_ID);
	if ((ver >> 8) != 0xDC) {
		pr_err("DCON ID not 0xDCxx: 0x%04x instead.\n", ver);
		rc = -ENXIO;
		goto err;
	}

	if (is_init) {
		pr_info("Discovered DCON version %x\n", ver & 0xFF);
		//rc = pdata->init(dcon);
		rc = dcon_init_xo_1_75(dcon);
		if (rc != 0) {
			pr_err("Unable to init.\n");
			goto err;
		}
	}

	if (ver < 0xdc02) {
		dev_err(&dcon->client->dev,
			"DCON v1 is unsupported, giving up..\n");
		rc = -ENODEV;
		goto err;
	}

	/* SDRAM setup/hold time */
	dcon_write(dcon, 0x3a, 0xc040);
	dcon_write(dcon, DCON_REG_MEM_OPT_A, 0x0000);  /* clear option bits */
	dcon_write(dcon, DCON_REG_MEM_OPT_A,
		   MEM_DLL_CLOCK_DELAY | MEM_POWER_DOWN);
	dcon_write(dcon, DCON_REG_MEM_OPT_B, MEM_SOFT_RESET);

	/* Colour swizzle, AA, no passthrough, backlight */
	if (is_init) {
		dcon->disp_mode = MODE_PASSTHRU | MODE_BL_ENABLE |
				MODE_CSWIZZLE | MODE_COL_AA;
	}
	dcon_write(dcon, DCON_REG_MODE, dcon->disp_mode);

	/* Set the scanline to interrupt on during resume */
	dcon_write(dcon, DCON_REG_SCAN_INT, resumeline);

err:
	return rc;
}

/*
 * The smbus doesn't always come back due to what is believed to be
 * hardware (power rail) bugs.  For older models where this is known to
 * occur, our solution is to attempt to wait for the bus to stabilize;
 * if it doesn't happen, cut power to the dcon, repower it, and wait
 * for the bus to stabilize.  Rinse, repeat until we have a working
 * smbus.  For newer models, we simply BUG(); we want to know if this
 * still happens despite the power fixes that have been made!
 */
static int dcon_bus_stabilize(struct dcon_priv *dcon, int is_powered_down)
{
	unsigned long timeout;
	u8 pm;
	int x;

power_up:
	if (is_powered_down) {
		pm = 1;
		x = olpc_ec_cmd(EC_DCON_POWER_MODE, &pm, 1, NULL, 0);
		if (x) {
			pr_warn("unable to force dcon to power up: %d!\n", x);
			return x;
		}
		usleep_range(10000, 11000);  /* we'll be conservative */
	}

	//pdata->bus_stabilize_wiggle();
	dcon_wiggle_xo_1_75();

	for (x = -1, timeout = 50; timeout && x < 0; timeout--) {
		usleep_range(1000, 1100);
		x = dcon_read(dcon, DCON_REG_ID);
	}
	if (x < 0) {
		pr_err("unable to stabilize dcon's smbus, reasserting power and praying.\n");
		//BUG_ON(olpc_board_at_least(olpc_board(0xc2)));
		pm = 0;
		olpc_ec_cmd(EC_DCON_POWER_MODE, &pm, 1, NULL, 0);
		msleep(100);
		is_powered_down = 1;
		goto power_up;	/* argh, stupid hardware.. */
	}

	if (is_powered_down)
		return dcon_hw_init(dcon, 0);
	return 0;
}

static void dcon_set_backlight(struct dcon_priv *dcon, u8 level)
{
	dcon->bl_val = level;
	dcon_write(dcon, DCON_REG_BRIGHT, dcon->bl_val);

	/* Purposely turn off the backlight when we go to level 0 */
	if (dcon->bl_val == 0) {
		dcon->disp_mode &= ~MODE_BL_ENABLE;
		dcon_write(dcon, DCON_REG_MODE, dcon->disp_mode);
	} else if (!(dcon->disp_mode & MODE_BL_ENABLE)) {
		dcon->disp_mode |= MODE_BL_ENABLE;
		dcon_write(dcon, DCON_REG_MODE, dcon->disp_mode);
	}
}

/* Set the output type to either color or mono */
static int dcon_set_mono_mode(struct dcon_priv *dcon, bool enable_mono)
{
	if (dcon->mono == enable_mono)
		return 0;

	dcon->mono = enable_mono;

	if (enable_mono) {
		dcon->disp_mode &= ~(MODE_CSWIZZLE | MODE_COL_AA);
		dcon->disp_mode |= MODE_MONO_LUMA;
	} else {
		dcon->disp_mode &= ~(MODE_MONO_LUMA);
		dcon->disp_mode |= MODE_CSWIZZLE | MODE_COL_AA;
	}

	dcon_write(dcon, DCON_REG_MODE, dcon->disp_mode);
	return 0;
}

/* For now, this will be really stupid - we need to address how
 * DCONLOAD works in a sleep and account for it accordingly
 */

static void dcon_sleep(struct dcon_priv *dcon, bool sleep)
{
	int x;

	/* Turn off the backlight and put the DCON to sleep */

	if (dcon->asleep == sleep)
		return;

	//if (!olpc_board_at_least(olpc_board(0xc2)))
	//	return;

	if (sleep) {
		u8 pm = 0;

		x = olpc_ec_cmd(EC_DCON_POWER_MODE, &pm, 1, NULL, 0);
		if (x)
			pr_warn("unable to force dcon to power down: %d!\n", x);
		else
			dcon->asleep = sleep;
	} else {
		/* Only re-enable the backlight if the backlight value is set */
		if (dcon->bl_val != 0)
			dcon->disp_mode |= MODE_BL_ENABLE;
		x = dcon_bus_stabilize(dcon, 1);
		if (x)
			pr_warn("unable to reinit dcon hardware: %d!\n", x);
		else
			dcon->asleep = sleep;

		/* Restore backlight */
		dcon_set_backlight(dcon, dcon->bl_val);
	}

	/* We should turn off some stuff in the framebuffer - but what? */
}

/* the DCON seems to get confused if we change DCONLOAD too
 * frequently -- i.e., approximately faster than frame time.
 * normally we don't change it this fast, so in general we won't
 * delay here.
 */
static void dcon_load_holdoff(struct dcon_priv *dcon)
{
	ktime_t delta_t, now;

	while (1) {
		now = ktime_get();
		delta_t = ktime_sub(now, dcon->load_time);
		if (ktime_to_ns(delta_t) > NSEC_PER_MSEC * 20)
			break;
		mdelay(4);
	}
}

static bool dcon_blank_fb(struct dcon_priv *dcon, bool blank)
{
	int err;

	console_lock();
	if (!lock_fb_info(dcon->fbinfo)) {
		console_unlock();
		dev_err(&dcon->client->dev, "unable to lock framebuffer\n");
		return false;
	}

	dcon->ignore_fb_events = true;
	err = fb_blank(dcon->fbinfo,
		       blank ? FB_BLANK_POWERDOWN : FB_BLANK_UNBLANK);
	dcon->ignore_fb_events = false;
	unlock_fb_info(dcon->fbinfo);
	console_unlock();

	if (err) {
		dev_err(&dcon->client->dev, "couldn't %sblank framebuffer\n",
			blank ? "" : "un");
		return false;
	}
	return true;
}

/* Set the source of the display (CPU or DCON) */
static void dcon_source_switch(struct work_struct *work)
{
	struct dcon_priv *dcon = container_of(work, struct dcon_priv,
			switch_source);
	int source = dcon->pending_src;

	if (dcon->curr_src == source)
		return;

	dcon_load_holdoff(dcon);

	dcon->switched = false;

	switch (source) {
	case DCON_SOURCE_CPU:
		pr_info("%s to CPU\n", __func__);
		/* Enable the scanline interrupt bit */
		if (dcon_write(dcon, DCON_REG_MODE,
			       dcon->disp_mode | MODE_SCAN_INT))
			pr_err("couldn't enable scanline interrupt!\n");
		else
			/* Wait up to one second for the scanline interrupt */
			wait_event_timeout(dcon->waitq, dcon->switched, HZ);

		if (!dcon->switched)
			pr_err("Timeout entering CPU mode; expect a screen glitch.\n");

		/* Turn off the scanline interrupt */
		if (dcon_write(dcon, DCON_REG_MODE, dcon->disp_mode))
			pr_err("couldn't disable scanline interrupt!\n");

		/*
		 * Ideally we'd like to disable interrupts here so that the
		 * fb unblanking and DCON turn on happen at a known time value;
		 * however, we can't do that right now with fb_blank
		 * messing with semaphores.
		 *
		 * For now, we just hope..
		 */
		if (!dcon_blank_fb(dcon, false)) {
			pr_err("Failed to enter CPU mode\n");
			dcon->pending_src = DCON_SOURCE_DCON;
			return;
		}

		/* And turn off the DCON */
		//pdata->set_dconload(1);
		dcon_set_dconload_1_75(1);
		dcon->load_time = ktime_get();

		pr_info("The CPU has control\n");
		break;
	case DCON_SOURCE_DCON:
	{
		ktime_t delta_t;

		pr_info("%s to DCON\n", __func__);

		/* Clear DCONLOAD - this implies that the DCON is in control */
		//pdata->set_dconload(0);
		dcon_set_dconload_1_75(0);
		dcon->load_time = ktime_get();

		wait_event_timeout(dcon->waitq, dcon->switched, HZ / 2);

		if (!dcon->switched) {
			pr_err("Timeout entering DCON mode; expect a screen glitch.\n");
		} else {
			/* sometimes the DCON doesn't follow its own rules,
			 * and doesn't wait for two vsync pulses before
			 * ack'ing the frame load with an IRQ.  the result
			 * is that the display shows the *previously*
			 * loaded frame.  we can detect this by looking at
			 * the time between asserting DCONLOAD and the IRQ --
			 * if it's less than 20msec, then the DCON couldn't
			 * have seen two VSYNC pulses.  in that case we
			 * deassert and reassert, and hope for the best.
			 * see http://dev.laptop.org/ticket/9664
			 */
			delta_t = ktime_sub(dcon->irq_time, dcon->load_time);
			if (dcon->switched && ktime_to_ns(delta_t)
			    < NSEC_PER_MSEC * 20) {
				pr_err("missed loading, retrying\n");
				//pdata->set_dconload(1);
				dcon_set_dconload_1_75(1);
				mdelay(41);
				//pdata->set_dconload(0);
				dcon_set_dconload_1_75(0);
				dcon->load_time = ktime_get();
				mdelay(41);
			}
		}

		dcon_blank_fb(dcon, true);
		pr_info("The DCON has control\n");
		break;
	}
	default:
		BUG();
	}

	dcon->curr_src = source;
}

static void dcon_set_source(struct dcon_priv *dcon, int arg)
{
	if (dcon->pending_src == arg)
		return;

	dcon->pending_src = arg;

	if (dcon->curr_src != arg)
		schedule_work(&dcon->switch_source);
}

static void dcon_set_source_sync(struct dcon_priv *dcon, int arg)
{
	dcon_set_source(dcon, arg);
	flush_scheduled_work();
}

static ssize_t dcon_mode_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct dcon_priv *dcon = dev_get_drvdata(dev);

	return sprintf(buf, "%4.4X\n", dcon->disp_mode);
}

static ssize_t dcon_sleep_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct dcon_priv *dcon = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", dcon->asleep);
}

static ssize_t dcon_freeze_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct dcon_priv *dcon = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", dcon->curr_src == DCON_SOURCE_DCON ? 1 : 0);
}

static ssize_t dcon_mono_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct dcon_priv *dcon = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", dcon->mono);
}

static ssize_t dcon_resumeline_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	return sprintf(buf, "%d\n", resumeline);
}

static ssize_t dcon_mono_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned long enable_mono;
	int rc;

	rc = kstrtoul(buf, 10, &enable_mono);
	if (rc)
		return rc;

	dcon_set_mono_mode(dev_get_drvdata(dev), enable_mono ? true : false);

	return count;
}

static ssize_t dcon_freeze_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct dcon_priv *dcon = dev_get_drvdata(dev);
	unsigned long output;
	int ret;

	ret = kstrtoul(buf, 10, &output);
	if (ret)
		return ret;

	switch (output) {
	case 0:
		dcon_set_source(dcon, DCON_SOURCE_CPU);
		break;
	case 1:
		dcon_set_source_sync(dcon, DCON_SOURCE_DCON);
		break;
	case 2:  /* normally unused */
		dcon_set_source(dcon, DCON_SOURCE_DCON);
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static ssize_t dcon_resumeline_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned short rl;
	int rc;

	rc = kstrtou16(buf, 10, &rl);
	if (rc)
		return rc;

	resumeline = rl;
	dcon_write(dev_get_drvdata(dev), DCON_REG_SCAN_INT, resumeline);

	return count;
}

static ssize_t dcon_sleep_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long output;
	int ret;

	ret = kstrtoul(buf, 10, &output);
	if (ret)
		return ret;

	dcon_sleep(dev_get_drvdata(dev), output ? true : false);
	return count;
}

static struct device_attribute dcon_device_files[] = {
	__ATTR(mode, 0444, dcon_mode_show, NULL),
	__ATTR(sleep, 0644, dcon_sleep_show, dcon_sleep_store),
	__ATTR(freeze, 0644, dcon_freeze_show, dcon_freeze_store),
	__ATTR(monochrome, 0644, dcon_mono_show, dcon_mono_store),
	__ATTR(resumeline, 0644, dcon_resumeline_show, dcon_resumeline_store),
};

static int dcon_bl_update(struct backlight_device *dev)
{
	struct dcon_priv *dcon = bl_get_data(dev);
	u8 level = dev->props.brightness & 0x0F;

	if (dev->props.power != FB_BLANK_UNBLANK)
		level = 0;

	if (level != dcon->bl_val)
		dcon_set_backlight(dcon, level);

	/* power down the DCON when the screen is blanked */
	if (!dcon->ignore_fb_events)
		dcon_sleep(dcon, !!(dev->props.state & BL_CORE_FBBLANK));

	return 0;
}

static int dcon_bl_get(struct backlight_device *dev)
{
	struct dcon_priv *dcon = bl_get_data(dev);

	return dcon->bl_val;
}

static const struct backlight_ops dcon_bl_ops = {
	.update_status = dcon_bl_update,
	.get_brightness = dcon_bl_get,
};

static struct backlight_properties dcon_bl_props = {
	.max_brightness = 15,
	.type = BACKLIGHT_RAW,
	.power = FB_BLANK_UNBLANK,
};

static int dcon_reboot_notify(struct notifier_block *nb,
			      unsigned long foo, void *bar)
{
	struct dcon_priv *dcon = container_of(nb, struct dcon_priv, reboot_nb);

	if (!dcon || !dcon->client)
		return NOTIFY_DONE;

	/* Turn off the DCON. Entirely. */
	dcon_write(dcon, DCON_REG_MODE, 0x39);
	dcon_write(dcon, DCON_REG_MODE, 0x32);
	return NOTIFY_DONE;
}

static int unfreeze_on_panic(struct notifier_block *nb,
			     unsigned long e, void *p)
{
	//pdata->set_dconload(1);
	dcon_set_dconload_1_75(1);
	return NOTIFY_DONE;
}

static struct notifier_block dcon_panic_nb = {
	.notifier_call = unfreeze_on_panic,
};

static int hx8837_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strlcpy(info->type, "hx8837", I2C_NAME_SIZE);

	return 0;
}

static int hx8837_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct dcon_priv *dcon;
	int rc, i, j;

	dcon = kzalloc(sizeof(*dcon), GFP_KERNEL);
	if (!dcon)
		return -ENOMEM;

	dcon->client = client;
	init_waitqueue_head(&dcon->waitq);
	INIT_WORK(&dcon->switch_source, dcon_source_switch);
	dcon->reboot_nb.notifier_call = dcon_reboot_notify;
	dcon->reboot_nb.priority = -1;

	i2c_set_clientdata(client, dcon);

	if (num_registered_fb < 1) {
		dev_err(&client->dev, "DCON driver requires a registered fb\n");
		rc = -EIO;
		goto einit;
	}
	dcon->fbinfo = registered_fb[0];

	rc = dcon_hw_init(dcon, 1);
	if (rc)
		goto einit;

	/* Add the DCON device */

	dcon_device = platform_device_alloc("dcon", -1);

	if (!dcon_device) {
		pr_err("Unable to create the DCON device\n");
		rc = -ENOMEM;
		goto eirq;
	}
	rc = platform_device_add(dcon_device);
	platform_set_drvdata(dcon_device, dcon);

	if (rc) {
		pr_err("Unable to add the DCON device\n");
		goto edev;
	}

	for (i = 0; i < ARRAY_SIZE(dcon_device_files); i++) {
		rc = device_create_file(&dcon_device->dev,
					&dcon_device_files[i]);
		if (rc) {
			dev_err(&dcon_device->dev, "Cannot create sysfs file\n");
			goto ecreate;
		}
	}

	dcon->bl_val = dcon_read(dcon, DCON_REG_BRIGHT) & 0x0F;

	/* Add the backlight device for the DCON */
	dcon_bl_props.brightness = dcon->bl_val;
	dcon->bl_dev = backlight_device_register("dcon-bl", &dcon_device->dev,
						 dcon, &dcon_bl_ops,
						 &dcon_bl_props);
	if (IS_ERR(dcon->bl_dev)) {
		dev_err(&client->dev, "cannot register backlight dev (%ld)\n",
			PTR_ERR(dcon->bl_dev));
		dcon->bl_dev = NULL;
	}

	register_reboot_notifier(&dcon->reboot_nb);
	atomic_notifier_chain_register(&panic_notifier_list, &dcon_panic_nb);

	return 0;

 ecreate:
	for (j = 0; j < i; j++)
		device_remove_file(&dcon_device->dev, &dcon_device_files[j]);
 edev:
	platform_device_unregister(dcon_device);
	dcon_device = NULL;
 eirq:
	free_irq(DCON_IRQ, dcon);
 einit:
	kfree(dcon);
	return rc;
}

static int hx8837_remove(struct i2c_client *client)
{
	struct dcon_priv *dcon = i2c_get_clientdata(client);

	unregister_reboot_notifier(&dcon->reboot_nb);
	atomic_notifier_chain_unregister(&panic_notifier_list, &dcon_panic_nb);

	free_irq(DCON_IRQ, dcon);

	backlight_device_unregister(dcon->bl_dev);

	if (dcon_device)
		platform_device_unregister(dcon_device);
	cancel_work_sync(&dcon->switch_source);

	kfree(dcon);

	return 0;
}

#ifdef CONFIG_PM
static int hx8837_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dcon_priv *dcon = i2c_get_clientdata(client);

	if (!dcon->asleep) {
		/* Set up the DCON to have the source */
		dcon_set_source_sync(dcon, DCON_SOURCE_DCON);
	}

	return 0;
}

static int hx8837_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dcon_priv *dcon = i2c_get_clientdata(client);

	if (!dcon->asleep) {
		dcon_bus_stabilize(dcon, 0);
		dcon_set_source(dcon, DCON_SOURCE_CPU);
	}

	return 0;
}

#else

#define dcon_suspend NULL
#define dcon_resume NULL

#endif /* CONFIG_PM */

irqreturn_t dcon_interrupt(int irq, void *id)
{
	struct dcon_priv *dcon = id;
	u8 status;

	//if (pdata->read_status(&status))
	//	return IRQ_NONE;
	status = dcon_read_status_xo_1_75();

	switch (status & 3) {
	case 3:
		pr_debug("DCONLOAD_MISSED interrupt\n");
		break;

	case 2:	/* switch to DCON mode */
	case 1: /* switch to CPU mode */
		dcon->switched = true;
		dcon->irq_time = ktime_get();
		wake_up(&dcon->waitq);
		break;

	case 0:
		/* workaround resume case:  the DCON (on 1.5) doesn't
		 * ever assert status 0x01 when switching to CPU mode
		 * during resume.  this is because DCONLOAD is de-asserted
		 * _immediately_ upon exiting S3, so the actual release
		 * of the DCON happened long before this point.
		 * see http://dev.laptop.org/ticket/9869
		 */
		if (dcon->curr_src != dcon->pending_src && !dcon->switched) {
			dcon->switched = true;
			dcon->irq_time = ktime_get();
			wake_up(&dcon->waitq);
			pr_debug("switching w/ status 0/0\n");
		} else {
			pr_debug("scanline interrupt w/CPU\n");
		}
	}

	return IRQ_HANDLED;
}

static const struct dev_pm_ops hx8837_pm_ops = {
	.suspend = hx8837_suspend,
	.resume = hx8837_resume,
};




static const struct of_device_id hx8837_dt_ids[] = {
	{ .compatible = "himax,hx8837", },
	{ }
};
MODULE_DEVICE_TABLE(of, hx8837_dt_ids);

static const struct i2c_device_id hx8837_i2c_ids[] = {
	{ .name = "hx8837", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hx8837_i2c_ids);




static struct i2c_driver hx8837_driver = {
	.driver = {
		.name = "hx8837",
		.pm = &hx8837_pm_ops,
		.of_match_table = hx8837_dt_ids,
	},
	.class = I2C_CLASS_DDC | I2C_CLASS_HWMON,
	.id_table = hx8837_i2c_ids,
	.probe = hx8837_probe,
	.remove = hx8837_remove,
	.detect = hx8837_detect,
	.address_list = normal_i2c,
};

module_i2c_driver(hx8837_driver);

MODULE_DESCRIPTION("HX8837 Display Controller Driver");
MODULE_LICENSE("GPL");
