// SPDX-License-Identifier: GPL-2.0
/*
 * Mainly by David Woodhouse, somewhat modified by Jordan Crouse.
 * Modernized to use managed GPIO, device-tree, etc. by Lubomir Rintel.
 *
 * Copyright (C) 2006-2007  Red Hat, Inc.
 * Copyright (C) 2006-2007  Advanced Micro Devices, Inc.
 * Copyright (C) 2009       VIA Technology, Inc.
 * Copyright (C) 2010-2011  Andres Salomon <dilinger@queued.net>
 * Copyright (C) 2018       Lubomir Rintel <lkundrak@v3.sk>
 */

#include <linux/module.h>
#include <linux/backlight.h>
#include <linux/console.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>
#include <linux/olpc-ec.h>
#include <linux/reboot.h>

/* DCON registers */

#define DCON_REG_ID		 0
#define DCON_REG_MODE		 1

#define MODE_PASSTHRU		BIT(0)
#define MODE_SLEEP		BIT(1)
#define MODE_SLEEP_AUTO		BIT(2)
#define MODE_BL_ENABLE		BIT(3)
#define MODE_BLANK		BIT(4)
#define MODE_CSWIZZLE		BIT(5)
#define MODE_COL_AA		BIT(6)
#define MODE_MONO_LUMA		BIT(7)
#define MODE_SCAN_INT		BIT(8)
#define MODE_CLOCKDIV		BIT(9)
#define MODE_DEBUG		BIT(14)
#define MODE_SELFTEST		BIT(15)

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

	struct gpio_desc *stat0_gpio;
	struct gpio_desc *stat1_gpio;
	struct gpio_desc *load_gpio;
	struct gpio_desc *irq_gpio;
};

static irqreturn_t dcon_interrupt(int irq, void *id);

#if 0
#define GPIO_SMBCLK	161
#define GPIO_SMBDATA	110

static void dcon_wiggle(void)
{
	int x;

	/*
	 * According to HiMax, when powering the DCON up we should hold
	 * SMB_DATA high for 8 SMB_CLK cycles.  This will force the DCON
	 * state machine to reset to a (sane) initial state.  Mitch Bradley
	 * did some testing and discovered that holding for 16 SMB_CLK cycles
	 * worked a lot more reliably, so that's what we do here.
	 */
	gpio_set_value(GPIO_SMBDATA, 1); /* should be high the entire time */

	for (x = 0; x < 16; x++) {
		udelay(5);
		gpio_set_value(GPIO_SMBCLK, 0);
		udelay(5);
		gpio_set_value(GPIO_SMBCLK, 1);
	}
	udelay(5);
}
#endif

/* Module definitions */

static ushort resumeline = 898;
module_param(resumeline, ushort, 0444);

/* I2C structures */

static unsigned short normal_i2c[] = { 0x0d, I2C_CLIENT_END };

static s32 dcon_write(struct dcon_priv *dcon, u8 reg, u16 val)
{
	return i2c_smbus_write_word_data(dcon->client, reg, val);
}

static s32 dcon_read(struct dcon_priv *dcon, u8 reg)
{
	return i2c_smbus_read_word_data(dcon->client, reg);
}

static void dcon_hw_init(struct dcon_priv *dcon, int is_init)
{
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
	struct device *dev = &dcon->client->dev;
	unsigned long timeout;
	u8 pm;
	int x;

power_up:
	if (is_powered_down) {
		pm = 1;
		x = olpc_ec_cmd(EC_DCON_POWER_MODE, &pm, 1, NULL, 0);
		if (x) {
			dev_warn(dev, "unable to power dcon up: %d!\n", x);
			return x;
		}
		usleep_range(10000, 11000);  /* we'll be conservative */
	}

	//dcon_wiggle();

	for (x = -1, timeout = 50; timeout && x < 0; timeout--) {
		usleep_range(1000, 1100);
		x = dcon_read(dcon, DCON_REG_ID);
	}
	if (x < 0) {
		dev_err(dev, "unable to stabilize smbus, reasserting power.\n");
		//BUG_ON(olpc_board_at_least(olpc_board(0xc2)));
		pm = 0;
		olpc_ec_cmd(EC_DCON_POWER_MODE, &pm, 1, NULL, 0);
		msleep(100);
		is_powered_down = 1;
		goto power_up;	/* argh, stupid hardware.. */
	}

	if (is_powered_down)
		dcon_hw_init(dcon, 0);
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
	struct device *dev = &dcon->client->dev;
	int x;

	/* Turn off the backlight and put the DCON to sleep */

	if (dcon->asleep == sleep)
		return;

	if (sleep) {
		u8 pm = 0;

		x = olpc_ec_cmd(EC_DCON_POWER_MODE, &pm, 1, NULL, 0);
		if (x)
			dev_warn(dev, "unable to power dcon down: %d!\n", x);
		else
			dcon->asleep = sleep;
	} else {
		/* Only re-enable the backlight if the backlight value is set */
		if (dcon->bl_val != 0)
			dcon->disp_mode |= MODE_BL_ENABLE;
		x = dcon_bus_stabilize(dcon, 1);
		if (x)
			dev_warn(dev, "unable to reinit dcon: %d!\n", x);
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
	struct device *dev = &dcon->client->dev;
	int source = dcon->pending_src;

	if (dcon->curr_src == source)
		return;

	dcon_load_holdoff(dcon);

	dcon->switched = false;

	switch (source) {
	case DCON_SOURCE_CPU:
		dev_info(dev, "%s to CPU\n", __func__);
		/* Enable the scanline interrupt bit */
		if (dcon_write(dcon, DCON_REG_MODE,
			       dcon->disp_mode | MODE_SCAN_INT))
			dev_err(dev, "couldn't enable scanline interrupt!\n");
		else
			/* Wait up to one second for the scanline interrupt */
			wait_event_timeout(dcon->waitq, dcon->switched, HZ);

		if (!dcon->switched)
			dev_err(dev, "Timeout entering CPU mode; expect a screen glitch.\n");

		/* Turn off the scanline interrupt */
		if (dcon_write(dcon, DCON_REG_MODE, dcon->disp_mode))
			dev_err(dev, "couldn't disable scanline interrupt!\n");

		/*
		 * Ideally we'd like to disable interrupts here so that the
		 * fb unblanking and DCON turn on happen at a known time value;
		 * however, we can't do that right now with fb_blank
		 * messing with semaphores.
		 *
		 * For now, we just hope..
		 */
		if (!dcon_blank_fb(dcon, false)) {
			dev_err(dev, "Failed to enter CPU mode\n");
			dcon->pending_src = DCON_SOURCE_DCON;
			return;
		}

		/* And turn off the DCON */
		gpiod_set_value(dcon->load_gpio, 1);
		dcon->load_time = ktime_get();

		dev_info(dev, "The CPU has control\n");
		break;
	case DCON_SOURCE_DCON:
		dev_info(dev, "%s to DCON\n", __func__);

		/* Clear DCONLOAD - this implies that the DCON is in control */
		gpiod_set_value(dcon->load_gpio, 0);
		dcon->load_time = ktime_get();

		wait_event_timeout(dcon->waitq, dcon->switched, HZ / 2);

		if (!dcon->switched) {
			dev_err(dev, "Timeout entering DCON mode; expect a screen glitch.\n");
		} else {
			ktime_t delta_t;

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
				dev_err(dev, "missed loading, retrying\n");
				gpiod_set_value(dcon->load_gpio, 1);
				mdelay(41);
				gpiod_set_value(dcon->load_gpio, 0);
				dcon->load_time = ktime_get();
				mdelay(41);
			}
		}

		dcon_blank_fb(dcon, true);
		dev_info(dev, "The DCON has control\n");
		break;
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
	struct device_attribute *attr, char *buf)
{
	struct dcon_priv *dcon = dev_get_drvdata(dev);

	return sprintf(buf, "%4.4X\n", dcon->disp_mode);
}

static ssize_t dcon_sleep_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dcon_priv *dcon = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", dcon->asleep);
}

static ssize_t dcon_freeze_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct dcon_priv *dcon = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", dcon->curr_src == DCON_SOURCE_DCON ? 1 : 0);
}

static ssize_t dcon_mono_show(struct device *dev,
	struct device_attribute *attr, char *buf)
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
	struct device_attribute *attr, const char *buf, size_t count)
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
	struct device_attribute *attr, const char *buf, size_t count)
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
	struct device_attribute *attr, const char *buf, size_t count)
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
	struct device_attribute *attr, const char *buf, size_t count)
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

static int dcon_reboot_notify(struct notifier_block *nb, unsigned long foo,
								void *bar)
{
	struct dcon_priv *dcon = container_of(nb, struct dcon_priv, reboot_nb);

	if (!dcon || !dcon->client)
		return NOTIFY_DONE;

	/* Turn off the DCON. Entirely. */
	dcon_write(dcon, DCON_REG_MODE, 0x39);
	dcon_write(dcon, DCON_REG_MODE, 0x32);
	return NOTIFY_DONE;
}

static int unfreeze_on_panic(struct notifier_block *nb, unsigned long e,
								void *p)
{
	struct dcon_priv *dcon = container_of(nb, struct dcon_priv, reboot_nb);

	gpiod_set_value(dcon->load_gpio, 1);
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

static int hx8837_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct dcon_priv *dcon;
	u16 ver;
	int rc, i;

	if (num_registered_fb < 1)
		return -EPROBE_DEFER;

	dcon = devm_kzalloc(&client->dev, sizeof(*dcon), GFP_KERNEL);
	if (!dcon)
		return -ENOMEM;

	dcon->client = client;
	init_waitqueue_head(&dcon->waitq);
	INIT_WORK(&dcon->switch_source, dcon_source_switch);
	dcon->reboot_nb.notifier_call = dcon_reboot_notify;
	dcon->reboot_nb.priority = -1;

	i2c_set_clientdata(client, dcon);

	dcon->fbinfo = registered_fb[0];

	dcon->stat0_gpio = devm_gpiod_get_index(&client->dev, "stat", 0, GPIOD_IN);
	if (IS_ERR(dcon->stat0_gpio)) {
		dev_err(&client->dev, "olpc-dcon: failed to request STAT0 GPIO\n");
		return PTR_ERR(dcon->stat0_gpio);
	};

	dcon->stat1_gpio = devm_gpiod_get_index(&client->dev, "stat", 1, GPIOD_IN);
	if (IS_ERR(dcon->stat1_gpio)) {
		dev_err(&client->dev, "olpc-dcon: failed to request STAT1 GPIO\n");
		return PTR_ERR(dcon->stat1_gpio);
	};

	dcon->load_gpio = devm_gpiod_get(&client->dev, "load", GPIOD_IN);
	if (IS_ERR(dcon->load_gpio)) {
		dev_err(&client->dev, "olpc-dcon: failed to request LOAD GPIO\n");
		return PTR_ERR(dcon->load_gpio);
	};

	dcon->irq_gpio = devm_gpiod_get(&client->dev, "irq", GPIOD_IN);
	if (IS_ERR(dcon->irq_gpio)) {
		dev_err(&client->dev, "olpc-dcon: failed to request IRQ GPIO\n");
		return PTR_ERR(dcon->irq_gpio);
	};

	ver = dcon_read(dcon, DCON_REG_ID);
	if ((ver >> 8) != 0xDC) {
		dev_err(&client->dev, "DCON ID not 0xDCxx: 0x%04x.\n", ver);
		return -ENXIO;
	}

	dev_info(&client->dev, "Discovered DCON version %x\n", ver & 0xFF);

	if (ver < 0xdc02) {
		dev_err(&client->dev, "DCON v1 is unsupported, giving up.\n");
		return -ENODEV;
	}

	/*
	 * Determine the current state by reading the GPIO bit; earlier
	 * stages of the boot process have established the state.
	 */
	dcon->curr_src = gpiod_get_value(dcon->load_gpio)
			? DCON_SOURCE_CPU : DCON_SOURCE_DCON;
	dcon->pending_src = dcon->curr_src;

	/* Set the directions for the GPIO pins */
	gpiod_direction_input(dcon->stat0_gpio);
	gpiod_direction_input(dcon->stat1_gpio);
	gpiod_direction_input(dcon->irq_gpio);
	gpiod_direction_output(dcon->load_gpio,
		dcon->curr_src == DCON_SOURCE_CPU);

	/* Register the interrupt handler */
	rc = devm_request_irq(&client->dev, gpiod_to_irq(dcon->irq_gpio),
			&dcon_interrupt, IRQF_TRIGGER_FALLING, "DCON", dcon);
	if (rc) {
		dev_err(&client->dev, "failed to request DCON's irq\n");
		return rc;
	}

	dcon_hw_init(dcon, 1);

	for (i = 0; i < ARRAY_SIZE(dcon_device_files); i++) {
		rc = device_create_file(&client->dev,
					&dcon_device_files[i]);
		if (rc) {
			dev_err(&client->dev, "Cannot create sysfs file\n");
			goto ecreate;
		}
	}

	dcon->bl_val = dcon_read(dcon, DCON_REG_BRIGHT) & 0x0F;

	/* Add the backlight device for the DCON */
	dcon_bl_props.brightness = dcon->bl_val;
	dcon->bl_dev = devm_backlight_device_register(&client->dev, "dcon-bl",
			&client->dev, dcon, &dcon_bl_ops, &dcon_bl_props);
	if (IS_ERR(dcon->bl_dev)) {
		dev_err(&client->dev, "cannot register backlight dev (%ld)\n",
			PTR_ERR(dcon->bl_dev));
		dcon->bl_dev = NULL;
	}

	devm_register_reboot_notifier(&client->dev, &dcon->reboot_nb);
	atomic_notifier_chain_register(&panic_notifier_list, &dcon_panic_nb);

	return 0;

ecreate:
	for (; i > 0; i--)
		device_remove_file(&client->dev, &dcon_device_files[i]);
	return rc;
}

static int hx8837_remove(struct i2c_client *client)
{
	struct dcon_priv *dcon = i2c_get_clientdata(client);

	atomic_notifier_chain_unregister(&panic_notifier_list, &dcon_panic_nb);

	cancel_work_sync(&dcon->switch_source);

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
	struct device *dev = &dcon->client->dev;
	u8 status;

	status = gpiod_get_value(dcon->stat0_gpio);
	status |= gpiod_get_value(dcon->stat1_gpio) << 1;


	switch (status & 3) {
	case 3:
		dev_dbg(dev, "DCONLOAD_MISSED interrupt\n");
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
			dev_dbg(dev, "switching w/ status 0/0\n");
		} else {
			dev_dbg(dev, "scanline interrupt w/CPU\n");
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
MODULE_LICENSE("GPL v2");
