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
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/component.h>
#include <drm/drm_panel.h>
#include <drm/drm_encoder.h>
#include <drm/drm_connector.h>
#include <drm/drm_of.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>

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

struct hx8837_priv {
	struct i2c_client *client;
	struct backlight_device *bl_dev;

	wait_queue_head_t waitq;
	struct work_struct switch_source;
	struct notifier_block panic_nb;

	/* Scanline to interrupt on during resume */
	ushort resumeline;

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
	/* This get set while controlling fb blank state from the driver */
	bool ignore_fb_events;

	struct gpio_desc *stat0_gpio;
	struct gpio_desc *stat1_gpio;
	struct gpio_desc *load_gpio;

        struct drm_panel *panel;

	struct drm_encoder encoder;
	struct drm_connector connector;
};

static irqreturn_t hx8837_interrupt(int irq, void *id);

/* I2C structures */

static unsigned short normal_i2c[] = { 0x0d, I2C_CLIENT_END };

static s32 hx8837_write(struct hx8837_priv *priv, u8 reg, u16 val)
{
	return i2c_smbus_write_word_data(priv->client, reg, val);
}

static s32 hx8837_read(struct hx8837_priv *priv, u8 reg)
{
	return i2c_smbus_read_word_data(priv->client, reg);
}

static void hx8837_set_backlight(struct hx8837_priv *priv, u8 level)
{
	hx8837_write(priv, DCON_REG_BRIGHT, level);

	/* Purposely turn off the backlight when we go to level 0 */
	if (level == 0) {
		priv->disp_mode &= ~MODE_BL_ENABLE;
		hx8837_write(priv, DCON_REG_MODE, priv->disp_mode);
	} else if (!(priv->disp_mode & MODE_BL_ENABLE)) {
		priv->disp_mode |= MODE_BL_ENABLE;
		hx8837_write(priv, DCON_REG_MODE, priv->disp_mode);
	}
}

/* Set the output type to either color or mono */
static int hx8837_set_mono_mode(struct hx8837_priv *priv, bool enable_mono)
{
	if (priv->mono == enable_mono)
		return 0;

	priv->mono = enable_mono;

	if (enable_mono) {
		priv->disp_mode &= ~(MODE_CSWIZZLE | MODE_COL_AA);
		priv->disp_mode |= MODE_MONO_LUMA;
	} else {
		priv->disp_mode &= ~(MODE_MONO_LUMA);
		priv->disp_mode |= MODE_CSWIZZLE | MODE_COL_AA;
	}

	hx8837_write(priv, DCON_REG_MODE, priv->disp_mode);
	return 0;
}

/* the DCON seems to get confused if we change DCONLOAD too
 * frequently -- i.e., approximately faster than frame time.
 * normally we don't change it this fast, so in general we won't
 * delay here.
 */
static void hx8837_load_holdoff(struct hx8837_priv *priv)
{
	ktime_t delta_t, now;

	while (1) {
		now = ktime_get();
		delta_t = ktime_sub(now, priv->load_time);
		if (ktime_to_ns(delta_t) > NSEC_PER_MSEC * 20)
			break;
		mdelay(4);
	}
}

/* Set the source of the display (CPU or DCON) */
static void hx8837_source_switch(struct work_struct *work)
{
	struct hx8837_priv *priv = container_of(work, struct hx8837_priv,
							switch_source);
	struct device *dev = &priv->client->dev;
	int source = priv->pending_src;

	if (priv->curr_src == source)
		return;

	hx8837_load_holdoff(priv);

	priv->switched = false;

	switch (source) {
	case DCON_SOURCE_CPU:
		dev_info(dev, "%s to CPU\n", __func__);
		/* Enable the scanline interrupt bit */
		if (hx8837_write(priv, DCON_REG_MODE,
			       priv->disp_mode | MODE_SCAN_INT))
			dev_err(dev, "couldn't enable scanline interrupt!\n");
		else
			/* Wait up to one second for the scanline interrupt */
			wait_event_timeout(priv->waitq, priv->switched, HZ);

		if (!priv->switched)
			dev_err(dev, "Timeout entering CPU mode; expect a screen glitch.\n");

		/* Turn off the scanline interrupt */
		if (hx8837_write(priv, DCON_REG_MODE, priv->disp_mode))
			dev_err(dev, "couldn't disable scanline interrupt!\n");

		/* And turn off the DCON */
		gpiod_set_value(priv->load_gpio, 1);
		priv->load_time = ktime_get();

		dev_info(dev, "The CPU has control\n");
		break;
	case DCON_SOURCE_DCON:
		dev_info(dev, "%s to DCON\n", __func__);

		/* Clear DCONLOAD - this implies that the DCON is in control */
		gpiod_set_value(priv->load_gpio, 0);
		priv->load_time = ktime_get();

		wait_event_timeout(priv->waitq, priv->switched, HZ / 2);

		if (!priv->switched) {
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
			delta_t = ktime_sub(priv->irq_time, priv->load_time);
			if (priv->switched && ktime_to_ns(delta_t)
			    < NSEC_PER_MSEC * 20) {
				dev_err(dev, "missed loading, retrying\n");
				gpiod_set_value(priv->load_gpio, 1);
				mdelay(41);
				gpiod_set_value(priv->load_gpio, 0);
				priv->load_time = ktime_get();
				mdelay(41);
			}
		}

		dev_info(dev, "The DCON has control\n");
		break;
	default:
		WARN_ON(1);
	}

	priv->curr_src = source;
}

static void hx8837_set_source(struct hx8837_priv *priv, int arg)
{
	if (priv->pending_src == arg)
		return;

	priv->pending_src = arg;

	if (priv->curr_src != arg)
		schedule_work(&priv->switch_source);
}

static void hx8837_set_source_sync(struct hx8837_priv *priv, int arg)
{
	hx8837_set_source(priv, arg);
	flush_scheduled_work();
}

static int hx8837_bl_update(struct backlight_device *dev)
{
	struct hx8837_priv *priv = bl_get_data(dev);
	u8 level = dev->props.brightness & 0x0F;

	priv->bl_val = level;

	if (dev->props.power != FB_BLANK_UNBLANK)
		level = 0;

	if (dev->props.state & BL_CORE_FBBLANK)
		level = 0;

	hx8837_set_backlight(priv, level);

	return 0;
}

static int hx8837_bl_get(struct backlight_device *dev)
{
	struct hx8837_priv *priv = bl_get_data(dev);

	return priv->bl_val;
}

static const struct backlight_ops hx8837_bl_ops = {
	.update_status = hx8837_bl_update,
	.get_brightness = hx8837_bl_get,
};

static struct backlight_properties hx8837_bl_props = {
	.max_brightness = 15,
	.type = BACKLIGHT_RAW,
	.power = FB_BLANK_UNBLANK,
};

static int hx8837_panic_notify(struct notifier_block *nb, unsigned long e,
								void *p)
{
	struct hx8837_priv *priv = container_of(nb, struct hx8837_priv,
								panic_nb);

	gpiod_set_value(priv->load_gpio, 1);
	return NOTIFY_DONE;
}

static int hx8837_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strlcpy(info->type, "hx8837", I2C_NAME_SIZE);

	return 0;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,
								char *buf)
{
	struct hx8837_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%4.4X\n", priv->disp_mode);
}

static ssize_t freeze_show(struct device *dev, struct device_attribute *attr,
								char *buf)
{
	struct hx8837_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", priv->curr_src == DCON_SOURCE_DCON ? 1 : 0);
}

static ssize_t monochrome_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct hx8837_priv *priv = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", priv->mono);
}

static ssize_t monochrome_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned long enable_mono;
	int rc;

	rc = kstrtoul(buf, 10, &enable_mono);
	if (rc)
		return rc;

	hx8837_set_mono_mode(dev_get_drvdata(dev), enable_mono ? true : false);

	return count;
}

static ssize_t freeze_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct hx8837_priv *priv = dev_get_drvdata(dev);
	unsigned long output;
	int ret;

	ret = kstrtoul(buf, 10, &output);
	if (ret)
		return ret;

	switch (output) {
	case 0:
		hx8837_set_source(priv, DCON_SOURCE_CPU);
		break;
	case 1:
		hx8837_set_source_sync(priv, DCON_SOURCE_DCON);
		break;
	case 2:  /* normally unused */
		hx8837_set_source(priv, DCON_SOURCE_DCON);
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static DEVICE_ATTR_RO(mode);
static DEVICE_ATTR_RW(freeze);
static DEVICE_ATTR_RW(monochrome);

static struct attribute *hx8837_attrs[] = {
	&dev_attr_mode.attr,
	&dev_attr_freeze.attr,
	&dev_attr_monochrome.attr,
	NULL,
};
ATTRIBUTE_GROUPS(hx8837);

static void hx8837_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs hx8837_encoder_funcs = {
	.destroy = hx8837_encoder_destroy,
};

static void hx8837_encoder_helper_enable(struct drm_encoder *encoder)
{
	struct hx8837_priv *priv = container_of(encoder, struct hx8837_priv, encoder);

	priv->disp_mode |= MODE_BL_ENABLE;
	hx8837_write(priv, DCON_REG_MODE, priv->disp_mode);
}

static void hx8837_encoder_helper_disable(struct drm_encoder *encoder)
{
	struct hx8837_priv *priv = container_of(encoder, struct hx8837_priv, encoder);

	priv->disp_mode &= ~MODE_BL_ENABLE;
	hx8837_write(priv, DCON_REG_MODE, priv->disp_mode);
}

static const struct drm_encoder_helper_funcs hx8837_encoder_helper_funcs = {
	.enable = hx8837_encoder_helper_enable,
	.disable = hx8837_encoder_helper_disable,
};

static const struct drm_connector_funcs hx8837_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int hx8837_connector_get_modes(struct drm_connector *connector)
{
	struct hx8837_priv *priv = container_of(connector, struct hx8837_priv, connector);

	return priv->panel->funcs->get_modes(priv->panel);
}

static struct drm_encoder *
hx8837_connector_best_encoder(struct drm_connector *connector)
{
	struct hx8837_priv *priv = container_of(connector, struct hx8837_priv, connector);

	return &priv->encoder;
}

static const struct drm_connector_helper_funcs hx8837_connector_helper_funcs = {
	.get_modes = hx8837_connector_get_modes,
	.best_encoder = hx8837_connector_best_encoder,
};

static int hx8837_bind(struct device *dev, struct device *master, void *data)
{
	struct hx8837_priv *priv = dev_get_drvdata(dev);
	struct drm_device *drm = data;
	int ret;

	priv->encoder.possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);

	drm_encoder_helper_add(&priv->encoder, &hx8837_encoder_helper_funcs);
	ret = drm_encoder_init(drm, &priv->encoder, &hx8837_encoder_funcs, DRM_MODE_ENCODER_LVDS, NULL);
	if (ret) {
		dev_err(dev, "failed to init encoder\n");
		return ret;
	}

	priv->connector.dpms = DRM_MODE_DPMS_OFF;
	priv->connector.polled = 0;
	drm_connector_helper_add(&priv->connector, &hx8837_connector_helper_funcs);
	ret = drm_connector_init(drm, &priv->connector, &hx8837_connector_funcs, DRM_MODE_CONNECTOR_LVDS);
	if (ret) {
		dev_err(dev, "failed to init connector\n");
		return ret;
	}

	ret = drm_panel_attach(priv->panel, &priv->connector);
	if (ret) {
		dev_err(dev, "failed to attach panel\n");
		return ret;
	}

	ret = drm_connector_attach_encoder(&priv->connector, &priv->encoder);
	if (ret) {
		dev_err(dev, "failed to attach connector\n");
		return ret;
	}

	return 0;
}

static void hx8837_unbind(struct device *dev, struct device *master,
			   void *data)
{
	struct hx8837_priv *priv = dev_get_drvdata(dev);

	drm_encoder_cleanup(&priv->encoder);
}

static const struct component_ops hx8837_ops = {
	.bind = hx8837_bind,
	.unbind = hx8837_unbind,
};

static int hx8837_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct hx8837_priv *priv;
	struct drm_panel *panel;
	u16 ver;
	int rc;

	rc = drm_of_find_panel_or_bridge(client->dev.of_node, 1, 0, &panel, NULL);
	if (rc) {

		if (rc != -EPROBE_DEFER)
			dev_err(&client->dev, "no panel connected\n");
		return rc;
	}

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(&client->dev, priv);

	priv->client = client;
	init_waitqueue_head(&priv->waitq);
	INIT_WORK(&priv->switch_source, hx8837_source_switch);
	priv->panic_nb.notifier_call = hx8837_panic_notify;
	priv->panel = panel;

	i2c_set_clientdata(client, priv);

	priv->stat0_gpio = devm_gpiod_get_index(&client->dev, "stat", 0,
								GPIOD_IN);
	if (IS_ERR(priv->stat0_gpio)) {
		dev_err(&client->dev, "failed to request STAT0 GPIO\n");
		return PTR_ERR(priv->stat0_gpio);
	};

	priv->stat1_gpio = devm_gpiod_get_index(&client->dev, "stat", 1,
								GPIOD_IN);
	if (IS_ERR(priv->stat1_gpio)) {
		dev_err(&client->dev, "failed to request STAT1 GPIO\n");
		return PTR_ERR(priv->stat1_gpio);
	};

	priv->load_gpio = devm_gpiod_get(&client->dev, "load", GPIOD_IN);
	if (IS_ERR(priv->load_gpio)) {
		dev_err(&client->dev, "failed to request LOAD GPIO\n");
		return PTR_ERR(priv->load_gpio);
	};

	ver = hx8837_read(priv, DCON_REG_ID);
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
	priv->curr_src = gpiod_get_value(priv->load_gpio)
			? DCON_SOURCE_CPU : DCON_SOURCE_DCON;
	priv->pending_src = priv->curr_src;

	/* Set the directions for the GPIO pins */
	gpiod_direction_input(priv->stat0_gpio);
	gpiod_direction_input(priv->stat1_gpio);
	gpiod_direction_output(priv->load_gpio,
		priv->curr_src == DCON_SOURCE_CPU);

	/* Register the interrupt handler */
	rc = devm_request_irq(&client->dev, client->irq, &hx8837_interrupt, 0,
								"DCON", priv);
	if (rc) {
		dev_err(&client->dev, "IRQ request failed: %d\n", rc);
		return rc;
	}

	/* SDRAM setup/hold time */
	hx8837_write(priv, 0x3a, 0xc040);
	hx8837_write(priv, DCON_REG_MEM_OPT_A, 0x0000);  /* clear option bits */
	hx8837_write(priv, DCON_REG_MEM_OPT_A,
		   MEM_DLL_CLOCK_DELAY | MEM_POWER_DOWN);
	hx8837_write(priv, DCON_REG_MEM_OPT_B, MEM_SOFT_RESET);

	/* Colour swizzle, AA, no passthrough, backlight */
	priv->disp_mode = MODE_PASSTHRU | MODE_BL_ENABLE | MODE_CSWIZZLE | MODE_COL_AA;
	hx8837_write(priv, DCON_REG_MODE, priv->disp_mode);

	/* Set the scanline to interrupt on during resume */
	/* FIXME: Calculate this from the mode. */
	hx8837_write(priv, DCON_REG_SCAN_INT, 898);

	priv->bl_val = hx8837_read(priv, DCON_REG_BRIGHT) & 0x0F;

	/* Add the backlight device for the DCON */
	hx8837_bl_props.brightness = priv->bl_val;
	priv->bl_dev = devm_backlight_device_register(&client->dev, "hx8837-bl",
			&client->dev, priv, &hx8837_bl_ops, &hx8837_bl_props);
	if (IS_ERR(priv->bl_dev)) {
		dev_err(&client->dev, "cannot register backlight dev (%ld)\n",
			PTR_ERR(priv->bl_dev));
		priv->bl_dev = NULL;
	}

	rc = devm_device_add_groups(&client->dev, hx8837_groups);
	if (rc) {
		dev_err(&client->dev, "failed to register sysfs groups\n");
		return rc;
	}

        rc = component_add(&client->dev, &hx8837_ops);
        if (rc) {
		dev_err(&client->dev, "failed to register sysfs groups\n");
		return rc;
	}

	atomic_notifier_chain_register(&panic_notifier_list, &priv->panic_nb);

	return 0;
}

static int hx8837_remove(struct i2c_client *client)
{
	struct hx8837_priv *priv = i2c_get_clientdata(client);


	component_del(&client->dev, &hx8837_ops);



	atomic_notifier_chain_unregister(&panic_notifier_list,
					&priv->panic_nb);

	cancel_work_sync(&priv->switch_source);

	return 0;
}

static irqreturn_t hx8837_interrupt(int irq, void *id)
{
	struct hx8837_priv *priv = id;
	struct device *dev = &priv->client->dev;
	u8 status;

	status = gpiod_get_value(priv->stat0_gpio);
	status |= gpiod_get_value(priv->stat1_gpio) << 1;


	switch (status & 3) {
	case 3:
		dev_dbg(dev, "DCONLOAD_MISSED interrupt\n");
		break;

	case 2:	/* switch to DCON mode */
	case 1: /* switch to CPU mode */
		priv->switched = true;
		priv->irq_time = ktime_get();
		wake_up(&priv->waitq);
		break;

	case 0:
		/* workaround resume case:  the DCON (on 1.5) doesn't
		 * ever assert status 0x01 when switching to CPU mode
		 * during resume.  this is because DCONLOAD is de-asserted
		 * _immediately_ upon exiting S3, so the actual release
		 * of the DCON happened long before this point.
		 * see http://dev.laptop.org/ticket/9869
		 */
		if (priv->curr_src != priv->pending_src && !priv->switched) {
			priv->switched = true;
			priv->irq_time = ktime_get();
			wake_up(&priv->waitq);
			dev_dbg(dev, "switching w/ status 0/0\n");
		} else {
			dev_dbg(dev, "scanline interrupt w/CPU\n");
		}
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
static int hx8837_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct hx8837_priv *priv = i2c_get_clientdata(client);

	hx8837_set_source_sync(priv, DCON_SOURCE_DCON);

	return 0;
}

static int hx8837_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct hx8837_priv *priv = i2c_get_clientdata(client);

	hx8837_set_source(priv, DCON_SOURCE_CPU);

	return 0;
}
#endif /* CONFIG_PM */

static SIMPLE_DEV_PM_OPS(hx8837_pm_ops, hx8837_suspend, hx8837_resume);

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
