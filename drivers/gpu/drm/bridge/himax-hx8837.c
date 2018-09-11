// SPDX-License-Identifier: GPL-2.0
/*
 * Mainly by David Woodhouse, somewhat modified by Jordan Crouse.
 * Reqorked into a bridge driver and modernized to use managed GPIO,
 * device-tree, etc. by Lubomir Rintel.
 *
 * Copyright (C) 2006-2007  Red Hat, Inc.
 * Copyright (C) 2006-2007  Advanced Micro Devices, Inc.
 * Copyright (C) 2009	    VIA Technology, Inc.
 * Copyright (C) 2010-2011  Andres Salomon <dilinger@queued.net>
 * Copyright (C) 2018-2020  Lubomir Rintel <lkundrak@v3.sk>
 */

#include <linux/backlight.h>
#include <linux/component.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#define bridge_to_hx8837_priv(x) \
	container_of(x, struct hx8837_priv, bridge)
#define conn_to_hx8837_priv(x) \
	container_of(x, struct hx8837_priv, connector)

/* DCON registers */
enum {
	DCON_REG_ID		=  0,
	DCON_REG_MODE		=  1,
};

enum {
	MODE_PASSTHRU		= BIT(0),
	MODE_SLEEP		= BIT(1),
	MODE_SLEEP_AUTO		= BIT(2),
	MODE_BL_ENABLE		= BIT(3),
	MODE_BLANK		= BIT(4),
	MODE_CSWIZZLE		= BIT(5),
	MODE_COL_AA		= BIT(6),
	MODE_MONO_LUMA		= BIT(7),
	MODE_SCAN_INT		= BIT(8),
	MODE_CLOCKDIV		= BIT(9),
	MODE_DEBUG		= BIT(14),
	MODE_SELFTEST		= BIT(15),
};

enum {
	DCON_REG_HRES		= 0x2,
	DCON_REG_HTOTAL		= 0x3,
	DCON_REG_HSYNC_WIDTH	= 0x4,
	DCON_REG_VRES		= 0x5,
	DCON_REG_VTOTAL		= 0x6,
	DCON_REG_VSYNC_WIDTH	= 0x7,
	DCON_REG_TIMEOUT	= 0x8,
	DCON_REG_SCAN_INT	= 0x9,
	DCON_REG_BRIGHT		= 0xa,
	DCON_REG_MEM_OPT_A	= 0x41,
	DCON_REG_MEM_OPT_B	= 0x42,
};

enum {
	/* Load Delay Locked Loop (DLL) settings for clock delay */
	MEM_DLL_CLOCK_DELAY	= BIT(0),
	/* Memory controller power down function */
	MEM_POWER_DOWN		= BIT(8),
	/* Memory controller software reset */
	MEM_SOFT_RESET		= BIT(0),
};

/* Status values */
enum {
	DCONSTAT_SCANINT	= 0,
	DCONSTAT_SCANINT_DCON	= 1,
	DCONSTAT_DISPLAYLOAD	= 2,
	DCONSTAT_MISSED		= 3,
};

/* Source values */
enum {
	DCON_SOURCE_DCON	 = 0,
	DCON_SOURCE_CPU		 = 1,
};

struct hx8837_priv {
	struct i2c_client *client;
	struct backlight_device *bl_dev;

	wait_queue_head_t waitq;
	struct work_struct switch_source;
	struct notifier_block panic_nb;

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

	struct gpio_desc *stat0_gpio;
	struct gpio_desc *stat1_gpio;
	struct gpio_desc *load_gpio;

	struct regmap *regmap;
	struct drm_panel *panel;
	struct drm_encoder encoder;
	struct drm_bridge bridge;
	struct drm_connector connector;
};

static irqreturn_t hx8837_interrupt(int irq, void *id);

/* I2C structures */

static void hx8837_set_backlight(struct hx8837_priv *priv, u8 level)
{
	regmap_write(priv->regmap, DCON_REG_BRIGHT, level);

	/* Purposely turn off the backlight when we go to level 0 */
	if (level == 0) {
		priv->disp_mode &= ~MODE_BL_ENABLE;
		regmap_write(priv->regmap, DCON_REG_MODE, priv->disp_mode);
	} else if (!(priv->disp_mode & MODE_BL_ENABLE)) {
		priv->disp_mode |= MODE_BL_ENABLE;
		regmap_write(priv->regmap, DCON_REG_MODE, priv->disp_mode);
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

	regmap_write(priv->regmap, DCON_REG_MODE, priv->disp_mode);
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
		DRM_DEV_INFO(dev, "%s to CPU\n", __func__);
		/* Enable the scanline interrupt bit */
		if (regmap_write(priv->regmap, DCON_REG_MODE,
				 priv->disp_mode | MODE_SCAN_INT)) {
			DRM_DEV_ERROR(dev, "couldn't enable scanline interrupt!\n");
		} else {
			/* Wait up to one second for the scanline interrupt */
			wait_event_timeout(priv->waitq, priv->switched, HZ);
		}

		if (!priv->switched)
			DRM_DEV_ERROR(dev, "timeout entering CPU mode; expect a screen glitch.\n");

		/* Turn off the scanline interrupt */
		if (regmap_write(priv->regmap, DCON_REG_MODE,
				 priv->disp_mode))
			DRM_DEV_ERROR(dev, "couldn't disable scanline interrupt!\n");

		/* And turn off the DCON */
		gpiod_set_value(priv->load_gpio, 1);
		priv->load_time = ktime_get();

		DRM_DEV_INFO(dev, "The CPU has control\n");
		break;
	case DCON_SOURCE_DCON:
		DRM_DEV_INFO(dev, "%s to DCON\n", __func__);

		/* Clear DCONLOAD - this implies that the DCON is in control */
		gpiod_set_value(priv->load_gpio, 0);
		priv->load_time = ktime_get();

		wait_event_timeout(priv->waitq, priv->switched, HZ / 2);

		if (!priv->switched) {
			DRM_DEV_ERROR(dev, "Timeout entering DCON mode; expect a screen glitch.\n");
		} else {
			ktime_t delta_t;

			/* sometimes the DCON doesn't follow its own rules,
			 * and doesn't wait for two vsync pulses before
			 * ack'ing the frame load with an IRQ.	the result
			 * is that the display shows the *previously*
			 * loaded frame.  we can detect this by looking at
			 * the time between asserting DCONLOAD and the IRQ --
			 * if it's less than 20msec, then the DCON couldn't
			 * have seen two VSYNC pulses.	in that case we
			 * deassert and reassert, and hope for the best.
			 * see http://dev.laptop.org/ticket/9664
			 */
			delta_t = ktime_sub(priv->irq_time, priv->load_time);
			if (priv->switched && ktime_to_ns(delta_t)
			    < NSEC_PER_MSEC * 20) {
				DRM_DEV_ERROR(dev, "missed loading, retrying\n");
				gpiod_set_value(priv->load_gpio, 1);
				mdelay(41);
				gpiod_set_value(priv->load_gpio, 0);
				priv->load_time = ktime_get();
				mdelay(41);
			}
		}

		DRM_DEV_INFO(dev, "The DCON has control\n");
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
	u8 level = dev->props.brightness & 0x0f;

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
	case 2:	 /* normally unused */
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

static const struct drm_connector_funcs hx8837_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static irqreturn_t hx8837_interrupt(int irq, void *id)
{
	struct hx8837_priv *priv = id;
	struct device *dev = &priv->client->dev;
	u8 status;

	status = gpiod_get_value(priv->stat0_gpio);
	status |= gpiod_get_value(priv->stat1_gpio) << 1;


	switch (status & 3) {
	case 3:
		DRM_DEV_DEBUG(dev, "DCONLOAD_MISSED interrupt\n");
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
			DRM_DEV_DEBUG(dev, "switching w/ status 0/0\n");
		} else {
			DRM_DEV_DEBUG(dev, "scanline interrupt w/CPU\n");
		}
	}

	return IRQ_HANDLED;
}

static int hx8837_connector_get_modes(struct drm_connector *connector)
{
	struct hx8837_priv *priv = conn_to_hx8837_priv(connector);

	return priv->panel->funcs->get_modes(priv->panel, connector);
}

static struct drm_encoder *hx8837_connector_best_encoder(
			struct drm_connector *connector)
{
	struct hx8837_priv *priv = conn_to_hx8837_priv(connector);

	return priv->bridge.encoder;
}

static const struct drm_connector_helper_funcs hx8837_connector_helper_funcs = {
	.get_modes = hx8837_connector_get_modes,
	.best_encoder = hx8837_connector_best_encoder,
};

static int hx8837_bridge_attach(struct drm_bridge *bridge,
				enum drm_bridge_attach_flags flags)
{
	struct hx8837_priv *priv = bridge_to_hx8837_priv(bridge);
	struct device *dev = &priv->client->dev;
	int ret;

	ret = drm_panel_attach(priv->panel, &priv->connector);
	if (ret < 0)
		return ret;

	if (flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR)
		return 0;

	priv->connector.dpms = DRM_MODE_DPMS_OFF;
	priv->connector.polled = 0;
	drm_connector_helper_add(&priv->connector,
				 &hx8837_connector_helper_funcs);
	ret = drm_connector_init(bridge->dev, &priv->connector,
				 &hx8837_connector_funcs,
				 DRM_MODE_CONNECTOR_LVDS);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to initialize connector\n");
		return ret;
	}

	return drm_connector_attach_encoder(&priv->connector, &priv->encoder);
}

static void hx8837_bridge_detach(struct drm_bridge *bridge)
{
	struct hx8837_priv *priv = bridge_to_hx8837_priv(bridge);

	drm_connector_cleanup(&priv->connector);
}

static enum drm_mode_status hx8837_bridge_mode_valid(struct drm_bridge *bridge,
				     const struct drm_display_mode *mode)
{
//XXX
//	if (mode->clock > 165000)
//		return MODE_CLOCK_HIGH;
//	if (mode->hdisplay >= 1920)
//		return MODE_BAD_HVALUE;
//	if (mode->vdisplay >= 1080)
//		return MODE_BAD_VVALUE;
	return MODE_OK;
}

static void hx8837_bridge_disable(struct drm_bridge *bridge)
{
	struct hx8837_priv *priv = bridge_to_hx8837_priv(bridge);

	priv->disp_mode &= ~MODE_BL_ENABLE;
	regmap_write(priv->regmap, DCON_REG_MODE, priv->disp_mode);
}

static void hx8837_bridge_enable(struct drm_bridge *bridge)
{
	struct hx8837_priv *priv = bridge_to_hx8837_priv(bridge);

	priv->disp_mode |= MODE_BL_ENABLE;
	regmap_write(priv->regmap, DCON_REG_MODE, priv->disp_mode);
}

static void hx8837_bridge_mode_set(struct drm_bridge *bridge,
				   const struct drm_display_mode *mode,
				   const struct drm_display_mode *adjusted_mode)
{
	// XXXX
	//struct hx8837_priv *priv = bridge_to_hx8837_priv(bridge);
	//regmap_write(priv->regmap, 0x03, 0x04);
}

static const struct drm_bridge_funcs hx8837_bridge_funcs = {
	.attach = hx8837_bridge_attach,
	.detach = hx8837_bridge_detach,
	.mode_valid = hx8837_bridge_mode_valid,
	.disable = hx8837_bridge_disable,
	.enable = hx8837_bridge_enable,
	.mode_set = hx8837_bridge_mode_set,
};

static const struct drm_encoder_funcs hx8837_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int hx8837_bind(struct device *dev, struct device *master,
		       void *data)
{
	struct drm_device *drm = data;
	struct hx8837_priv *priv = dev_get_drvdata(dev);
	uint32_t crtcs;
	int ret;

	crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);
	priv->encoder.possible_crtcs = crtcs;

	ret = drm_encoder_init(drm, &priv->encoder, &hx8837_encoder_funcs,
			       DRM_MODE_ENCODER_LVDS, NULL);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to init encoder\n");
		return ret;
	}

	ret = drm_bridge_attach(&priv->encoder, &priv->bridge, NULL, 0);
	if (ret)
		drm_encoder_cleanup(&priv->encoder);

	return ret;
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

static int hx8837_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct hx8837_priv *priv = i2c_get_clientdata(client);

	component_del(dev, &hx8837_ops);
	drm_bridge_remove(&priv->bridge);

	atomic_notifier_chain_unregister(&panic_notifier_list, &priv->panic_nb);

	cancel_work_sync(&priv->switch_source);

	return 0;
}

static const struct regmap_config hx8837_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0x4c,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int hx8837_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct hx8837_priv *priv;
	unsigned int val;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);
	i2c_set_clientdata(client, priv);

	priv->client = client;
	init_waitqueue_head(&priv->waitq);
	INIT_WORK(&priv->switch_source, hx8837_source_switch);
	priv->panic_nb.notifier_call = hx8837_panic_notify;

	ret = drm_of_find_panel_or_bridge(dev->of_node, 1, -1,
					  &priv->panel, NULL);
	if (ret)
		return ret;

	priv->stat0_gpio = devm_gpiod_get_index(dev, "stat", 0, GPIOD_IN);
	if (IS_ERR(priv->stat0_gpio))
		return PTR_ERR(priv->stat0_gpio);

	priv->stat1_gpio = devm_gpiod_get_index(dev, "stat", 1, GPIOD_IN);
	if (IS_ERR(priv->stat1_gpio))
		return PTR_ERR(priv->stat1_gpio);

	priv->load_gpio = devm_gpiod_get(dev, "load", GPIOD_IN);
	if (IS_ERR(priv->load_gpio))
		return PTR_ERR(priv->load_gpio);


	priv->regmap = devm_regmap_init_i2c(client, &hx8837_regmap_config);
	if (IS_ERR(priv->regmap)) {
		DRM_DEV_ERROR(dev, "regmap init failed\n");
		return PTR_ERR(priv->regmap);
	}
	ret = regmap_read(priv->regmap, DCON_REG_ID, &val);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "error reading the model id: %d\n", ret);
		return ret;
	}
	if ((val >> 8) != 0xdc) {
		DRM_DEV_ERROR(dev, "DCON ID not 0xdcxx: 0x%04x.\n", val);
		return -ENODEV;
	}

	DRM_DEV_INFO(dev, "HiMax HX8837 Display Controller version %x\n", val & 0xff);

	if (val < 0xdc02) {
		DRM_DEV_ERROR(dev, "DCON v1 is unsupported, giving up.\n");
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
	ret = devm_request_irq(dev, client->irq, &hx8837_interrupt, 0,
			       "DCON", priv);
	if (ret) {
		DRM_DEV_ERROR(dev, "IRQ request failed: %d\n", ret);
		return ret;
	}

	/* SDRAM setup/hold time */
	regmap_write(priv->regmap, 0x3a, 0xc040);
	/* clear option bits */
	regmap_write(priv->regmap, DCON_REG_MEM_OPT_A, 0x0000);
	regmap_write(priv->regmap, DCON_REG_MEM_OPT_A, MEM_DLL_CLOCK_DELAY |
						       MEM_POWER_DOWN);
	regmap_write(priv->regmap, DCON_REG_MEM_OPT_B, MEM_SOFT_RESET);

	/* Colour swizzle, AA, no passthrough, backlight */
	priv->disp_mode = MODE_PASSTHRU | MODE_BL_ENABLE | MODE_CSWIZZLE |
			  MODE_COL_AA;
	regmap_write(priv->regmap, DCON_REG_MODE, priv->disp_mode);

	/* Set the scanline to interrupt on during resume */
	/* FIXME: Calculate this from the mode. */
	regmap_write(priv->regmap, DCON_REG_SCAN_INT, 898);

	ret = regmap_read(priv->regmap, DCON_REG_BRIGHT, &val);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "error reading the brightness: %d\n", ret);
		return ret;
	}


	priv->bl_val = val & 0x0f;

	/* Add the backlight device for the DCON */
	hx8837_bl_props.brightness = priv->bl_val;
	priv->bl_dev = devm_backlight_device_register(dev, "hx8837-bl",
			dev, priv, &hx8837_bl_ops, &hx8837_bl_props);
	if (IS_ERR(priv->bl_dev)) {
		DRM_DEV_ERROR(dev, "cannot register backlight dev (%ld)\n",
			PTR_ERR(priv->bl_dev));
		priv->bl_dev = NULL;
	}

	ret = devm_device_add_groups(dev, hx8837_groups);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to register sysfs groups\n");
		return ret;
	}

	INIT_LIST_HEAD(&priv->bridge.list);
	priv->bridge.funcs = &hx8837_bridge_funcs;
	priv->bridge.of_node = dev->of_node;
	drm_bridge_add(&priv->bridge);

	ret = component_add(dev, &hx8837_ops);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to add component\n");
		drm_bridge_remove(&priv->bridge);
		return ret;
	}

	atomic_notifier_chain_register(&panic_notifier_list, &priv->panic_nb);
	return 0;
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
	.probe = hx8837_probe,
	.remove = hx8837_remove,
	.driver = {
		.name = "hx8837",
		.pm = &hx8837_pm_ops,
		.of_match_table = of_match_ptr(hx8837_dt_ids),
	},
	.id_table = hx8837_i2c_ids,
};

module_i2c_driver(hx8837_driver);

MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_DESCRIPTION("HiMax HX8837 Display Controller Driver");
MODULE_LICENSE("GPL v2");
