// SPDX-License-Identifier: GPL-2.0
/*
 * Based on a driver by David Woodhouse and Jordan Crouse. Bugs are by
 * Lubomir Rintel who essentially rewrote it while turning it to a DRM
 * bridge driver.
 *
 * Copyright (C) 2006-2007 Red Hat, Inc.
 * Copyright (C) 2006-2007 Advanced Micro Devices, Inc.
 * Copyright (C) 2009 VIA Technology, Inc.
 * Copyright (C) 2010-2011 Andres Salomon <dilinger@queued.net>
 * Copyright (C) 2018-2019 Lubomir Rintel <lkundrak@v3.sk>
 */

#include <linux/module.h>
#include <linux/backlight.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <drm/drm_panel.h>
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

#define DCON_SOURCE_DCON	0
#define DCON_SOURCE_CPU		1

struct hx8837_priv {
	struct i2c_client *client;
	struct backlight_device *bl;
	struct drm_panel *panel;
	struct drm_bridge bridge;
	struct drm_connector connector;

	/* Shadow register for the DCON_REG_MODE register */
	u8 disp_mode;

	/* The current backlight value - this saves us some smbus traffic */
	u8 bl_val;
};

static inline struct hx8837_priv *bridge_to_hx8837(struct drm_bridge *bridge)
{
	return container_of(bridge, struct hx8837_priv, bridge);
}

static inline struct hx8837_priv *connector_to_hx8837(struct drm_connector
								*connector)
{
	return container_of(connector, struct hx8837_priv, connector);
}

static s32 hx8837_write(struct hx8837_priv *priv, u8 reg, u16 val)
{
	return i2c_smbus_write_word_data(priv->client, reg, val);
}

static s32 hx8837_read(struct hx8837_priv *priv, u8 reg)
{
	return i2c_smbus_read_word_data(priv->client, reg);
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

	hx8837_write(priv, DCON_REG_BRIGHT, level);

	/* Purposely turn off the backlight when we go to level 0 */
	/// XXX REFORMAT
	if (level == 0) {
		priv->disp_mode &= ~MODE_BL_ENABLE;
		hx8837_write(priv, DCON_REG_MODE, priv->disp_mode);
	} else if (!(priv->disp_mode & MODE_BL_ENABLE)) {
		priv->disp_mode |= MODE_BL_ENABLE;
		hx8837_write(priv, DCON_REG_MODE, priv->disp_mode);
	}

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

static void hx8837_pre_enable(struct drm_bridge *bridge)
{
	struct hx8837_priv *priv = bridge_to_hx8837(bridge);

	drm_panel_prepare(priv->panel);
}

static void hx8837_enable(struct drm_bridge *bridge)
{
	struct hx8837_priv *priv = bridge_to_hx8837(bridge);

	drm_panel_enable(priv->panel);
}

static void hx8837_disable(struct drm_bridge *bridge)
{
	struct hx8837_priv *priv = bridge_to_hx8837(bridge);

	drm_panel_disable(priv->panel);
}

static void hx8837_post_disable(struct drm_bridge *bridge)
{
	struct hx8837_priv *priv = bridge_to_hx8837(bridge);

	drm_panel_unprepare(priv->panel);
}

static int hx8837_get_modes(struct drm_connector *connector)
{
	struct hx8837_priv *priv = connector_to_hx8837(connector);

	return drm_panel_get_modes(priv->panel);
}

static const struct drm_connector_helper_funcs hx8837_connector_helper_funcs = {
	.get_modes = hx8837_get_modes,
};

static const struct drm_connector_funcs hx8837_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int hx8837_attach(struct drm_bridge *bridge)
{
	struct hx8837_priv *priv = bridge_to_hx8837(bridge);
	struct device *dev = &priv->client->dev;
	int ret;

	if (!bridge->encoder) {
		dev_err(dev, "Parent encoder object not found");
		return -ENODEV;
	}

	priv->connector.polled = DRM_CONNECTOR_POLL_HPD;
	ret = drm_connector_init(bridge->dev, &priv->connector,
			&hx8837_connector_funcs, DRM_MODE_CONNECTOR_LVDS);
	if (ret) {
		dev_err(dev, "Failed to initialize connector with drm\n");
		return ret;
	}
	drm_connector_helper_add(&priv->connector,
					&hx8837_connector_helper_funcs);
	drm_connector_register(&priv->connector);
	drm_connector_attach_encoder(&priv->connector, bridge->encoder);
	drm_panel_attach(priv->panel, &priv->connector);

	// XXX useless??
	drm_helper_hpd_irq_event(priv->connector.dev);

	return ret;
}

static const struct drm_bridge_funcs hx8837_bridge_funcs = {
	.pre_enable = hx8837_pre_enable,
	.enable = hx8837_enable,
	.disable = hx8837_disable,
	.post_disable = hx8837_post_disable,
	.attach = hx8837_attach,
};

static int hx8837_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct hx8837_priv *priv;
	int ret;
	u16 ver;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = drm_of_find_panel_or_bridge(dev->of_node, 1, 0, &priv->panel,
								NULL);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "no panel connected\n");
		return ret;
	}

	dev_set_drvdata(dev, priv);

	priv->client = client;
	i2c_set_clientdata(client, priv);

	ver = hx8837_read(priv, DCON_REG_ID);
	if ((ver >> 8) != 0xDC) {
		dev_err(dev, "DCON ID not 0xDCxx: 0x%04x.\n", ver);
		return -ENXIO;
	}

	dev_info(dev, "Discovered DCON version %x\n", ver & 0xFF);

	if (ver < 0xdc02) {
		dev_err(dev, "DCON v1 is unsupported, giving up.\n");
		return -ENODEV;
	}

	/* SDRAM setup/hold time */
	hx8837_write(priv, 0x3a, 0xc040);
	hx8837_write(priv, DCON_REG_MEM_OPT_A, 0x0000);  /* clear option bits */
	hx8837_write(priv, DCON_REG_MEM_OPT_A, MEM_DLL_CLOCK_DELAY
						| MEM_POWER_DOWN);
	hx8837_write(priv, DCON_REG_MEM_OPT_B, MEM_SOFT_RESET);

	/* Colour swizzle, AA, no passthrough, backlight */
	priv->disp_mode = MODE_PASSTHRU | MODE_BL_ENABLE;
	priv->disp_mode |= MODE_CSWIZZLE | MODE_COL_AA;
	hx8837_write(priv, DCON_REG_MODE, priv->disp_mode);

	/* Set the scanline to interrupt on during resume */
	/* FIXME: Calculate this from the mode. */
	hx8837_write(priv, DCON_REG_SCAN_INT, 898);

	priv->bl_val = hx8837_read(priv, DCON_REG_BRIGHT) & 0x0F;

	/* Add the backlight device for the DCON */
	hx8837_bl_props.brightness = priv->bl_val;
	priv->bl = devm_backlight_device_register(dev, "hx8837-bl",
			dev, priv, &hx8837_bl_ops, &hx8837_bl_props);
	if (IS_ERR(priv->bl)) {
		dev_err(dev, "cannot register backlight dev (%ld)\n",
			PTR_ERR(priv->bl));
		priv->bl = NULL;
	}

	priv->bridge.funcs = &hx8837_bridge_funcs;
	priv->bridge.of_node = dev->of_node;
	drm_bridge_add(&priv->bridge);

	return 0;
}

static int hx8837_remove(struct i2c_client *client)
{
	struct hx8837_priv *priv = i2c_get_clientdata(client);

	backlight_device_unregister(priv->bl);
	drm_bridge_remove(&priv->bridge);

	return 0;
}

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
		.of_match_table = hx8837_dt_ids,
	},
	.id_table = hx8837_i2c_ids,
	.probe = hx8837_probe,
	.remove = hx8837_remove,
};

module_i2c_driver(hx8837_driver);

MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_DESCRIPTION("HX8837 Display Controller Driver");
MODULE_LICENSE("GPL v2");
