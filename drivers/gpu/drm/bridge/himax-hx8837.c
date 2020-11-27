// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * HiMax HX8837 Display Controller Driver
 *
 * Datasheet: http://wiki.laptop.org/images/0/09/DCON_datasheet_HX8837-A.pdf
 *
 * Copyright (C) 2020 Lubomir Rintel
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define bridge_to_hx8837_priv(x) \
	container_of(x, struct hx8837_priv, bridge)

/* DCON registers */
enum {
	DCON_REG_ID		= 0x00,
	DCON_REG_MODE		= 0x01,
	DCON_REG_HRES		= 0x02,
	DCON_REG_HTOTAL		= 0x03,
	DCON_REG_HSYNC_WIDTH	= 0x04,
	DCON_REG_VRES		= 0x05,
	DCON_REG_VTOTAL		= 0x06,
	DCON_REG_VSYNC_WIDTH	= 0x07,
	DCON_REG_TIMEOUT	= 0x08,
	DCON_REG_SCAN_INT	= 0x09,
	DCON_REG_BRIGHT		= 0x0a,
	DCON_REG_MEM_OPT_A	= 0x41,
	DCON_REG_MEM_OPT_B	= 0x42,
};

/* DCON_REG_MODE */
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

/* DCON_REG_BRIGHT */
enum {
	BRIGHT_MASK		= GENMASK(7, 0),
};

struct hx8837_priv {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *load_gpio;
	struct regulator_bulk_data supplies[4];

	struct drm_bridge *panel_bridge;
	struct drm_bridge bridge;
};

static int hx8837_bridge_attach(struct drm_bridge *bridge,
				enum drm_bridge_attach_flags flags)
{
	struct hx8837_priv *priv = bridge_to_hx8837_priv(bridge);

	return drm_bridge_attach(bridge->encoder, priv->panel_bridge,
				 bridge, flags);
}

static enum drm_mode_status hx8837_bridge_mode_valid(
				struct drm_bridge *bridge,
				const struct drm_display_info *info,
				const struct drm_display_mode *mode)
{
	if (mode->hdisplay > 0xffff)
		return MODE_BAD_HVALUE;
	if (mode->htotal > 0xffff)
		return MODE_BAD_HVALUE;
	if (mode->hsync_start - mode->hdisplay > 0xff)
		return MODE_HBLANK_WIDE;
	if (mode->hsync_end - mode->hsync_start > 0xff)
		return MODE_HSYNC_WIDE;
	if (mode->vdisplay > 0xffff)
		return MODE_BAD_VVALUE;
	if (mode->vtotal > 0xffff)
		return MODE_BAD_VVALUE;
	if (mode->vsync_start - mode->vdisplay > 0xff)
		return MODE_VBLANK_WIDE;
	if (mode->vsync_end - mode->vsync_start > 0xff)
		return MODE_VSYNC_WIDE;

	return MODE_OK;
}

static void hx8837_bridge_post_disable(struct drm_bridge *bridge)
{
	struct hx8837_priv *priv = bridge_to_hx8837_priv(bridge);

	printk("==== POST DISABLE ===\n");
	pm_runtime_put_sync(priv->dev);
}

static void hx8837_bridge_disable(struct drm_bridge *bridge)
{
	struct hx8837_priv *priv = bridge_to_hx8837_priv(bridge);
	int ret;

	printk("==== DISABLE ===\n");
	ret = gpiod_direction_output(priv->load_gpio, 0);
	if (ret)
		dev_err(priv->dev, "error disabling the dcon load: %d\n", ret);

	ret = regmap_update_bits(priv->regmap, DCON_REG_MODE,
					       MODE_PASSTHRU |
					       MODE_SLEEP,
					       MODE_PASSTHRU |
					       MODE_SLEEP);
	if (ret)
		dev_err(priv->dev, "error disabling the dcon: %d\n", ret);
}

static void hx8837_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct hx8837_priv *priv = bridge_to_hx8837_priv(bridge);

	printk("==== PRE ENABLE ===\n");
	pm_runtime_get_sync(priv->dev);
}

static void hx8837_bridge_enable(struct drm_bridge *bridge)
{
	struct hx8837_priv *priv = bridge_to_hx8837_priv(bridge);
	int ret;

	printk("==== ENABLE ===\n");
	ret = regmap_update_bits(priv->regmap, DCON_REG_MODE,
					       MODE_PASSTHRU |
					       MODE_SLEEP |
					       MODE_SLEEP_AUTO |
					       MODE_BLANK |
					       MODE_SCAN_INT |
					       MODE_CLOCKDIV |
					       MODE_DEBUG |
					       MODE_SELFTEST,
					       MODE_PASSTHRU);
	if (ret)
		dev_err(priv->dev, "error enabling the dcon: %d\n", ret);

	ret = gpiod_direction_output(priv->load_gpio, 1);
	if (ret)
		dev_err(priv->dev, "error enabling the dcon load: %d\n", ret);
}

static void hx8837_bridge_mode_set(struct drm_bridge *bridge,
			   const struct drm_display_mode *mode,
			   const struct drm_display_mode *adjusted_mode)
{
	struct hx8837_priv *priv = bridge_to_hx8837_priv(bridge);

	printk("==== MODE SET ===\n");
	regmap_write(priv->regmap, DCON_REG_HRES, mode->hdisplay);
	regmap_write(priv->regmap, DCON_REG_HTOTAL, mode->htotal);
	regmap_write(priv->regmap, DCON_REG_HSYNC_WIDTH,
			(mode->hsync_start - mode->hdisplay) << 8 |
			(mode->hsync_end - mode->hsync_start));
	regmap_write(priv->regmap, DCON_REG_VRES, mode->vdisplay);
	regmap_write(priv->regmap, DCON_REG_VTOTAL, mode->vtotal);
	regmap_write(priv->regmap, DCON_REG_VSYNC_WIDTH,
			(mode->vsync_start - mode->vdisplay) << 8 |
			(mode->vsync_end - mode->vsync_start));
}

static const struct drm_bridge_funcs hx8837_bridge_funcs = {
	.attach = hx8837_bridge_attach,
	.mode_valid = hx8837_bridge_mode_valid,
	.post_disable = hx8837_bridge_post_disable,
	.disable = hx8837_bridge_disable,
	.pre_enable = hx8837_bridge_pre_enable,
	.enable = hx8837_bridge_enable,
	.mode_set = hx8837_bridge_mode_set,
};

static int hx8837_bl_update_status(struct backlight_device *bl)
{
	struct hx8837_priv *priv = bl_get_data(bl);
	unsigned int val;
	int ret;

	ret = regmap_update_bits(priv->regmap, DCON_REG_BRIGHT,
					       BRIGHT_MASK,
					       backlight_get_brightness(bl));
	if (ret) {
		dev_err(&bl->dev, "error setting the backlight: %d\n", ret);
		return ret;
	}

	if (backlight_get_brightness(bl))
		val = MODE_CSWIZZLE | MODE_COL_AA;
	else
		val = MODE_MONO_LUMA;

	ret = regmap_update_bits(priv->regmap, DCON_REG_MODE,
					       MODE_CSWIZZLE |
					       MODE_COL_AA |
					       MODE_MONO_LUMA,
					       val);
	if (ret) {
		dev_err(&bl->dev, "error setting color mode: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct backlight_ops hx8837_bl_ops = {
	.update_status = hx8837_bl_update_status,
};

static const struct regmap_config hx8837_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0x4c,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int hx8837_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct backlight_properties bl_props = {
		.type = BACKLIGHT_RAW,
		.max_brightness = BRIGHT_MASK,
	};
	struct device *dev = &client->dev;
	struct backlight_device *bl;
	struct hx8837_priv *priv;
	struct drm_panel *panel;
	unsigned int val;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);
	priv->dev = dev;

	priv->supplies[0].supply = "vddp18";
	priv->supplies[1].supply = "vddm25";
	priv->supplies[2].supply = "vdd33";
	priv->supplies[3].supply = "vddk18";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(priv->supplies),
				      priv->supplies);
	if (ret)
		return ret;

	priv->load_gpio = devm_gpiod_get(dev, "load", GPIOD_ASIS);
	if (IS_ERR(priv->load_gpio))
		return PTR_ERR(priv->load_gpio);

	ret = drm_of_find_panel_or_bridge(dev->of_node, 1, -1, &panel, NULL);
	if (ret)
		return ret;

	if (panel->backlight) {
		dev_err(dev, "the panel already has a backlight controller\n");
		//return -ENODEV;
	}

	priv->panel_bridge = devm_drm_panel_bridge_add(dev, panel);
	if (IS_ERR(priv->panel_bridge))
		return PTR_ERR(priv->panel_bridge);

	//pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	priv->regmap = devm_regmap_init_i2c(client, &hx8837_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "regmap_init_i2c failed\n");
		ret = PTR_ERR(priv->regmap);
		goto error;
	}

	ret = regmap_read(priv->regmap, DCON_REG_ID, &val);
	if (ret < 0) {
		dev_err(dev, "error reading the model id: %d\n", ret);
		goto error;
	}
	if ((val & 0xff00) != 0xdc00) {
		dev_err(dev, "the device is not a hx8837\n");
		ret = -ENODEV;
		goto error;
	}

	ret = regmap_read(priv->regmap, DCON_REG_BRIGHT, &val);
	if (ret < 0) {
		dev_err(&bl->dev, "error getting the backlight: %d\n", ret);
		goto error;
	}
	bl_props.brightness = val & 0xf;

	bl = devm_backlight_device_register(dev, dev_name(dev), dev, priv,
					    &hx8837_bl_ops, &bl_props);
	if (IS_ERR(bl)) {
		dev_err(dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto error;
	}

	panel->backlight = bl;

	INIT_LIST_HEAD(&priv->bridge.list);
	priv->bridge.funcs = &hx8837_bridge_funcs;
	priv->bridge.of_node = dev->of_node;
	drm_bridge_add(&priv->bridge);

	pm_runtime_idle(dev);
	return 0;

error:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	return ret;
}

static int hx8837_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct hx8837_priv *priv = dev_get_drvdata(dev);

	drm_bridge_remove(&priv->bridge);

	printk("=== REMOVE ===\n");
	//pm_runtime_get_sync(dev);
	pm_runtime_disable(dev);
	//pm_runtime_set_suspended(dev);
	//pm_runtime_put_noidle(dev);

	return 0;
}

static int __maybe_unused hx8837_runtime_suspend(struct device *dev)
{
	struct hx8837_priv *priv = dev_get_drvdata(dev);
	int ret;

	printk("=== RUNTIME SUSPEND ===\n");
	ret = regulator_bulk_disable(ARRAY_SIZE(priv->supplies),
				     priv->supplies);
	if (ret)
		dev_err(priv->dev,"cannot disable regulators %d\n", ret);

	return ret;
}

static int __maybe_unused hx8837_runtime_resume(struct device *dev)
{
	struct hx8837_priv *priv = dev_get_drvdata(dev);
	int ret;

	printk("=== RUNTIME RESUME ===\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(priv->supplies),
				    priv->supplies);
	if (ret)
		dev_err(priv->dev,"cannot enable regulators %d\n", ret);

	return ret;
}

static const struct dev_pm_ops hx8837_pm_ops = {
        SET_RUNTIME_PM_OPS(hx8837_runtime_suspend,
			   hx8837_runtime_resume, NULL)
        SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
                                pm_runtime_force_resume)
};

static const struct of_device_id hx8837_dt_ids[] = {
	{ .compatible = "himax,hx8837", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hx8837_dt_ids);

static const struct i2c_device_id hx8837_ids[] = {
	{ "hx8837", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, hx8837_ids);

static struct i2c_driver hx8837_driver = {
	.probe = hx8837_probe,
	.remove = hx8837_remove,
	.driver = {
		.name = "hx8837",
		.of_match_table = of_match_ptr(hx8837_dt_ids),
		.pm = &hx8837_pm_ops,
	},
	.id_table = hx8837_ids,
};

module_i2c_driver(hx8837_driver);

MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_DESCRIPTION("HiMax HX8837 Display Controller Driver");
MODULE_LICENSE("GPL");
