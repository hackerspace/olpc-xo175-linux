// SPDX-License-Identifier: GPL-2.0+
/*
 * Marvell MMP3 HDMI Encoder Driver
 *
 * Copyright (C) 2020 Lubomir Rintel
 */

#include <linux/module.h>
#include <linux/platform_device.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

struct mmp3_hdmi_priv {
	void __iomem *base;
	struct drm_bridge *next_bridge;
	struct drm_bridge bridge;
	struct drm_connector connector;
};

#define conn_to_mmp3_hdmi_priv(x) \
	container_of(x, struct mmp3_hdmi_priv, connector)
#define bridge_to_mmp3_hdmi_priv(x) \
	container_of(x, struct mmp3_hdmi_priv, bridge)


static enum drm_connector_status mmp3_hdmi_connector_detect(
	struct drm_connector *connector, bool force)
{
	struct mmp3_hdmi_priv *priv = conn_to_mmp3_hdmi_priv(connector);

	return drm_bridge_detect(priv->next_bridge);
}

static const struct drm_connector_funcs mmp3_hdmi_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = mmp3_hdmi_connector_detect,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int mmp3_hdmi_connector_get_modes(struct drm_connector *connector)
{
	struct mmp3_hdmi_priv *priv = conn_to_mmp3_hdmi_priv(connector);
	struct edid *edid;
	int ret;

	edid = drm_bridge_get_edid(priv->next_bridge, connector);
	drm_connector_update_edid_property(connector, edid);
	if (edid) {
		ret = drm_add_edid_modes(connector, edid);
		kfree(edid);
	} else {
		ret = drm_add_modes_noedid(connector, 1920, 1080);
		drm_set_preferred_mode(connector, 1024, 768);
	}

	return ret;
}

static struct drm_encoder *mmp3_hdmi_connector_best_encoder(
			struct drm_connector *connector)
{
	struct mmp3_hdmi_priv *priv = conn_to_mmp3_hdmi_priv(connector);

	return priv->bridge.encoder;
}

static const struct drm_connector_helper_funcs mmp3_hdmi_connector_helper_funcs = {
	.get_modes = mmp3_hdmi_connector_get_modes,
	.best_encoder = mmp3_hdmi_connector_best_encoder,
};

static void mmp3_hdmi_hpd_event(void *arg, enum drm_connector_status status)
{
	struct mmp3_hdmi_priv *priv = arg;

	if (priv->bridge.dev)
		drm_helper_hpd_irq_event(priv->connector.dev);
}

static int mmp3_hdmi_bridge_attach(struct drm_bridge *bridge,
				enum drm_bridge_attach_flags flags)
{
	struct mmp3_hdmi_priv *priv = bridge_to_mmp3_hdmi_priv(bridge);
	struct drm_connector *connector = &priv->connector;
	int ret;

	ret = drm_bridge_attach(bridge->encoder, priv->next_bridge, bridge,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return ret;

	if (flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR)
		return 0;

	if (priv->next_bridge->ops & DRM_BRIDGE_OP_DETECT) {
		connector->polled = DRM_CONNECTOR_POLL_HPD;
	} else {
		connector->polled = DRM_CONNECTOR_POLL_CONNECT |
				    DRM_CONNECTOR_POLL_DISCONNECT;
	}

	if (priv->next_bridge->ops & DRM_BRIDGE_OP_HPD) {
		drm_bridge_hpd_enable(priv->next_bridge, mmp3_hdmi_hpd_event,
				      priv);
	}

	drm_connector_helper_add(connector,
				 &mmp3_hdmi_connector_helper_funcs);
	ret = drm_connector_init_with_ddc(bridge->dev, &priv->connector,
					  &mmp3_hdmi_connector_funcs,
					  priv->next_bridge->type,
					  priv->next_bridge->ddc);
	if (ret) {
		DRM_ERROR("Failed to initialize connector\n");
		return ret;
	}

	return drm_connector_attach_encoder(&priv->connector, bridge->encoder);
}

static void mmp3_hdmi_bridge_detach(struct drm_bridge *bridge)
{
	struct mmp3_hdmi_priv *priv = bridge_to_mmp3_hdmi_priv(bridge);

	if (priv->next_bridge->ops & DRM_BRIDGE_OP_HPD)
		drm_bridge_hpd_disable(priv->next_bridge);
	drm_connector_cleanup(&priv->connector);
}

static enum drm_mode_status mmp3_hdmi_bridge_mode_valid(struct drm_bridge *bridge,
				     const struct drm_display_info *info,
				     const struct drm_display_mode *mode)
{
#if 0
	if (mode->clock > 165000)
		return MODE_CLOCK_HIGH;
	if (mode->hdisplay >= 1920)
		return MODE_BAD_HVALUE;
	if (mode->vdisplay >= 1080)
		return MODE_BAD_VVALUE;
#endif
	return MODE_OK;
}

static void mmp3_hdmi_bridge_disable(struct drm_bridge *bridge)
{
//	struct mmp3_hdmi_priv *priv = bridge_to_mmp3_hdmi_priv(bridge);

//	regmap_write(priv->regmap, 0x03, 0x04);
//	regmap_update_bits(priv->regmap, 0x52, RESETDB, 0x00);
}

static void mmp3_hdmi_bridge_enable(struct drm_bridge *bridge)
{
//	struct mmp3_hdmi_priv *priv = bridge_to_mmp3_hdmi_priv(bridge);

//	regmap_write(priv->regmap, 0x03, 0x04);
//	regmap_update_bits(priv->regmap, 0x52, RESETDB, RESETDB);
}

static void mmp3_hdmi_bridge_mode_set(struct drm_bridge *bridge,
				   const struct drm_display_mode *mode,
				   const struct drm_display_mode *adjusted_mode)
{
	struct mmp3_hdmi_priv *priv = bridge_to_mmp3_hdmi_priv(bridge);

        writel(0x0000aad5, priv->base + 0x1c);
        writel(0x418084b8, priv->base + 0x24);
        writel(0x0240190b, priv->base + 0x2c);
}

static const struct drm_bridge_funcs mmp3_hdmi_bridge_funcs = {
	.attach = mmp3_hdmi_bridge_attach,
	.detach = mmp3_hdmi_bridge_detach,
	.mode_valid = mmp3_hdmi_bridge_mode_valid,
	.disable = mmp3_hdmi_bridge_disable,
	.enable = mmp3_hdmi_bridge_enable,
	.mode_set = mmp3_hdmi_bridge_mode_set,
};

static int mmp3_hdmi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mmp3_hdmi_priv *priv;
	struct resource *resource;
	int ret;

printk("XXX PROBE0\n");

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);

printk("XXX PROBE1\n");
	ret = drm_of_find_panel_or_bridge(dev->of_node, 1, -1, NULL,
					  &priv->next_bridge);
	if (ret)
		return ret;

printk("XXX PROBE2\n");
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, resource);
	if (IS_ERR(priv->base)) {
		dev_err(dev, "failed to remap HDMI regs\n");
		return PTR_ERR(priv->base);
	}

printk("XXX PROBE3\n");
	INIT_LIST_HEAD(&priv->bridge.list);
	priv->bridge.funcs = &mmp3_hdmi_bridge_funcs;
	priv->bridge.of_node = dev->of_node;
	drm_bridge_add(&priv->bridge);

printk("XXX PROBE4\n");
	dev_info(dev, "Marvell MMP3 HDMI Encoder\n");
	return 0;
}

static int mmp3_hdmi_remove(struct platform_device *pdev)
{
	struct mmp3_hdmi_priv *priv = platform_get_drvdata(pdev);

	drm_bridge_remove(&priv->bridge);

	return 0;
}

static const struct of_device_id mmp3_hdmi_of_match[] = {
	{ .compatible = "marvell,mmp3-hdmi", },
	{ },
};
MODULE_DEVICE_TABLE(of, mmp3_hdmi_of_match);

static struct platform_driver mmp3_hdmi_driver = {
	.probe = mmp3_hdmi_probe,
	.remove	= mmp3_hdmi_remove,
	.driver = {
		.name = "mmp3-hdmi",
		.of_match_table = mmp3_hdmi_of_match,
	},
};
module_platform_driver(mmp3_hdmi_driver);

MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_DESCRIPTION("Marvell MMP3 HDMI Encoder Driver");
MODULE_LICENSE("GPL");
