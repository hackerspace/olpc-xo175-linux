#include <linux/component.h>
#include <linux/module.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

struct ch7033_priv {
        struct i2c_client *client;
	struct drm_encoder encoder;
	struct drm_bridge bridge;
	struct drm_connector connector;
};

#define conn_to_ch7033_priv(x) \
	container_of(x, struct ch7033_priv, connector)
#define enc_to_ch7033_priv(x) \
	container_of(x, struct ch7033_priv, encoder)
#define bridge_to_ch7033_priv(x) \
	container_of(x, struct ch7033_priv, bridge)

static enum drm_connector_status ch7033_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
	//return connector_status_disconnected;
}

static void ch7033_connector_destroy(struct drm_connector *connector)
{
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs ch7033_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = ch7033_connector_detect,
	.destroy = ch7033_connector_destroy,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int ch7033_connector_get_modes(struct drm_connector *connector)
{
	struct ch7033_priv *priv = conn_to_ch7033_priv(connector);
	struct edid *edid;
	int n;

	//edid = drm_do_get_edid(connector, read_edid_block, priv);
	edid = NULL;
	if (!edid) {
		dev_warn(&priv->client->dev, "failed to read EDID\n");
		return 0;
	}

	drm_connector_update_edid_property(connector, edid);

	n = drm_add_edid_modes(connector, edid);

	kfree(edid);

	return n;
}

static struct drm_encoder *ch7033_connector_best_encoder(struct drm_connector *connector)
{
	struct ch7033_priv *priv = conn_to_ch7033_priv(connector);

	return priv->bridge.encoder;
}

static const struct drm_connector_helper_funcs ch7033_connector_helper_funcs = {
	.get_modes = ch7033_connector_get_modes,
	.best_encoder = ch7033_connector_best_encoder,
};





static int ch7033_bridge_attach(struct drm_bridge *bridge)
{
	struct ch7033_priv *priv = bridge_to_ch7033_priv(bridge);
	struct drm_connector *connector = &priv->connector;
	int ret;

	connector->polled = DRM_CONNECTOR_POLL_CONNECT |
		DRM_CONNECTOR_POLL_DISCONNECT;

	drm_connector_helper_add(connector, &ch7033_connector_helper_funcs);
	ret = drm_connector_init(bridge->dev, connector, &ch7033_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret)
		return ret;

	drm_connector_attach_encoder(&priv->connector,
				     priv->bridge.encoder);

	return 0;
}

static void ch7033_bridge_detach(struct drm_bridge *bridge)
{
	struct ch7033_priv *priv = bridge_to_ch7033_priv(bridge);

	drm_connector_cleanup(&priv->connector);
}

static enum drm_mode_status ch7033_bridge_mode_valid(struct drm_bridge *bridge,
				     const struct drm_display_mode *mode)
{
//	if (mode->clock > 150000)
//		return MODE_CLOCK_HIGH;
//	if (mode->htotal >= BIT(13))
//		return MODE_BAD_HVALUE;
//	if (mode->vtotal >= BIT(11))
//		return MODE_BAD_VVALUE;
	return MODE_OK;
}

static void ch7033_bridge_enable(struct drm_bridge *bridge)
{
//	struct ch7033_priv *priv = bridge_to_ch7033_priv(bridge);
}

static void ch7033_bridge_disable(struct drm_bridge *bridge)
{
//	struct ch7033_priv *priv = bridge_to_ch7033_priv(bridge);
}

static void ch7033_bridge_mode_set(struct drm_bridge *bridge,
				    const struct drm_display_mode *mode,
				    const struct drm_display_mode *adjusted_mode)
{
}

static const struct drm_bridge_funcs ch7033_bridge_funcs = {
	.attach = ch7033_bridge_attach,
	.detach = ch7033_bridge_detach,
	.mode_valid = ch7033_bridge_mode_valid,
	.enable = ch7033_bridge_enable,
	.disable = ch7033_bridge_disable,
	.mode_set = ch7033_bridge_mode_set,
};







static void ch7033_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs ch7033_encoder_funcs = {
	.destroy = ch7033_encoder_destroy,
};





static int ch7033_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = data;
	struct ch7033_priv *priv = dev_get_drvdata(dev);
	u32 crtcs = 0;
	int ret;

	if (dev->of_node)
		crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);

	/* If no CRTCs were found, fall back to our old behaviour */
	if (crtcs == 0) {
		dev_warn(dev, "Falling back to first CRTC\n");
		crtcs = 1 << 0;
	}

	priv->encoder.possible_crtcs = crtcs;

	ret = drm_encoder_init(drm, &priv->encoder, &ch7033_encoder_funcs,
			       DRM_MODE_ENCODER_TMDS, NULL);
	if (ret)
		goto err_encoder;

	ret = drm_bridge_attach(&priv->encoder, &priv->bridge, NULL);
	if (ret)
		goto err_bridge;

	return 0;

err_bridge:
	drm_encoder_cleanup(&priv->encoder);
err_encoder:
	return ret;
}

static void ch7033_unbind(struct device *dev, struct device *master,
			   void *data)
{
	struct ch7033_priv *priv = dev_get_drvdata(dev);

	drm_encoder_cleanup(&priv->encoder);
}

static const struct component_ops ch7033_ops = {
	.bind = ch7033_bind,
	.unbind = ch7033_unbind,
};







static int ch7033_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ch7033_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);

	INIT_LIST_HEAD(&priv->bridge.list);

	priv->client = client;

	priv->bridge.funcs = &ch7033_bridge_funcs;
	priv->bridge.of_node = dev->of_node;

	drm_bridge_add(&priv->bridge);

	ret = component_add(dev, &ch7033_ops);
	if (ret)
		goto fail;

	return 0;

fail:
	drm_bridge_remove(&priv->bridge);
	return ret;
}

static int ch7033_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ch7033_priv *priv = dev_get_drvdata(dev);

	component_del(dev, &ch7033_ops);
	drm_bridge_remove(&priv->bridge);

	return 0;
}

static const struct of_device_id ch7033_dt_ids[] = {
	{ .compatible = "chrontel,ch7033", },
	{ }
};
MODULE_DEVICE_TABLE(of, ch7033_dt_ids);

static const struct i2c_device_id ch7033_ids[] = {
	{ "ch7033", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ch7033_ids);

static struct i2c_driver ch7033_driver = {
	.probe = ch7033_probe,
	.remove = ch7033_remove,
	.driver = {
		.name = "ch7033",
		.of_match_table = of_match_ptr(ch7033_dt_ids),
	},
	.id_table = ch7033_ids,
};

module_i2c_driver(ch7033_driver);

MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_DESCRIPTION("Chrontel CH7033 Encoder Driver");
MODULE_LICENSE("GPL v2");
