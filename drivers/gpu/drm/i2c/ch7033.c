#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/component.h>
#include <drm/drm_panel.h>
#include <drm/drm_encoder.h>
#include <drm/drm_connector.h>
#include <drm/drm_of.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_probe_helper.h>
#include <video/display_timing.h>

struct ch7033_priv {
	struct i2c_client *client;
	
        struct drm_panel *panel;

	struct drm_encoder encoder;
	struct drm_connector connector;
};

/* I2C structures */

static unsigned short normal_i2c[] = { 0x76, I2C_CLIENT_END };



static void ch7033_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs ch7033_encoder_funcs = {
	.destroy = ch7033_encoder_destroy,
};

static void ch7033_encoder_helper_enable(struct drm_encoder *encoder)
{
	//struct ch7033_priv *priv = container_of(encoder, struct ch7033_priv, encoder);
	//ch7033_write(priv, DCON_REG_MODE, priv->disp_mode);
}

static void ch7033_encoder_helper_disable(struct drm_encoder *encoder)
{
	//struct ch7033_priv *priv = container_of(encoder, struct ch7033_priv, encoder);
	//ch7033_write(priv, DCON_REG_MODE, priv->disp_mode);
}

static const struct drm_encoder_helper_funcs ch7033_encoder_helper_funcs = {
	.enable = ch7033_encoder_helper_enable,
	.disable = ch7033_encoder_helper_disable,
};

static const struct drm_connector_funcs ch7033_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_display_mode ariel_mode = {
	.clock = 65000,

	.hdisplay = 1024,
	.hsync_start = 1048,
	.hsync_end = 1184,
	.htotal = 1344,

	.vdisplay = 768,
	.vsync_start = 771,
	.vsync_end = 777,
	.vtotal = 806,

	.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
	.vrefresh = 60,
};

struct drm_display_mode mode17 = {
	.clock = 108000,
	//.clock = 65000,

	.hdisplay = 1280,
	.hsync_start = 1328,
	.hsync_end = 1440,
	.htotal = 1688,

	.vdisplay = 1024,
	.vsync_start = 1025,
	.vsync_end = 1028,
	.vtotal = 1066,

	.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
};

static int ch7033_connector_get_modes(struct drm_connector *connector)
{
//	struct ch7033_priv *priv = container_of(connector, struct ch7033_priv, connector);
	struct drm_device *dev = connector->dev;
	struct drm_display_mode *mode;

#if 1
	mode = drm_mode_duplicate(dev, &mode17);
	if (!mode) {
		dev_err(dev->dev, "failed to add mode %ux%u@%u\n",
			mode17.hdisplay, mode17.vdisplay, mode17.vrefresh);
		return -1;
	}
#endif
#if 0
	mode = drm_mode_duplicate(dev, &ariel_mode);
	if (!mode) {
		dev_err(dev->dev, "failed to add mode %ux%u@%u\n",
			ariel_mode.hdisplay, ariel_mode.vdisplay, ariel_mode.vrefresh);
		return -1;
	}
#endif

	mode->type |= DRM_MODE_TYPE_DRIVER;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);

	drm_mode_probed_add(connector, mode);
//	return priv->panel->funcs->get_modes(priv->panel);

	return 1;
}

static struct drm_encoder *
ch7033_connector_best_encoder(struct drm_connector *connector)
{
	struct ch7033_priv *priv = container_of(connector, struct ch7033_priv, connector);

	return &priv->encoder;
}

static const struct drm_connector_helper_funcs ch7033_connector_helper_funcs = {
	.get_modes = ch7033_connector_get_modes,
	.best_encoder = ch7033_connector_best_encoder,
};

static int ch7033_bind(struct device *dev, struct device *master, void *data)
{
	struct ch7033_priv *priv = dev_get_drvdata(dev);
	struct drm_device *drm = data;
	int ret;

	priv->encoder.possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);

	drm_encoder_helper_add(&priv->encoder, &ch7033_encoder_helper_funcs);
	ret = drm_encoder_init(drm, &priv->encoder, &ch7033_encoder_funcs, DRM_MODE_ENCODER_LVDS, NULL);
	if (ret) {
		dev_err(dev, "failed to init encoder\n");
		return ret;
	}

	priv->connector.dpms = DRM_MODE_DPMS_OFF;
	priv->connector.polled = 0;
	drm_connector_helper_add(&priv->connector, &ch7033_connector_helper_funcs);
	ret = drm_connector_init(drm, &priv->connector, &ch7033_connector_funcs, DRM_MODE_CONNECTOR_LVDS);
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

static int ch7033_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ch7033_priv *priv;
	struct drm_panel *panel;
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
	priv->panel = panel;

	i2c_set_clientdata(client, priv);





#if 1
        rc = component_add(&client->dev, &ch7033_ops);
        if (rc) {
		dev_err(&client->dev, "failed to register component: %d\n", rc);
		return rc;
	}
#endif


	return 0;
}

static int ch7033_remove(struct i2c_client *client)
{
	//struct ch7033_priv *priv = i2c_get_clientdata(client);

	component_del(&client->dev, &ch7033_ops);


	return 0;
}

static int ch7033_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strlcpy(info->type, "ch7033", I2C_NAME_SIZE);

	return 0;
}

static const struct of_device_id ch7033_dt_ids[] = {
	{ .compatible = "chrontel,ch7033", },
	{ }
};
MODULE_DEVICE_TABLE(of, ch7033_dt_ids);

static const struct i2c_device_id ch7033_i2c_ids[] = {
	{ .name = "ch7033", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ch7033_i2c_ids);

static struct i2c_driver ch7033_driver = {
	.driver = {
		.name = "ch7033",
		.of_match_table = ch7033_dt_ids,
	},
	.class = I2C_CLASS_DDC | I2C_CLASS_HWMON,
	.id_table = ch7033_i2c_ids,
	.probe = ch7033_probe,
	.remove = ch7033_remove,
	.detect = ch7033_detect,
	.address_list = normal_i2c,
};

module_i2c_driver(ch7033_driver);

MODULE_DESCRIPTION("Chrontel CH7033 Encoder Driver");
MODULE_LICENSE("GPL v2");
