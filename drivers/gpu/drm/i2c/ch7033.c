#include <linux/component.h>
#include <linux/module.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#define CH7033_PAGE_SEL_REG		0x03

#define CH7033_POWER_STATE_4_REG	0x0a
#define CH7033_POWER_STATE_4_MEM_INIT		BIT(7)
#define CH7033_POWER_STATE_4_MEM_STOP		BIT(4)

#define CH7033_INPUT_TIMING_1_REG	0x0b
#define CH7033_INPUT_TIMING_1_HTI(val)		(((val >> 8) & 0xf) << 3)
#define CH7033_INPUT_TIMING_1_HAI(val)		((val >> 8) & 0x7)

#define CH7033_INPUT_TIMING_2_REG	0x0c
#define CH7033_INPUT_TIMING_2_HAI(val)		(val & 0xff)

#define CH7033_INPUT_TIMING_3_REG	0x0d
#define CH7033_INPUT_TIMING_3_HTI(val)		(val & 0xff)

#define CH7033_INPUT_TIMING_4_REG	0x0e
#define CH7033_INPUT_TIMING_4_HWI(val)		(((val >> 8) & 0x7) << 3)
#define CH7033_INPUT_TIMING_4_HOI(val)		((val >> 8) & 0x7)

#define CH7033_INPUT_TIMING_5_REG	0x0f
#define CH7033_INPUT_TIMING_5_HOI(val)		(val & 0xff)

#define CH7033_INPUT_TIMING_6_REG	0x10
#define CH7033_INPUT_TIMING_6_HWI(val)		(val & 0xff)

#define CH7033_INPUT_TIMING_7_REG	0x11
#define CH7033_INPUT_TIMING_7_VTI(val)		(((val >> 8) & 0x7) << 3)
#define CH7033_INPUT_TIMING_7_VAI(val)		((val >> 8) & 0x7)

#define CH7033_INPUT_TIMING_8_REG	0x12
#define CH7033_INPUT_TIMING_8_VAI(val)		(val & 0xff)

#define CH7033_INPUT_TIMING_9_REG	0x13
#define CH7033_INPUT_TIMING_9_VTI(val)		(val & 0xff)

#define CH7033_INPUT_TIMING_10_REG	0x14
#define CH7033_INPUT_TIMING_10_VWI(val)		(((val >> 8) & 0x7) << 3)
#define CH7033_INPUT_TIMING_10_VOI(val)		((val >> 8) & 0x7)

#define CH7033_INPUT_TIMING_11_REG	0x15
#define CH7033_INPUT_TIMING_11_VOI(val)		(val & 0xff)

#define CH7033_INPUT_TIMING_12_REG	0x16
#define CH7033_INPUT_TIMING_12_VWI(val)		(val & 0xff)

#define CH7033_INPUT_POL_REG		0x19
#define CH7033_INPUT_POL_HSYNC_HI		BIT(5)
#define CH7033_INPUT_POL_VSYNC_HI		BIT(4)
#define CH7033_INPUT_POL_DE_HI			BIT(3)
#define CH7033_INPUT_POL_GCLK(val)		((val >> 16) & 0x3)

#define CH7033_GCLK_1_REG		0x1a
#define CH7033_GCLK_1_FREQ(val)			((val >> 8) & 0xff)

#define CH7033_GCLK_2_REG		0x1b
#define CH7033_GCLK_2_FREQ(val)			((val) & 0xff)

#define CH7033_OUTPUT_TIMING_1_REG	0x1f
#define CH7033_OUTPUT_TIMING_1_HTO(val)		(((val >> 8) & 0xf) << 3)
#define CH7033_OUTPUT_TIMING_1_HAO(val)		((val >> 8) & 0x7)

#define CH7033_OUTPUT_TIMING_2_REG	0x20
#define CH7033_OUTPUT_TIMING_2_HAO(val)		(val & 0xff)

#define CH7033_OUTPUT_TIMING_3_REG	0x21
#define CH7033_OUTPUT_TIMING_3_HTO(val)		(val & 0xff)

#define CH7033_OUTPUT_TIMING_7_REG	0x25
#define CH7033_OUTPUT_TIMING_7_VTO(val)		(((val >> 8) & 0x7) << 3)
#define CH7033_OUTPUT_TIMING_7_VAO(val)		((val >> 8) & 0x7)

#define CH7033_OUTPUT_TIMING_8_REG	0x26
#define CH7033_OUTPUT_TIMING_8_VAO(val)		(val & 0xff)

#define CH7033_OUTPUT_TIMING_9_REG	0x27
#define CH7033_OUTPUT_TIMING_9_VTO(val)		(val & 0xff)

#define CH7033_OUTPUT_TIMING_4_REG	0x54
#define CH7033_OUTPUT_TIMING_4_HWO(val)		(((val >> 8) & 0x7) << 3)
#define CH7033_OUTPUT_TIMING_4_HOO(val)		((val >> 8) & 0x7)

#define CH7033_OUTPUT_TIMING_5_REG	0x55
#define CH7033_OUTPUT_TIMING_5_HOO(val)		(val & 0xff)

#define CH7033_OUTPUT_TIMING_6_REG	0x56
#define CH7033_OUTPUT_TIMING_6_HWO(val)		(val & 0xff)

#define CH7033_OUTPUT_TIMING_10_REG	0x57
#define CH7033_OUTPUT_TIMING_10_VWO(val)	(((val >> 8) & 0x7) << 3)
#define CH7033_OUTPUT_TIMING_10_VOO(val)	((val >> 8) & 0x7)

#define CH7033_OUTPUT_TIMING_11_REG	0x58
#define CH7033_OUTPUT_TIMING_11_VOO(val)	(val & 0xff)

#define CH7033_OUTPUT_TIMING_12_REG	0x59
#define CH7033_OUTPUT_TIMING_12_VWO(val)	(val & 0xff)

struct ch7033_priv {
        struct i2c_client *client;
	struct i2c_adapter *ddc;
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

static u8 ch7033_write(struct i2c_client *client, u8 page, u8 reg, u8 value)
{
	i2c_smbus_write_byte_data(client, CH7033_PAGE_SEL_REG, page);

	return i2c_smbus_write_byte_data(client, reg, value);
}

static enum drm_connector_status ch7033_connector_detect(struct drm_connector *connector, bool force)
{
	struct ch7033_priv *priv = conn_to_ch7033_priv(connector);

        if (drm_probe_ddc(priv->ddc))
                return connector_status_connected;

        return connector_status_unknown;
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
	int ret;

	edid = drm_get_edid(connector, priv->ddc);
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

        drm_connector_helper_add(connector,
                                 &ch7033_connector_helper_funcs);
        ret = drm_connector_init_with_ddc(bridge->dev, connector,
                                          &ch7033_connector_funcs,
                                          DRM_MODE_CONNECTOR_DVII,
                                          priv->ddc);
        if (ret) {
                DRM_ERROR("Failed to initialize connector\n");
                return ret;
        }

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
	if (mode->clock > 165000)
		return MODE_CLOCK_HIGH;
	if (mode->hdisplay >= 1920)
		return MODE_BAD_HVALUE;
	if (mode->vdisplay >= 1080)
		return MODE_BAD_VVALUE;
	return MODE_OK;
}

static void ch7033_bridge_mode_set(struct drm_bridge *bridge,
				    const struct drm_display_mode *mode,
				    const struct drm_display_mode *adjusted_mode)
{
	struct ch7033_priv *priv = bridge_to_ch7033_priv(bridge);
	u16 hbp = mode->hsync_start - mode->hdisplay;
	u16 hsync_len = mode->hsync_end - mode->hsync_start;
	u16 vbp = mode->vsync_start - mode->vdisplay;
	u16 vsync_len = mode->vsync_end - mode->vsync_start;
	u32 val;

	/* Setup the horizontal timings ... */
	ch7033_write(priv->client, 0, CH7033_INPUT_TIMING_1_REG,
		     CH7033_INPUT_TIMING_1_HTI(mode->htotal) |
		     CH7033_INPUT_TIMING_1_HAI(mode->hdisplay));

	ch7033_write(priv->client, 0, CH7033_INPUT_TIMING_2_REG,
		     CH7033_INPUT_TIMING_2_HAI(mode->hdisplay));

	ch7033_write(priv->client, 0, CH7033_INPUT_TIMING_3_REG,
		     CH7033_INPUT_TIMING_3_HTI(mode->htotal));

	ch7033_write(priv->client, 0, CH7033_INPUT_TIMING_4_REG,
		     CH7033_INPUT_TIMING_4_HOI(hbp) |
		     CH7033_INPUT_TIMING_4_HWI(hsync_len));

	ch7033_write(priv->client, 0, CH7033_INPUT_TIMING_5_REG,
		     CH7033_INPUT_TIMING_5_HOI(hbp));

	ch7033_write(priv->client, 0, CH7033_INPUT_TIMING_6_REG,
		     CH7033_INPUT_TIMING_6_HWI(hsync_len));

	/* ... And the vertical ones */
	ch7033_write(priv->client, 0, CH7033_INPUT_TIMING_7_REG,
		     CH7033_INPUT_TIMING_7_VTI(mode->vtotal) |
		     CH7033_INPUT_TIMING_7_VAI(mode->vdisplay));

	ch7033_write(priv->client, 0, CH7033_INPUT_TIMING_8_REG,
		     CH7033_INPUT_TIMING_8_VAI(mode->vdisplay));

	ch7033_write(priv->client, 0, CH7033_INPUT_TIMING_9_REG,
		     CH7033_INPUT_TIMING_9_VTI(mode->vtotal));

	ch7033_write(priv->client, 0, CH7033_INPUT_TIMING_10_REG,
		     CH7033_INPUT_TIMING_10_VOI(vbp) |
		     CH7033_INPUT_TIMING_10_VWI(vsync_len));

	ch7033_write(priv->client, 0, CH7033_INPUT_TIMING_11_REG,
		     CH7033_INPUT_TIMING_11_VOI(vbp));

	ch7033_write(priv->client, 0, CH7033_INPUT_TIMING_12_REG,
		     CH7033_INPUT_TIMING_12_VWI(vsync_len));

	/* Setup polarities and clock */
	val = CH7033_INPUT_POL_DE_HI;
	val |= (mode->flags & DRM_MODE_FLAG_PHSYNC) ? CH7033_INPUT_POL_HSYNC_HI : 0;
	val |= (mode->flags & DRM_MODE_FLAG_PVSYNC) ? CH7033_INPUT_POL_VSYNC_HI : 0;
	val |= CH7033_INPUT_POL_GCLK(mode->clock);
	ch7033_write(priv->client, 0, CH7033_INPUT_POL_REG, val);

	ch7033_write(priv->client, 0, CH7033_GCLK_1_REG,
		     CH7033_GCLK_1_FREQ(mode->clock));

	ch7033_write(priv->client, 0, CH7033_GCLK_2_REG,
		     CH7033_GCLK_2_FREQ(mode->clock));

	/* Horizontal output timings ... */
	ch7033_write(priv->client, 0, CH7033_OUTPUT_TIMING_1_REG,
		     CH7033_OUTPUT_TIMING_1_HTO(mode->htotal) |
		     CH7033_OUTPUT_TIMING_1_HAO(mode->hdisplay));

	ch7033_write(priv->client, 0, CH7033_OUTPUT_TIMING_2_REG,
		     CH7033_OUTPUT_TIMING_2_HAO(mode->hdisplay));

	ch7033_write(priv->client, 0, CH7033_OUTPUT_TIMING_3_REG,
		     CH7033_OUTPUT_TIMING_3_HTO(mode->htotal));

	ch7033_write(priv->client, 0, CH7033_OUTPUT_TIMING_4_REG,
		     CH7033_OUTPUT_TIMING_4_HOO(hbp) |
		     CH7033_OUTPUT_TIMING_4_HWO(hsync_len));

	ch7033_write(priv->client, 0, CH7033_OUTPUT_TIMING_5_REG,
		     CH7033_OUTPUT_TIMING_5_HOO(hbp));

	ch7033_write(priv->client, 0, CH7033_OUTPUT_TIMING_6_REG,
		     CH7033_OUTPUT_TIMING_6_HWO(hsync_len));

	/* ... And the vertical ones */
	ch7033_write(priv->client, 0, CH7033_OUTPUT_TIMING_7_REG,
		     CH7033_OUTPUT_TIMING_7_VTO(mode->vtotal) |
		     CH7033_OUTPUT_TIMING_7_VAO(mode->vdisplay));

	ch7033_write(priv->client, 0, CH7033_OUTPUT_TIMING_8_REG,
		     CH7033_OUTPUT_TIMING_8_VAO(mode->vdisplay));

	ch7033_write(priv->client, 0, CH7033_OUTPUT_TIMING_9_REG,
		     CH7033_OUTPUT_TIMING_9_VTO(mode->vtotal));

	ch7033_write(priv->client, 0, CH7033_OUTPUT_TIMING_10_REG,
		     CH7033_OUTPUT_TIMING_10_VOO(vbp) |
		     CH7033_OUTPUT_TIMING_10_VWO(vsync_len));

	ch7033_write(priv->client, 0, CH7033_OUTPUT_TIMING_11_REG,
		     CH7033_OUTPUT_TIMING_11_VOO(vbp));

	ch7033_write(priv->client, 0, CH7033_OUTPUT_TIMING_12_REG,
		     CH7033_OUTPUT_TIMING_12_VWO(vsync_len));


	if (mode->hdisplay >= 1280) {
		i2c_smbus_write_byte_data(priv->client, 0x03, 0x03);
		// 3: 28 (3 - 3) set_output_info() CHANNEL_VGA bypass ? 1 : 0
		i2c_smbus_write_byte_data(priv->client, 0x28, 0x0c); //i2cset -f -y 1 0x76 0x28 0x0c # 0x04  weird clocking artifacts
		// 0: 2b (0 - 3) set_output_info() CHANNEL_VGA bypass ? 9 : 8
		i2c_smbus_write_byte_data(priv->client, 0x03, 0x00);
		i2c_smbus_write_byte_data(priv->client, 0x2b, 0x09); //i2cset -f -y 1 0x76 0x2b 0x09 # 0x08 no signal
	} else {
		i2c_smbus_write_byte_data(priv->client, 0x03, 0x03);
		i2c_smbus_write_byte_data(priv->client, 0x28, 0x04); //i2cset -f -y 1 0x76 0x28 0x0c # 0x04  weird clocking artifacts
		i2c_smbus_write_byte_data(priv->client, 0x03, 0x00);
		i2c_smbus_write_byte_data(priv->client, 0x2b, 0x08); //i2cset -f -y 1 0x76 0x2b 0x09 # 0x08 no signal
	}

	i2c_smbus_write_byte_data(priv->client, 0x03, 0x04);
	i2c_smbus_write_byte_data(priv->client, 0x52, 0xce);
	i2c_smbus_write_byte_data(priv->client, 0x52, 0xcf);
}

static const struct drm_bridge_funcs ch7033_bridge_funcs = {
	.attach = ch7033_bridge_attach,
	.detach = ch7033_bridge_detach,
	.mode_valid = ch7033_bridge_mode_valid,
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
		return ret;

	ret = drm_bridge_attach(&priv->encoder, &priv->bridge, NULL);
	if (ret)
		drm_encoder_cleanup(&priv->encoder);

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
	struct device_node *ddc_node, *remote;
	struct device *dev = &client->dev;
	struct ch7033_priv *priv;
	struct i2c_adapter *ddc;
	int ret;

	remote = of_graph_get_remote_node(dev->of_node, 1, -1);
	if (!remote) {
		dev_err(&client->dev, "XXX MISSING REMOTE\n");
		return -EINVAL;
	}

	ddc_node = of_parse_phandle(remote, "ddc-i2c-bus", 0);
	of_node_put(remote);
	if (!ddc_node) {
		dev_err(&client->dev, "XXX NO I2C BUS\n");
		return -ENODEV;
	}

	ddc = of_get_i2c_adapter_by_node(ddc_node);
	if (!ddc) {
		dev_warn(&client->dev, "DEFER: NO I2C\n");
		return -EPROBE_DEFER;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);

	INIT_LIST_HEAD(&priv->bridge.list);

	priv->client = client;
	priv->ddc = ddc;

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
