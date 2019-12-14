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
	struct i2c_adapter *ddc;
	struct drm_encoder encoder;
	struct drm_bridge bridge;
	struct drm_connector connector;
};

#define conn_to_ch7033_priv(x) \
	container_of(x, struct ch7033_priv, connector)
#define bridge_to_ch7033_priv(x) \
	container_of(x, struct ch7033_priv, bridge)

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

	// Mode set 1280x1024@75

	// Reset
	i2c_smbus_write_byte_data(priv->client, 0x03, 0x04);
	i2c_smbus_write_byte_data(priv->client, 0x52, 0x00);
	i2c_smbus_write_byte_data(priv->client, 0x52, 0xce);


	// Page 0
	i2c_smbus_write_byte_data(priv->client, 0x03, 0x00);

	// bring up drivers and i/o from power down

#define DRI_PD BIT(3)
#define IO_PD  BIT(5)
	//i2c_smbus_write_byte_data(priv->client, 0x07, 0xd0);
	//i2c_smbus_write_byte_data(priv->client, 0x07, 0xd7); <--
	i2c_smbus_write_byte_data(priv->client, 0x07, 0xff & ~DRI_PD & ~IO_PD);

#define DRI_PDDRI (BIT(4) | BIT(5) | BIT(6) | BIT(7))
#define PDDAC (BIT(1) | BIT(2) | BIT(3))
#define PANEN BIT(0)
//	i2c_smbus_write_byte_data(priv->client, 0x08, 0x01);
//	i2c_smbus_write_byte_data(priv->client, 0x08, 0xff & ~DRI_PDDRI & ~PDDAC | PANEN);
//	i2c_smbus_write_byte_data(priv->client, 0x08, 0x00);
	i2c_smbus_write_byte_data(priv->client, 0x08, 0xff & ~DRI_PDDRI & ~PDDAC & ~PANEN);


#define DRI_PDDRI (BIT(4) | BIT(5) | BIT(6) | BIT(7))
#define PDDAC (BIT(1) | BIT(2) | BIT(3))
#define PANEN BIT(0)
//	i2c_smbus_write_byte_data(priv->client, 0x08, 0x01);
//	i2c_smbus_write_byte_data(priv->client, 0x08, 0xff & ~DRI_PDDRI & ~PDDAC | PANEN);
//	i2c_smbus_write_byte_data(priv->client, 0x08, 0x00);
	i2c_smbus_write_byte_data(priv->client, 0x08, 0xff & ~DRI_PDDRI & ~PDDAC & ~PANEN);


#define DPD BIT(7) //
#define GCKOFF BIT(6) //
#define TV_BP BIT(5) // ok
#define SCLPD BIT(4) //xx
#define SDPD BIT(3) // ok
#define VGA_PD BIT(2) // ok
#define HDBKPD BIT(1) //xx
#define HDMI_PD BIT(0) //
        //i2c_smbus_write_byte_data(priv->client, 0x09, 0x12);
	//i2c_smbus_write_byte_data(priv->client, 0x09, 0x3a);
	i2c_smbus_write_byte_data(priv->client, 0x09, 0xff & ~HDMI_PD & ~VGA_PD & ~DPD & ~GCKOFF);


#define MEMINIT BIT(7)
#define MEMIDLE BIT(6)
#define MEMPD BIT(5)
#define STOP BIT(4)
#define LVDS_PD BIT(3)
#define HD_DVIB BIT(2) //
#define HDCP_PD BIT(1)
#define MCU_PD BIT(0)
	//i2c_smbus_write_byte_data(priv->client, 0x0a, 0x2a);
	//i2c_smbus_write_byte_data(priv->client, 0x0a, 0xfb);
	i2c_smbus_write_byte_data(priv->client, 0x0a, 0xff & ~HD_DVIB);


	// input horizontal
	i2c_smbus_write_byte_data(priv->client, 0x0b, 0x34);
	i2c_smbus_write_byte_data(priv->client, 0x0c, 0x80);
	i2c_smbus_write_byte_data(priv->client, 0x0d, 0x40);
	i2c_smbus_write_byte_data(priv->client, 0x0e, 0x00);
	i2c_smbus_write_byte_data(priv->client, 0x0f, 0x40);
	i2c_smbus_write_byte_data(priv->client, 0x10, 0x80);

	// input vertical
	i2c_smbus_write_byte_data(priv->client, 0x11, 0x1b);
	i2c_smbus_write_byte_data(priv->client, 0x12, 0x60);
	i2c_smbus_write_byte_data(priv->client, 0x13, 0x84);
	i2c_smbus_write_byte_data(priv->client, 0x14, 0x00);
	i2c_smbus_write_byte_data(priv->client, 0x15, 0x01);
	i2c_smbus_write_byte_data(priv->client, 0x16, 0x03);

	// color swap
	i2c_smbus_write_byte_data(priv->client, 0x18, 0x05); // bad colors otherwise

	// input clock
	i2c_smbus_write_byte_data(priv->client, 0x19, 0xf9);
	i2c_smbus_write_byte_data(priv->client, 0x1a, 0xa5);
	i2c_smbus_write_byte_data(priv->client, 0x1b, 0xe0);

	// output vertical
	i2c_smbus_write_byte_data(priv->client, 0x1f, 0x34);
	i2c_smbus_write_byte_data(priv->client, 0x20, 0x80);
	i2c_smbus_write_byte_data(priv->client, 0x21, 0x40);
	i2c_smbus_write_byte_data(priv->client, 0x25, 0x1b);
	i2c_smbus_write_byte_data(priv->client, 0x26, 0x60);
	i2c_smbus_write_byte_data(priv->client, 0x27, 0x84);

	// VGA bypass [NO]
	i2c_smbus_write_byte_data(priv->client, 0x2b, 0x09);

	// polarity, TE (???) [NO]
	i2c_smbus_write_byte_data(priv->client, 0x2e, 0x3f);

	// hdmi horizontal output
	//i2c_smbus_write_byte_data(priv->client, 0x54, 0x70); // vga bad
	i2c_smbus_write_byte_data(priv->client, 0x54, 0x80); // should be default...
	//i2cset -f -y 1 0x76 0x54 0x80 # just blue ghosts

	i2c_smbus_write_byte_data(priv->client, 0x55, 0x40);
	i2c_smbus_write_byte_data(priv->client, 0x56, 0x80);

	// hdmi vertical output
	i2c_smbus_write_byte_data(priv->client, 0x57, 0x00);
	i2c_smbus_write_byte_data(priv->client, 0x58, 0x01);
	i2c_smbus_write_byte_data(priv->client, 0x59, 0x03);

	// hdmi lvds sel [ch7033_unknown_init]
	i2c_smbus_write_byte_data(priv->client, 0x7e, 0x8f);


	// Page 1
	i2c_smbus_write_byte_data(priv->client, 0x03, 0x01);

	i2c_smbus_write_byte_data(priv->client, 0x07, 0x66); // XXX just turn on CKINV, otherwise we get blue ghost [ch7033_unknown_init]
	i2c_smbus_write_byte_data(priv->client, 0x08, 0x05); // otherwise shit clock on vga

	// PLLM
	i2c_smbus_write_byte_data(priv->client, 0x0c, 0x74); // [ch7033_unknown_init]
	i2c_smbus_write_byte_data(priv->client, 0x12, 0xf6);
	i2c_smbus_write_byte_data(priv->client, 0x13, 0x28);
	i2c_smbus_write_byte_data(priv->client, 0x14, 0x81);
	i2c_smbus_write_byte_data(priv->client, 0x15, 0x23);

	// otherwise different colors are bland on vga
	i2c_smbus_write_byte_data(priv->client, 0x64, 0x29); // BC1 [blue compensation?, default = 0x40]
	i2c_smbus_write_byte_data(priv->client, 0x65, 0x29); // GC1
	i2c_smbus_write_byte_data(priv->client, 0x66, 0x29); // RC1

	i2c_smbus_write_byte_data(priv->client, 0x6b, 0x10); // "Power up DRISER"
	i2c_smbus_write_byte_data(priv->client, 0x6c, 0x00); // "Power up DRI PLL"

	// Page 3
	i2c_smbus_write_byte_data(priv->client, 0x03, 0x03);
	// VGA clock bypass [ch7033_unknown_init]
	i2c_smbus_write_byte_data(priv->client, 0x28, 0x0c);
	// HDMI & HDMI clock bypass [ch7033_unknown_init]
	i2c_smbus_write_byte_data(priv->client, 0x2a, 0x28);

	// Page 4
	i2c_smbus_write_byte_data(priv->client, 0x03, 0x04);

	// (hdmi?) output clock [NO]
	i2c_smbus_write_byte_data(priv->client, 0x11, 0xa5);
	i2c_smbus_write_byte_data(priv->client, 0x12, 0xe0);

	// [HV]D?INC[ABC] what that even is [NO]
	i2c_smbus_write_byte_data(priv->client, 0x2a, 0x8f);
	i2c_smbus_write_byte_data(priv->client, 0x2c, 0x8f);
	i2c_smbus_write_byte_data(priv->client, 0x2e, 0x6b);
	i2c_smbus_write_byte_data(priv->client, 0x30, 0x6b);
	i2c_smbus_write_byte_data(priv->client, 0x32, 0x90);
	i2c_smbus_write_byte_data(priv->client, 0x34, 0x90);
	i2c_smbus_write_byte_data(priv->client, 0x3c, 0x10);

	// POWON VID IDBD
	//i2c_smbus_write_byte_data(priv->client, 0x51, 0xc2);

	// DIV4_PD down. sigh. doesn't matter. but also some scaler related clk thing
	//i2c_smbus_write_byte_data(priv->client, 0x61, 0xe6);

	// Apply
	i2c_smbus_write_byte_data(priv->client, 0x03, 0x04);
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
