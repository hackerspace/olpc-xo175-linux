#include <linux/component.h>
#include <linux/module.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

/* Page 0, Register 0x07 */
enum {
	DRI_PD		= BIT(3),
	IO_PD		= BIT(5),
};

/* Page 0, Register 0x08 */
enum {
	DRI_PDDRI	= GENMASK(7, 4),
	PDDAC		= GENMASK(3, 1),
	PANEN		= BIT(0),
};

/* Page 0, Register 0x09 */
enum {
	DPD		= BIT(7),
	GCKOFF		= BIT(6),
	TV_BP		= BIT(5),
	SCLPD		= BIT(4),
	SDPD		= BIT(3),
	VGA_PD		= BIT(2),
	HDBKPD		= BIT(1),
	HDMI_PD		= BIT(0),
};

/* Page 0, Register 0x0a */
enum {
	MEMINIT		= BIT(7),
	MEMIDLE		= BIT(6),
	MEMPD		= BIT(5),
	STOP		= BIT(4),
	LVDS_PD		= BIT(3),
	HD_DVIB		= BIT(2),
	HDCP_PD		= BIT(1),
	MCU_PD		= BIT(0),
};

/* Page 0, Register 0x18 */
enum {
	IDF		= GENMASK(7, 4),
	INTEN		= BIT(3),
	SWAP	 	= GENMASK(2, 0),
};

enum {
	BYTE_SWAP_RGB	= 0,
	BYTE_SWAP_RBG	= 1,
	BYTE_SWAP_GRB	= 2,
	BYTE_SWAP_GBR	= 3,
	BYTE_SWAP_BRG	= 4,
	BYTE_SWAP_BGR	= 5,
};

/* Page 0, Register 0x2b */
enum {
	SWAPS		= GENMASK(7, 4),
	VFMT	 	= GENMASK(3, 0),
};

/* Page 0, Register 0x54 */
enum {
	COMP_BP		= BIT(7),
	DAC_EN_T	= BIT(6),
	HWO_HDMI_HI	= GENMASK(5, 3),
	HOO_HDMI_HI	= GENMASK(2, 0),
};

/* Page 0, Register 0x57 */
enum {
	FLDSEN		= BIT(7),
	VWO_HDMI_HI	= GENMASK(5, 3),
	VOO_HDMI_HI	= GENMASK(2, 0),
};

/* Page 0, Register 0x7e */
enum {
	HDMI_LVDS_SEL	= BIT(7),
	DE_GEN		= BIT(6),
	PWM_INDEX_HI	= BIT(5),
	USE_DE		= BIT(4),
	R_INT		= GENMASK(3, 0),
};

/* Page 1, Register 0x07 */
enum {
	BPCKSEL		= BIT(7),
	DRI_CMFB_EN	= BIT(6),
	CEC_PUEN	= BIT(5),
	CEC_T		= BIT(3),
	CKINV		= BIT(2),
	CK_TVINV	= BIT(1),
	DRI_CKS2	= BIT(0),
};

/* Page 1, Register 0x08 */
enum {
	DACG		= BIT(6),
	DACKTST		= BIT(5),
	DEDGEB		= BIT(4),
	SYO		= BIT(3),
	DRI_IT_LVDS	= GENMASK(2, 1),
	DISPON		= BIT(0),
};

/* Page 1, Register 0x0c */
enum {
	DRI_PLL_CP	= GENMASK(7, 6),
	DRI_PLL_DIVSEL	= BIT(5),
	DRI_PLL_N1_1	= BIT(4),
	DRI_PLL_N1_0	= BIT(3),
	DRI_PLL_N3_1	= BIT(2),
	DRI_PLL_N3_0	= BIT(1),
	DRI_PLL_CKTSTEN	= BIT(0),
};

/* Page 1, Register 0x6b */
enum {
	VCO3CS		= GENMASK(7, 6),
	ICPGBK2_0	= GENMASK(5, 3),
	DRI_VCO357SC	= BIT(2),
	PDPLL2		= BIT(1),
	DRI_PD_SER	= BIT(0),
};

/* Page 1, Register 0x6c */
enum {
	PLL2N11		= GENMASK(7, 4),
	PLL2N5_4	= BIT(3),
	PLL2N5_TOP	= BIT(2),
	DRI_PLL_PD	= BIT(1),
	PD_I2CM		= BIT(0),
};

/* Page 3, Register 0x28 */
enum {
	DIFF_EN		= GENMASK(7, 6),
	CORREC_EN	= GENMASK(5, 4),
	VGACLK_BP	= BIT(3),
	HM_LV_SEL	= BIT(2),
	HD_VGA_SEL	= BIT(1),
};

/* Page 3, Register 0x2a */
enum {
	LVDSCLK_BP	= BIT(7),
	HDTVCLK_BP	= BIT(6),
	HDMICLK_BP	= BIT(5),
	HDTV_BP		= BIT(4),
	HDMI_BP		= BIT(3),
	THRWL		= GENMASK(2, 0),
};

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

static int32_t ch7033_update_reg (struct i2c_client *client,
				u8 command, u8 value, u8 mask)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, command);
	if (ret < 0)
		return ret;

	ret &= ~mask;
	ret |= value;
	return i2c_smbus_write_byte_data(client, command, ret);
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
	int hbporch = mode->hsync_start - mode->hdisplay;
	int hsynclen = mode->hsync_end - mode->hsync_start;
	int vbporch = mode->vsync_start - mode->vdisplay;
	int vsynclen = mode->vsync_end - mode->vsync_start;

	/*
	 * Page 4
	 */
	i2c_smbus_write_byte_data(priv->client, 0x03, 0x04);

	/* Turn everything off to set all the registers to their defaults. */
	i2c_smbus_write_byte_data(priv->client, 0x52, 0x00);
	/* Back up, but keep the display disabled. */
	i2c_smbus_write_byte_data(priv->client, 0x52, 0xce);

	/*
	 * Page 0
	 */
	i2c_smbus_write_byte_data(priv->client, 0x03, 0x00);

	/* Bring up parts we need from the power down. */
	ch7033_update_reg(priv->client, 0x07, 0x00, DRI_PD | IO_PD);
	ch7033_update_reg(priv->client, 0x08, 0x00, DRI_PDDRI | PDDAC | PANEN);
	ch7033_update_reg(priv->client, 0x09, 0x00, HDMI_PD | VGA_PD | DPD | GCKOFF);
	ch7033_update_reg(priv->client, 0x0a, 0x00, HD_DVIB);

	/* Horizontal input timing. */
	i2c_smbus_write_byte_data(priv->client, 0x0b, ((mode->htotal >> 8) << 3) | (mode->hdisplay >> 8));
	i2c_smbus_write_byte_data(priv->client, 0x0c, mode->hdisplay & 0xff);
	i2c_smbus_write_byte_data(priv->client, 0x0d, mode->htotal & 0xff);
	i2c_smbus_write_byte_data(priv->client, 0x0e, ((hsynclen >> 8) << 3) | (hbporch >> 8));
	i2c_smbus_write_byte_data(priv->client, 0x0f, hbporch & 0xff);
	i2c_smbus_write_byte_data(priv->client, 0x10, hsynclen & 0xff);

	/* Vertical input timing. */
	i2c_smbus_write_byte_data(priv->client, 0x11, ((mode->vtotal >> 8) << 3) | (mode->vdisplay >> 8));
	i2c_smbus_write_byte_data(priv->client, 0x12, mode->vdisplay & 0xff);
	i2c_smbus_write_byte_data(priv->client, 0x13, mode->vtotal & 0xff);
	i2c_smbus_write_byte_data(priv->client, 0x14, ((vsynclen >> 8) << 3) | (vbporch >> 8));
	i2c_smbus_write_byte_data(priv->client, 0x15, vbporch & 0xff);
	i2c_smbus_write_byte_data(priv->client, 0x16, vsynclen & 0xff);

	/* Input clock. */
	ch7033_update_reg(priv->client, 0x19, mode->clock >> 16, 0x1);
	i2c_smbus_write_byte_data(priv->client, 0x1a, (mode->clock >> 8) & 0xff);
	i2c_smbus_write_byte_data(priv->client, 0x1b, mode->clock & 0xff);

	// input color swap
	ch7033_update_reg(priv->client, 0x18, BYTE_SWAP_BGR, SWAP);

	/* Horizontal output timing. */
	i2c_smbus_write_byte_data(priv->client, 0x1f, ((mode->htotal >> 8) << 3) | (mode->hdisplay >> 8));
	i2c_smbus_write_byte_data(priv->client, 0x20, mode->hdisplay & 0xff);
	i2c_smbus_write_byte_data(priv->client, 0x21, mode->htotal & 0xff);

	/* Vertical output timing. */
	i2c_smbus_write_byte_data(priv->client, 0x25, ((mode->vtotal >> 8) << 3) | (mode->vdisplay >> 8));
	i2c_smbus_write_byte_data(priv->client, 0x26, mode->vdisplay & 0xff);
	i2c_smbus_write_byte_data(priv->client, 0x27, mode->vtotal & 0xff);

	/* VGA channel bypass */
	ch7033_update_reg(priv->client, 0x2b, 9, VFMT);

	// polarity, TE (???) [NO]
	// doesn't do anything
	//i2c_smbus_write_byte_data(priv->client, 0x2e, 0x3f);

	/* HDMI horizontal output timing. */
	ch7033_update_reg(priv->client, 0x54, ((hsynclen >> 8) << 3) | (hbporch >> 8), HWO_HDMI_HI | HOO_HDMI_HI);
	i2c_smbus_write_byte_data(priv->client, 0x55, hbporch & 0xff);
	i2c_smbus_write_byte_data(priv->client, 0x56, hsynclen & 0xff);

	/* HDMI vertical output timing. */
	ch7033_update_reg(priv->client, 0x57, ((vsynclen >> 8) << 3) | (vbporch >> 8), VWO_HDMI_HI | VOO_HDMI_HI);
	i2c_smbus_write_byte_data(priv->client, 0x58, vbporch & 0xff);
	i2c_smbus_write_byte_data(priv->client, 0x59, vsynclen & 0xff);

	/* Pick HDMI, not LVDS. */
	ch7033_update_reg(priv->client, 0x7e, HDMI_LVDS_SEL, HDMI_LVDS_SEL);

	/*
	 * Page 1
	 */
	i2c_smbus_write_byte_data(priv->client, 0x03, 0x01);

	/* No idea what these do, but VGA is wobbly blinky without them. */
	ch7033_update_reg(priv->client, 0x07, CKINV, CKINV);
	ch7033_update_reg(priv->client, 0x08, DISPON, DISPON);

	/* DRI PLL */
	ch7033_update_reg(priv->client, 0x0c, DRI_PLL_DIVSEL, DRI_PLL_DIVSEL);
	if (mode->clock <= 40000) {
		ch7033_update_reg(priv->client, 0x0c, 0, DRI_PLL_N1_1 | DRI_PLL_N1_0 | DRI_PLL_N3_1 | DRI_PLL_N3_0);
	} else if(mode->clock < 80000) {
		ch7033_update_reg(priv->client, 0x0c, DRI_PLL_N3_0 | DRI_PLL_N1_0, DRI_PLL_N1_1 | DRI_PLL_N1_0 | DRI_PLL_N3_1 | DRI_PLL_N3_0);
	} else {
		ch7033_update_reg(priv->client, 0x0c, DRI_PLL_N3_1 | DRI_PLL_N1_1, DRI_PLL_N1_1 | DRI_PLL_N1_0 | DRI_PLL_N3_1 | DRI_PLL_N3_0);
	}

	/* This seems to be color calibration for VGA. */
	i2c_smbus_write_byte_data(priv->client, 0x64, 0x29); /* LSB Blue */
	i2c_smbus_write_byte_data(priv->client, 0x65, 0x29); /* LSB Green */
	i2c_smbus_write_byte_data(priv->client, 0x66, 0x29); /* LSB Red */
	i2c_smbus_write_byte_data(priv->client, 0x67, 0x00); /* MSB Blue */
	i2c_smbus_write_byte_data(priv->client, 0x68, 0x00); /* MSB Green */
	i2c_smbus_write_byte_data(priv->client, 0x69, 0x00); /* MSB Red */

	ch7033_update_reg(priv->client, 0x6b, 0x00, DRI_PD_SER);
	ch7033_update_reg(priv->client, 0x6c, 0x00, DRI_PLL_PD);

	/*
	 * Page 3
	 */
	i2c_smbus_write_byte_data(priv->client, 0x03, 0x03);

	/* More bypasses and apparently another HDMI/LVDS selector. */
	ch7033_update_reg(priv->client, 0x28, VGACLK_BP | HM_LV_SEL, VGACLK_BP | HM_LV_SEL);
	ch7033_update_reg(priv->client, 0x2a, HDMICLK_BP | HDMI_BP, HDMICLK_BP | HDMI_BP);

	/*
	 * Page 4
	 */
	i2c_smbus_write_byte_data(priv->client, 0x03, 0x04);

	/* Output clock. */
	i2c_smbus_write_byte_data(priv->client, 0x10, (mode->clock >> 16) & 0xff);
	i2c_smbus_write_byte_data(priv->client, 0x11, (mode->clock >> 8) & 0xff);
	i2c_smbus_write_byte_data(priv->client, 0x12, mode->clock & 0xff);

	/* Finally, turn the output back on. */
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
