// SPDXe-Identifier: GPL-2.0+
/*
 * Marvell MMP3 HDMI Encoder Driver
 *
 * Copyright (C) 2020 Lubomir Rintel
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rational.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#define MMP3_HDMI_PLL_CFG0			0x20
#define MMP3_HDMI_PLL_CFG0_CK			BIT(0)
#define MMP3_HDMI_PLL_CFG0_VDDL_SHIFT		6

#define MMP3_HDMI_PLL_CFG1			0x24
#define MMP3_HDMI_PLL_CFG1_EN_HDMI		BIT(23)
#define MMP3_HDMI_PLL_CFG1_EN_PANEL		BIT(24)

#define MMP3_HDMI_PLL_CFG2			0x28

#define MMP3_HDMI_PLL_CFG3			0x2c
#define MMP3_HDMI_PLL_CFG3_PLL_ON		BIT(0)
#define MMP3_HDMI_PLL_CFG3_PLL_RESET		BIT(1)
#define MMP3_HDMI_PLL_CFG3_REFDIV_MASK		0x1f
#define MMP3_HDMI_PLL_CFG3_REFDIV_SHIFT	2
#define MMP3_HDMI_PLL_CFG3_FBDIV_MASK		0x1ff
#define MMP3_HDMI_PLL_CFG3_FBDIV_SHIFT		7
#define MMP3_HDMI_PLL_CFG3_PLL_LOCK		BIT(22)

struct mmp3_hdmi_priv {
	void __iomem *base;
	struct drm_bridge *next_bridge;
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct clk_hw pll;
};

#define conn_to_mmp3_hdmi_priv(x) \
	container_of(x, struct mmp3_hdmi_priv, connector)
#define bridge_to_mmp3_hdmi_priv(x) \
	container_of(x, struct mmp3_hdmi_priv, bridge)
#define pll_to_mmp3_hdmi_priv(x) \
	container_of(x, struct mmp3_hdmi_priv, pll)

static int mmp3_hdmi_pll_enable(struct clk_hw *hw)
{
	struct mmp3_hdmi_priv *priv = pll_to_mmp3_hdmi_priv(hw);
	u32 val;

	// HDMI_CLOCK_CFG // 0x0000aa10 also works
	//                        5 5`--- 1 hd enable 
	writel(0x0000aad5, priv->base + 0x1c);
	//writel(0x0000aa10, priv->base + 0x1c);

	// HDMI_PLL_CFG0
	//writel(0x2c442651, priv->base + MMP3_HDMI_PLL_CFG0);
	val = MMP3_HDMI_PLL_CFG0_CK;
	val |= 8 << MMP3_HDMI_PLL_CFG0_VDDL_SHIFT;
	writel(val, priv->base + MMP3_HDMI_PLL_CFG0);

	// 0x01800000 works
	// writel(0x418084b8, priv->base + MMP3_HDMI_PLL_CFG1);
	val = MMP3_HDMI_PLL_CFG1_EN_HDMI;
	val |= MMP3_HDMI_PLL_CFG1_EN_PANEL;
	writel(val, priv->base + MMP3_HDMI_PLL_CFG1);

	writel(0, priv->base + MMP3_HDMI_PLL_CFG2);

	val = readl(priv->base + MMP3_HDMI_PLL_CFG3);
	val &= ~MMP3_HDMI_PLL_CFG3_PLL_RESET;
	val |= MMP3_HDMI_PLL_CFG3_PLL_ON;
	writel(val, priv->base + MMP3_HDMI_PLL_CFG3);
	udelay(100);
	val |= MMP3_HDMI_PLL_CFG3_PLL_RESET;
	writel(val, priv->base + MMP3_HDMI_PLL_CFG3);

	return 0;
}

static void mmp3_hdmi_pll_disable(struct clk_hw *hw)
{
	struct mmp3_hdmi_priv *priv = pll_to_mmp3_hdmi_priv(hw);
	u32 val;

	val = readl(priv->base + MMP3_HDMI_PLL_CFG3);
	val &= ~MMP3_HDMI_PLL_CFG3_PLL_RESET;
	val &= ~MMP3_HDMI_PLL_CFG3_PLL_ON;
	writel(val, priv->base + MMP3_HDMI_PLL_CFG3);

	writel(0, priv->base + MMP3_HDMI_PLL_CFG2);
	writel(0, priv->base + MMP3_HDMI_PLL_CFG1);
	writel(0, priv->base + MMP3_HDMI_PLL_CFG0);
}

static inline void mmp3_hdmi_pll_calc_divs(unsigned long parent_rate,
					   unsigned long rate,
					   unsigned long *refdiv,
					   unsigned long *fbdiv)
{
	rational_best_approximation(parent_rate, rate,
				    MMP3_HDMI_PLL_CFG3_REFDIV_MASK,
				    MMP3_HDMI_PLL_CFG3_FBDIV_MASK,
				    refdiv, fbdiv);
}

static inline long mmp3_hdmi_pll_calc_rate(unsigned long parent_rate,
					   unsigned long refdiv,
					   unsigned long fbdiv)
{
	uint64_t rate = parent_rate;

	rate *= fbdiv;
	do_div(rate, refdiv);

	return rate;
}

static unsigned long mmp3_hdmi_pll_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct mmp3_hdmi_priv *priv = pll_to_mmp3_hdmi_priv(hw);
	unsigned long refdiv;
	unsigned long fbdiv;
	u32 val;

	val = readl(priv->base + MMP3_HDMI_PLL_CFG3);
	refdiv = (val >> MMP3_HDMI_PLL_CFG3_REFDIV_SHIFT) &
		  MMP3_HDMI_PLL_CFG3_REFDIV_MASK;
	fbdiv = (val >> MMP3_HDMI_PLL_CFG3_FBDIV_SHIFT) &
		MMP3_HDMI_PLL_CFG3_FBDIV_MASK;

	return mmp3_hdmi_pll_calc_rate(parent_rate, refdiv, fbdiv);
}

static long mmp3_hdmi_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	unsigned long refdiv;
	unsigned long fbdiv;

	mmp3_hdmi_pll_calc_divs(*parent_rate, rate, &refdiv, &fbdiv);

	return mmp3_hdmi_pll_calc_rate(*parent_rate, refdiv, fbdiv);
}

static int mmp3_hdmi_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct mmp3_hdmi_priv *priv = pll_to_mmp3_hdmi_priv(hw);
	unsigned long refdiv;
	unsigned long fbdiv;
	int retries = 10000;
	u32 val;

	mmp3_hdmi_pll_calc_divs(parent_rate, rate, &refdiv, &fbdiv);

	val = readl(priv->base + MMP3_HDMI_PLL_CFG3);
	val &= ~(MMP3_HDMI_PLL_CFG3_REFDIV_MASK << MMP3_HDMI_PLL_CFG3_REFDIV_SHIFT);
	val &= ~(MMP3_HDMI_PLL_CFG3_FBDIV_MASK << MMP3_HDMI_PLL_CFG3_FBDIV_SHIFT);
	val |= refdiv << MMP3_HDMI_PLL_CFG3_REFDIV_SHIFT;
	val |= fbdiv << MMP3_HDMI_PLL_CFG3_FBDIV_SHIFT;
	writel(val, priv->base + MMP3_HDMI_PLL_CFG3);

	if (!clk_hw_is_enabled(hw))
		return 0;

	while (retries--) {
		val = readl(priv->base + MMP3_HDMI_PLL_CFG3);
		if (val & MMP3_HDMI_PLL_CFG3_PLL_LOCK)
			return 0;
		udelay(1);
	}

	return -ETIMEDOUT;
}

struct clk_ops mmp3_hdmi_pll_clk_ops = {
	.enable = mmp3_hdmi_pll_enable,
	.disable = mmp3_hdmi_pll_disable,
	.recalc_rate = mmp3_hdmi_pll_recalc_rate,
	.round_rate = mmp3_hdmi_pll_round_rate,
	.set_rate = mmp3_hdmi_pll_set_rate,
};


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
	struct mmp3_hdmi_priv *priv = bridge_to_mmp3_hdmi_priv(bridge);

	// HDMI_PHY_CFG2
	//writel(0x00003c30, priv->base + 0x10);
	writel(0x00001000, priv->base + 0x10);

//	regmap_write(priv->regmap, 0x03, 0x04);
//	regmap_update_bits(priv->regmap, 0x52, RESETDB, RESETDB);
}

static void mmp3_hdmi_bridge_mode_set(struct drm_bridge *bridge,
				   const struct drm_display_mode *mode,
				   const struct drm_display_mode *adjusted_mode)
{
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

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);

	ret = drm_of_find_panel_or_bridge(dev->of_node, 1, -1, NULL,
					  &priv->next_bridge);
	if (ret)
		return ret;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, resource);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->pll.init = CLK_HW_INIT_FW_NAME("hdmi_pll", "vctcxo",
					     &mmp3_hdmi_pll_clk_ops, 0);
	ret = devm_clk_hw_register(dev, &priv->pll);
	if (ret)
		return ret;

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get,
					  &priv->pll);
	if (ret)
		return ret;

	INIT_LIST_HEAD(&priv->bridge.list);
	priv->bridge.funcs = &mmp3_hdmi_bridge_funcs;
	priv->bridge.of_node = dev->of_node;
	drm_bridge_add(&priv->bridge);

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
