// SPDX-License-Identifier: BSD-2-Clause OR GPL-2.0-or-later
/*
 * MMP2 Random Number Generator Driver
 *
 * Copyright (C) 2020 Lubomir Rintel
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define RETRY_COUNT	20
#define RETRY_INTERVAL	3

/* Config register */
enum {
	POSTPROC_ENABLE		= BIT(6),
	SLOW_OSC_ENABLE		= BIT(7),
	FAST_OSC_ENABLE		= BIT(8),
	BIAS_POWER_UP		= BIT(9),
	RNG_RESET_DISABLE	= BIT(10),
	ENABLE			= BIT(12),
	SLOW_OSC_DIVIDER	= (BIT(16) | BIT(20)),
	DOWNSAMPLING_RATIO	= BIT(25),
};

/* Data register, low word is random data */
enum {
	READY = BIT(31),
};

struct mmp2_rng {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct hwrng rng;
};

static inline struct mmp2_rng *rng_to_priv(struct hwrng *rng)
{
	return container_of(rng, struct mmp2_rng, rng);
}

static int mmp2_rng_init(struct hwrng *rng)
{
	struct mmp2_rng *priv = rng_to_priv(rng);
	int ret;

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;

	writel(POSTPROC_ENABLE | SLOW_OSC_ENABLE | FAST_OSC_ENABLE |
	       BIAS_POWER_UP | RNG_RESET_DISABLE | ENABLE |
	       SLOW_OSC_DIVIDER | DOWNSAMPLING_RATIO, priv->base);

	return 0;
}

static int mmp2_rng_read(struct hwrng *rng, void *data, size_t max, bool wait)
{
	struct mmp2_rng *priv = rng_to_priv(rng);
	int retries;
	u32 random;
	int i = 0;

	while (i < max) {
		for (retries = 0; retries < RETRY_COUNT; retries++) {
			random = readl_relaxed(priv->base + 4);
			if (random & READY)
				break;
			udelay(RETRY_INTERVAL);
		}

		if ((random & READY) == 0) {
			dev_err(priv->dev, "RNG read timed out\n");
			return i;
		}

		*(u8 *)(data + i++) = random;
		if (i == max)
			break;
		*(u8 *)(data + i++) = random >> 8;
	}

	return i;
}

static void mmp2_rng_cleanup(struct hwrng *rng)
{
	struct mmp2_rng *priv = rng_to_priv(rng);

	writel(0, priv->base);
	clk_disable_unprepare(priv->clk);
}

static int mmp2_rng_probe(struct platform_device *pdev)
{
	struct mmp2_rng *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->clk = devm_clk_get_optional(&pdev->dev, NULL);
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	priv->rng.name = pdev->name;
	priv->rng.init = mmp2_rng_init;
	priv->rng.read = mmp2_rng_read;
	priv->rng.cleanup = mmp2_rng_cleanup;

	ret = devm_hwrng_register(&pdev->dev, &priv->rng);
	if (ret)
		dev_err(priv->dev, "Failed to register RNG\n");
	else
		dev_info(priv->dev, "MMP2 Random Number Generator\n");

	return ret;
}

static const struct of_device_id mmp2_rng_match[] = {
	{ .compatible = "marvell,mmp2-rng" },
	{},
};
MODULE_DEVICE_TABLE(of, mmp2_rng_match);

static struct platform_driver mmp2_rng_driver = {
	.driver = {
		.name = "mmp2-rng",
		.of_match_table = mmp2_rng_match,
	},
	.probe = mmp2_rng_probe,
};

module_platform_driver(mmp2_rng_driver);

MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_DESCRIPTION("MMP2 Random Number Generator (RNG) driver");
MODULE_LICENSE("Dual BSD/GPL");
