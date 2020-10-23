// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2012 Marvell Semiconductor, Inc.
 * Copyright 2012 One Laptop per Child
 * Copyright 2020 Lubomir Rintel <lkundrak@v3.sk>
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>

struct mmp3_thermal {
	struct device *dev;
	struct thermal_zone_device *tz;
	u32 __iomem *regs;
	struct clk_bulk_data *clks;
	int nr_clks;
};

#define TEMP_RESERVED		BIT(31) // undoc'd, appears in original marvell code
#define TEMP_START		BIT(30)
#define TEMP_STATUS		BIT(29)
#define TEMP_OVER_INT		BIT(28)
#define TEMP_LOWRANGE		BIT(27)
#define TEMP_EN_WDOG		BIT(26)
#define TEMP_TEMP_INT		BIT(25)
#define TEMP_EN_OVER_INT	BIT(24)
#define TEMP_EN_TEMP_INT	BIT(23)
#define TEMP_AUTO_READ		BIT(21) // undoc'd, useful for watchdog but causes interrupt storm
#define TEMP_WDOG_THSHLD	0x0f00
#define TEMP_WDOG_SHIFT		8
#define TEMP_INT_TSHLD_MASK	0x00f0
#define TEMP_TEMP_VALUE_MASK	0x000f

enum {
        HIRANGE = 0,
        LORANGE = 1,
};

static unsigned int gray_to_temp[2][16] = {{
        780,  805,  855,  830,  955,  930,  880,  905,
          0, 1130, 1080, 1105,  980, 1005, 1055, 1030
}, {
        260,  285,  335,  310,  435,  410,  360,  385,
          0,  610,  560,  585,  460,  485,  535,  510,
}};

static DEFINE_MUTEX(mmp3_thermal_sensor_mutex);

static unsigned long read_temperature_sensor(struct mmp3_thermal *priv, int range, int index)
{
	int i, j, gray;
	unsigned long temp;
	u32 val;

	mutex_lock(&mmp3_thermal_sensor_mutex);

	val = readl(priv->regs + index);

	if (val & TEMP_EN_WDOG) {
		if ((range == HIRANGE) == ((val & TEMP_LOWRANGE) == 0))
			return 0;
	}

	if (range == HIRANGE) {
		val &= ~TEMP_LOWRANGE;
	} else {
		val |= TEMP_LOWRANGE;
	}

	val &= ~TEMP_START;
	writel(val, priv->regs + index);
	usleep_range(300, 1000);

	for (i = 0; i < 10; i++) {
		writel(val | TEMP_START, priv->regs + index);
		for (j = 0; j < 10; j++) {
			usleep_range(300, 1000);
			if (readl(priv->regs + index) & TEMP_STATUS)
				goto done;
		}
		usleep_range(300, 1000);
		writel(val, priv->regs + index);
	}
    done:

	if (i == 10) {
		printk(KERN_ERR "mmp3_thermal: timeout reading sensor %d, range %d\n",
			index, range);
		writel(val, priv->regs + index);
		temp = 0;  // filtering may fix this
	} else {
		gray = readl(priv->regs + index) & TEMP_TEMP_VALUE_MASK;
		temp = gray_to_temp[range][gray];
	}

	mutex_unlock(&mmp3_thermal_sensor_mutex);

	return temp;	// return value is tenths, C
}

static unsigned long read_filtered_temp(struct mmp3_thermal *priv, int range, int sensor)
{
	int i;
	unsigned long tvals[3];

	for (i = 0; i < 3; i++)
		tvals[i] = read_temperature_sensor(priv, range, sensor);

	// sort the values 
	if (tvals[0] > tvals[1]) swap(tvals[0], tvals[1]);
	if (tvals[1] > tvals[2]) swap(tvals[1], tvals[2]);
	if (tvals[0] > tvals[1]) swap(tvals[0], tvals[1]);

	// return the middle value 
	return tvals[1];
}

static int mmp3_thermal_get_temp(void *data, int *temp)
{
	struct mmp3_thermal *priv = data;
	unsigned long t, t0, t1, t2;

	t0 = read_filtered_temp(priv, HIRANGE, 0);
	t1 = read_filtered_temp(priv, HIRANGE, 1);
	t2 = read_filtered_temp(priv, HIRANGE, 2);
        t = max(t0,t1);
        t = max(t,t2);

	if (t < 800) {
		t0 = read_filtered_temp(priv, LORANGE, 0);
		t1 = read_filtered_temp(priv, LORANGE, 1);
		t2 = read_filtered_temp(priv, LORANGE, 2);
		t = max(t0,t1);
		t = max(t,t2);

		// the sensors can't report values between 60 and 80
		if (t > 600)
			t = 700;
	}

	*temp = t * 100; // return value is in millidegrees C

	return 0;
}

static void mmp3_thermal_set_watchdog(struct mmp3_thermal *priv, unsigned int temp)
{
	int range = HIRANGE;
        int grey = 8;
	int t = 0;
        int r, i;
	u32 val;

        for (r = HIRANGE; r <= LORANGE; r++) {
                for (i = 0; i < 16; i++) {
                        if (gray_to_temp[r][i] > temp)
                                continue;
                        if (t > gray_to_temp[r][i])
                                continue;
                        range = r;
                        grey = i;
                        t = gray_to_temp[range][grey];
                }
        }

	dev_info(priv->dev, "Enabling thermal watchdog at %d.%dC\n", t / 10, t % 10);

	val = readl(priv->regs + 1);
	val &= ~TEMP_LOWRANGE;
	val |= grey << TEMP_WDOG_SHIFT;
	val |= TEMP_AUTO_READ;
	val |= TEMP_EN_WDOG;
	writel(val, priv->regs + 1);
}

static const struct thermal_zone_of_device_ops mmp3_thermal_ops = {
	.get_temp = mmp3_thermal_get_temp,
};

static int mmp3_thermal_probe(struct platform_device *pdev)
{
	struct mmp3_thermal *priv;
	int index;
	int temp;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	platform_set_drvdata(pdev, priv);

	priv->regs = devm_of_iomap(priv->dev, dev_of_node(priv->dev), 0, NULL);
	if (IS_ERR(priv->regs)) {
		dev_warn(priv->dev, "Unable to get I/O range\n");
		return PTR_ERR(priv->regs);
	}

	priv->nr_clks = devm_clk_bulk_get_all(priv->dev, &priv->clks);
	if (priv->nr_clks < 0) {
		dev_warn(priv->dev, "Unable to get clocks\n");
		return priv->nr_clks;
	}

	ret = clk_bulk_prepare_enable(priv->nr_clks, priv->clks);
	if (ret) {
		dev_warn(priv->dev, "Unable to enable clocks\n");
		return ret;
	}

	for (index = 0; index < 3; index++)
		writel(0, priv->regs + index);

	priv->tz = devm_thermal_zone_of_sensor_register(priv->dev, 0, priv, &mmp3_thermal_ops);
	if (IS_ERR(priv->tz)) {
		dev_warn(priv->dev, "Unable to register a thermal zone\n");
		return PTR_ERR(priv->tz);
	}

	if (priv->tz->ops->get_crit_temp(priv->tz, &temp))
		mmp3_thermal_set_watchdog(priv, temp);

	dev_info(priv->dev, "MMP3 Thermal Sensor\n");
	return 0;
}

static int mmp3_thermal_remove(struct platform_device *pdev)
{
	struct mmp3_thermal *priv = platform_get_drvdata(pdev);

	clk_bulk_disable_unprepare(priv->nr_clks, priv->clks);

	return 0;
}

static const struct of_device_id mmp3_thermal_of_match[] = {
	{ .compatible = "marvell,mmp3-thermal", },
	{},
};
MODULE_DEVICE_TABLE(of, mmp3_thermal_of_match);

static struct platform_driver mmp3_thermal_driver = {
	.probe		= mmp3_thermal_probe,
	.remove		= mmp3_thermal_remove,
	.driver = {
		.name = "mmp3-thermal",
		.of_match_table = mmp3_thermal_of_match,
	},
};
module_platform_driver(mmp3_thermal_driver);

MODULE_AUTHOR("Marvell Semiconductor");
MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_DESCRIPTION("MMP3 Thermal Sensor Driver");
MODULE_LICENSE("GPL");
