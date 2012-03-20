/*
 * max77601-regulator.c
 * Maxim LDO and Buck regulators driver
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */
#include <linux/err.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mfd/max77601.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

struct max77601_vreg {
	struct regulator_dev *rdev;
	struct max77601_chip *chip;
	u8 id;
	u8 type;
	u8 volt_reg;
	u8 cfg_reg;
	u32 min_uV;
	u32 max_uV;
	u32 step_uV;
	u8 volt_shadow;
	u8 cfg_shadow;
	u8 power_mode;
};

static int max77601_regulator_set_voltage(struct regulator_dev *rdev,
					    int min_uV, int max_uV,
					    unsigned *selector);
static int max77601_regulator_get_voltage(struct regulator_dev *dev);
static int max77601_regulator_enable(struct regulator_dev *dev);
static int max77601_regulator_disable(struct regulator_dev *dev);
static int max77601_regulator_is_enabled(struct regulator_dev *dev);
static int max77601_regulator_set_mode(struct regulator_dev *dev,
					unsigned int power_mode);
static unsigned int max77601_regulator_get_mode(struct regulator_dev *dev);

static struct regulator_ops max77601_ldo_ops = {
	.set_voltage =  max77601_regulator_set_voltage,
	.get_voltage = max77601_regulator_get_voltage,
	.enable = max77601_regulator_enable,
	.disable = max77601_regulator_disable,
	.is_enabled = max77601_regulator_is_enabled,
	.set_mode = max77601_regulator_set_mode,
	.get_mode = max77601_regulator_get_mode,
};

#define VREG_DESC(_id, _name, _ops)		\
	[_id] = {				\
		.name = _name,		\
		.id = _id,			\
		.ops = _ops,		\
		.type = REGULATOR_VOLTAGE,	\
		.owner = THIS_MODULE,		\
	}

static struct regulator_desc max77601_regulator_desc[MAX77601_VREG_MAX] = {
	VREG_DESC(MAX77601_ID_SD0, "max77601_sd0", &max77601_ldo_ops),
	VREG_DESC(MAX77601_ID_DVSSD0, "max77601_dvssd0", &max77601_ldo_ops),
	VREG_DESC(MAX77601_ID_SD1, "max77601_sd1", &max77601_ldo_ops),
	VREG_DESC(MAX77601_ID_DVSSD1, "max77601_dvssd1", &max77601_ldo_ops),
	VREG_DESC(MAX77601_ID_SD2, "max77601_sd2", &max77601_ldo_ops),
	VREG_DESC(MAX77601_ID_SD3, "max77601_sd3", &max77601_ldo_ops),
	VREG_DESC(MAX77601_ID_SD4, "max77601_sd4", &max77601_ldo_ops),

	VREG_DESC(MAX77601_ID_L0, "max77601_ldo0", &max77601_ldo_ops),
	VREG_DESC(MAX77601_ID_L1, "max77601_ldo1", &max77601_ldo_ops),
	VREG_DESC(MAX77601_ID_L2, "max77601_ldo2", &max77601_ldo_ops),
	VREG_DESC(MAX77601_ID_L3, "max77601_ldo3", &max77601_ldo_ops),
	VREG_DESC(MAX77601_ID_L4, "max77601_ldo4", &max77601_ldo_ops),
	VREG_DESC(MAX77601_ID_L5, "max77601_ldo5", &max77601_ldo_ops),
	VREG_DESC(MAX77601_ID_L6, "max77601_ldo6", &max77601_ldo_ops),
	VREG_DESC(MAX77601_ID_L7, "max77601_ldo7", &max77601_ldo_ops),
	VREG_DESC(MAX77601_ID_L8, "max77601_ldo8", &max77601_ldo_ops),
};

#define VREG(_id, _volt_reg, _cfg_reg, _type, _min, _max, _step) \
    [_id]  = {			\
		.id = _id,			\
		.min_uV = _min,		\
		.max_uV = _max,		\
		.step_uV = _step,	\
		.volt_reg = _volt_reg,		\
		.cfg_reg = _cfg_reg,		\
		.power_mode = MAX77601_MODE_NORMAL,	\
		.type = _type,	\
	}

static struct max77601_vreg max77601_regulators[MAX77601_VREG_MAX] = {
	VREG(MAX77601_ID_SD0, MAX77601_VREG_SD0, MAX77601_VREG_SD0_CFG,
		MAX77601_VREG_TYPE_SD, 600000, 3387500, 12500),
	VREG(MAX77601_ID_DVSSD0, MAX77601_VREG_DVSSD0, MAX77601_VREG_DVSSD0_CFG,
		MAX77601_VREG_TYPE_SD, 600000, 3387500, 12500),
	VREG(MAX77601_ID_SD1, MAX77601_VREG_SD1, MAX77601_VREG_SD1_CFG,
		MAX77601_VREG_TYPE_SD, 800000, 1587500, 12500),
	VREG(MAX77601_ID_DVSSD1, MAX77601_VREG_DVSSD1, MAX77601_VREG_DVSSD1_CFG,
		MAX77601_VREG_TYPE_SD, 800000, 1587500, 12500),
	VREG(MAX77601_ID_SD2, MAX77601_VREG_SD2, MAX77601_VREG_SD2_CFG,
	    MAX77601_VREG_TYPE_SD, 600000, 3387500, 12500),
	VREG(MAX77601_ID_SD3, MAX77601_VREG_SD3, MAX77601_VREG_SD3_CFG,
		MAX77601_VREG_TYPE_SD, 600000, 3387500, 12500),
	VREG(MAX77601_ID_SD4, MAX77601_VREG_SD4, MAX77601_VREG_SD4_CFG,
		MAX77601_VREG_TYPE_SD, 600000, 3387500, 12500),

	VREG(MAX77601_ID_L0, MAX77601_VREG_LDO0, MAX77601_VREG_LDO0_CFG2,
		MAX77601_VREG_TYPE_LDO, 800000, 2350000, 25000),
	VREG(MAX77601_ID_L1, MAX77601_VREG_LDO1, MAX77601_VREG_LDO1_CFG2,
		MAX77601_VREG_TYPE_LDO, 800000, 2350000, 25000),
	VREG(MAX77601_ID_L2, MAX77601_VREG_LDO2, MAX77601_VREG_LDO2_CFG2,
		MAX77601_VREG_TYPE_LDO, 800000, 3950000, 50000),
	VREG(MAX77601_ID_L3, MAX77601_VREG_LDO3, MAX77601_VREG_LDO3_CFG2,
		MAX77601_VREG_TYPE_LDO, 800000, 3950000, 50000),
	VREG(MAX77601_ID_L4, MAX77601_VREG_LDO4, MAX77601_VREG_LDO4_CFG2,
		MAX77601_VREG_TYPE_LDO, 800000, 1587500, 12500),
	VREG(MAX77601_ID_L5, MAX77601_VREG_LDO5, MAX77601_VREG_LDO5_CFG2,
		MAX77601_VREG_TYPE_LDO, 800000, 3950000, 50000),
	VREG(MAX77601_ID_L6, MAX77601_VREG_LDO6, MAX77601_VREG_LDO6_CFG2,
		MAX77601_VREG_TYPE_LDO, 800000, 3950000, 50000),
	VREG(MAX77601_ID_L7, MAX77601_VREG_LDO7, MAX77601_VREG_LDO7_CFG2,
		MAX77601_VREG_TYPE_LDO, 800000, 3950000, 50000),
	VREG(MAX77601_ID_L8, MAX77601_VREG_LDO8, MAX77601_VREG_LDO8_CFG2,
		MAX77601_VREG_TYPE_LDO, 800000, 3950000, 50000),
};

static int max77601_vreg_write(struct max77601_chip *chip, u16 addr, u8 val,
				 u8 mask, u8 *bak)
{
	u8 reg = (*bak & ~mask) | (val & mask);
	int ret = max77601_write(chip, addr, &reg, 1);
	if (!ret)
		*bak = reg;
	return ret;
}

static int max77601_regulator_set_voltage(struct regulator_dev *rdev,
					    int min_uV, int max_uV,
					    unsigned *selector)
{
	struct max77601_vreg *vreg = rdev_get_drvdata(rdev);
	struct max77601_chip *chip = vreg->chip;
	u8 val;
	int rc = -EIO;

	if (min_uV < vreg->min_uV || max_uV > vreg->max_uV) {
		dev_err(chip->dev, "invalid voltage range (%d, %d) uV\n", \
			min_uV, max_uV);
		return -EDOM;
	}

	val = (min_uV - vreg->min_uV) / vreg->step_uV;
	dev_dbg(chip->dev, "%s RequV:%d, MinuV:%d, StpuV:%d, val:0x%x\n", \
		__func__, min_uV, vreg->min_uV, vreg->step_uV, val);

	if (vreg->type == MAX77601_VREG_TYPE_SD) {
		rc = max77601_write(chip, vreg->volt_reg, &val, 1);
		if (rc == 0)
			vreg->volt_shadow = val;
	} else {
		rc =
		    max77601_vreg_write(chip, vreg->volt_reg, val,
					MAX77601_LDO_VOLT_M,
					&(vreg->volt_shadow));
	}
	return rc;
}

static int max77601_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct max77601_vreg *vreg = rdev_get_drvdata(rdev);
	struct max77601_chip *chip = vreg->chip;
	u8 val;
	int volt;
	int rc = -EIO;

	rc = max77601_read(chip, vreg->volt_reg, &val, 1);
	if (rc < 0)
		return rc;
	vreg->volt_shadow = val;
	if (vreg->type == MAX77601_VREG_TYPE_SD) {
		volt = val * vreg->step_uV + vreg->min_uV;
	} else {
		volt =
		    (val & MAX77601_LDO_VOLT_M) * vreg->step_uV + vreg->min_uV;
	}
	dev_dbg(chip->dev, "%s: val:0x%x volt:%d\n", __func__, val, volt);
	return volt;
}

static int max77601_regulator_enable(struct regulator_dev *rdev)
{
	int rc = -EDOM;
	rc = max77601_regulator_set_mode(rdev, MAX77601_MODE_NORMAL);
	return rc;
}

static int max77601_regulator_disable(struct regulator_dev *rdev)
{
	int rc = -EDOM;
	rc = max77601_regulator_set_mode(rdev, MAX77601_MODE_DISABLE);
	return rc;
}

static int max77601_regulator_is_enabled(struct regulator_dev *rdev)
{
	return (max77601_regulator_get_mode(rdev) == MAX77601_MODE_NORMAL);
}

static int max77601_regulator_set_mode(struct regulator_dev *rdev,
					 unsigned int power_mode)
{
	struct max77601_vreg *vreg = rdev_get_drvdata(rdev);
	struct max77601_chip *chip = vreg->chip;
	int rc = -EIO;

	if (vreg->type == MAX77601_VREG_TYPE_SD) {
		vreg->cfg_shadow = (vreg->cfg_shadow & ~MAX77601_SD_MODE_M) |
			(vreg->power_mode << MAX77601_SD_MODE_SHIFT);
		rc =
		    max77601_write(chip, vreg->cfg_reg, &vreg->cfg_shadow, 1);
	} else {
		vreg->volt_shadow = (vreg->volt_shadow & ~MAX77601_LDO_MODE_M) |
			(vreg->power_mode << MAX77601_LDO_MODE_SHIFT);
		rc =
		    max77601_write(chip, vreg->volt_reg, &vreg->volt_shadow, 1);
	}
	if (rc == 0)
		vreg->power_mode = power_mode;
	dev_dbg(chip->dev, "%s id:%d, mode:%d\n", __func__, vreg->id,
		  vreg->power_mode);
	return rc;
}

static unsigned int max77601_regulator_get_mode(struct regulator_dev
						  *rdev)
{
	struct max77601_vreg *vreg = rdev_get_drvdata(rdev);
	struct max77601_chip *chip = vreg->chip;
	int rc = -EIO;
	u8 val = -1;
	if (vreg->type == MAX77601_VREG_TYPE_SD) {
		rc = max77601_read(chip, vreg->cfg_reg, &val, 1);
		if (rc == 0) {
			vreg->cfg_shadow = val;
			vreg->power_mode =
			    (vreg->cfg_shadow & MAX77601_SD_MODE_M) >> \
				MAX77601_SD_MODE_SHIFT;
		}
	} else {
		rc = max77601_read(chip, vreg->volt_reg, &val, 1);
		if (rc == 0) {
			vreg->volt_shadow = val;
			vreg->power_mode =
			    (vreg->volt_shadow & MAX77601_LDO_MODE_M) >> \
				MAX77601_LDO_MODE_SHIFT;
		}
	}
	if (rc < 0)
		return rc;
	dev_dbg(chip->dev, "%s id:%d, val:0x%x, mode:%d\n", __func__,
		  vreg->id, val, vreg->power_mode);

	return (unsigned int)vreg->power_mode;
}

static int max77601_init_regulator(struct max77601_chip *chip,
					struct max77601_vreg *vreg)
{
	int rc = 0;
	rc = max77601_read(chip, vreg->volt_reg, &vreg->volt_shadow, 1);
	if (rc < 0)
		return rc;
	rc = max77601_read(chip, vreg->cfg_reg, &vreg->cfg_shadow, 1);
	return rc;
}

static int max77601_regulator_probe(struct platform_device *pdev)
{
	struct max77601_chip *chip;
	struct max77601_vreg *vreg = NULL;
	struct regulator_desc *rdesc;
	struct regulator_dev *rdev;
	struct regulator_init_data *pdata;
	const char *reg_name = NULL;
	int rc = 0;

	if (pdev == NULL)
		return -EINVAL;
	if (pdev->id >= MAX77601_ID_SD0 && pdev->id < MAX77601_VREG_MAX) {
		chip = dev_get_drvdata(pdev->dev.parent);
		pdata = pdev->dev.platform_data;
		rdesc = &max77601_regulator_desc[pdev->id];
		vreg = &max77601_regulators[pdev->id];
		vreg->chip = chip;
		reg_name = max77601_regulator_desc[pdev->id].name;

		rc = max77601_init_regulator(chip, vreg);
		if (rc < 0)
			goto error;
		dev_dbg(&pdev->dev, "%s pdata name : %s\n", __func__, \
			 pdata->consumer_supplies->supply);
		rdev = regulator_register(rdesc, &pdev->dev, pdata, vreg);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev, "%s regulator register err.\n",
				 __func__);
			rc = PTR_ERR(rdev);
		}
		platform_set_drvdata(pdev, rdev);
	} else {
		rc = -ENODEV;
	}

error:
	if (rc) {
		dev_err(&pdev->dev, "%s: id=%d, name=%s, rc=%d\n", __func__,
			 pdev->id, reg_name, rc);
	}
	return rc;
}

static int max77601_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, NULL);
	regulator_unregister(rdev);
	return 0;
}

static struct platform_driver max77601_regulator_driver = {
	.probe = max77601_regulator_probe,
	.remove = __devexit_p(max77601_regulator_remove),
	.driver = {
		.name = "max77601-regulator",
		.owner = THIS_MODULE,
	},
};

static int __init max77601_regulator_init(void)
{
	return platform_driver_register(&max77601_regulator_driver);
}
subsys_initcall(max77601_regulator_init);

static void __exit max77601_reg_exit(void)
{
	platform_driver_unregister(&max77601_regulator_driver);
}
module_exit(max77601_reg_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("max77601 regulator driver");
MODULE_VERSION("1.0");
