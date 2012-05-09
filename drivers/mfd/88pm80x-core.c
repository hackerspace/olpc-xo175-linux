/*
 * Base driver for Marvell 88PM800
 *
 * Copyright (C) 2011 Marvell International Ltd.
 * Joseph(Yossi) Hanin <yhanin@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/mfd/88pm80x.h>
#include <linux/switch.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/delay.h>

static struct resource rtc_resources[] = {
	{
	 .name = "88pm80x-rtc",
	 .start = PM800_IRQ_RTC,
	 .end = PM800_IRQ_RTC,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct mfd_cell rtc_devs[] = {
	{
	 .name = "88pm80x-rtc",
	 .num_resources = ARRAY_SIZE(rtc_resources),
	 .resources = &rtc_resources[0],
	 .id = -1,
	 },
};

static struct resource dvc_resources[] = {
	{
	 .name = "dvc",
	},
};

static struct mfd_cell dvc_devs[] = {
	{
	 .name = "dvc",
	 .num_resources = ARRAY_SIZE(dvc_resources),
	 .resources = &dvc_resources[0],
	 .id = -1,
	},
};

static struct resource vbus_resources[] = {
	{
	 .name = "88pm80x-vbus",
	 .start = PM800_IRQ_CHG,
	 .end = PM800_IRQ_CHG,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct mfd_cell vbus_devs[] = {
	{
	 .name = "88pm80x-vbus",
	 .num_resources = 1,
	 .resources = &vbus_resources[0],
	 .id = -1,
	 },
};
static struct resource pm80x_gpio_resources[] = {
	{
		.name = "gpio-00",
		.start = PM800_IRQ_GPIO0,
		.end = PM800_IRQ_GPIO0,
		.flags = IORESOURCE_IRQ,
	}, {
		.name = "gpio-01",
		.start = PM800_IRQ_GPIO1,
		.end = PM800_IRQ_GPIO1,
		.flags = IORESOURCE_IRQ,
	}, {
		.name = "gpio-02",
		.start = PM800_IRQ_GPIO2,
		.end = PM800_IRQ_GPIO2,
		.flags = IORESOURCE_IRQ,
	}, {
		.name = "gpio-03",
		.start = PM800_IRQ_GPIO3,
		.end = PM800_IRQ_GPIO3,
		.flags = IORESOURCE_IRQ,
	}, {
		.name = "gpio-04",
		.start = PM800_IRQ_GPIO4,
		.end = PM800_IRQ_GPIO4,
		.flags = IORESOURCE_IRQ,
	},
};

static struct mfd_cell pm80x_gpio_devs[] = {
	{
	.name = "88pm80x-gpio",
	.num_resources = ARRAY_SIZE(pm80x_gpio_resources),
	.resources = &pm80x_gpio_resources[0],
	.id = -1,
	},
};

static struct resource onkey_resources[] = {
	{
	 .name = "88pm8xxx-onkey",
	 .start = PM800_IRQ_ONKEY,
	 .end = PM800_IRQ_ONKEY,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct mfd_cell onkey_devs[] = {
	{
	 .name = "88pm8xxx-onkey",
	 .num_resources = 1,
	 .resources = &onkey_resources[0],
	 .id = -1,
	 },
};

static struct mfd_cell vibrator_devs[] = {
	{
	 .name = "android-vibrator",
	 .id = -1,
	},
};

static struct resource codec_resources[] = {
	{
	 /* Headset microphone insertion or removal */
	 .name = "micin",
	 .start = PM805_IRQ_MIC_DET,
	 .end = PM805_IRQ_MIC_DET,
	 .flags = IORESOURCE_IRQ,
	 }, {

	     /* Audio short HP1 */
	     .name = "audio-short1",
	     .start = PM805_IRQ_HP1_SHRT,
	     .end = PM805_IRQ_HP1_SHRT,
	     .flags = IORESOURCE_IRQ,
	     }, {
		 /* Audio short HP2 */
		 .name = "audio-short2",
		 .start = PM805_IRQ_HP2_SHRT,
		 .end = PM805_IRQ_HP2_SHRT,
		 .flags = IORESOURCE_IRQ,
		 },
};

static struct resource headset_resources[] = {
	{
	 /* Hook-switch press or release */
	 .name = "hook",
	 .start = PM805_IRQ_SHRT_BTN_DET,
	 .end = PM805_IRQ_SHRT_BTN_DET,
	 .flags = IORESOURCE_IRQ,
	 }, {
	     /* Hook-switch press or release */
	     .name = "volup",
	     .start = PM805_IRQ_VOLP_BTN_DET,
	     .end = PM805_IRQ_VOLP_BTN_DET,
	     .flags = IORESOURCE_IRQ,
	     }, {
		 /* Hook-switch press or release */
		 .name = "voldown",
		 .start = PM805_IRQ_VOLM_BTN_DET,
		 .end = PM805_IRQ_VOLM_BTN_DET,
		 .flags = IORESOURCE_IRQ,
		 },
};

static struct resource headset_resources_800[] = {
	{
		.name = "gpio-03",
		.start = PM800_IRQ_GPIO3,
		.end = PM800_IRQ_GPIO3,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "gpadc4",
		.start = PM800_IRQ_GPADC4,
		.end = PM800_IRQ_GPADC4,
		.flags = IORESOURCE_IRQ,
	},
};

static struct gpio_switch_platform_data headset_switch_device_data[] = {
	{
	 /* headset switch */
	 .name = "h2w",
	 .gpio = 0,
	 .name_on = NULL,
	 .name_off = NULL,
	 .state_on = NULL,
	 .state_off = NULL,
	 }, {
	     /* hook switch */
	     .name = "h3w",
	     .gpio = 0,
	     .name_on = NULL,
	     .name_off = NULL,
	     .state_on = NULL,
	     .state_off = NULL,
	     },
};

static struct mfd_cell codec_devs[] = {
	{
	 .name = "88pm80x-codec",
	 .num_resources = ARRAY_SIZE(codec_resources),
	 .resources = &codec_resources[0],
	 .id = -1,
	 },
};

static struct mfd_cell headset_devs[] = {
	{
	 .name = "88pm80x-headset",
	 .num_resources = ARRAY_SIZE(headset_resources),
	 .resources = &headset_resources[0],
	 .id = -1,
	 .platform_data = headset_switch_device_data,
	 .pdata_size = sizeof(headset_switch_device_data),
	 },
};

static struct mfd_cell headset_devs_800[] = {
	{
	 .name = "88pm800-headset",
	 .num_resources = ARRAY_SIZE(headset_resources_800),
	 .resources = &headset_resources_800[0],
	 .id = -1,
	 .platform_data = headset_switch_device_data,
	 .pdata_size = sizeof(headset_switch_device_data),
	 },
};

static struct resource regulator_resources[] = {
	{PM800_ID_BUCK1, PM800_ID_BUCK1, "buck-1", IORESOURCE_IO,},
	{PM800_ID_BUCK2, PM800_ID_BUCK2, "buck-2", IORESOURCE_IO,},
	{PM800_ID_BUCK3, PM800_ID_BUCK3, "buck-3", IORESOURCE_IO,},
	{PM800_ID_BUCK4, PM800_ID_BUCK4, "buck-4", IORESOURCE_IO,},
	{PM800_ID_BUCK5, PM800_ID_BUCK5, "buck-5", IORESOURCE_IO,},
	{PM800_ID_LDO1, PM800_ID_LDO1, "ldo-01", IORESOURCE_IO,},
	{PM800_ID_LDO2, PM800_ID_LDO2, "ldo-02", IORESOURCE_IO,},
	{PM800_ID_LDO3, PM800_ID_LDO3, "ldo-03", IORESOURCE_IO,},
	{PM800_ID_LDO4, PM800_ID_LDO4, "ldo-04", IORESOURCE_IO,},
	{PM800_ID_LDO5, PM800_ID_LDO5, "ldo-05", IORESOURCE_IO,},
	{PM800_ID_LDO6, PM800_ID_LDO6, "ldo-06", IORESOURCE_IO,},
	{PM800_ID_LDO7, PM800_ID_LDO7, "ldo-07", IORESOURCE_IO,},
	{PM800_ID_LDO8, PM800_ID_LDO8, "ldo-08", IORESOURCE_IO,},
	{PM800_ID_LDO9, PM800_ID_LDO9, "ldo-09", IORESOURCE_IO,},
	{PM800_ID_LDO10, PM800_ID_LDO10, "ldo-10", IORESOURCE_IO,},
	{PM800_ID_LDO11, PM800_ID_LDO11, "ldo-11", IORESOURCE_IO,},
	{PM800_ID_LDO12, PM800_ID_LDO12, "ldo-12", IORESOURCE_IO,},
	{PM800_ID_LDO13, PM800_ID_LDO13, "ldo-13", IORESOURCE_IO,},
	{PM800_ID_LDO14, PM800_ID_LDO14, "ldo-14", IORESOURCE_IO,},
	{PM800_ID_LDO15, PM800_ID_LDO15, "ldo-15", IORESOURCE_IO,},
	{PM800_ID_LDO16, PM800_ID_LDO16, "ldo-16", IORESOURCE_IO,},
	{PM800_ID_LDO17, PM800_ID_LDO17, "ldo-17", IORESOURCE_IO,},
	{PM800_ID_LDO18, PM800_ID_LDO18, "ldo-18", IORESOURCE_IO,},
	{PM800_ID_LDO19, PM800_ID_LDO19, "ldo-19", IORESOURCE_IO,},
};

static struct mfd_cell regulator_devs[] = {
	{"88pm80x-regulator", 0,},
	{"88pm80x-regulator", 1,},
	{"88pm80x-regulator", 2,},
	{"88pm80x-regulator", 3,},
	{"88pm80x-regulator", 4,},
	{"88pm80x-regulator", 5,},
	{"88pm80x-regulator", 6,},
	{"88pm80x-regulator", 7,},
	{"88pm80x-regulator", 8,},
	{"88pm80x-regulator", 9,},
	{"88pm80x-regulator", 10,},
	{"88pm80x-regulator", 11,},
	{"88pm80x-regulator", 12,},
	{"88pm80x-regulator", 13,},
	{"88pm80x-regulator", 14,},
	{"88pm80x-regulator", 15,},
	{"88pm80x-regulator", 16,},
	{"88pm80x-regulator", 17,},
	{"88pm80x-regulator", 18,},
	{"88pm80x-regulator", 19,},
	{"88pm80x-regulator", 20,},
	{"88pm80x-regulator", 21,},
	{"88pm80x-regulator", 22,},
	{"88pm80x-regulator", 23,},
};

static struct regulator_init_data regulator_pdata[ARRAY_SIZE(regulator_devs)];

struct pm80x_irq_data {
	int reg;
	int mask_reg;
	int enable;		/* enable or not */
	int offs;		/* bit offset in mask register */
};

static struct pm80x_irq_data pm800_irqs[] = {
	[PM800_IRQ_ONKEY] = {	/*0 */
			     .reg = PM800_INT_STATUS1,
			     .mask_reg = PM800_INT_ENA_1,
			     .offs = 1 << 0,
			     },
	[PM800_IRQ_EXTON] = {
			     .reg = PM800_INT_STATUS1,
			     .mask_reg = PM800_INT_ENA_1,
			     .offs = 1 << 1,
			     },
	[PM800_IRQ_CHG] = {
			   .reg = PM800_INT_STATUS1,
			   .mask_reg = PM800_INT_ENA_1,
			   .offs = 1 << 2,
			   },
	[PM800_IRQ_BAT] = {
			   .reg = PM800_INT_STATUS1,
			   .mask_reg = PM800_INT_ENA_1,
			   .offs = 1 << 3,
			   },
	[PM800_IRQ_RTC] = {
			   .reg = PM800_INT_STATUS1,
			   .mask_reg = PM800_INT_ENA_1,
			   .offs = 1 << 4,
			   },
	[PM800_IRQ_CLASSD] = {	/*5 */
			      .reg = PM800_INT_STATUS1,
			      .mask_reg = PM800_INT_ENA_1,
			      .offs = 1 << 5,
			      },
	[PM800_IRQ_VBAT] = {
			    .reg = PM800_INT_STATUS2,
			    .mask_reg = PM800_INT_ENA_2,
			    .offs = 1 << 0,
			    },
	[PM800_IRQ_VSYS] = {
			    .reg = PM800_INT_STATUS2,
			    .mask_reg = PM800_INT_ENA_2,
			    .offs = 1 << 1,
			    },
	[PM800_IRQ_VCHG] = {
			    .reg = PM800_INT_STATUS2,
			    .mask_reg = PM800_INT_ENA_2,
			    .offs = 1 << 2,
			    },
	[PM800_IRQ_TINT] = {
			    .reg = PM800_INT_STATUS2,
			    .mask_reg = PM800_INT_ENA_2,
			    .offs = 1 << 3,
			    },
	[PM800_IRQ_GPADC0] = {	/*10 */
			      .reg = PM800_INT_STATUS3,
			      .mask_reg = PM800_INT_ENA_3,
			      .offs = 1 << 0,
			      },
	[PM800_IRQ_GPADC1] = {
			      .reg = PM800_INT_STATUS3,
			      .mask_reg = PM800_INT_ENA_3,
			      .offs = 1 << 1,
			      },
	[PM800_IRQ_GPADC2] = {
			      .reg = PM800_INT_STATUS3,
			      .mask_reg = PM800_INT_ENA_3,
			      .offs = 1 << 2,
			      },
	[PM800_IRQ_GPADC3] = {
			      .reg = PM800_INT_STATUS3,
			      .mask_reg = PM800_INT_ENA_3,
			      .offs = 1 << 3,
			      },
	[PM800_IRQ_GPADC4] = {
			      .reg = PM800_INT_STATUS3,
			      .mask_reg = PM800_INT_ENA_3,
			      .offs = 1 << 4,
			      },
	[PM800_IRQ_GPIO0] = {	/*15 */
			     .reg = PM800_INT_STATUS4,
			     .mask_reg = PM800_INT_ENA_4,
			     .offs = 1 << 0,
			     },
	[PM800_IRQ_GPIO1] = {
			     .reg = PM800_INT_STATUS4,
			     .mask_reg = PM800_INT_ENA_4,
			     .offs = 1 << 1,
			     },
	[PM800_IRQ_GPIO2] = {
			     .reg = PM800_INT_STATUS4,
			     .mask_reg = PM800_INT_ENA_4,
			     .offs = 1 << 2,
			     },
	[PM800_IRQ_GPIO3] = {
			     .reg = PM800_INT_STATUS4,
			     .mask_reg = PM800_INT_ENA_4,
			     .offs = 1 << 3,
			     },
	[PM800_IRQ_GPIO4] = {	/*19 */
			     .reg = PM800_INT_STATUS4,
			     .mask_reg = PM800_INT_ENA_4,
			     .offs = 1 << 4,
			     },
};

static inline struct pm80x_irq_data *irq_to_pm800(struct pm80x_chip *chip,
						  int irq)
{
	if (!chip->pm800_chip || irq < chip->pm800_chip->irq_base)
		return NULL;
	return &pm800_irqs[irq - chip->pm800_chip->irq_base];
}

static irqreturn_t pm800_irq(int irq, void *data)
{
	struct pm80x_chip *chip = data;
	struct pm80x_subchip *pm800_chip = chip->pm800_chip;
	struct pm80x_irq_data *irq_data;
	struct i2c_client *i2c = chip->base_page;
	int read_reg = -1, value = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(pm800_irqs); i++) {
		irq_data = &pm800_irqs[i];
		if (read_reg != irq_data->reg) {
			read_reg = irq_data->reg;
			value = pm80x_reg_read(i2c, irq_data->reg);
		}
		if (value & irq_data->enable)
			handle_nested_irq(pm800_chip->irq_base + i);
	}
	return IRQ_HANDLED;
}

static void pm800_irq_lock(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);

	mutex_lock(&chip->pm800_irq_lock);
}

static void pm800_irq_sync_unlock(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);
	struct pm80x_irq_data *irq_data;
	struct i2c_client *i2c;
	static unsigned char cached[PM800_INT_REG_NUM] = { 0x0, 0x0, 0x0, 0x0 };
	unsigned char mask[PM800_INT_REG_NUM];
	int i;

	i2c = chip->base_page;
	/* Load cached value. In initial, all IRQs are masked */
	for (i = 0; i < PM800_INT_REG_NUM; i++)
		mask[i] = cached[i];
	for (i = 0; i < ARRAY_SIZE(pm800_irqs); i++) {
		irq_data = &pm800_irqs[i];
		switch (irq_data->mask_reg) {
		case PM800_INT_ENA_1:
			mask[0] &= ~irq_data->offs;
			mask[0] |= irq_data->enable;
			break;
		case PM800_INT_ENA_2:
			mask[1] &= ~irq_data->offs;
			mask[1] |= irq_data->enable;
			break;
		case PM800_INT_ENA_3:
			mask[2] &= ~irq_data->offs;
			mask[2] |= irq_data->enable;
			break;
		case PM800_INT_ENA_4:
			mask[3] &= ~irq_data->offs;
			mask[3] |= irq_data->enable;
			break;
		default:
			dev_err(chip->dev, "wrong IRQ\n");
			break;
		}
	}
	/* update mask into registers */
	for (i = 0; i < PM800_INT_REG_NUM; i++) {
		if (mask[i] != cached[i]) {
			cached[i] = mask[i];
			pm80x_reg_write(i2c, PM800_INT_ENA_1 + i, mask[i]);
		}
	}

	mutex_unlock(&chip->pm800_irq_lock);
}

static void pm800_irq_enable(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);
	struct pm80x_subchip *pm800_chip = chip->pm800_chip;
	pm800_irqs[data->irq - pm800_chip->irq_base].enable
	    = pm800_irqs[data->irq - pm800_chip->irq_base].offs;
}

static void pm800_irq_disable(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);
	struct pm80x_subchip *pm800_chip = chip->pm800_chip;
	pm800_irqs[data->irq - pm800_chip->irq_base].enable = 0;
}

static struct irq_chip pm800_irq_chip = {
	.name = "88pm80x",
	.irq_bus_lock = pm800_irq_lock,
	.irq_bus_sync_unlock = pm800_irq_sync_unlock,
	.irq_enable = pm800_irq_enable,
	.irq_disable = pm800_irq_disable,
};

static struct pm80x_irq_data pm805_irqs[] = {
	[PM805_IRQ_LDO_OFF] = {/*0*/
		.reg		= PM805_INT_STATUS1,
		.mask_reg	= PM805_INT_MASK1,
		.offs		= 1 << 5,
	},
	[PM805_IRQ_SRC_DPLL_LOCK] = {
		.reg		= PM805_INT_STATUS1,
		.mask_reg	= PM805_INT_MASK1,
		.offs		= 1 << 4,
	},
	[PM805_IRQ_CLIP_FAULT] = {
		.reg		= PM805_INT_STATUS1,
		.mask_reg	= PM805_INT_MASK1,
		.offs		= 1 << 3,
	},
	[PM805_IRQ_MIC_CONFLICT] = {
		.reg		= PM805_INT_STATUS1,
		.mask_reg	= PM805_INT_MASK1,
		.offs		= 1 << 2,
	},
	[PM805_IRQ_HP2_SHRT] = {
		.reg		= PM805_INT_STATUS1,
		.mask_reg	= PM805_INT_MASK1,
		.offs		= 1 << 1,
	},
	[PM805_IRQ_HP1_SHRT] = {/*5*/
		.reg		= PM805_INT_STATUS1,
		.mask_reg	= PM805_INT_MASK1,
		.offs		= 1 << 0,
	},
	[PM805_IRQ_FINE_PLL_FAULT] = {
		.reg		= PM805_INT_STATUS2,
		.mask_reg	= PM805_INT_MASK2,
		.offs		= 1 << 5,
	},
	[PM805_IRQ_RAW_PLL_FAULT] = {
		.reg		= PM805_INT_STATUS2,
		.mask_reg	= PM805_INT_MASK2,
		.offs		= 1 << 4,
	},
	[PM805_IRQ_VOLP_BTN_DET] = {
		.reg		= PM805_INT_STATUS2,
		.mask_reg	= PM805_INT_MASK2,
		.offs		= 1 << 3,
	},
	[PM805_IRQ_VOLM_BTN_DET] = {
		.reg		= PM805_INT_STATUS2,
		.mask_reg	= PM805_INT_MASK2,
		.offs		= 1 << 2,
	},
	[PM805_IRQ_SHRT_BTN_DET] = {/*10*/
		.reg		= PM805_INT_STATUS2,
		.mask_reg	= PM805_INT_MASK2,
		.offs		= 1 << 1,
	},
	[PM805_IRQ_MIC_DET] = {/*11*/
		.reg		= PM805_INT_STATUS2,
		.mask_reg	= PM805_INT_MASK2,
		.offs		= 1 << 0,
	},
};

static inline struct pm80x_irq_data *irq_to_pm805(struct pm80x_chip *chip,
						  int irq)
{
	if (!chip->pm805_chip || irq < chip->pm805_chip->irq_base)
		return NULL;
	return &pm805_irqs[irq - chip->pm805_chip->irq_base];
}

static irqreturn_t pm805_irq(int irq, void *data)
{
	struct pm80x_chip *chip = data;
	struct pm80x_subchip *pm805_chip = chip->pm805_chip;
	struct pm80x_irq_data *irq_data;
	struct i2c_client *i2c;
	int read_reg = -1, value = 0;
	int i;

	i2c = pm805_chip->client;

	for (i = 0; i < ARRAY_SIZE(pm805_irqs); i++) {
		irq_data = &pm805_irqs[i];
		if (read_reg != irq_data->reg) {
			read_reg = irq_data->reg;
			value = pm80x_reg_read(i2c, irq_data->reg);
		}
		if (value & irq_data->enable)
			handle_nested_irq(pm805_chip->irq_base + i);
	}
	return IRQ_HANDLED;
}

static void pm805_irq_lock(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);

	mutex_lock(&chip->pm805_irq_lock);
}

static void pm805_irq_sync_unlock(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);
	struct pm80x_subchip *pm805_chip = chip->pm805_chip;
	struct pm80x_irq_data *irq_data;
	struct i2c_client *i2c;
	static unsigned char cached[PM805_INT_REG_NUM] = {0x0};
	unsigned char mask[PM805_INT_REG_NUM];
	int i;

	i2c = pm805_chip->client;

	/* Load cached value. In initial, all IRQs are masked */
	for (i = 0; i < PM805_INT_REG_NUM; i++)
		mask[i] = cached[i];
	for (i = 0; i < ARRAY_SIZE(pm805_irqs); i++) {
		irq_data = &pm805_irqs[i];
		switch (irq_data->mask_reg) {
		case PM805_INT_MASK1:
			mask[0] &= ~irq_data->offs;
			mask[0] |= irq_data->enable;
			break;
		case PM805_INT_MASK2:
			mask[1] &= ~irq_data->offs;
			mask[1] |= irq_data->enable;
			break;
		default:
			dev_err(chip->dev, "wrong IRQ\n");
			break;
		}
	}
	/* update mask into registers */
	for (i = 0; i < PM805_INT_REG_NUM; i++) {
		if (mask[i] != cached[i]) {
			cached[i] = mask[i];
			pm80x_reg_write(i2c, PM805_INT_MASK1 + i, mask[i]);
		}
	}

	mutex_unlock(&chip->pm805_irq_lock);
}

static void pm805_irq_enable(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);
	struct pm80x_subchip *pm805_chip = chip->pm805_chip;
	pm805_irqs[data->irq - pm805_chip->irq_base].enable
		= pm805_irqs[data->irq - pm805_chip->irq_base].offs;
}

static void pm805_irq_disable(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);
	struct pm80x_subchip *pm805_chip = chip->pm805_chip;
	pm805_irqs[data->irq - pm805_chip->irq_base].enable = 0;
}

static struct irq_chip pm805_irq_chip = {
	.name			= "88pm805",
	.irq_bus_lock		= pm805_irq_lock,
	.irq_bus_sync_unlock	= pm805_irq_sync_unlock,
	.irq_enable		= pm805_irq_enable,
	.irq_disable		= pm805_irq_disable,
};

static int __devinit device_gpadc_init(struct pm80x_chip *chip,
				       struct pm80x_platform_data *pdata)
{
	struct i2c_client *i2c_gpadc = chip->gpadc_page;
	int data = 0, mask = 0, ret = 0;

	if (!i2c_gpadc) {
		dev_warn(chip->dev, "Warning: I2C gpdac page is not available!\n");
		return -EINVAL;
	}
	/* initialize GPADC without activating it */
	/* turn on GPADC measurments */
	ret = pm80x_set_bits(i2c_gpadc,
			     PM800_GPADC_MISC_CONFIG2,
			     PM800_GPADC_MISC_GPFSM_EN,
			     PM800_GPADC_MISC_GPFSM_EN);
	if (ret < 0)
		goto out;
	/*
	   This function configures the ADC as requires for
	   CP implementation.CP does not "own" the ADC configuration
	   registers and relies on AP.
	   Reason: enable automatic ADC measurements needed
	   for CP to get VBAT and RF temperature readings.
	 */
	ret = pm80x_set_bits(i2c_gpadc, PM800_GPADC_MEAS_EN1,
			     PM800_MEAS_EN1_VBAT, PM800_MEAS_EN1_VBAT);
	if (ret < 0)
		goto out;
	ret = pm80x_set_bits(i2c_gpadc, PM800_GPADC_MEAS_EN2,
			     (PM800_MEAS_EN2_RFTMP | PM800_MEAS_GP0_EN),
			     (PM800_MEAS_EN2_RFTMP | PM800_MEAS_GP0_EN));
	if (ret < 0)
		goto out;

	/* the defult of PM800 is GPADC operates at 100Ks/s rate */
	/* and Number of GPADC slots with active current bias prior to
	   GPADC sampling = 1 slot for all GPADCs */

	/* set for Temprature mesurmants */
	mask = (PM800_GPADC_GP_BIAS_EN0 | PM800_GPADC_GP_BIAS_EN1 |
			PM800_GPADC_GP_BIAS_EN2 | PM800_GPADC_GP_BIAS_EN3);

	if (pdata && (pdata->batt_det == 0)) {
		data = (PM800_GPADC_GP_BIAS_EN0 | PM800_GPADC_GP_BIAS_EN1 |
			PM800_GPADC_GP_BIAS_EN2 | PM800_GPADC_GP_BIAS_EN3);
	} else {
		data = (PM800_GPADC_GP_BIAS_EN0 | PM800_GPADC_GP_BIAS_EN2 |
				PM800_GPADC_GP_BIAS_EN3);
	}
	ret = pm80x_set_bits(i2c_gpadc, PM800_GP_BIAS_ENA1, mask, data);
	if (ret < 0)
		goto out;

	dev_info(chip->dev, "pm80x device_gpadc_init: Done\n");
	return 0;

out:
	dev_info(chip->dev, "pm80x device_gpadc_init: Failed!\n");
	return ret;
}

static void genirq_init_800(struct pm80x_chip *chip, int irq_base)
{
	int i, __irq;
	for (i = 0; i < ARRAY_SIZE(pm800_irqs); i++) {
		__irq = i + irq_base;
		irq_set_chip_data(__irq, chip);
		irq_set_chip_and_handler(__irq, &pm800_irq_chip,
						handle_edge_irq);
		irq_set_nested_thread(__irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(__irq, IRQF_VALID);
#else
		irq_set_noprobe(__irq);
#endif
	}
}

static void genirq_exit_800(int irq_base)
{
	int i, __irq;
	for (i = 0; i < ARRAY_SIZE(pm800_irqs); i++) {
		__irq = i + irq_base;
#ifdef CONFIG_ARM
		set_irq_flags(__irq, 0);
#endif
		irq_set_chip_and_handler(__irq, NULL, NULL);
		irq_set_chip_data(__irq, NULL);
	}
}

static int __devinit device_irq_init_800(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata)
{
	struct pm80x_subchip *pm800_chip = chip->pm800_chip;
	struct i2c_client *i2c_base = chip->base_page;
	unsigned char status_buf[PM800_INT_REG_NUM];
	unsigned long flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED;
	struct irq_desc *desc;
	int data, mask, ret = -EINVAL;
	int irq, irq_base;

	if (!pdata) {
		dev_warn(chip->dev, "missing platform data\n");
		return -EINVAL;
	}
	if (!i2c_base) {
		dev_warn(chip->dev, "missing base_page\n");
		return -EINVAL;
	}

	irq = pm800_chip->irq;
	irq_base = pm800_chip->irq_base;

	if (!irq) {
		dev_warn(chip->dev, "No interrupt IRQ for pm800\n");
		return -EINVAL;
	}

	mask = PM800_WAKEUP2_INV_INT | PM800_WAKEUP2_INT_CLEAR
	    | PM800_WAKEUP2_INT_MASK;
	data = 0;
	if (pdata && pdata->irq_mode) {
		/*
		 * irq_mode defines the way of clearing interrupt. If it's 1,
		 * clear IRQ by write. Otherwise, clear it by read.
		 */
		data |= PM800_WAKEUP2_INT_CLEAR;
		pm800_chip->irq_mode = 1;
	}
	ret = pm80x_set_bits(i2c_base, PM800_WAKEUP2, mask, data);
	if (ret < 0)
		goto out;

	/* mask all IRQs */
	memset(status_buf, 0, PM800_INT_REG_NUM);
	ret = pm80x_bulk_write(i2c_base, PM800_INT_ENA_1,
			       PM800_INT_REG_NUM, status_buf);
	if (ret < 0)
		goto out;

	if (pm800_chip->irq_mode) {
		/* clear interrupt status by write */
		memset(status_buf, 0xFF, PM800_INT_REG_NUM);
		ret = pm80x_bulk_write(i2c_base, PM800_INT_STATUS1,
				       PM800_INT_REG_NUM, status_buf);
	} else {
		/* clear interrupt status by read */
		ret = pm80x_bulk_read(i2c_base, PM800_INT_STATUS1,
				      PM800_INT_REG_NUM, status_buf);
	}
	if (ret < 0)
		goto out;

	mutex_init(&chip->pm800_irq_lock);

	desc = irq_to_desc(irq);
	pm800_irq_chip.irq_set_wake = desc->irq_data.chip->irq_set_wake;

	/* Register IRQ by genirq */
	genirq_init_800(chip, irq_base);
	/* Request IRQ */
	ret = request_threaded_irq(irq, NULL, pm800_irq, flags,
				   "88pm800", chip);
	if (ret) {
		pm800_chip->irq = 0;
		dev_err(chip->dev, "Failed to request pm800 IRQ: %d\n", ret);
		goto out_request_irq;
	}
	return 0;

out_request_irq:
	genirq_exit_800(irq_base);
out:
	return ret;
}

static void device_irq_exit_800(struct pm80x_chip *chip)
{
	if (chip->pm800_chip && chip->pm800_chip->irq) {
		free_irq(chip->pm800_chip->irq, chip);
		genirq_exit_800(chip->pm800_chip->irq_base);
	}
}

static void genirq_init_805(struct pm80x_chip *chip, int irq_base)
{
	int i, __irq;
	for (i = 0; i < ARRAY_SIZE(pm805_irqs); i++) {
		__irq = i + irq_base;
		irq_set_chip_data(__irq, chip);
		irq_set_chip_and_handler(__irq, &pm805_irq_chip,
					 handle_edge_irq);
		irq_set_nested_thread(__irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(__irq, IRQF_VALID);
#else
		irq_set_noprobe(__irq);
#endif
	}
}

static void genirq_exit_805(int irq_base)
{
	int i, __irq;
	for (i = 0; i < ARRAY_SIZE(pm805_irqs); i++) {
		__irq = i + irq_base;
#ifdef CONFIG_ARM
		set_irq_flags(__irq, 0);
#endif
		irq_set_chip_and_handler(__irq, NULL, NULL);
		irq_set_chip_data(__irq, NULL);
	}
}

static int __devinit device_irq_init_805(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata)
{
	struct pm80x_subchip *pm805_chip = chip->pm805_chip;
	struct i2c_client *i2c = pm805_chip->client;
	unsigned char status_buf[PM805_INT_REG_NUM];
	unsigned long flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	struct irq_desc *desc;
	int data, mask, ret = -EINVAL;
	int irq, irq_base;

	if (!pdata) {
		dev_warn(chip->dev, "missing platform data\n");
		return -EINVAL;
	}

	irq = pm805_chip->irq;
	irq_base = pm805_chip->irq_base;
	if (!irq) {
		dev_warn(chip->dev, "No interrupt IRQ for pm805\n");
		return -EINVAL;
	}
	mask = PM805_STATUS0_INT_CLEAR | PM805_STATUS0_INV_INT
			| PM800_STATUS0_INT_MASK;

	data = 0;

	if (pdata->irq_mode) {
		data |= PM805_STATUS0_INT_CLEAR;
		pm805_chip->irq_mode = 1;
	}

	/* Set active low */
	/* data |= PM805_STATUS0_INV_INT; */

	ret = pm80x_set_bits(i2c, PM805_INT_STATUS0, mask, data);
	if (ret < 0)
		goto out;

	/* Need to use delay between accesses to 32K-registers */
	msleep(1);

	/* mask all IRQs */
	memset(status_buf, 0, PM805_INT_REG_NUM);
	ret = pm80x_bulk_write(i2c, PM805_INT_MASK1,
				PM805_INT_REG_NUM, status_buf);
	if (ret < 0)
		goto out;

	/* Need to use delay between accesses to 32K-registers */
	msleep(1);

	if (pm805_chip->irq_mode) {
		/* clear interrupt status by write */
		memset(status_buf, 0xFF, PM805_INT_REG_NUM);
		ret = pm80x_bulk_write(i2c, PM805_INT_STATUS1,
					PM805_INT_REG_NUM, status_buf);
	} else {
		/* clear interrupt status by read */
		ret = pm80x_bulk_read(i2c, PM805_INT_STATUS1,
					PM805_INT_REG_NUM, status_buf);
	}
	if (ret < 0)
		goto out;

	mutex_init(&chip->pm805_irq_lock);

	if (!irq)
		goto out;

	desc = irq_to_desc(irq);
	pm805_irq_chip.irq_set_wake = desc->irq_data.chip->irq_set_wake;

	/* Register IRQ by genirq */
	genirq_init_805(chip, irq_base);
	/* Request IRQ */
	ret = request_threaded_irq(irq, NULL, pm805_irq, flags,
				   "88pm805", chip);
	if (ret) {
		pm805_chip->irq = 0;
		dev_err(chip->dev, "Failed to request pm805 IRQ: %d\n", ret);
		goto out_request_irq;
	}
	return 0;

out_request_irq:
	genirq_exit_805(irq_base);
out:
	return ret;
}

static void device_irq_exit_805(struct pm80x_chip *chip)
{
	if (chip->pm805_chip && chip->pm805_chip->irq) {
		free_irq(chip->pm805_chip->irq, chip);
		genirq_exit_805(chip->pm805_chip->irq_base);
	}
}

static int __devinit device_805_init(struct pm80x_chip *chip,
				     struct i2c_client *i2c,
				     struct pm80x_platform_data *pdata)
{
	int ret = 0;
	struct pm80x_subchip *pm805_chip;

	dev_info(chip->dev,
		"pm80x:%s slave addr[0x%x]\n", __func__, i2c->addr);

	/* Init PM805 subchip */
	pm805_chip = kzalloc(sizeof(struct pm80x_subchip), GFP_KERNEL);
	if (!pm805_chip)
		return -ENOMEM;
	chip->pm805_chip = pm805_chip;
	pm805_chip->dev = chip->dev;
	pm805_chip->chip = chip;
	pm805_chip->pdata = pdata;
	pm805_chip->client = i2c;
	pm805_chip->irq = (chip->id == CHIP_PM805) ? chip->client->irq
			: chip->irq_companion;
	pm805_chip->irq_base = (chip->id == CHIP_PM805) ? chip->irq_base
			: chip->irq_base + PM800_MAX_IRQ;

	ret = pm80x_reg_read(i2c, PM805_CHIP_ID);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read CHIP ID: %d\n", ret);
		goto out_chip_id;
	}
	chip->chip805_version = ret;

	ret = device_irq_init_805(chip, pdata);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to init pm805 irq!\n");
		goto out_irq_init;
	}

	chip->pm805_wqueue = create_singlethread_workqueue("88pm805");
	if (!chip->pm805_wqueue) {
		dev_info(chip->dev,
			"[%s]Failed to create pm805_wqueue\n", __func__);
		ret = -ESRCH;
		goto out_work;
	}

	ret = mfd_add_devices(chip->dev, 0, &codec_devs[0],
			      ARRAY_SIZE(codec_devs), &codec_resources[0], 0);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add codec subdev\n");
		goto out_codec;
	} else
		dev_info(chip->dev,
			"[%s]:Added mfd codec_devs\n", __func__);

	if (chip->chip805_version == PM805_CHIP_B0) {
		ret = mfd_add_devices(chip->dev, 0, &headset_devs[0],
				      ARRAY_SIZE(headset_devs),
				      &headset_resources[0], 0);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add headset subdev\n");
			goto out_headset;
		} else
			dev_info(chip->dev, "[%s]:Added mfd headset_devs\n", __func__);
	}
	if (pdata->pm805_plat_config)
		pdata->pm805_plat_config(chip, pdata);

	return 0;

out_headset:
	mfd_remove_devices(chip->dev);
out_codec:
	destroy_workqueue(chip->pm805_wqueue);
out_work:
	device_irq_exit_805(chip);
out_irq_init:
out_chip_id:
	kfree(chip->pm805_chip);
	chip->pm805_chip = NULL;
	return ret;
}

static int __devinit device_regulator_init(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata)
{
	struct regulator_init_data *initdata;
	int ret = 0;
	int i, seq;

	if (!pdata || !pdata->regulator) {
		dev_warn(chip->dev, "Warning: Regulator pdata is not available!\n");
		return 0;
	}
	if (!chip->power_page) {
		dev_warn(chip->dev, "Warning: I2C power page is not available!\n");
		return 0;
	}

	if (pdata->num_regulators > ARRAY_SIZE(regulator_devs))
		pdata->num_regulators = ARRAY_SIZE(regulator_devs);

	for (i = 0, seq = -1; i < pdata->num_regulators; i++) {
		initdata = &pdata->regulator[i];
		seq = *(unsigned int *)initdata->driver_data;
		if ((seq < 0) || (seq > PM800_ID_RG_MAX)) {
			dev_err(chip->dev, "Wrong ID(%d) on regulator(%s)\n",
				seq, initdata->constraints.name);
			ret = -EINVAL;
			goto out_err;
		}
		memcpy(&regulator_pdata[i], &pdata->regulator[i],
		       sizeof(struct regulator_init_data));
		regulator_devs[i].platform_data = &regulator_pdata[i];
		regulator_devs[i].pdata_size = sizeof(struct regulator_init_data);
		regulator_devs[i].num_resources = 1;
		regulator_devs[i].resources = &regulator_resources[seq];

		ret = mfd_add_devices(chip->dev, 0, &regulator_devs[i], 1,
				      &regulator_resources[seq], 0);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add regulator subdev\n");
			goto out_err;
		}
	}
	dev_info(chip->dev, "[%s]:Added mfd regulator_devs\n", __func__);
	return 0;

out_err:
	return ret;
}

static int __devinit device_800_init(struct pm80x_chip *chip,
				      struct i2c_client *i2c,
				      struct pm80x_platform_data *pdata)
{
	struct i2c_client *i2c_base = chip->base_page;
	struct pm80x_subchip *pm800_chip;
	int ret, pmic_id;

	if (!i2c_base) {
		dev_err(chip->dev, "base_page is invalid\n");
		return -EINVAL;
	}

	/* Init PM800 subchip */
	pm800_chip = kzalloc(sizeof(struct pm80x_subchip), GFP_KERNEL);
	if (!pm800_chip)
		return -ENOMEM;
	chip->pm800_chip = pm800_chip;

	pm800_chip->dev = chip->dev;
	pm800_chip->chip = chip;
	pm800_chip->pdata = pdata;
	pm800_chip->client = i2c;
	pm800_chip->irq = (chip->id == CHIP_PM800) ? chip->client->irq
			: chip->irq_companion;
	pm800_chip->irq_base = (chip->id == CHIP_PM800) ? chip->irq_base
			: chip->irq_base + PM805_MAX_IRQ;

	ret = pm80x_reg_read(i2c, PM800_CHIP_ID);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read CHIP ID: %d\n", ret);
		goto out;
	}

	pmic_id = ret & PM8XXX_VERSION_MASK;

	if ((pmic_id >= PM800_CHIP_A0) && (pmic_id <= PM800_CHIP_END)) {
		chip->chip800_version = ret;
		dev_info(chip->dev,
			 "88PM80x:Marvell 88PM800 (ID:0x%x) detected\n", ret);
	} else {
		dev_err(chip->dev,
			"Failed to detect Marvell 88PM800:ChipID[0x%x]\n", ret);
		goto out;
	}

	/*
	 * alarm wake up bit will be clear in device_irq_init(),
	 * read before that
	 */
	ret = pm80x_reg_read(chip->base_page, PM800_RTC_CONTROL);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read RTC register: %d\n", ret);
		goto out;
	}
	if (ret & PM800_ALARM_WAKEUP) {
		if (pdata && pdata->rtc)
			pdata->rtc->rtc_wakeup = 1;
	}

	ret = device_gpadc_init(chip, pdata);
	if (ret < 0) {
		dev_info(chip->dev,
			 "[%s]Failed to init gpadc\n", __func__);
		goto out;
	}

	ret = device_irq_init_800(chip, pdata);
	if (ret < 0) {
		dev_info(chip->dev,
			"[%s]Failed to init pm800 irq\n", __func__);
		goto out;
	}

	/* PM800 common wqueue: used in headset, battery, rtc */
	chip->pm800_wqueue = create_singlethread_workqueue("88pm800");
	if (!chip->pm800_wqueue) {
		dev_info(chip->dev,
			"[%s]Failed to create pm800_wqueue\n", __func__);
		ret = -ESRCH;
		goto out_work;
	}

	if (device_regulator_init(chip, pdata)) {
		dev_err(chip->dev, "Failed to init regulators\n");
		goto out_dev;
	}

	if (pdata) {
		ret = mfd_add_devices(chip->dev, 0, &vbus_devs[0],
				      ARRAY_SIZE(vbus_devs),
				      &vbus_resources[0], pm800_chip->irq_base);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add vbus subdev\n");
			goto out_dev;
		} else
			dev_info(chip->dev,
				"[%s]:Added mfd vbus_devs\n", __func__);
	}

	ret = mfd_add_devices(chip->dev, 0, &onkey_devs[0],
			      ARRAY_SIZE(onkey_devs), &onkey_resources[0], 0);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add onkey subdev\n");
		goto out_dev;
	} else
		dev_info(chip->dev,
			"[%s]:Added mfd onkey_devs\n", __func__);

	if (chip->chip800_version >= PM800_CHIP_C0) {
		ret = mfd_add_devices(chip->dev, 0, &headset_devs_800[0],
					ARRAY_SIZE(headset_devs_800), &headset_resources_800[0], 0);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add headset subdev\n");
			goto out_dev;
		} else
			dev_info(chip->dev, "[%s]:Added mfd headset_devs\n", __func__);
	}

	if (pdata && pdata->vibrator) {
		vibrator_devs[0].platform_data = pdata->vibrator;
		vibrator_devs[0].pdata_size = sizeof(struct pm80x_vibrator_pdata);
		ret = mfd_add_devices(chip->dev, 0, &vibrator_devs[0],
				ARRAY_SIZE(vibrator_devs), NULL, 0);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add vibrator subdev\n");
			goto out_dev;
		} else
			dev_info(chip->dev,
				"[%s]:Added mfd vibrator_devs\n", __func__);
	}

	if (pdata && pdata->rtc) {
		rtc_devs[0].platform_data = pdata->rtc;
		rtc_devs[0].pdata_size = sizeof(struct pm80x_rtc_pdata);
		ret = mfd_add_devices(chip->dev, 0, &rtc_devs[0],
				ARRAY_SIZE(rtc_devs), NULL, pm800_chip->irq_base);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add rtc subdev\n");
			goto out_dev;
		} else
			dev_info(chip->dev,
				"[%s]:Added mfd rtc_devs\n", __func__);
	}

	if (pdata && pdata->dvc) {
		dvc_devs[0].platform_data = pdata->dvc;
		dvc_devs[0].pdata_size = sizeof(struct pm80x_dvc_pdata);
		ret = mfd_add_devices(chip->dev, 0, &dvc_devs[0],
				      ARRAY_SIZE(dvc_devs), NULL,
				      pm800_chip->irq_base);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add dvc subdev\n");
			goto out_dev;
		}
	}
	if (chip->chip800_version == PM800_CHIP_B0) {
		ret = mfd_add_devices(chip->dev, 0, &pm80x_gpio_devs[0],
					ARRAY_SIZE(pm80x_gpio_devs), NULL, pm800_chip->irq_base);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add gpio subdev\n");
			goto out_dev;
		} else
			dev_info(chip->dev, "[%s]:Added mfd gpio_devs\n", __func__);
	}

	if (pdata->pm800_plat_config)
		pdata->pm800_plat_config(chip, pdata);

	return 0;
out_dev:
	mfd_remove_devices(chip->dev);
	destroy_workqueue(chip->pm800_wqueue);
out_work:
	device_irq_exit_800(chip);
out:
	kfree(chip->pm800_chip);
	chip->pm800_chip = NULL;
	return ret;
}

int __devinit pm80x_device_init(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata)
{
	int ret = 0;

	switch (chip->id) {
	case CHIP_PM800:
		/* set PM800 as main chip */
		ret = device_800_init(chip, chip->client, pdata);
		break;
	case CHIP_PM805:
		/* set PM805 as main chip */
		ret = device_805_init(chip, chip->client, pdata);
		break;
	}
	if (ret) {
		dev_err(chip->dev, "%s failed!\n", __func__);
		return ret;
	}

	if (chip->companion) {
		switch (chip->id) {
		case CHIP_PM800:
			/* PM800 is main chip, PM805 is companion chip */
			ret = device_805_init(chip, chip->companion, pdata);
			break;
		case CHIP_PM805:
			/* PM805 is main chip, PM800 is companion chip */
			ret = device_800_init(chip, chip->companion, pdata);
			break;
		}
	}
	if (ret)
		dev_err(chip->dev, "%s failed!\n", __func__);

	return ret;
}

void __devexit pm80x_device_exit(struct pm80x_chip *chip)
{
	if (chip->pm800_chip) {
		device_irq_exit_800(chip);
		if (chip->pm800_wqueue) {
			flush_workqueue(chip->pm800_wqueue);
			destroy_workqueue(chip->pm800_wqueue);
		}
		kfree(chip->pm800_chip);
	}
	if (chip->pm805_chip) {
		device_irq_exit_805(chip);
		if (chip->pm805_wqueue) {
			flush_workqueue(chip->pm805_wqueue);
			destroy_workqueue(chip->pm805_wqueue);
		}
		kfree(chip->pm805_chip);
	}
	mfd_remove_devices(chip->dev);
}

MODULE_DESCRIPTION("PMIC Driver for Marvell 88PM80x");
MODULE_AUTHOR("Joseph(Yossi) Hanin <yhanin@marvell.com>");
MODULE_LICENSE("GPL");
