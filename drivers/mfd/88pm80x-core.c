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

static struct resource onkey_resources[] = {
	{
	 .name = "88pm80x-onkey",
	 .start = PM800_IRQ_ONKEY,
	 .end = PM800_IRQ_ONKEY,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct mfd_cell onkey_devs[] = {
	{
	 .name = "88pm80x-onkey",
	 .num_resources = 1,
	 .resources = &onkey_resources[0],
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
	 }
	,
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

static struct pm80x_irq_data pm80x_irqs[] = {
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

static inline struct pm80x_irq_data *irq_to_pm80x(struct pm80x_chip *chip,
						  int irq)
{
	return &pm80x_irqs[irq - chip->irq_base];
}

static irqreturn_t pm80x_irq(int irq, void *data)
{
	struct pm80x_chip *chip = data;
	struct pm80x_irq_data *irq_data;
	struct i2c_client *i2c = chip->base_page;
	int read_reg = -1, value = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(pm80x_irqs); i++) {
		irq_data = &pm80x_irqs[i];
		if (read_reg != irq_data->reg) {
			read_reg = irq_data->reg;
			value = pm80x_reg_read(i2c, irq_data->reg);
		}
		if (value & irq_data->enable)
			handle_nested_irq(chip->irq_base + i);
	}
	return IRQ_HANDLED;
}

static void pm80x_irq_lock(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);

	mutex_lock(&chip->irq_lock);
}

static void pm80x_irq_sync_unlock(struct irq_data *data)
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
	for (i = 0; i < ARRAY_SIZE(pm80x_irqs); i++) {
		irq_data = &pm80x_irqs[i];
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

	mutex_unlock(&chip->irq_lock);
}

static void pm80x_irq_enable(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);
	pm80x_irqs[data->irq - chip->irq_base].enable
	    = pm80x_irqs[data->irq - chip->irq_base].offs;
}

static void pm80x_irq_disable(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);
	pm80x_irqs[data->irq - chip->irq_base].enable = 0;
}

static struct irq_chip pm80x_irq_chip = {
	.name = "88pm80x",
	.irq_bus_lock = pm80x_irq_lock,
	.irq_bus_sync_unlock = pm80x_irq_sync_unlock,
	.irq_enable = pm80x_irq_enable,
	.irq_disable = pm80x_irq_disable,
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

static irqreturn_t pm805_irq(int irq, void *data)
{
	struct pm80x_chip *chip = data;
	struct pm80x_irq_data *irq_data;
	struct i2c_client *i2c;
	int read_reg = -1, value = 0;
	int i;

	i2c = chip->companion;

	for (i = 0; i < ARRAY_SIZE(pm805_irqs); i++) {
		irq_data = &pm805_irqs[i];
		if (read_reg != irq_data->reg) {
			read_reg = irq_data->reg;
			value = pm80x_reg_read(i2c, irq_data->reg);
		}
		if (value & irq_data->enable)
			handle_nested_irq(chip->irq_companion_base + i);
	}
	return IRQ_HANDLED;
}

static void pm805_irq_lock(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);

	mutex_lock(&chip->companion_irq_lock);
}

static void pm805_irq_sync_unlock(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);
	struct pm80x_irq_data *irq_data;
	struct i2c_client *i2c;
	static unsigned char cached[PM805_INT_REG_NUM] = {0x0};
	unsigned char mask[PM805_INT_REG_NUM];
	int i;

	i2c = chip->companion;

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

	mutex_unlock(&chip->companion_irq_lock);
}

static void pm805_irq_enable(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);
	pm805_irqs[data->irq - chip->irq_companion_base].enable
		= pm805_irqs[data->irq - chip->irq_companion_base].offs;
}

static void pm805_irq_disable(struct irq_data *data)
{
	struct pm80x_chip *chip = irq_data_get_irq_chip_data(data);
	pm805_irqs[data->irq - chip->irq_companion_base].enable = 0;
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
	mask = (PM800_GPADC_GP_BIAS_EN0 |
		PM800_GPADC_GP_BIAS_EN1 |
		PM800_GPADC_GP_BIAS_EN2 | PM800_GPADC_GP_BIAS_EN3);

	if (pdata && (pdata->batt_det == 0)) {
		data = (PM800_GPADC_GP_BIAS_EN0 |
			PM800_GPADC_GP_BIAS_EN1 |
			PM800_GPADC_GP_BIAS_EN2 | PM800_GPADC_GP_BIAS_EN3);
	} else {
		data = (PM800_GPADC_GP_BIAS_EN0 |
			PM800_GPADC_GP_BIAS_EN2 | PM800_GPADC_GP_BIAS_EN3);
	}
	pm80x_set_bits(i2c_gpadc, PM800_GP_BIAS_ENA1, mask, data);
	dev_info(chip->dev, "pm80x device_gpadc_init:initialize GPADC!!!\n");
	return 0;

out:
	dev_info(chip->dev, "pm80x device_gpadc_init:FAIL initialize !!!\n");

	return ret;
}

static int __devinit device_irq_init_800(struct pm80x_chip *chip,
				     struct pm80x_platform_data *pdata)
{
	struct i2c_client *i2c = chip->client;
	struct i2c_client *i2c_base = chip->base_page;
	unsigned char status_buf[PM800_INT_REG_NUM];
	unsigned long flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED;
	struct irq_desc *desc;
	int i, data, mask, ret = -EINVAL;
	int __irq;

	if (!pdata || !pdata->irq_base) {
		dev_warn(chip->dev, "No interrupt support on IRQ base\n");
		return -EINVAL;
	}

	mask = PM800_WAKEUP2_INV_INT | PM800_WAKEUP2_INT_CLEAR
	    | PM800_WAKEUP2_INT_MASK;
	data = 0;
	chip->irq_mode = 0;
	if (pdata && pdata->irq_mode) {
		/*
		 * irq_mode defines the way of clearing interrupt. If it's 1,
		 * clear IRQ by write. Otherwise, clear it by read.
		 */
		data |= PM800_WAKEUP2_INT_CLEAR;
		chip->irq_mode = 1;
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

	if (chip->irq_mode) {
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

	mutex_init(&chip->irq_lock);
	chip->irq_base = pdata->irq_base;
	chip->core_irq = i2c->irq;
	chip->irq_companion = pdata->irq_companion;
	chip->irq_companion_base = pdata->irq_base + ARRAY_SIZE(pm80x_irqs);
	if (!chip->core_irq)
		goto out;

	desc = irq_to_desc(chip->core_irq);

	/* register IRQ by genirq */
	for (i = 0; i < ARRAY_SIZE(pm80x_irqs); i++) {
		__irq = i + chip->irq_base;
		irq_set_chip_data(__irq, chip);
		irq_set_chip_and_handler(__irq, &pm80x_irq_chip,
					 handle_edge_irq);
		irq_set_nested_thread(__irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(__irq, IRQF_VALID);
#else
		irq_set_noprobe(__irq);
#endif
	}

	ret = request_threaded_irq(chip->core_irq, NULL, pm80x_irq, flags,
				   "88pm80x", chip);
	if (ret) {
		dev_err(chip->dev, "Failed to request IRQ: %d\n", ret);
		chip->core_irq = 0;
		goto out;
	}

	return 0;
out:
	chip->core_irq = 0;
	return ret;
}

static void device_irq_exit_800(struct pm80x_chip *chip)
{
	if (chip->core_irq)
		free_irq(chip->core_irq, chip);
}

static int __devinit device_irq_init_805(struct pm80x_chip *chip,
				     struct pm80x_platform_data *pdata)
{
	struct i2c_client *i2c = chip->companion;
	unsigned char status_buf[PM805_INT_REG_NUM];
	unsigned long flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	struct irq_desc *desc;
	int i, data, mask, ret = -EINVAL;
	int __irq;

	if (!pdata || !chip->irq_companion_base) {
		dev_warn(chip->dev, "No interrupt support on IRQ base\n");
		return -EINVAL;
	}

	mask = PM805_STATUS0_INT_CLEAR | PM805_STATUS0_INV_INT
			| PM800_STATUS0_INT_MASK;

	data = 0;
	chip->irq_mode = 0;

	if (pdata->irq_mode) {
		data |= PM805_STATUS0_INT_CLEAR;
		chip->irq_mode = 1;
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

	if (chip->irq_mode) {
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

	mutex_init(&chip->companion_irq_lock);

	if (!chip->irq_companion)
		goto out;

	desc = irq_to_desc(chip->irq_companion);
	pm805_irq_chip.irq_set_wake = desc->irq_data.chip->irq_set_wake;

	/* register IRQ by genirq */
	for (i = 0; i < ARRAY_SIZE(pm805_irqs); i++) {
		__irq = i + chip->irq_companion_base;
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

	ret = request_threaded_irq(chip->irq_companion, NULL, pm805_irq, flags,
				   "88pm805", chip);
	if (ret) {
		dev_err(chip->dev, "Failed to request IRQ: %d\n", ret);
		chip->irq_companion = 0;
	}

	return 0;
out:
	chip->irq_companion = 0;
	return ret;
}

static void device_irq_exit_805(struct pm80x_chip *chip)
{
	if (chip->irq_companion)
		free_irq(chip->irq_companion, chip);
}

static int __devinit device_805_init(struct pm80x_chip *chip,
				     struct i2c_client *i2c,
				     struct pm80x_platform_data *pdata)
{
	int ret = 0;
	struct i2c_client *i2c_comp = chip->companion;
	struct pm805_chip *chip805;

	dev_info(chip->dev, "pm80x:device_805_init slave[%x]\n", (int)i2c_comp);

	chip805 = kzalloc(sizeof(struct pm805_chip), GFP_KERNEL);
	if (chip805 == NULL)
		return -ENOMEM;

	chip805->dev = chip->dev;
	chip805->chip = chip;
	chip805->pdata = pdata;
	chip805->client = chip->companion;
	/* this is for the interrupt look for this is for the interrupt
	   the last paramter is a unique ID - if this is not working need
	   to platform_driver_register(&chip805);
	   * - since only PM805 will use this interrupt line
	   and it is not shared we can put there NULL */
	chip->companion_chip = chip805;

	ret = device_irq_init_805(chip, pdata);

	if (ret < 0)
		goto out_dev;

	dev_info(chip->dev, "[%s][%s]mfd_add_devices codec_devs\n",
		 __FILE__, __func__);

	ret = mfd_add_devices(chip->dev, 0, &codec_devs[0],
			      ARRAY_SIZE(codec_devs), &codec_resources[0], 0);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add codec subdev\n");
		goto out_dev;
	}

	dev_info(chip->dev, "[%s][%s]mfd_add_devices headset_devs\n",
		 __FILE__, __func__);

	ret = mfd_add_devices(chip->dev, 0, &headset_devs[0],
			      ARRAY_SIZE(headset_devs),
			      &headset_resources[0], 0);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add headset subdev\n");
		goto out_dev;
	}

	return 0;

out_dev:
	mfd_remove_devices(chip->dev);
	device_irq_exit_805(chip);
	kfree(chip805);
	return 1;
}

static int __devinit device_regulator_init(struct pm80x_chip *chip,
					   struct i2c_client *i2c,
					   struct pm80x_platform_data *pdata)
{
	struct regulator_init_data *initdata;
	int ret = 0;
	int i, seq;

	if ((pdata == NULL) || (pdata->regulator == NULL))
		return 0;

	if (pdata->num_regulators > ARRAY_SIZE(regulator_devs))
		pdata->num_regulators = ARRAY_SIZE(regulator_devs);

	for (i = 0, seq = -1; i < pdata->num_regulators; i++) {
		initdata = &pdata->regulator[i];
		seq = *(unsigned int *)initdata->driver_data;
		if ((seq < 0) || (seq > PM800_ID_RG_MAX)) {
			dev_err(chip->dev, "Wrong ID(%d) on regulator(%s)\n",
				seq, initdata->constraints.name);
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
	return 0;

out_err:
	return 1;
}

static void __devinit device_800_init(struct pm80x_chip *chip,
				      struct i2c_client *i2c,
				      struct pm80x_platform_data *pdata)
{
	int ret, pmic_id;
	struct i2c_client *i2c_base = chip->base_page;

	ret = pm80x_reg_read(i2c, PM800_CHIP_ID);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read CHIP ID: %d\n", ret);
		goto out;
	}

	pmic_id = ret & PM8XXX_VERSION_MASK;

	if ((pmic_id >= PM800_CHIP_A0) && (pmic_id <= PM800_CHIP_END)) {
		chip->chip_version = ret;
		dev_info(chip->dev,
			 "pm80x:Marvell 88PM800 (ID: %02x) detected\n", ret);
	} else {
		dev_err(chip->dev,
			"Failed to detect Marvell 88PM800:Chip ID: %02x\n",
			ret);
		goto out;
	}

	ret = device_gpadc_init(chip, pdata);
	if (ret < 0) {
		dev_info(chip->dev,
			 "[%s][%s]FAIL device_gpadc_init:out\n",
			 __FILE__, __func__);
		goto out;
	}

	ret = device_irq_init_800(chip, pdata);
	if (ret < 0) {
		dev_info(chip->dev,
			 "[%s][%s]FAIL device_irq_init:out\n",
			 __FILE__, __func__);
		goto out;
	}

	/* PM800 common wqueue: used in headset, battery, rtc */
	chip->monitor_wqueue = create_singlethread_workqueue("88pm80x");
	if (!chip->monitor_wqueue) {
		dev_info(chip->dev,
			 "[%s][%s]FAIL monitor_wqueue:out_work\n",
			 __FILE__, __func__);
		ret = -ESRCH;
		goto out_work;
	}

	if (device_regulator_init(chip, i2c_base, pdata)) {
		dev_err(chip->dev, "Failed to init regulators\n");
		goto out_dev;
	}

	if (pdata && pdata->vbus) {
		ret = mfd_add_devices(chip->dev, 0, &vbus_devs[0],
				      ARRAY_SIZE(vbus_devs),
				      &vbus_resources[0], chip->irq_base);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add vbus subdev\n");
			goto out_dev;
		}
	}

	ret = mfd_add_devices(chip->dev, 0, &onkey_devs[0],
			      ARRAY_SIZE(onkey_devs), &onkey_resources[0], 0);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add onkey subdev\n");
		goto out_dev;
	}

	ret = mfd_add_devices(chip->dev, 0, &rtc_devs[0],
			      ARRAY_SIZE(rtc_devs), NULL, chip->irq_base);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add rtc subdev\n");
		goto out_dev;
	}

	/* Initializain actions to enable 88pm805 */

	/* Clear WDT */
	pm80x_reg_write(chip->base_page, 0x0E, 0x00);
	/* Enable 32Khz-out-1 and resetoutn */
	pm80x_reg_write(chip->base_page, 0xE1, 0xB0);
	/* Enable 32Khz-out-3  low jitter */
	pm80x_reg_write(chip->base_page, 0x21, 0x20);
	/* Enable 32Khz-out-3 */
	pm80x_reg_write(chip->base_page, 0xE2, 0x22);

	return;
out_dev:
	mfd_remove_devices(chip->dev);
	destroy_workqueue(chip->monitor_wqueue);
out_work:
	device_irq_exit_800(chip);
out:
	return;
}

int __devinit pm80x_device_init(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata)
{
	chip->core_irq = 0;

	switch (chip->id) {

	case CHIP_PM800:
		/* set PM800 as main chip */
		device_800_init(chip, chip->client, pdata);
		break;

	case CHIP_PM805:
		/* set PM805 as main chip */
		device_805_init(chip, chip->client, pdata);
		break;

	}

	if (chip->companion) {

		switch (chip->id) {

		case CHIP_PM800:
			/* PM800 is main chip, PM805 is companion chip */
			device_805_init(chip, chip->companion, pdata);
			break;

		case CHIP_PM805:
			/* PM805 is main chip, PM800 is companion chip */
			device_800_init(chip, chip->companion, pdata);
			break;

		}
	}

	return 0;
}

void __devexit pm80x_device_exit(struct pm80x_chip *chip)
{
	flush_workqueue(chip->monitor_wqueue);
	destroy_workqueue(chip->monitor_wqueue);
	device_irq_exit_800(chip);
	device_irq_exit_805(chip);
	mfd_remove_devices(chip->dev);
	kfree(chip->companion_chip);
}

MODULE_DESCRIPTION("PMIC Driver for Marvell 88PM80x");
MODULE_AUTHOR("Joseph(Yossi) Hanin <yhanin@marvell.com>");
MODULE_LICENSE("GPL");
