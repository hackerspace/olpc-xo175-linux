/*
 * Max77601 mfd core driver
 *
 * Copyright 2011 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77601.h>
#include <linux/switch.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>

#define MAX77601_REG_DEVS(_id)     \
{                                   \
	.name   = "max77601-regulator",   \
	.id     = MAX77601_ID_##_id,      \
}

static struct mfd_cell regulator_devs[] = {
	MAX77601_REG_DEVS(SD0),
	MAX77601_REG_DEVS(DVSSD0),
	MAX77601_REG_DEVS(SD1),
	MAX77601_REG_DEVS(DVSSD1),
	MAX77601_REG_DEVS(SD2),
	MAX77601_REG_DEVS(SD3),
	MAX77601_REG_DEVS(SD4),
	MAX77601_REG_DEVS(L0),
	MAX77601_REG_DEVS(L1),
	MAX77601_REG_DEVS(L2),
	MAX77601_REG_DEVS(L3),
	MAX77601_REG_DEVS(L4),
	MAX77601_REG_DEVS(L5),
	MAX77601_REG_DEVS(L6),
	MAX77601_REG_DEVS(L7),
	MAX77601_REG_DEVS(L8),
};

static struct resource onkey_resources[] = {
	{
		.name	= "max77601-onkey",
		.start	= MAX77601_IRQTOP_ONOFF,
		.end	= MAX77601_IRQTOP_ONOFF,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct mfd_cell onkey_devs[] = {
	{
		.name		= "max77601-onkey",
		.num_resources	= ARRAY_SIZE(onkey_resources),
		.resources	= &onkey_resources[0],
		.id		= -1,
	},
};


struct max77601_irq_data {
	int reg;
	int mask_reg;
	int enable;		/* enable interrupt */
	int mask_bit;		/* bit in mask register */
	int irq_bit;		/* bit in interrupt register */
};

static struct max77601_irq_data max77601_irqtop[] = {
	[MAX77601_IRQTOP_NVER] = {
				  .reg = MAX77601_IRQTOP_REG,
				  .mask_reg = MAX77601_IRQTOPM_REG,
				  .irq_bit = MAX77601_IRQ_NVER_MASK,
				  .mask_bit = MAX77601_IRQ_NVER_MASK,
				  .enable = MAX77601_IRQ_NVER_MASK,
				  },
	[MAX77601_IRQTOP_ONOFF] = {
				   .reg = MAX77601_IRQTOP_REG,
				   .mask_reg = MAX77601_IRQTOPM_REG,
				   .irq_bit = MAX77601_IRQ_ONOFF_MASK,
				   .mask_bit = MAX77601_IRQ_ONOFF_MASK,
				   .enable = MAX77601_IRQ_ONOFF_MASK,
				   },
	[MAX77601_IRQTOP_32K] = {
				 .reg = MAX77601_IRQTOP_REG,
				 .mask_reg = MAX77601_IRQTOPM_REG,
				 .irq_bit = MAX77601_IRQ_32K_MASK,
				 .mask_bit = MAX77601_IRQ_32K_MASK,
				 .enable = 0x00,
				 },
	[MAX77601_IRQTOP_RTC] = {
				 .reg = MAX77601_IRQTOP_REG,
				 .mask_reg = MAX77601_IRQTOPM_REG,
				 .irq_bit = MAX77601_IRQ_RTC_MASK,
				 .mask_bit = MAX77601_IRQ_RTC_MASK,
				 .enable = MAX77601_IRQ_RTC_MASK,
				 },
	[MAX77601_IRQTOP_GPIO] = {
				  .reg = MAX77601_IRQTOP_REG,
				  .mask_reg = MAX77601_IRQTOPM_REG,
				  .irq_bit = MAX77601_IRQ_GPIO_MASK,
				  .mask_bit = MAX77601_IRQ_GPIO_MASK,
				  .enable = 0x00,
				  },
	[MAX77601_IRQTOP_LDO] = {
				 .reg = MAX77601_IRQTOP_REG,
				 .mask_reg = MAX77601_IRQTOPM_REG,
				 .irq_bit = MAX77601_IRQ_LDO_MASK,
				 .mask_bit = MAX77601_IRQ_LDO_MASK,
				 .enable = 0x00,
				 },
	[MAX77601_IRQTOP_SD] = {
				.reg = MAX77601_IRQTOP_REG,
				.mask_reg = MAX77601_IRQTOPM_REG,
				.irq_bit = MAX77601_IRQ_SD_MASK,
				.mask_bit = MAX77601_IRQ_SD_MASK,
				.enable = 0x00,
				},
	[MAX77601_IRQTOP_GLBL] = {
				  .reg = MAX77601_IRQTOP_REG,
				  .mask_reg = MAX77601_IRQTOPM_REG,
				  .irq_bit = MAX77601_IRQ_GLBL_MASK,
				  .mask_bit = MAX77601_IRQ_GLBL_MASK,
				  .enable = MAX77601_IRQ_GLBL_MASK,
				  },
};

static irqreturn_t max77601_irqtop_isr(int irq, void *data)
{
	struct max77601_chip *chip = data;
	struct max77601_irq_data *irq_data;
	u8 irqtop = 0;
	int i = 0, ret = 0, handled = 0;
	u16 irqs_to_handle[MAX77601_IRQTOP_NR_INTS];

	/* mask max77601 global interrupt mask GLBLM */
	ret = max77601_set_bits(chip, MAX77601_INTLBTM_REG, \
		MAX77601_INTLBT_GLB_MASK, MAX77601_INTLBT_GLB_MASK);
	if (ret < 0)
		dev_err(chip->dev, "Failed to mask GLBLM\n");

	ret = max77601_read(chip, MAX77601_IRQTOP_REG, &irqtop, 1);
	if (ret < 0)
		dev_err(chip->dev, "Top level IRQ read error\n");
	dev_dbg(chip->dev, "Top level IRQ read ret :%x \n", irqtop);

	for (i = MAX77601_IRQTOP_START; i <= MAX77601_IRQTOP_END; i++) {
		irq_data = &max77601_irqtop[i];
		if ((irqtop & irq_data->irq_bit)
		    && (irq_data->enable != 0)) {
			irqs_to_handle[handled] = i + chip->irq_base;
			handled++;
		}
	}

	for (i = 0; i < handled; i++) {
		dev_dbg(chip->dev, "Top level IRQ nested = 0x%x\n",
			irqs_to_handle[i]);
		handle_nested_irq(irqs_to_handle[i]);
	}

	/* unmask max77601 global interrupt mask GLBLM  */
	ret = max77601_set_bits(chip, MAX77601_INTLBTM_REG, \
		MAX77601_INTLBT_GLB_MASK, 0x0);
	if (ret < 0)
		dev_err(chip->dev, "Failed to unmask GLBLM\n");

	return IRQ_HANDLED;
}

static void max77601_irqtop_disable(struct irq_data *data)
{
	struct max77601_chip *chip = irq_data_get_irq_chip_data(data);
	max77601_irqtop[data->irq - chip->irq_base].enable = 0;
}

static void max77601_irqtop_enable(struct irq_data *data)
{
	struct max77601_chip *chip = irq_data_get_irq_chip_data(data);
	max77601_irqtop[data->irq - chip->irq_base].enable =
	    max77601_irqtop[data->irq - chip->irq_base].mask_bit;
}

static void max77601_irqtop_lock(struct irq_data *data)
{
	struct max77601_chip *chip = irq_data_get_irq_chip_data(data);
	mutex_lock(&chip->irq_lock);
}

static void max77601_irqtop_sync_unlock(struct irq_data *data)
{
	struct max77601_irq_data *irq_data;
	struct max77601_chip *chip = irq_data_get_irq_chip_data(data);

	u8 config = 0;
	/* default mask value */
	static u8 cache_mask = 0x74;

	irq_data = &max77601_irqtop[data->irq - chip->irq_base];
	if (irq_data->enable == 0) {
		/* disable IRQ : mask bit - 1 */
		config = cache_mask | irq_data->mask_bit;
	} else {
		/* Enable IRQ : mask bit - 0 */
		config = cache_mask & ~irq_data->mask_bit;
	}

	if (cache_mask != config) {
		max77601_write(chip, irq_data->mask_reg, &config, 1);
		cache_mask = config;
		dev_dbg(chip->dev, "TOPIRQM write irq_num = %d\n",
			data->irq - chip->irq_base);
	}
	mutex_unlock(&chip->irq_lock);
}

static struct irq_chip max77601_irqtop_chip = {
	.name = "max77601-irqtop",
	.irq_bus_lock = max77601_irqtop_lock,
	.irq_bus_sync_unlock = max77601_irqtop_sync_unlock,
	.irq_enable = max77601_irqtop_enable,
	.irq_disable = max77601_irqtop_disable,
};

static int max77601_irq_init(struct max77601_chip *chip, int irq,
			     struct max77601_platform_data *pdata)
{
	int i, ret = 0;
	unsigned long flags =
	    IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_DISABLED;
	u8 data;
	struct irq_desc *desc;

	if (!pdata || !pdata->irq_base) {
		dev_warn(chip->dev, "No interrupt support on IRQ base\n");
		return -EINVAL;
	}

	/* clear all interrupts */
	max77601_set_bits(chip, MAX77601_INTLBTM_REG, \
		MAX77601_INTLBT_GLB_MASK, MAX77601_INTLBT_GLB_MASK);
	max77601_read(chip, MAX77601_IRQTOP_REG, &data, 1);
	max77601_read(chip, MAX77601_INTLBT_REG, &data, 1);
	max77601_read(chip, MAX77601_ONOFFIRQ_REG, &data, 1);
	max77601_set_bits(chip, MAX77601_INTLBTM_REG, \
		MAX77601_INTLBT_GLB_MASK, 0x0);

	mutex_init(&chip->irq_lock);
	chip->irq_base = pdata->irq_base;
	chip->core_irq = irq;

	if (!chip->core_irq) {
		dev_warn(chip->dev, "No interrupt support on core IRQ\n");
		return -EINVAL;
	}

	desc = irq_to_desc(chip->core_irq);
	max77601_irqtop_chip.irq_set_wake = desc->irq_data.chip->irq_set_wake;

	/* register with genirq */
	for (i = pdata->irq_base;
	     i < (pdata->irq_base + MAX77601_IRQTOP_NR_INTS); i++) {
		irq_set_chip_data(i, chip);
		irq_set_chip_and_handler(i, &max77601_irqtop_chip,
					 handle_edge_irq);
		set_irq_flags(i, IRQF_VALID);
		irq_set_nested_thread(i, 1);
	}

	ret = request_threaded_irq(irq, NULL, max77601_irqtop_isr, flags,
				   "max77601-irqtop", chip);
	if (ret) {
		dev_err(chip->dev, "Failed to request IRQ: %d\n", irq);
		chip->core_irq = 0;
	}

	return ret;
}


int __devinit max77601_device_init(struct max77601_chip *chip,
				   struct max77601_platform_data *pdata)
{
	int i, ret;

	ret = max77601_irq_init(chip, chip->i2c->irq, pdata);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to init irq!\n");
		return ret;
	}

	/* Regulator devices */
	if (pdata) {
		for (i = 0; i < ARRAY_SIZE(regulator_devs); i++) {
			regulator_devs[i].platform_data =
			    &pdata->regulator[i];
			regulator_devs[i].pdata_size =
			    sizeof(struct regulator_init_data);
			ret =
			    mfd_add_devices(chip->dev, 0,
					    &regulator_devs[i], 1, NULL,
					    0);
			if (ret < 0) {
				dev_err(chip->dev,
					"Failed to init regulator %d!\n",
					regulator_devs[i].id);
				return ret;
			}
		}
	}

	/* onkey device */
	ret = mfd_add_devices(chip->dev, 0, &onkey_devs[0],
		      ARRAY_SIZE(onkey_devs),
		      &onkey_resources[0], 0);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add onkey subdev\n");
		return ret;
	}

	/* RTC device */

	return ret;
}

void __devexit max77601_device_exit(struct max77601_chip *chip)
{
	if (chip->core_irq)
		free_irq(chip->core_irq, chip);
	mfd_remove_devices(chip->dev);
}


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MAX77601 Multi Function Device Core Driver");
MODULE_VERSION("1.0");
