/*
 * Copyright (C) 2011 Marvell International Ltd.
 *
 * Author: David Sayada <dsayada@marvell.com>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * LED driver for various PCA963x I2C LED devices
 *
 * Supported devices:
 *
 *	Device	Description				7-bit slave address
 *	------	--------------------------------------	-------------------
 *	PCA9632	4-bit Fm+ I2C-bus low power LED driver	0x60 to 0x63
 *	PCA9633	4-bit Fm+ I2C-bus LED driver		0x60 to 0x63 or any
 *	PCA9634	8-bit Fm+ I2C-bus LED driver		any
 *	PCA9635	16-bit Fm+ I2C-bus LED driver		any
 *
 * Datasheets:
 *
 * PCA9632: http://www.nxp.com/documents/data_sheet/PCA9632.pdf
 * PCA9633: http://www.nxp.com/documents/data_sheet/PCA9633.pdf
 * PCA9634: http://www.nxp.com/documents/data_sheet/PCA9634.pdf
 * PCA9635: http://www.nxp.com/documents/data_sheet/PCA9635.pdf
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/leds-pca963x.h>

enum pca963x_type {
	PCA9632_DP1,
	PCA9632_DP2,
	PCA9632_TK,
	PCA9632_TK2,
	PCA9633_DP1,
	PCA9633_DP2,
	PCA9633_PW,
	PCA9633_D16,
	PCA9633_BS,
	PCA9633_TK,
	PCA9634_D,
	PCA9634_PW,
	PCA9634_BS,
	PCA9635_PW
};

struct pca963x_chipdef {
	u8 num_leds;
	u8 slv_addr;		/* 7-bit slave address mask */
	u8 slv_addr_shift;	/* Number of bits to ignore */
};

static struct pca963x_chipdef pca963x_chipdefs[] = {
	[PCA9632_DP1] = {
			 .num_leds = 4,
			 .slv_addr = /* 1100010 */ 0x62,
			 .slv_addr_shift = 0,
			 },
	[PCA9632_DP2] = {
			 .num_leds = 4,
			 .slv_addr = /* 11000xx */ 0x60,
			 .slv_addr_shift = 2,
			 },
	[PCA9632_TK] = {
			.num_leds = 4,
			.slv_addr = /* 1100010 */ 0x62,
			.slv_addr_shift = 0,
			},
	[PCA9632_TK2] = {
			 .num_leds = 4,
			 .slv_addr = /* 11000xx */ 0x60,
			 .slv_addr_shift = 2,
			 },
	[PCA9633_DP1] = {
			 .num_leds = 4,
			 .slv_addr = /* 1100010 */ 0x62,
			 .slv_addr_shift = 0,
			 },
	[PCA9633_DP2] = {
			 .num_leds = 4,
			 .slv_addr = /* 11000xx */ 0x60,
			 .slv_addr_shift = 2,
			 },
	[PCA9633_PW] = {
			.num_leds = 4,
			.slv_addr = /* xxxxxxx */ 0x00,
			.slv_addr_shift = 7,
			},
	[PCA9633_D16] = {
			 .num_leds = 4,
			 .slv_addr = /* xxxxxxx */ 0x00,
			 .slv_addr_shift = 7,
			 },
	[PCA9633_BS] = {
			.num_leds = 4,
			.slv_addr = /* xxxxxxx */ 0x00,
			.slv_addr_shift = 7,
			},
	[PCA9633_TK] = {
			.num_leds = 4,
			.slv_addr = /* 1100010 */ 0x62,
			.slv_addr_shift = 0,
			},
	[PCA9634_D] = {
		       .num_leds = 8,
		       .slv_addr = /* xxxxxxx */ 0x00,
		       .slv_addr_shift = 7,
		       },
	[PCA9634_PW] = {
			.num_leds = 8,
			.slv_addr = /* xxxxxxx */ 0x00,
			.slv_addr_shift = 7,
			},
	[PCA9634_BS] = {
			.num_leds = 8,
			.slv_addr = /* xxxxxxx */ 0x00,
			.slv_addr_shift = 7,
			},
	[PCA9635_PW] = {
			.num_leds = 16,
			.slv_addr = /* xxxxxxx */ 0x00,
			.slv_addr_shift = 7,
			},
};

static const struct i2c_device_id pca963x_id[] = {
	{"pca9632dp1", PCA9632_DP1},
	{"pca9632dp2", PCA9632_DP2},
	{"pca9632tk", PCA9632_TK},
	{"pca9632tk2", PCA9632_TK2},
	{"pca9633dp1", PCA9633_DP1},
	{"pca9633dp2", PCA9633_DP2},
	{"pca9633pw", PCA9633_PW},
	{"pca9633d16", PCA9633_D16},
	{"pca9633bs", PCA9633_BS},
	{"pca9633tk", PCA9633_TK},
	{"pca9634d", PCA9634_D},
	{"pca9634pw", PCA9634_PW},
	{"pca9634bs", PCA9634_BS},
	{"pca9635pw", PCA9635_PW},
	{}
};

MODULE_DEVICE_TABLE(i2c, pca963x_id);

struct pca963x_led {
	struct i2c_client *client;
	char name[32];
	u8 led_num;		/* 0 .. 15 potentially */
	enum pca963x_led_state state;
	struct led_classdev ldev;
	struct work_struct work;
	enum led_brightness brightness;
};

struct pca963x_chip {
	struct i2c_client *client;
	struct pca963x_led *leds;
	struct mutex update_lock;
	u8 num_leds;
	u8 sub1;
	u8 sub2;
	u8 sub3;
	u8 allcall;
	u8 props;
};

static int pca963x_software_reset(struct i2c_client *client)
{
	u8 buf[] = { 0xa5, 0x5a };
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = 0x03,
		 .flags = 0,
		 .buf = &buf[0],
		 .len = 2,
		 }
	};

	ret = i2c_transfer(client->adapter, msg, 1);

	if (ret == 1)
		ret = 0;

	return ret;
}

static int __pca963x_read(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "Failed reading at 0x%02x\n", reg);
		return ret;
	}

	*val = (u8) ret;
	return 0;
}

static int __pca963x_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing 0x%02x to 0x%02x\n",
			val, reg);
		return ret;
	}

	return 0;
}

static int pca963x_read(struct i2c_client *client, int reg, u8 *val)
{
	struct pca963x_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->update_lock);
	ret = __pca963x_read(chip->client, reg, val);
	mutex_unlock(&chip->update_lock);

	return ret;
}

static int pca963x_set_bit(struct i2c_client *client, u8 reg, u8 bit_mask)
{
	struct pca963x_chip *chip = i2c_get_clientdata(client);
	u8 reg_val;
	int ret;

	mutex_lock(&chip->update_lock);
	ret = __pca963x_read(chip->client, reg, &reg_val);

	if (!ret && ((reg_val & bit_mask) == 0)) {
		reg_val |= bit_mask;
		ret = __pca963x_write(chip->client, reg, reg_val);
	}
	mutex_unlock(&chip->update_lock);

	return ret;
}

static int pca963x_set_val(struct i2c_client *client, u8 reg, u8 bit_mask,
			   u8 val)
{
	struct pca963x_chip *chip = i2c_get_clientdata(client);
	u8 reg_val;
	int ret;

	val &= bit_mask;

	mutex_lock(&chip->update_lock);
	ret = __pca963x_read(chip->client, reg, &reg_val);

	if (!ret) {
		reg_val &= ~bit_mask;
		reg_val |= val;
		ret = __pca963x_write(chip->client, reg, reg_val);
	}
	mutex_unlock(&chip->update_lock);

	return ret;
}

static ssize_t pca963x_store_grpfreq(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca963x_chip *chip = i2c_get_clientdata(client);
	u8 mode2 = 0, grpfreq = simple_strtoul(buf, NULL, 10);
	int ret = 0;

	ret |= pca963x_read(client, PCA963X_MODE2, &mode2);

	if (PCA963X_IS_BLINKING())
		ret |= pca963x_set_val(client, PCA963X_GRPFREQ,
				       PCA963X_256_STEPS_FREQ_MASK, grpfreq);

	if (ret)
		return -EIO;

	return count;
}

static ssize_t pca963x_show_grpfreq(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca963x_chip *chip = i2c_get_clientdata(client);
	u8 grpfreq;
	u32 period;
	int ret;

	ret = pca963x_read(client, PCA963X_GRPFREQ, &grpfreq);

	if (ret)
		return snprintf(buf, PAGE_SIZE, "I2C error");

	period = ((grpfreq + 1) * 1000) / 24;
	return snprintf(buf, PAGE_SIZE, "%d ms\n", period);
}

static ssize_t pca963x_store_grppwm(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca963x_chip *chip = i2c_get_clientdata(client);
	u8 mode2 = 0, grpfreq, grppwm = simple_strtoul(buf, NULL, 10);
	int ret = 0;

	ret |= pca963x_read(client, PCA963X_MODE2, &mode2);
	ret |= pca963x_read(client, PCA963X_GRPFREQ, &grpfreq);

	if (PCA963X_IS_BLINKING()) {
		if (grpfreq < PCA963X_6_HZ_FREQ) {
			if (grppwm <= PCA963X_98_4_PERCENT_DC)
				ret |= pca963x_set_val(client, PCA963X_GRPPWM,
						       PCA963X_64_STEPS_DC_MASK,
						       grppwm << 2);
		} else
			ret |= pca963x_set_val(client, PCA963X_GRPPWM,
					       PCA963X_256_STEPS_DC_MASK,
					       grppwm);
	} else {
		if (grppwm <= PCA963X_93_75_PERCENT_DC)
			ret |= pca963x_set_val(client, PCA963X_GRPPWM,
					       PCA963X_16_STEPS_DC_MASK,
					       grppwm << 4);
	}

	if (ret)
		return -EIO;

	return count;
}

static inline int pca963x_calc_dc(u8 idc)
{
	return (idc * 100) / 256;
}

static ssize_t pca963x_show_grppwm(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca963x_chip *chip = i2c_get_clientdata(client);
	u8 mode2, grppwm, grpfreq;
	int ret = 0;

	ret |= pca963x_read(client, PCA963X_MODE2, &mode2);
	ret |= pca963x_read(client, PCA963X_GRPPWM, &grppwm);
	ret |= pca963x_read(client, PCA963X_GRPFREQ, &grpfreq);

	if (ret)
		return snprintf(buf, PAGE_SIZE, "I2C error");

	if (PCA963X_IS_BLINKING()) {
		if (grpfreq < PCA963X_6_HZ_FREQ)
			return snprintf(buf, PAGE_SIZE, "%d%% \n",
					pca963x_calc_dc(grppwm &
							PCA963X_64_STEPS_DC_MASK));

		return snprintf(buf, PAGE_SIZE, "%d%% \n",
				pca963x_calc_dc(grppwm));
	}

	return snprintf(buf, PAGE_SIZE, "%d%% \n",
			pca963x_calc_dc(grppwm & PCA963X_16_STEPS_DC_MASK));
}

static ssize_t pca963x_store_blink(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 blink = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = pca963x_set_val(client, PCA963X_MODE2, PCA963X_DMBLNK_MASK,
			      blink << PCA963X_DMBLNK_SHIFT);

	if (ret)
		return -EIO;

	return count;
}

static ssize_t pca963x_show_blink(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 mode2 = 0;
	int ret;

	ret = pca963x_read(client, PCA963X_MODE2, &mode2);

	if (ret)
		return snprintf(buf, PAGE_SIZE, "I2C error");

	if (PCA963X_IS_BLINKING())
		return snprintf(buf, PAGE_SIZE, "blinking\n");
	else
		return snprintf(buf, PAGE_SIZE, "dimming\n");
}

static DEVICE_ATTR(grpfreq, S_IWUSR | S_IRUGO, pca963x_show_grpfreq,
		   pca963x_store_grpfreq);
static DEVICE_ATTR(grppwm, S_IWUSR | S_IRUGO, pca963x_show_grppwm,
		   pca963x_store_grppwm);
static DEVICE_ATTR(blink, S_IWUSR | S_IRUGO, pca963x_show_blink,
		   pca963x_store_blink);

static struct attribute *pca963x_sysfs_entries[] = {
	&dev_attr_grpfreq.attr,
	&dev_attr_grppwm.attr,
	&dev_attr_blink.attr,
	NULL
};

static struct attribute_group pca963x_attribute_group = {
	.attrs = pca963x_sysfs_entries,
};

static void pca963x_led_work(struct work_struct *work)
{
	struct pca963x_led *led = container_of(work, struct pca963x_led, work);
	u8 mode2;
	int ret = 0;

	ret |= pca963x_read(led->client, PCA963X_MODE2, &mode2);

	if (led->state == PCA963X_LED_PWM)
		ret |= pca963x_set_val(led->client, PCA963X_PWM(led->led_num),
				       PCA963X_256_STEPS_DC_MASK,
				       led->brightness);
	else if (led->state == PCA963X_LED_PWM_GRPPWM) {
		if (PCA963X_IS_BLINKING())
			ret |= pca963x_set_val(led->client,
					       PCA963X_PWM(led->led_num),
					       PCA963X_256_STEPS_DC_MASK,
					       led->brightness);
		else {
			if (led->brightness <= PCA963X_98_4_PERCENT_DC)
				ret |= pca963x_set_val(led->client,
						       PCA963X_PWM(led->led_num),
						       PCA963X_64_STEPS_DC_MASK,
						       (led->brightness << 2));
		}
	}

	if (ret)
		dev_err(&led->client->dev, "I2C error\n");
}

static void pca963x_set_brightness(struct led_classdev *led_cdev,
				   enum led_brightness value)
{
	struct pca963x_led *led =
	    container_of(led_cdev, struct pca963x_led, ldev);
	led->brightness = value;
	schedule_work(&led->work);
}

static void pca963x_unregister_leds(struct pca963x_chip *chip, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		cancel_work_sync(&chip->leds[i].work);
		led_classdev_unregister(&chip->leds[i].ldev);
	}
}

static int __devinit pca963x_init_led_classdev(struct pca963x_chip *chip,
					       struct pca963x_led *led,
					       int led_num)
{
	int ret = 0;

	led->ldev.name = led->name;
	led->ldev.brightness = LED_OFF;
	led->ldev.brightness_set = pca963x_set_brightness;

	INIT_WORK(&led->work, pca963x_led_work);

	ret = led_classdev_register(&chip->client->dev, &led->ldev);
	if (ret < 0) {
		dev_err(&chip->client->dev, "couldn't register LED %s\n",
			led->name);
		goto failed_register_classdev;
	}

	return 0;

failed_register_classdev:

	cancel_work_sync(&led->work);
	pca963x_unregister_leds(chip, led_num);

	return ret;
}

static int __devinit pca963x_set_led(struct pca963x_chip *chip, int led_num)
{
	u8 ldrx_bit_mask[] = { PCA963X_LDR0_MASK, PCA963X_LDR1_MASK,
		PCA963X_LDR2_MASK, PCA963X_LDR3_MASK
	};

	u8 ldrx_bit_shift[] = { PCA963X_LDR0_SHIFT, PCA963X_LDR1_SHIFT,
		PCA963X_LDR2_SHIFT, PCA963X_LDR3_SHIFT
	};

	return pca963x_set_val(chip->client, PCA963X_LEDOUT(led_num),
			       ldrx_bit_mask[PCA93X_LED_NUM],
			       chip->leds[led_num].state <<
			       ldrx_bit_shift[PCA93X_LED_NUM]);
}

static int __devinit pca963x_configure(struct pca963x_chip *chip)
{
	int ret = 0;

	ret |= pca963x_set_val(chip->client, PCA963X_MODE1,
			       PCA963X_SLEEP_MASK,
			       PCA963X_NORMAL << PCA963X_SLEEP_SHIFT);

	if (chip->sub1) {
		if (chip->sub1 & 1)
			ret |= pca963x_set_bit(chip->client, PCA963X_MODE1,
					       PCA963X_SUB1);
		ret |= pca963x_set_val(chip->client, PCA963X_SUBADR1,
				       PCA963X_ADR_MASK, chip->sub1);
	}

	if (chip->sub2) {
		if (chip->sub2 & 1)
			ret |= pca963x_set_bit(chip->client, PCA963X_MODE1,
					       PCA963X_SUB2);
		ret |= pca963x_set_val(chip->client, PCA963X_SUBADR2,
				       PCA963X_ADR_MASK, chip->sub2);
	}

	if (chip->sub3) {
		if (chip->sub3 & 1)
			ret |= pca963x_set_bit(chip->client, PCA963X_MODE1,
					       PCA963X_SUB3);
		ret |= pca963x_set_val(chip->client, PCA963X_SUBADR3,
				       PCA963X_ADR_MASK, chip->sub3);
	}

	if (chip->allcall) {
		if (chip->allcall & 1)
			ret |= pca963x_set_bit(chip->client, PCA963X_MODE1,
					       PCA963X_ALLCALL);
		ret |=
		    pca963x_set_val(chip->client, PCA963X_ALLCALLADR,
				    PCA963X_ADR_MASK, chip->allcall);
	}

	if (chip->props & PCA963X_INVRT)
		ret |= pca963x_set_bit(chip->client, PCA963X_MODE2,
				       PCA963X_INVRT);

	if (chip->props & PCA963X_OCH)
		ret |= pca963x_set_bit(chip->client, PCA963X_MODE2,
				       PCA963X_OCH);

	if (chip->props & PCA963X_OUTDRV)
		ret |= pca963x_set_bit(chip->client, PCA963X_MODE2,
				       PCA963X_OUTDRV);

	return ret;
}

static int __devinit pca963x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct pca963x_platform_data *pdata = client->dev.platform_data;
	struct pca963x_chipdef *chipdef = &pca963x_chipdefs[id->driver_data];
	struct pca963x_chip *chip;
	char *led_colour[] = { "red", "green", "blue", "amber" };
	int i, ret = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not Supported\n");
		return -EIO;
	}

	if (!pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -EINVAL;
	}

	if ((client->addr & ~((1 << chipdef->slv_addr_shift) - 1)) !=
	    chipdef->slv_addr) {
		dev_err(&client->dev, "invalid slave address %02x\n",
			client->addr);
		return -ENODEV;
	}

	if (pdata->num_leds != chipdef->num_leds) {
		dev_err(&client->dev, "board info claims %d LEDs"
			" on a %d-bit chip\n",
			pdata->num_leds, chipdef->num_leds);
		return -ENODEV;
	}

	ret = pca963x_software_reset(client);
	if (ret) {
		dev_err(&client->dev, "software reset failure.\n");
		return ret;
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->leds =
	    kzalloc(sizeof(*chip->leds) * chipdef->num_leds, GFP_KERNEL);
	if (!chip->leds) {
		kfree(chip);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, chip);
	chip->num_leds = chipdef->num_leds;
	chip->sub1 = pdata->sub1;
	chip->sub2 = pdata->sub2;
	chip->sub3 = pdata->sub3;
	chip->allcall = pdata->allcall;
	chip->props = pdata->props;
	chip->client = client;

	mutex_init(&chip->update_lock);

	for (i = 0; i < pdata->num_leds; i++) {
		struct pca963x_led *led = &chip->leds[i];

		led->client = client;
		led->led_num = i;
		led->state = pdata->leds[i].state;

		if (pdata->leds[i].name)
			snprintf(led->name, sizeof(led->name), "%s",
				 pdata->leds[i].name);
		else
			snprintf(led->name, sizeof(led->name), "%s%d",
				 led_colour[i & 3], (i / 4));

		ret = pca963x_set_led(chip, led->led_num);
		if (ret < 0)
			goto exit_free;

		ret = pca963x_init_led_classdev(chip, led, led->led_num);
		if (ret < 0)
			goto exit_free;
	}

	ret = sysfs_create_group(&client->dev.kobj, &pca963x_attribute_group);
	if (ret < 0) {
		dev_err(&client->dev, "registering with sysfs failed!\n");
		goto exit_classdev;
	}

	ret = pca963x_configure(chip);
	if (ret < 0) {
		dev_err(&client->dev, "pca963x configuration failed!\n");
		goto exit_sysfs;
	}

	return 0;

exit_sysfs:
	sysfs_remove_group(&client->dev.kobj, &pca963x_attribute_group);

exit_classdev:

	pca963x_unregister_leds(chip, chip->num_leds);

exit_free:
	kfree(chip->leds);
	kfree(chip);

	return ret;
}

static int __devexit pca963x_remove(struct i2c_client *client)
{
	struct pca963x_chip *chip = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &pca963x_attribute_group);

	pca963x_unregister_leds(chip, chip->num_leds);

	kfree(chip->leds);
	kfree(chip);

	return 0;
}

#ifdef CONFIG_PM
static int pca963x_suspend(struct i2c_client *client, pm_message_t state)
{
	return pca963x_set_val(client, PCA963X_MODE1, PCA963X_SLEEP_MASK,
			       PCA963X_LOW_POWER << PCA963X_SLEEP_SHIFT);
}

static int pca963x_resume(struct i2c_client *client)
{
	return pca963x_set_val(client, PCA963X_MODE1, PCA963X_SLEEP_MASK,
			       PCA963X_NORMAL << PCA963X_SLEEP_SHIFT);
}
#else
#define pca963x_suspend NULL
#define pca963x_resume  NULL
#endif

static struct i2c_driver pca963x_driver = {
	.driver = {
		   .name = "pca963x",
		   .owner = THIS_MODULE,
		   },
	.probe = pca963x_probe,
	.remove = __devexit_p(pca963x_remove),
	.suspend = pca963x_suspend,
	.resume = pca963x_resume,
	.id_table = pca963x_id,
};

static int __init pca963x_init(void)
{
	return i2c_add_driver(&pca963x_driver);
}

module_init(pca963x_init);

static void __exit pca963x_exit(void)
{
	i2c_del_driver(&pca963x_driver);
}

module_exit(pca963x_exit);

MODULE_AUTHOR("David Sayada <dsayada@marvell.com>");
MODULE_DESCRIPTION("PCA963X LED driver");
MODULE_LICENSE("GPL");
