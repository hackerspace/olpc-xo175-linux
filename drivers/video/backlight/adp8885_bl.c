/*
 * Backlight driver for Analog Devices ADP8885 chip
 *
 * Copyright (C) 2011 Marvell Internation Ltd.
 *
 * Michael Zaidman <zmichael@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/i2c/adp8885.h>

#define DRV_NAME		"adp8885_bl"
#define VERSION			"1.0.0.0"

/*
 * ADP8885 registers
 */
#define DVID			0x00	/* Manufacturer and device ID */
#define MDCR			0x01	/* Mode control */
#define STATUS			0x03	/* Status */
#define BOOSTCFG		0x04	/* Boost configuration */
#define OVPCFG			0x05	/* Over-voltage configuration */
#define FBCFG			0x06	/* Feedback configuration */
#define DIMCFG			0x07	/* External DIM configuration */
#define IMAX1			0x10	/* Channel 1 maximum output current */
#define CH1_SET			0x11	/* Channel 1 output current */
#define CH1_FADE		0x12	/* Channel 1 fade configuration */
#define IMAX2			0x13	/* Channel 2 maximum output current */
#define CH2_SET			0x14	/* Channel 2 output current */
#define CH2_FADE		0x15	/* Channel 2 fade configuration */
#define LEDOUT_H		0x50	/* LED out over current MSB */
#define LEDOUT_L		0x51	/* LED out over current LSB */
#define LEDOVR			0x52	/* LED output current override */

#define CH_REG(base, ch)	(base + (IMAX2 - IMAX1) * ch)

#define IMAX(ch)		CH_REG(IMAX1, ch)
#define CH_SET(ch)		CH_REG(CH1_SET, ch)
#define CH_FADE(ch)		CH_REG(CH1_FADE, ch)

#define MANUF_ID		0x1A	/* Analog Devices AD8885 device ID */
#define REVID(x)		((x) & 0x7)
#define MANID(x)		((x) >> 3)

#define NSTBY			1

#define MDCR_CH_EN(x)		(1 << (x + 1))
#define OVPCFG_CH(x)		(1 << (x + 1))
#define FBCFG_DIS(x)		(1 << x)
#define DIMCFG_EN(x)		(1 << x)
#define LEDOVR_CH(x)		(1 << x)

#define BOOSTCFG_VAL(psm_en, fsw, ipeak) \
		((psm_en << 4) | (fsw << 2) | ipeak)

#define OVPCFG_VAL(ch2_sofp, ch1_sofp, ovp_lvl) \
		((ch2_sofp) << 2 | (ch1_sofp << 1) | ovp_lvl)

#define FADE_VAL(in, out)	((0xF & (in)) | ((0xF & (out)) << 4))

struct adp8885_ch {
	struct adp8885_bl *bl;
	int current_brightness;
	int id;
};

struct adp8885_bl {
	struct adp8885_bl_platform_data *pdata;
	struct backlight_device *ch[2];
	struct i2c_client *client;
	struct mutex lock;
	int revid;
	int id;
};

static const char *adp8885_ch_name[] = {
	"backlight-0",
	"backlight-1",
	""
};

static int adp8885_read(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
		return ret;
	}
	*val = (u8) ret;

	return 0;
}

static int adp8885_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret)
		dev_err(&client->dev, "failed to write\n");

	dev_dbg(&client->dev, "%s: 0x%x : 0x%x : 0x%x\n",
		client->name, client->addr, reg, val);

	return ret;
}

static int adp8885_set_bits(struct i2c_client *client, u8 reg, u8 mask)
{
	struct adp8885_bl *data = i2c_get_clientdata(client);
	u8 reg_val;
	int ret;

	mutex_lock(&data->lock);
	ret = adp8885_read(client, reg, &reg_val);
	if (!ret && ((reg_val & mask) != mask)) {
		reg_val |= mask;
		ret = adp8885_write(client, reg, reg_val);
	}
	mutex_unlock(&data->lock);

	return ret;
}

static int adp8885_clr_bits(struct i2c_client *client, u8 reg, u8 mask)
{
	struct adp8885_bl *data = i2c_get_clientdata(client);
	u8 reg_val;
	int ret;

	mutex_lock(&data->lock);
	ret = adp8885_read(client, reg, &reg_val);
	if (!ret && (reg_val & mask)) {
		reg_val &= ~mask;
		ret = adp8885_write(client, reg, reg_val);
	}
	mutex_unlock(&data->lock);

	return ret;
}

static int adp8885_bl_set(struct backlight_device *bl, int brightness)
{
	struct adp8885_ch *data = bl_get_data(bl);
	struct i2c_client *client = data->bl->client;
	int ch = data->id;
	int ret = 0;

	ret = adp8885_write(client, CH_SET(ch), (u8) brightness);
	if (ret)
		return ret;

	if (data->current_brightness && brightness == 0)
		ret = adp8885_set_bits(client, DIMCFG, DIMCFG_EN(ch));
	else if (data->current_brightness == 0 && brightness)
		ret = adp8885_clr_bits(client, DIMCFG, DIMCFG_EN(ch));

	if (!ret)
		data->current_brightness = brightness;

	return ret;
}

static int adp8885_bl_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;
	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	return adp8885_bl_set(bl, brightness);
}

static int adp8885_bl_get_brightness(struct backlight_device *bl)
{
	struct adp8885_ch *data = bl_get_data(bl);

	return data->current_brightness;
}

static const struct backlight_ops adp8885_bl_ops = {
	.update_status = adp8885_bl_update_status,
	.get_brightness = adp8885_bl_get_brightness,
};

static int __devinit adp8885_bl_setup(struct i2c_client *client)
{
	struct adp8885_bl *data = i2c_get_clientdata(client);
	struct adp8885_bl_platform_data *pdata = data->pdata;
	int ret = 0;

	ret = adp8885_clr_bits(client, MDCR, NSTBY);
	if (ret)
		return ret;

	ret = adp8885_write(client, BOOSTCFG,
			    BOOSTCFG_VAL(pdata->psm_en, pdata->fsw,
					 pdata->ipeak));
	if (ret)
		return ret;

	ret = adp8885_write(client, OVPCFG, pdata->ovp_lvl);
	if (ret)
		return ret;

	ret = adp8885_write(client, LEDOUT_H, pdata->ledout_h);
	if (ret)
		return ret;

	ret = adp8885_write(client, LEDOUT_L, pdata->ledout_l);

	return ret;
}

static int __devinit adp8885_ch_setup(struct backlight_device *bl, int ch)
{
	struct adp8885_ch *data = bl_get_data(bl);
	struct i2c_client *client = data->bl->client;
	struct adp8885_bl_platform_data *pdata = data->bl->pdata;
	struct adp8885_bl_channel_data *pchan = pdata->ch + ch;

	int ret = 0;

	if (pchan->sovp) {
		ret = adp8885_set_bits(client, OVPCFG, OVPCFG_CH(ch));
		if (ret)
			return ret;
	}

	if (pchan->fb_dis) {
		ret = adp8885_set_bits(client, FBCFG, FBCFG_DIS(ch));
		if (ret)
			return ret;
	}

	if (pchan->dim_en) {
		ret = adp8885_set_bits(client, DIMCFG, DIMCFG_EN(ch));
		if (ret)
			return ret;
	}

	ret = adp8885_write(client, IMAX(ch), pchan->imax);
	if (ret)
		return ret;

	ret = adp8885_write(client, CH_SET(ch), pchan->iset);
	if (ret)
		return ret;

	if (pchan->iovr) {
		ret = adp8885_set_bits(client, LEDOVR, LEDOVR_CH(ch));
	} else {
		ret = adp8885_clr_bits(client, LEDOVR, LEDOVR_CH(ch));
		if (ret)
			return ret;

		ret = adp8885_write(client, CH_FADE(ch),
				    FADE_VAL(pchan->fade_in, pchan->fade_out));
	}
	if (ret)
		return ret;

	ret = adp8885_set_bits(client, MDCR, MDCR_CH_EN(ch) | NSTBY);

	return ret;
}

static int __devinit adp8885_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct adp8885_bl_platform_data *pdata = client->dev.platform_data;
	struct backlight_properties props;
	struct backlight_device *bl;
	struct adp8885_bl *data;
	struct adp8885_ch *chan;
	int ret, i;
	u8 reg_val;

	dev_info(&client->dev, "Ver. %s:\n", VERSION);

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}

	if (!pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -EINVAL;
	}

	if (pdata->num_chs > MAX_CHAN_NUM) {
		dev_err(&client->dev, "invalid number of channels!\n");
		return -EINVAL;
	}

	if (pdata->chip_enable) {
		ret = pdata->chip_enable(true);
		if (ret) {
			dev_err(&client->dev, "failed to enable chip!\n");
			return ret;
		}
	}

	ret = adp8885_read(client, DVID, &reg_val);
	if (ret)
		return ret;

	if (MANID(reg_val) != MANUF_ID) {
		dev_err(&client->dev, "failed to probe!\n");
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct adp8885_bl), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev, "failed to allocate adp8885_bl!\n");
		return -ENOMEM;
	}

	chan = kzalloc(sizeof(struct adp8885_ch) * pdata->num_chs, GFP_KERNEL);
	if (chan == NULL) {
		dev_err(&client->dev, "failed to allocate adp8885_ch!\n");
		kfree(data);
		return -ENOMEM;
	}

	data->revid = REVID(reg_val);
	data->client = client;
	data->pdata = pdata;
	data->id = id->driver_data;

	i2c_set_clientdata(client, data);

	mutex_init(&data->lock);

	ret = adp8885_bl_setup(client);
	if (ret) {
		dev_err(&client->dev, "failed to adp8885_bl_setup!\n");
		goto out;
	}

	for (i = 0; i < data->pdata->num_chs; i++) {
		chan[i].id = i;
		chan[i].bl = data;
		chan[i].current_brightness = 0;

		memset(&props, 0, sizeof(props));
		props.max_brightness = ADP8885_MAX_BRIGHTNESS;

		if (pdata->ch[i].iset)
			props.brightness = pdata->ch[i].iset;
		else
			props.brightness = ADP8885_MAX_BRIGHTNESS;

		props.type = BACKLIGHT_RAW;
		bl = backlight_device_register(adp8885_ch_name[i],
					       &client->dev, &chan[i],
					       &adp8885_bl_ops, &props);
		if (IS_ERR(bl)) {
			dev_err(&client->dev,
				"failed to backlight_device_register %s!\n",
				adp8885_ch_name[i]);
			ret = PTR_ERR(bl);
			goto out1;
		}
		data->ch[i] = bl;

		ret = adp8885_ch_setup(bl, i);
		if (ret) {
			dev_err(&client->dev, "failed to adp8885_ch_setup!\n");
			goto out1;
		}
		backlight_update_status(bl);
	}

	return 0;

out1:
	for (i = 0; i < data->pdata->num_chs; i++) {
		if (data->ch[i])
			backlight_device_unregister(data->ch[i]);
	}
out:
	i2c_set_clientdata(client, NULL);
	kfree(chan);
	kfree(data);

	return ret;
}

static int __devexit adp8885_remove(struct i2c_client *client)
{
	struct adp8885_bl *data = i2c_get_clientdata(client);
	struct adp8885_ch *chan = bl_get_data(data->ch[0]);
	int i;

	adp8885_clr_bits(client, MDCR, NSTBY);

	for (i = 0; i < data->pdata->num_chs; i++)
		if (data->ch[i])
			backlight_device_unregister(data->ch[i]);

	i2c_set_clientdata(client, NULL);
	kfree(chan);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int adp8885_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
	int ret;
	struct adp8885_bl *data = i2c_get_clientdata(client);

	ret = adp8885_clr_bits(client, MDCR, NSTBY);

	if (data->pdata->chip_enable)
		ret = data->pdata->chip_enable(false);

	return ret;
}

static int adp8885_i2c_resume(struct i2c_client *client)
{
	int ret;
	struct adp8885_bl *data = i2c_get_clientdata(client);

	if (data->pdata->chip_enable)
		ret = data->pdata->chip_enable(true);
	ret = adp8885_set_bits(client, MDCR, NSTBY);

	return ret;
}
#else
#define adp8885_i2c_suspend NULL
#define adp8885_i2c_resume NULL
#endif

static const struct i2c_device_id adp8885_id[] = {
	{"adp8885", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, adp8885_id);

static struct i2c_driver adp8885_driver = {
	.driver = {
		   .name = DRV_NAME,
		   },
	.probe = adp8885_probe,
	.remove = __devexit_p(adp8885_remove),
	.suspend = adp8885_i2c_suspend,
	.resume = adp8885_i2c_resume,
	.id_table = adp8885_id,
};

static int __init adp8885_init(void)
{
	return i2c_add_driver(&adp8885_driver);
}

module_init(adp8885_init);

static void __exit adp8885_exit(void)
{
	i2c_del_driver(&adp8885_driver);
}

module_exit(adp8885_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Zaidman <zmichael@marvell.com>");
MODULE_DESCRIPTION("adp8885 backlight driver");
