/*
 * Driver for the OLPC XO-1.75 Embedded Controller.
 *
 * The EC protocol is documented at:
 * http://wiki.laptop.org/go/XO_1.75_HOST_to_EC_Protocol
 *
 * Copyright (C) 2010 One Laptop per Child Foundation.
 * Copyright (C) 2018 Lubomir Rintel <lkundrak@v3.sk>
 *
 * Licensed under the GPL v2 or later.
 */

#include <linux/completion.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

struct olpc_xo175_ec_priv {
	struct spi_device *spi;

	struct gpio_desc *gpio_ack;
	struct gpio_desc *gpio_cmd;

	struct completion finished;
	struct spi_transfer xfer;
	struct spi_message msg;
	u8 buf[2];
};

static int olpc_xo175_ec_submit(struct olpc_xo175_ec_priv *priv);

static void olpc_xo175_ec_complete(void *arg)
{
	struct olpc_xo175_ec_priv *priv = arg;
	int ret;

	ret = priv->msg.status;
	if (ret)
		goto terminate;

	printk ("XXX [%08x] [%08x]\n", priv->buf[0], priv->buf[1]);

	ret = olpc_xo175_ec_submit(priv);
	if (ret)
		goto terminate;

	/* We're ready to get another command. */
	gpiod_set_value_cansleep(priv->gpio_ack, 1);
	udelay(1);
	gpiod_set_value_cansleep(priv->gpio_ack, 0);


	return;

terminate:
	dev_info(&priv->spi->dev, "Terminating\n");
	complete(&priv->finished);
}

static int olpc_xo175_ec_submit(struct olpc_xo175_ec_priv *priv)
{
	int ret;

	spi_message_init_with_transfers(&priv->msg, &priv->xfer, 1);

	priv->msg.complete = olpc_xo175_ec_complete;
	priv->msg.context = priv;

	ret = spi_async(priv->spi, &priv->msg);
	if (ret)
		dev_err(&priv->spi->dev, "spi_async() failed %d\n", ret);

	return ret;
}

static int olpc_xo175_ec_probe(struct spi_device *spi)
{
	struct olpc_xo175_ec_priv *priv;
	int ret;

	priv = devm_kzalloc(&spi->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->gpio_ack = devm_gpiod_get(&spi->dev, "ack", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpio_ack)) {
		dev_err(&spi->dev, "failed to get ack gpio: %ld\n", PTR_ERR(priv->gpio_ack));
		return PTR_ERR(priv->gpio_ack);
	}

	priv->gpio_cmd = devm_gpiod_get(&spi->dev, "cmd", GPIOD_OUT_LOW);
	if (IS_ERR(priv->gpio_cmd)) {
		dev_err(&spi->dev, "failed to get cmd gpio: %ld\n", PTR_ERR(priv->gpio_cmd));
		return PTR_ERR(priv->gpio_cmd);
	}

	priv->spi = spi;
	init_completion(&priv->finished);
	priv->xfer.rx_buf = priv->buf;
	priv->xfer.len = sizeof(priv->buf);

	ret = olpc_xo175_ec_submit(priv);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(priv->gpio_cmd, 0);
	gpiod_set_value_cansleep(priv->gpio_ack, 0);

	dev_info(&spi->dev, "OLPC XO-1.75 Embedded Controller driver\n");

	spi_set_drvdata(spi, priv);
	return 0;
}

static int olpc_xo175_ec_remove(struct spi_device *spi)
{
	struct olpc_xo175_ec_priv *priv = spi_get_drvdata(spi);

	spi_slave_abort(spi);
	wait_for_completion(&priv->finished);
	return 0;
}

static const struct of_device_id olpc_xo175_ec_of_match[] = {
        { .compatible = "olpc,xo1.75-ec" },
        { },
};
MODULE_DEVICE_TABLE(of, pxa_usb_phy_of_match);

static struct spi_driver olpc_xo175_ec_driver = {
	.driver = {
		.name	= "olpc-xo175-ec",
		.of_match_table = olpc_xo175_ec_of_match,
	},
	.probe		= olpc_xo175_ec_probe,
	.remove		= olpc_xo175_ec_remove,
};
module_spi_driver(olpc_xo175_ec_driver);

MODULE_DESCRIPTION("OLPC XO-1.75 Embedded Controller driver");
MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_AUTHOR("Lennert Buytenhek <buytenh@wantstofly.org>");
MODULE_LICENSE("GPL");
