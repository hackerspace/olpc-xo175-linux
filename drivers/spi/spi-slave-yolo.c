#include <linux/completion.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>

struct spi_slave_yolo_priv {
	struct spi_device *spi;

	struct gpio_desc *gpio_ack;
	struct gpio_desc *gpio_cmd;

	struct completion finished;
	struct spi_transfer xfer;
	struct spi_message msg;
	u8 buf[2];
};

static int spi_slave_yolo_submit(struct spi_slave_yolo_priv *priv);

static void spi_slave_yolo_complete(void *arg)
{
	struct spi_slave_yolo_priv *priv = arg;
	int ret;

	ret = priv->msg.status;
	if (ret)
		goto terminate;

	printk ("XXX [%08x] [%08x]\n", priv->buf[0], priv->buf[1]);

	ret = spi_slave_yolo_submit(priv);
	if (ret)
		goto terminate;

	return;

terminate:
	dev_info(&priv->spi->dev, "Terminating\n");
	complete(&priv->finished);
}

static int spi_slave_yolo_submit(struct spi_slave_yolo_priv *priv)
{
	int ret;

	spi_message_init_with_transfers(&priv->msg, &priv->xfer, 1);

	priv->msg.complete = spi_slave_yolo_complete;
	priv->msg.context = priv;

	ret = spi_async(priv->spi, &priv->msg);
	if (ret)
		dev_err(&priv->spi->dev, "spi_async() failed %d\n", ret);

	return ret;
}

static int spi_slave_yolo_probe(struct spi_device *spi)
{
	struct spi_slave_yolo_priv *priv;
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

	ret = spi_slave_yolo_submit(priv);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(priv->gpio_cmd, 0);
	gpiod_set_value_cansleep(priv->gpio_ack, 0);

	dev_info(&spi->dev, "Riverbed will run red with the blood of the saints and the holy\n");

	spi_set_drvdata(spi, priv);
	return 0;
}

static int spi_slave_yolo_remove(struct spi_device *spi)
{
	struct spi_slave_yolo_priv *priv = spi_get_drvdata(spi);

	spi_slave_abort(spi);
	wait_for_completion(&priv->finished);
	return 0;
}

static const struct of_device_id spi_slave_yolo_of_match[] = {
        { .compatible = "olpc,ec-spi" },
        { },
};
MODULE_DEVICE_TABLE(of, pxa_usb_phy_of_match);

static struct spi_driver spi_slave_yolo_driver = {
	.driver = {
		.name	= "spi-slave-yolo",
		.of_match_table = spi_slave_yolo_of_match,
	},
	.probe		= spi_slave_yolo_probe,
	.remove		= spi_slave_yolo_remove,
};
module_spi_driver(spi_slave_yolo_driver);

MODULE_AUTHOR("Konsky Kokot");
MODULE_DESCRIPTION("Barania Kloaka");
MODULE_LICENSE("GPL v2");
