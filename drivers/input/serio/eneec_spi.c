#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>

enum {
	PORT_KBD = 0x30,
	PORT_MOU = 0x40,
	PORT_XBI = 0xF0,
};

struct rspdata {
	u8 resv;
	u8 toggle : 2;
	u8 count : 2;
	u8 type : 4; // RSP_XXX
	u8 data[3];
} __packed;

enum {
	RSP_UNKNOWN		= 0,
	RSP_WRITE_COMPLETE	= 1,
	RSP_READ		= 2,
	RSP_KB_RESPONSE		= 3,
	RSP_AUX_RESPONSE	= 4,
	RSP_PORT_RESPONSE	= 5,
	RSP_KB_DATA		= 12,
	RSP_AUX_DATA		= 13,
	RSP_PORT_DATA		= 14,
	RSP_RESET		= 15,
};

struct eneec {
	struct spi_device *client;
	int toggle;
};

static int eneec_read_rspdata(struct eneec *eneec, struct rspdata *rspdata)
{
#define RSP_LEN	5
	u8	tx_buf[RSP_LEN] = { 0x00, 0x5a, 0xa5, 0x00, 0x00 };
	int   ret;
	struct spi_device   *spi = eneec->client;
	struct spi_transfer t = {
		.tx_buf = tx_buf,
		.rx_buf = rspdata,
		.len = RSP_LEN,
	};
#if 0	
	struct spi_message  m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(spi, &m);
#endif
	ret = spi_sync_transfer(spi, &t, 1);

	if (ret < 0) {
		pr_err("ERROR: spi_sync fail\n");
		return ret;
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Codes for interfacing with kernel kbd

static irqreturn_t eneec_interrupt(int irq, void *dev_id)
{
	struct eneec *eneec = dev_id;
	struct rspdata rspdata;
	int i;

	printk ("INTERRUPTH\n");

	if (eneec_read_rspdata(eneec, &rspdata) < 0) {
		pr_err("ERROR: eneec_read_rspdata fail \n");
		return IRQ_HANDLED;
	}

	if (eneec->toggle == rspdata.toggle) {
		printk("WARNING: got the same toggle bit %d\n", rspdata.toggle);
		return IRQ_HANDLED;
	}

	eneec->toggle = rspdata.toggle;

	if ((rspdata.type != RSP_KB_RESPONSE) && (rspdata.type != RSP_KB_DATA)) {
		pr_warn(   "WARNING: Not kbd data\n");
		return IRQ_HANDLED;
	}

	for (i = 0; i < rspdata.count; i++) {
		printk("scan code %02x\n",rspdata.data[i]);
	}

	return IRQ_HANDLED;
}

static int eneec_probe(struct spi_device *spi)
{
	struct eneec *eneec = 0;
	struct rspdata rspdata;
	int err;

	printk("eneec_spi_probe with modalias = %s, irq = %d\n", spi->modalias, spi->irq);

	if (!spi->irq) {
		pr_err("ERROR: spi->irq is not specified.\n");
		return -ENXIO;
	}

	eneec = devm_kzalloc(&spi->dev, sizeof(struct eneec), GFP_KERNEL);
	if (!eneec)
		return -ENOMEM;


	eneec->client = spi;
	spi_set_drvdata(spi, eneec);

	// Read rspdata to sync toggle bit.
	err = eneec_read_rspdata(eneec, &rspdata);
	if (err < 0) {
		printk("eneec_spi_read_rspdata fail \n");
		return err;
	}
	eneec->toggle = rspdata.toggle;

	err = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
					eneec_interrupt,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"eneec_spi", eneec);

	if (err) {
		printk("Failed to request IRQ %d -- %d\n", spi->irq, err);
		return err;
	}

	return 0;
}

static const struct of_device_id ariel_ec_of_match[] = {
	{ .compatible = "dell,wyse-ariel-ec-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, ariel_ec_of_match);

static const struct spi_device_id ariel_ec_id_table[] = {
	{ "wyse-ariel-ec-spi", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ariel_ec_id_table);

static struct spi_driver ariel_ec_spi_driver = {
	.driver = {
		.name	= "dell-wyse-ariel-ec-spi",
		.of_match_table = ariel_ec_of_match,
	},
	.probe		= eneec_probe,
};
module_spi_driver(ariel_ec_spi_driver);


MODULE_AUTHOR("Victoria/flychen");
MODULE_DESCRIPTION("ENEEC SPI driver");
MODULE_LICENSE("GPL");
