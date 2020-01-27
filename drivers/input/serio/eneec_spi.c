// SPDX-License-Identifier: BSD-2-Clause OR GPL-2.0-or-later

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/input.h>

struct eneec_response {
	u8 reserved;
	u8 toggle:2;
	u8 count:2;
	u8 type:4;
	u8 data[3];
} __packed;

struct eneec {
	struct spi_device *client;
	struct input_dev *input;
	int toggle;
};

static int eneec_read(struct eneec *eneec,
		      struct eneec_response *response)
{
	u8 read_request[] = { 0x00, 0x5a, 0xa5, 0x00, 0x00 };
	struct spi_device   *spi = eneec->client;
	struct spi_transfer t = {
		.tx_buf = read_request,
		.rx_buf = response,
		.len = sizeof(read_request),
	};

	compiletime_assert(sizeof(read_request) == sizeof(*response),
			   "SPI xfer request/response size mismatch");

	return spi_sync_transfer(spi, &t, 1);
}

static irqreturn_t eneec_interrupt(int irq, void *dev_id)
{
	struct eneec *eneec = dev_id;
	struct spi_device *spi = eneec->client;
	struct eneec_response eneec_response;
	int i;

	if (eneec_read(eneec, &eneec_response) < 0) {
		dev_err(&spi->dev, "EC read failed.\n");
		return IRQ_HANDLED;
	}

	if (eneec->toggle == eneec_response.toggle) {
		dev_warn(&spi->dev, "No new data to read?\n");
		return IRQ_HANDLED;
	}

	eneec->toggle = eneec_response.toggle;

	if (eneec_response.type != 0x03) {
		dev_dbg(&spi->dev, "Ignoring message that's not kbd data.\n");
		return IRQ_HANDLED;
	}

	if (eneec_response.count > ARRAY_SIZE(eneec_response.data)) {
		eneec_response.count = ARRAY_SIZE(eneec_response.data);
		dev_warn(&spi->dev, "Truncating a long response\n");
	}

	for (i = 0; i < eneec_response.count; i++) {
		dev_err(&spi->dev, "scan code %02x\n", eneec_response.data[i]);
		switch (eneec_response.data[i]) {
		case 0x74:
			input_report_key(eneec->input, KEY_POWER, 1);
			input_sync(eneec->input);
			break;
		case 0xf4:
			input_report_key(eneec->input, KEY_POWER, 0);
			input_sync(eneec->input);
			break;
		default:
			dev_dbg(&spi->dev, "Unknown scan code: %02x\n",
					   eneec_response.data[i]);
		}
	}

	return IRQ_HANDLED;
}

static int eneec_probe(struct spi_device *spi)
{
	struct eneec *eneec = 0;
	struct eneec_response eneec_response;
	int ret;

	if (!spi->irq) {
		dev_err(&spi->dev, "Missing IRQ.\n");
		return -ENXIO;
	}

	eneec = devm_kzalloc(&spi->dev, sizeof(struct eneec), GFP_KERNEL);
	if (!eneec)
		return -ENOMEM;

	eneec->client = spi;
	spi_set_drvdata(spi, eneec);

	eneec->input = devm_input_allocate_device(&spi->dev);
	if (!eneec->input)
		return -ENOMEM;
	eneec->input->name = "Power Button";
	eneec->input->dev.parent = &spi->dev;
	input_set_capability(eneec->input, EV_KEY, KEY_POWER);
	ret = input_register_device(eneec->input);
	if (ret) {
		dev_err(&spi->dev, "error registering input device: %d\n", ret);
		return ret;
	}

	ret = eneec_read(eneec, &eneec_response);
	if (ret < 0) {
		dev_err(&spi->dev, "EC read failed: %d\n", ret);
		return ret;
	}
	eneec->toggle = eneec_response.toggle;

	ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
					eneec_interrupt,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"eneec_spi", eneec);

	if (ret) {
		dev_err(&spi->dev, "Failed to request IRQ %d: %d\n",
				   spi->irq, ret);
		return ret;
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

MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_DESCRIPTION("ENEEC SPI driver");
MODULE_LICENSE("Dual BSD/GPL");
