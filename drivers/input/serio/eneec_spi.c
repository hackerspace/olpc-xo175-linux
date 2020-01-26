#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/spi/spi.h>

enum {
	PORT_KBD = 0x30,
	PORT_MOU = 0x40,
	PORT_XBI = 0xF0,
};

#pragma pack(1)
struct rspdata {
    u8 resv;
    u8 toggle : 2;
    u8 count : 2;
    u8 type : 4; // RSP_XXX
    u8 data[3];
};
#pragma pack()

enum { RSP_UNKNOWN = 0,
       RSP_WRITE_COMPLETE = 1,
       RSP_READ           = 2,
       RSP_KB_RESPONSE    = 3,
       RSP_AUX_RESPONSE   = 4,
       RSP_PORT_RESPONSE  = 5,
       RSP_KB_DATA        = 12,
       RSP_AUX_DATA       = 13,
       RSP_PORT_DATA      = 14,
       RSP_RESET          = 15,
};

struct eneec {
    struct spi_device *client;
    struct mutex lock;
    int irq; // don't remove it although also in client->irq. for indicating if irq requested.
    struct workqueue_struct *work_queue;
    struct work_struct read_work;
    struct serio *serio;
    int toggle;
};

static int eneec_read_rspdata(struct eneec *eneec, struct rspdata *rspdata);

///////////////////////////////////////////////////////////////////////////////////////////////////
// Utilities

static int eneec_read_rspdata(struct eneec *eneec, struct rspdata *rspdata)
{
#define RSP_LEN    5
    u8    tx_buf[RSP_LEN];
    int   ret;
    struct spi_device   *spi = eneec->client;
    struct spi_transfer t =
    {
        .tx_buf     = tx_buf,
        .rx_buf     = rspdata,
        .len        = RSP_LEN,
    };
    struct spi_message  m;

    tx_buf[0] = 0x00; // Read command
    tx_buf[1] = 0x5A; // Rx1
    tx_buf[2] = 0xA5; // Rx2
    tx_buf[3] = 0x00; // Rx3
    tx_buf[4] = 0x00; // dummy
    
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    mutex_lock(&eneec->lock);
    ret =spi_sync(spi, &m);
    mutex_unlock(&eneec->lock);

    if (ret < 0) {
        pr_err("ERROR: spi_sync fail\n");
        return ret;
    }

#if 0
    if (rspdata->type == 12) {
	pr_debug("type = %d, toggle = %d, count = %d\n", rspdata->type, rspdata->toggle, rspdata->count);
    	pr_debug("data = ");
    	for (i = 0; i < rspdata->count; i++)
        	pr_debug("%02x ", rspdata->data[i]);
    	pr_debug("\n");
    }
#endif

    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Codes for interfacing with kernel kbd

static void eneec_read_work(struct work_struct *work)
{ 
    struct eneec *eneec = container_of(work, struct eneec, read_work);

    int i;
    struct rspdata rspdata;

    if(eneec_read_rspdata(eneec, &rspdata) < 0) {
        pr_err("ERROR: eneec_read_rspdata fail \n");
        goto exit_enable_irq;
    }

    if (eneec->toggle == rspdata.toggle) {
        printk("WARNING: got the same toggle bit %d\n", rspdata.toggle);
        goto exit_enable_irq;
    }

    eneec->toggle = rspdata.toggle;

    if ((rspdata.type == RSP_KB_RESPONSE) || (rspdata.type == RSP_KB_DATA)) {
        if(eneec->serio) { // interrupt might assert before probe() done.
            for (i = 0; i < rspdata.count; i++) {
		printk("scan code %02x\n",rspdata.data[i]);
                serio_interrupt(eneec->serio, rspdata.data[i], 0);
            }
        }
    }
    else {
        pr_warn(   "WARNING: Not kbd data\n");
        goto exit_enable_irq;
    }

exit_enable_irq:

    enable_irq(eneec->irq);
}

static irqreturn_t eneec_interrupt(int irq, void *dev_id)
{
    struct spi_device *spi = dev_id;
    struct eneec *eneec = spi_get_drvdata(spi);

    disable_irq_nosync(irq);

    queue_work(eneec->work_queue, &eneec->read_work);

    return IRQ_HANDLED;
}

static int eneec_write(struct serio *serio, unsigned char byte)
{
#define WRITE_LEN    4
    u8    tx_buf[WRITE_LEN] = {0,0,0,0};
    u8    rx_buf[WRITE_LEN] = {0,0,0,0};
    int   ret;
    struct spi_transfer t = 
    {
            .tx_buf     = tx_buf,
            .rx_buf     = rx_buf,
            .len        = WRITE_LEN,
    };
    struct eneec *eneec = serio->port_data;
    struct spi_device   *spi = eneec->client;
    struct spi_message  m;

    tx_buf[0] = PORT_KBD; // write kbd
    tx_buf[1] = byte;
    tx_buf[2] = 0x00; // dummy
    tx_buf[3] = 0x00; // dummy

//TEST  
//tx_buf[0] = 0x5d; // dummy
//tx_buf[1] = 0x00; // dummy
    
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    mutex_lock(&eneec->lock);
    ret = spi_sync(spi, &m);
    mutex_unlock(&eneec->lock);

    if (ret < 0) {
        pr_err("ERROR: eneec_write() fail ..\n");
        return ret;
    }

#if 0
    printk( KERN_EMERG  "tx[0] = %02x, tx[1] = %02x, tx[2] = %02x, tx[3] = %02x\n", tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
    printk( KERN_EMERG  "rx[0] = %02x, rx[1] = %02x, rx[2] = %02x, rx[3] = %02x\n", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
#endif

    return 0;
}

static int eneec_probe(struct spi_device *spi)
{
    struct eneec *eneec = 0;
    struct rspdata rspdata;
    int err = 0 , retry = 0;
    struct serio *serio; // XXX

    printk("eneec_spi_probe with modalias = %s, irq = %d\n", spi->modalias, spi->irq);

    if(spi->irq <= 0) {
        pr_err("ERROR: spi->irq is not specified.\n");
        return -ENXIO;
    }

    if (!(eneec = kzalloc(sizeof(struct eneec), GFP_KERNEL))) {
        return -ENOMEM;
    }

    eneec->client = spi;
    spi_set_drvdata(spi, eneec);
    mutex_init(&eneec->lock);

    // Read rspdata to sync toggle bit.
    if(eneec_read_rspdata(eneec, &rspdata) < 0) {
        printk("eneec_spi_read_rspdata fail \n");
        goto error_exit; 
    }
    eneec->toggle = rspdata.toggle;
    printk("Drain data : 0x%04x\n", (u16) rspdata.data[1]);
    for(retry = 0; retry < 9; retry++) {
	eneec_read_rspdata(eneec, &rspdata);
        if(eneec->toggle == rspdata.toggle) // no new data
            break;
        else {
            printk("Drain data : 0x%04x\n", (u16) rspdata.data[1]);
            eneec->toggle=rspdata.toggle;
        }
    }

    if((eneec->work_queue = create_singlethread_workqueue("eneec_spi"))) {
        INIT_WORK(&eneec->read_work, eneec_read_work);
    } else
        goto error_exit;


    if ((err = request_irq(spi->irq, eneec_interrupt, IRQF_TRIGGER_RISING , "eneec_spi", spi ))) {
        printk("Failed to request IRQ %d -- %d\n", spi->irq, err);
        goto error_exit;
    }

    printk("IRQ %d requested\n", spi->irq);
    eneec->irq = spi->irq;

    serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
    if (!serio)
        return -ENOMEM;

    serio->id.type      = SERIO_8042_XL;
    serio->write        = eneec_write;
    serio->port_data    = eneec;
    serio->dev.parent   = &spi->dev;
    strlcpy(serio->name, "eneec spi kbd port", sizeof(serio->name));
    strlcpy(serio->phys, "eneec_spi/serio0", sizeof(serio->phys));

    eneec->serio = serio;

    serio_register_port(eneec->serio);

    // add by allen for test
    //eneec_read_rspdata(eneec, &rspdata);

    return 0;

error_exit:

    if(eneec && eneec->irq) // deregister irq first, so safe once if kbd is sending data.
        free_irq(eneec->irq, spi);

    if(eneec && eneec->serio) {
        serio_unregister_port(eneec->serio);
    }

    if(eneec && eneec->work_queue) {
        destroy_workqueue(eneec->work_queue);
    }

    if(eneec)
        kfree(eneec);

    return err;

}

static int eneec_remove(struct spi_device *spi)
{
    struct eneec *eneec = spi_get_drvdata(spi);

    pr_debug("eneec_spi_remove\n");

    if(eneec->serio) {
        serio_unregister_port(eneec->serio);
    }
    if(eneec->irq) {
        free_irq(eneec->irq, spi);
    }
    if(eneec->work_queue) {
        destroy_workqueue(eneec->work_queue);
    }

    kfree(eneec);

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
	.remove		= eneec_remove,
};
module_spi_driver(ariel_ec_spi_driver);



MODULE_AUTHOR("Victoria/flychen");
MODULE_DESCRIPTION("ENEEC SPI driver");
MODULE_LICENSE("GPL");
