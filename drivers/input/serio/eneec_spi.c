
/*
 *  ENE EC SPI driver for Linux
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/spi/spi.h>
#include "eneec_ioc.h"

#define DRIVER_DESC "ENEEC SPI driver"

MODULE_AUTHOR("Victoria/flychen");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

//#define POLLING
#define POLL_INTERVAL       100

/*
 * Be sure system is added with one SPI board info at mainboard arch_initcall() code, for example,
 *
 * static struct spi_board_info mini2440_spi_board_info[] __initdata = {
 *     { .modalias      = "eneec",
 *       .platform_data = NULL,
 *       .irq           = IRQ_EINT8,   // irq used for EC depending on customer platform.
 *       .max_speed_hz  = 1*1000*1000,
 *       .chip_select   = 0,           // chip-select for EC.
 *       .bus_num       = 0,           // SPI bus number EC attached to.
 *       .mode          = 0
 *     },
 *     ...
 * };
 *
 * static void __init mini2440_machine_init()
 * {
 *     spi_register_board_info(mini2440_spi_board_info, ARRAY_SIZE(mini2440_spi_board_info));
 * }
 */

#define PORT_KBD        0x30
#define PORT_MOU        0x40
#define PORT_XBI        0xF0
#define DATA_TO_PORT(data)  (data & 0x70)

struct eneec_port {
    struct serio *serio;
    struct eneec *eneec;
    unsigned char port_no;  // PORT_KBD or PORT_MOU.
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
    struct spi_device   *client;
    struct mutex lock;
    int irq; // don't remove it although also in client->irq. for indicating if irq requested.
    unsigned short chip_id;
    unsigned char rev_id;
    struct workqueue_struct *work_queue;
#ifdef POLLING
    struct delayed_work read_work;
    bool stop_polling;
#else
    struct work_struct read_work;
#endif
    struct eneec_port kbd_port;
    struct eneec_port mou_port;
    int toggle;
    bool is_37xx;

// For exposing our userspace API.
    // The main reason to have this class is to make mdev/udev create the
    // /dev/eneec character device nodes exposing our userspace API.
    struct class *eneec_class;
    struct cdev cdev;
    dev_t devt;
// ~
};

static int eneec_create_kbd_port(struct eneec *eneec);
static int eneec_create_mouse_port(struct eneec *eneec);
static int eneec_spi_create_cdev_node(struct eneec *eneec);
static int eneec_read_rspdata(struct eneec *eneec, struct rspdata *rspdata);

///////////////////////////////////////////////////////////////////////////////////////////////////
// Utilities

static int eneec_read_rspdata(struct eneec *eneec, struct rspdata *rspdata)
{
#define RSP_LEN    5
    u8    tx_buf[RSP_LEN];
    int   ret;
    int   i;
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

static long eneec_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return 0;
}

static int eneec_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static int eneec_release(struct inode *inode, struct file *filp)
{
    pr_debug("eneec_spi_release()\n");
    return 0;
}

static const struct file_operations eneec_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = eneec_ioctl,
    .open           = eneec_open,
    .release        = eneec_release,
};

//
// Undo eneec_create_cdev_node().
// Call this only if the char device node was ever created successfully.
void eneec_spi_destroy_cdev_node(struct eneec *eneec)
{
    device_destroy(eneec->eneec_class, eneec->devt);
    cdev_del(&eneec->cdev);
    unregister_chrdev_region(eneec->devt, 1);
    class_destroy(eneec->eneec_class);
}

int eneec_spi_create_cdev_node(struct eneec *eneec)
{
    int status;
    dev_t devt;
    struct device *dev;
    struct class *eneec_class;
    bool is_class_created = false, is_region_allocated = false, is_cdev_added = false, is_device_created = false;


    pr_debug("eneec_spi_create_cdev_node .. \n");

    // Create class
    eneec_class = class_create(THIS_MODULE, "eneec_spi");
    status = IS_ERR(eneec_class) ? PTR_ERR(eneec_class) : 0;
    if (status < 0) {
        pr_err("eneec_SPI class_create() failed -- %d\n", status);
        goto error_exit;
    }
    is_class_created = true;

    // Alloc chrdev region.
    status = alloc_chrdev_region(&devt, 0, 1, "eneec_spi");
    if(status < 0) {
        pr_err("eneec_spi alloc_chrdev_region() failed -- %d\n", status);
        goto error_exit;
    }
    is_region_allocated = true;

    // Add cdev.
    cdev_init(&eneec->cdev, &eneec_fops);
    status = cdev_add(&eneec->cdev, devt, 1);
    if(status < 0) {
        pr_err("cdev_add() failed -- %d\n", status);
        goto error_exit;
    }
    is_cdev_added = true;

    // Create device
    dev = device_create
                (
                eneec_class, 
                &eneec->client->dev, // parent device (struct device *)
                devt, 
                eneec,               // caller's context
                "eneec_spi"
                );
    status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    if (status < 0) {
        pr_err("device_create() failed -- %d\n", status);
        goto error_exit;
    }
    is_device_created = true;

    // Succeed.
    eneec->eneec_class = eneec_class;
    eneec->devt = devt;

    return 0;

error_exit:

    if(is_device_created)
        device_destroy(eneec_class, devt);
    if(is_cdev_added)
        cdev_del(&eneec->cdev);
    if(is_region_allocated)
        unregister_chrdev_region(devt, 1);
    if(is_class_created)
        class_destroy(eneec_class);

    return status;

}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Codes for interfacing with kernel kbd, mouse.

static void eneec_read_work(struct work_struct *work)
{ 
#ifdef POLLING
    struct eneec *eneec = container_of(work, struct eneec, read_work.work);
#else
    struct eneec *eneec = container_of(work, struct eneec, read_work);
#endif

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
        if(eneec->kbd_port.serio) { // interrupt might assert before probe() done.
            for (i = 0; i < rspdata.count; i++) {
		printk("scan code %02x\n",rspdata.data[i]);
                serio_interrupt(eneec->kbd_port.serio, rspdata.data[i], 0);
            }
        }
    } else if ((rspdata.type == RSP_AUX_RESPONSE) || (rspdata.type == RSP_AUX_DATA)) {   
        if(eneec->mou_port.serio) { // interrupt might assert before probe() done.
            for (i = 0; i < rspdata.count; i++) {
                serio_interrupt(eneec->mou_port.serio, rspdata.data[i], 0);
            }
        }
    }
    else {
        pr_warning(   "WARNING: Not mou or kbd data\n");
        goto exit_enable_irq;
    }

exit_enable_irq:

#ifdef POLLING
    if(!eneec->stop_polling)
        queue_delayed_work(eneec->work_queue, &eneec->read_work, msecs_to_jiffies(POLL_INTERVAL));
#else
    enable_irq(eneec->irq);
#endif
}

#ifndef POLLING

static irqreturn_t eneec_interrupt(int irq, void *dev_id)
{
    struct spi_device *spi = dev_id;
    struct eneec *eneec = spi_get_drvdata(spi);

    disable_irq_nosync(irq);

    queue_work(eneec->work_queue, &eneec->read_work);

    return IRQ_HANDLED;
}

#endif

static int eneec_write(struct serio *serio, unsigned char byte)
{
#define WRITE_LEN    4
    u8    portno;        
    u8    tx_buf[WRITE_LEN] = {0,0,0,0};
    u8    rx_buf[WRITE_LEN] = {0,0,0,0};
    int   ret;
    struct spi_transfer t = 
    {
            .tx_buf     = tx_buf,
            .rx_buf     = rx_buf,
            .len        = WRITE_LEN,
    };
    struct eneec_port *port = serio->port_data;
    struct eneec *eneec = port->eneec;      
    struct spi_device   *spi = eneec->client;
    struct spi_message  m;

    portno = port->port_no;

    if(portno == PORT_KBD) {
        tx_buf[0] = 0x30; // write kbd
    }
    else if(portno == PORT_MOU) {
        tx_buf[0] = 0x40; // write aux
	mdelay(2);
    }
    else {
        pr_err("ERROR: eneec_write with bad port no\n");
        return -1;
    }

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

static int eneec_create_kbd_port(struct eneec *eneec)
{
    struct spi_device   *client = eneec->client;
    struct serio *serio;

    serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
    if (!serio)
        return -ENOMEM;

    serio->id.type      = SERIO_8042_XL;
    serio->write        = eneec_write;
    serio->port_data    = &eneec->kbd_port;
    serio->dev.parent   = &client->dev;
    strlcpy(serio->name, "eneec spi kbd port", sizeof(serio->name));
    strlcpy(serio->phys, "eneec_spi/serio0", sizeof(serio->phys));

    eneec->kbd_port.serio = serio;
    eneec->kbd_port.port_no = PORT_KBD;
    eneec->kbd_port.eneec = eneec;

    return 0;

}

static int eneec_create_mouse_port(struct eneec *eneec)
{
    struct serio *serio;
    struct spi_device   *client = eneec->client;;
    
    serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
    if (!serio)
        return -ENOMEM;

    serio->id.type      = SERIO_PS_PSTHRU;//SERIO_8042;
    serio->write        = eneec_write;
    serio->port_data    = &eneec->mou_port;
    serio->dev.parent   = &client->dev;
    strlcpy(serio->name, "eneec spi mouse port", sizeof(serio->name));
    strlcpy(serio->phys, "eneec_spi/serio1", sizeof(serio->phys));

    eneec->mou_port.serio = serio;
    eneec->mou_port.port_no = PORT_MOU;
    eneec->mou_port.eneec = eneec;

    return 0;
}

static int eneec_probe(struct spi_device *spi)
{
    struct eneec *eneec = 0;
    struct rspdata rspdata;
    int err = 0 , retry = 0;

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
#ifdef POLLING
        INIT_DELAYED_WORK(&eneec->read_work, eneec_read_work);
        queue_delayed_work(eneec->work_queue, &eneec->read_work, msecs_to_jiffies(POLL_INTERVAL));
#else
        INIT_WORK(&eneec->read_work, eneec_read_work);
#endif
    } else
        goto error_exit;


#ifndef POLLING
    if ((err = request_irq(spi->irq, eneec_interrupt, IRQF_TRIGGER_RISING , "eneec_spi", spi ))) {
        printk("Failed to request IRQ %d -- %d\n", spi->irq, err);
        goto error_exit;
    } else {
        printk("IRQ %d requested\n", spi->irq);
        eneec->irq = spi->irq;
    }
#endif

    if ((err = eneec_create_kbd_port(eneec)))
        goto error_exit;

    serio_register_port(eneec->kbd_port.serio);

    if ((err = eneec_create_mouse_port(eneec)))
        goto error_exit;

    serio_register_port(eneec->mou_port.serio);

    if ((err = eneec_spi_create_cdev_node(eneec)))
        goto error_exit;
        
    // add by allen for test
    //eneec_read_rspdata(eneec, &rspdata);

    return 0;

error_exit:

    if(eneec && eneec->irq) // deregister irq first, so safe once if kbd/mou is sending data.
        free_irq(eneec->irq, spi);

    if(eneec && eneec->mou_port.serio) {
        serio_unregister_port(eneec->mou_port.serio);
    }

    if(eneec && eneec->kbd_port.serio) {
        serio_unregister_port(eneec->kbd_port.serio);
    }

    if(eneec && eneec->work_queue) {
#ifdef POLLING
        eneec->stop_polling = true;
        cancel_delayed_work_sync(&eneec->read_work);
#endif
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

    eneec_spi_destroy_cdev_node(eneec);

    if(eneec->mou_port.serio) {
        serio_unregister_port(eneec->mou_port.serio);
    }
    if(eneec->kbd_port.serio) {
        serio_unregister_port(eneec->kbd_port.serio);
    }
    if(eneec->irq) {
        free_irq(eneec->irq, spi);
    }
    if(eneec->work_queue) {
#ifdef POLLING
        eneec->stop_polling = true;
        cancel_delayed_work_sync(&eneec->read_work);
#endif
        destroy_workqueue(eneec->work_queue);
    }

    kfree(eneec);

    return 0;

}

static struct spi_driver eneec_driver = {
    .driver = {
        .name   = "eneec_spi",
        .owner  = THIS_MODULE,
    },
    .probe      = eneec_probe,
    .remove     = __devexit_p(eneec_remove),
};

static int __init eneec_init(void)
{
    printk("eneec_spi_init\n");

    return spi_register_driver(&eneec_driver);
}

static void __exit eneec_exit(void)
{
    pr_debug("eneec_spi_exit\n");
    
    spi_unregister_driver(&eneec_driver);
}

module_init(eneec_init);
module_exit(eneec_exit);
