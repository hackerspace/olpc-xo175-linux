/*
 *  ENE EC i2c driver for Linux
 *
 *  v.1.2
 *  - Add XBI access functions.
 *  v.1.1
 *  - for kernel 2.6.31 (new i2c model)
 */

static inline int machine_is_qseven(void) { return 1; }

#include <linux/module.h>
#include <linux/i2c.h>

MODULE_AUTHOR("Victoria/flychen");
MODULE_DESCRIPTION("ENEEC i2c driver");
MODULE_LICENSE("GPL");

static const struct i2c_device_id eneec_idtable[] = {
	//  name      driver_data
	{ "kb3930",     0x3926f0 }, // doesn't matter
	{ "kb3900",     0x3926f0 }, // doesn't matter
	{ "kb9010a",    0x9010a0 },
	{ "kb9010b",    0x9010b0 },
	{ "kb3700",     0x3730a0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, eneec_idtable);

/*
 * The word read from EC
 * bit[0]=toggle bit;
 * bit[1]=0: command ack or response from mouse or keyboard;
 *       =1: keyboard scancode or mouse position data;
 * bit[6:4]=3: from keyboard; =4: from mouse;
 */
#define PORT_ECRAM	0x80

/* Define the used RAM register address for reading content purpose */
//#define MAX_ECRAM_REG		4

static const u8 ariel_ram_reg[] = {
        0x01, 0x02, 0x03, 0x11, 0x12, 0x13, 0x14, 0x15,
        0x16, 0x20, 0x21, 0x22, 0x30, 0x31, 0x32
};

struct eneec {
	struct i2c_client *client;
	const u8 *ram_reg;
	int ram_reg_size;
	int toggle;
};

static struct i2c_client *ene_client = NULL ;

static int eneec_register_update_all(void);

///////////////////////////////////////////////////////////////////////////////////////////////////
// Utilities

static u8 ecram_val[256];

#if 0
static int eneec_register_update(u8 addr)
{
	int err;
	u16 data_1;

	struct i2c_client *client = ene_client;

	data_1 = addr | (((u16) 0x0) << 8);
	err = i2c_smbus_write_word_data(client, PORT_ECRAM|0x01, data_1);
	if (err) {
		pr_err("kb3930 i2c bus error(0x%04x)",err);
		return 1;
	}

//        ENE_DEBUG("kb3930 Read_Write: id:%02x reg:%02x \n", (PORT_ECRAM|0x01), addr );
	data_1 = i2c_smbus_read_word_data(client, 0);
//        ENE_DEBUG("kb3930 Read_Resp: id:%02x reg:%02x data:%02x\n", (data_1&0xFF), addr, (data_1>>8));
	ecram_val[addr]=(u8)(data_1>>8);

	return 0;
}
#endif

static int eneec_register_update_all(void)
{
	struct eneec *eneec = i2c_get_clientdata(ene_client);
	int i,err;
	u16 data_1;

	struct i2c_client *client = ene_client;

	for (i=0; i < eneec->ram_reg_size; i++) {
		u8 reg = eneec->ram_reg[i];
		data_1 = reg | (((u16) 0x0) << 8);
		err = i2c_smbus_write_word_data(client, PORT_ECRAM|0x01, data_1);
		if (err) {
			pr_err("kb3930 i2c bus error(0x%04x)",err);
			return 1;
		}
		data_1 = i2c_smbus_read_word_data(client, 0);
		ecram_val[reg]=(u8)(data_1>>8);
	}

	return 0;
}

/*
 * sysfs
 */
static ssize_t read_ec_ram_value(struct device *dev, struct device_attribute
	          *devattr, char *buf) {
	int i,count;
	struct eneec *eneec = i2c_get_clientdata(ene_client);

	eneec_register_update_all();
	count = sprintf(buf, "Reg:Data\n");

	for (i=0; i < eneec->ram_reg_size; i++) {
		u8 reg = eneec->ram_reg[i];
		count += sprintf(buf + count,"%02X:%02X\n",reg,ecram_val[reg]);
	}

	return count;
}

static ssize_t set_ec_ram_value(struct device *dev, struct device_attribute *devattr,
	         const char *buf, size_t count) {
	struct eneec *eneec = i2c_get_clientdata(ene_client);
	u8 cmd = 0;
	int ret;
	struct i2c_client *client = eneec->client;
	u8 buffer[2]="", *temp;
	u16 data_1;

	/* Write buffer value to ECRAM  */
	if (count>3) {
		cmd = PORT_ECRAM;
		buffer[0] = (u8) simple_strtoul(buf,NULL,16);
		temp = strchr(buf, 0x20); //string with space character
		buffer[1] = (u8) simple_strtoul(&temp[1],NULL,16);
		data_1 = buffer[0] | (((u16) buffer[1]) << 8);  // Register addr | data value
		ret = i2c_smbus_write_word_data(client, PORT_ECRAM, data_1);
		//ENE_DEBUG("kb3930 Write: ID:%02x Data:%04x\n", cmd, data_1);
	} else if (count) {
		/* Read ECRAM test */
		data_1 = (u16) simple_strtoul(buf,NULL,16);
		ret = i2c_smbus_write_word_data(client, PORT_ECRAM|0x01, data_1);
		data_1 = i2c_smbus_read_word_data(client, PORT_ECRAM);
		//ENE_DEBUG("kb3930: R:%04x\n", data_1);
	}

	return count;
}

struct battery_device_attribute{
       struct device_attribute dev_attr;
       int index;
};

#define BATTERY_ATTR(_name, _mode, _show, _store, _index)        \
       { .dev_attr = __ATTR(_name, _mode, _show, _store),      \
         .index = _index }

#define BATTERY_DEVICE_ATTR(_name, _mode, _show, _store, _index) \
struct battery_device_attribute battery_dev_attr_##_name          \
       = BATTERY_ATTR(_name, _mode, _show, _store, _index)

BATTERY_DEVICE_ATTR(ec_ram, S_IRUGO | S_IWUSR, read_ec_ram_value, set_ec_ram_value ,0);


static struct attribute *ariel_ec_attributes[] = {
        &battery_dev_attr_ec_ram.dev_attr.attr,
        NULL
};

static const struct attribute_group ariel_ec_group = {
        .attrs = ariel_ec_attributes,
};




static int eneec_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct eneec *eneec = 0;
	int err = 0;
	s32 data;
	int retry;

	pr_info("eneec_probe with name = %s, addr = 0x%x, irq = %d\n",
		    client->name, client->addr, client->irq);

	eneec = kzalloc(sizeof(struct eneec), GFP_KERNEL);
	if (eneec == NULL)
		return -ENOMEM;

	eneec->client = client;
	i2c_set_clientdata(client, eneec);

	if (machine_is_qseven()){
                eneec->ram_reg = ariel_ram_reg;
                eneec->ram_reg_size = ARRAY_SIZE(ariel_ram_reg);
	}

	ene_client = eneec->client;

	// Drain old data out of device.
	data = i2c_smbus_read_word_data(client, 0);
	//ENE_DEBUG("Drain data : 0x%04x\n", (u16) data);
	for (retry = 0; retry < 9; retry++) {
		eneec->toggle = i2c_smbus_read_word_data(client , 0);
		if (eneec->toggle == data) // no new data
			break;
		else {
			printk("Drain data : 0x%04x\n", (u16) eneec->toggle);
			data = eneec->toggle;
		}
	}
	eneec->toggle = data & 1;
//	eneec_register_update_all();

	/* create the sysfs file for ec_ram_write */
        if (machine_is_qseven())
                err = sysfs_create_group(&client->dev.kobj, &ariel_ec_group);

	return err;
}

static int eneec_remove(struct i2c_client *client)
{
	struct eneec *eneec = i2c_get_clientdata(client);

        if (machine_is_qseven())
                sysfs_remove_group(&client->dev.kobj, &ariel_ec_group);

	kfree(eneec);
	return 0;
}

static const struct of_device_id eneec_dt_ids[] = {
	{ .compatible = "ene,kb3930" },
	{ .compatible = "ene,kb3900" },
	{ .compatible = "ene,kb9010a" },
	{ .compatible = "ene,kb9010b" },
	{ .compatible = "ene,kb3700" },
	{ }
};
MODULE_DEVICE_TABLE(of, eneec_dt_ids);

static struct i2c_driver eneec_driver = {
	.driver = {
		.of_match_table = of_match_ptr(eneec_dt_ids),
		.owner  = THIS_MODULE,
		.name = "eneec",
	},
	.id_table   = eneec_idtable,
	.probe		= eneec_probe,
	.remove		= eneec_remove,
};

module_i2c_driver(eneec_driver);
