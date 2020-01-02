/*
 *  ENE EC i2c driver for Linux
 *
 *  v.1.2
 *  - Add XBI access functions.
 *  v.1.1
 *  - for kernel 2.6.31 (new i2c model)
 */

static inline int machine_is_titan(void)  { return 0; }
static inline int machine_is_mimas(void)  { return 0; }
static inline int machine_is_qseven(void) { return 1; }

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>

#include "eneec_ioc.h"

#define DRIVER_DESC "ENEEC i2c driver"
#include <linux/param.h>
#include <linux/pm.h>
#ifdef SUSPEND_RESUME
#include <mach/mfp-mmp2.h>
#endif
//#include <plat/mfp.h>

MODULE_AUTHOR("Victoria/flychen");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

//#define POLLING
#define POLLING
#define POLL_INTERVAL       100
//#define EC_BATTERY_SUPPORT
#define ENE_DEBUG(format, args...) \
	 pr_warn("%s(%d): "format"\n", __func__, __LINE__, ##args)
	 //pr_debug("%s(%d): "format"\n", __func__, __LINE__, ##args)

struct battery_device_attribute{
	struct device_attribute dev_attr;
	int index;
};
#define to_battery_dev_attr(_dev_attr) \
	container_of(_dev_attr, struct battery_device_attribute, dev_attr)

#define BATTERY_ATTR(_name, _mode, _show, _store, _index)        \
	{ .dev_attr = __ATTR(_name, _mode, _show, _store),      \
	  .index = _index }

#define BATTERY_DEVICE_ATTR(_name, _mode, _show, _store, _index) \
struct battery_device_attribute battery_dev_attr_##_name          \
	= BATTERY_ATTR(_name, _mode, _show, _store, _index)


/*
 * Be sure system is added with one i2c board info for one of the KBC chip
 * at mainboard arch_initcall() code, for example,
 *
 * static struct i2c_board_info system_i2c_devs[] = {
 *     { .type = "KB39XX", // one of i2c_device_id.name below;
 *       .addr = 0x18,     // i2c slave address for EC, which must be the same as EC's SMBRSA register(offset 0xFFA4) bit7:1.
 *       .irq  = 220,      // irq used for request_irq().
 *     },
 * };
 *
 * static void __init mini2440_machine_init()
 * {
 *     i2c_register_board_info(0, system_i2c_devs, ARRAY_SIZE(system_i2c_devs));
 * }
 */
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
#define PORT_KBD	0x30
#define PORT_MOU	0x40
#define PORT_ECRAM	0x80
#define PORT_XBI	0xF0
#define DATA_TO_PORT(data)  (data & 0xf0)

/* Define the used RAM register address for reading content purpose */
//#define MAX_ECRAM_REG		4
static const u8 mimas_ram_reg[] = {
	0x01, 0x02, 0x03, 0x11, 0x12, 0x13, 0x14, 0x15,
	0x16, 0x20, 0x21, 0x22, 0x30, 0x31, 0x32
};

static const u8 titan_ram_reg[] = {
	0x01, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
	0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
	0x19 ,0x20, 0x21, 0x22, 0x23, 0x24, 0x30, 0x31,
	0x32, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46,
	0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x51 ,0x52,
	0x53, 0x54, 0x55, 0x56, 0x57, 0x5A, 0x5B, 0x5C,
	0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63, 0x64,
	0x65, 0x66 ,0x67, 0x68, 0x69, 0x74, 0x75, 0xF0,
	0xF1, 0xF2, 0xF3
};

static const u8 ariel_ram_reg[] = {
        0x01, 0x02, 0x03, 0x11, 0x12, 0x13, 0x14, 0x15,
        0x16, 0x20, 0x21, 0x22, 0x30, 0x31, 0x32
};

struct eneec_port {
	struct serio *serio;
	struct eneec *eneec;
	unsigned char port_no;  // PORT_KBD or PORT_MOU.
};

struct eneec_device_info {
	struct device *dev;

	/* Data is valid after calling eneec_battery_read_status() */
	unsigned long update_time;      /* jiffies when data read */
	int charge_status;
	int temp_tK;
	int current_mV;
	int current_mA;
	int average_mA;
	int capacity_P;
	int current_mAh;
	int absolute_mAh;
	int current_empty_time_Min;
	int average_empty_time_Min;
	int average_full_time_Min;
	int design_mAh;

	struct power_supply bat;
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
};

struct eneec {
	struct i2c_client *client;
	const u8 *ram_reg;
	int ram_reg_size;
	int serio_over_i2c;
	struct eneec_device_info *battery_info;
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
//    struct power_supply bat;
	int charge_status;
	int temp_tC;
	u16 Manufacture;
	u16 Battery_mode;
	u16 current_mV;
	short current_mA;
	short average_mA;
	u16 charge_mV;
	short charge_mA;
	int capacity_P;
	u8 abs_capacity_P;
	int current_mAh;
	int absolute_mAh;
	int current_empty_time_Min;
	int average_empty_time_Min;
	int average_full_time_Min;
	int design_mAh;
	u16 Cell_1_volt;
	u16 Cell_2_volt;
	u16 Cell_3_volt;
	u16 Cell_4_volt;
	u16 Max_Error;

};

static struct i2c_client *ene_client = NULL ;

static struct eneec_device_info *di_proc = NULL ;
static unsigned int cache_time = 20000;
static int eneec_register_update_all(void);
struct power_supply *ec_psy = NULL;

#ifdef EC_BATTERY_SUPPORT
static int eneec_probe_battery(struct eneec *);
static int eneec_remove_battery(struct eneec *);
#else
static inline int eneec_probe_battery(struct eneec *eneec) { return 0; }
static inline int eneec_remove_battery(struct eneec *eneec) { return 0; }
#endif /*EC_BATTERY_SUPPORT */

int ene_bat_proc_write(struct file *file, const char *buffer,unsigned long count, void *data)
{
	return count;
}

int ene_bat_proc_read(char* page, char** start, off_t off, int count,int* eof,
	    void* data)
{
	int len = 0;
//	struct eceec_device_info *di = to_eneec_device_info(ec_psy);
//       len += sprintf(page+len, "Charge_status = %d\n", (struct eceec_device_info *)di_proc->charge_status);


	len += sprintf(page+len, "Charge_status = %d\n", di_proc->charge_status);
	len += sprintf(page+len, "temp_tK = %d\n", di_proc->temp_tK);
	len += sprintf(page+len, "current_mV = %d\n", di_proc->current_mV);
	len += sprintf(page+len, "current_mA = %d\n", di_proc->current_mA);
	len += sprintf(page+len, "average_mA = %d\n", di_proc->average_mA);
	len += sprintf(page+len, "capacity_P = %d\n", di_proc->capacity_P);
	len += sprintf(page+len, "current_mAh = %d\n", di_proc->current_mAh);
	len += sprintf(page+len, "absolute_mAh = %d\n", di_proc->absolute_mAh);
	len += sprintf(page+len, "current_empty_time_Min = %d\n", di_proc->current_empty_time_Min);
	len += sprintf(page+len, "average_empty_time_Min = %d\n", di_proc->average_empty_time_Min);
	len += sprintf(page+len, "average_full_time_Min = %d\n", di_proc->average_full_time_Min);
	len += sprintf(page+len, "design_mAh = %d\n", di_proc->design_mAh);

	return len;
}

/*
 * Battery Driver
 */

static int eneec_battery_read_status(struct eneec_device_info *di)
{
	int old_charge_status = di->charge_status;
	struct eneec *eneec ;

	if (ene_client == NULL)
		return 0;

	eneec = i2c_get_clientdata(ene_client);

	/* Poll the device at most once in a cache_time */
	if (time_before(jiffies, di->update_time + msecs_to_jiffies(cache_time)))
		return 0;

	/* Read the Gauge status through the I2C */
	di->update_time = jiffies;              /* last update time */

	eneec_register_update_all();

	di->temp_tK	= (int)eneec->temp_tC*10+2730;
	di->current_mV 	= (int)eneec->current_mV;
	di->current_mA  = (int)eneec->current_mA;
	di->average_mA  = (int)eneec->average_mA;
	di->current_mAh = (int)eneec->current_mAh;
	di->absolute_mAh= (int)eneec->absolute_mAh;
	di->capacity_P	= (int)eneec->capacity_P;
	di->current_empty_time_Min	= 0 ;
	di->average_empty_time_Min	= 0 ;
	di->average_full_time_Min 	= 0 ;
	di->design_mAh	= (int)eneec->design_mAh;

	if (di->current_mA & 0x80000000)
		di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		di->charge_status = POWER_SUPPLY_STATUS_CHARGING;

	if ((di->current_mA & ~0x80000000) < 10) {
		pr_debug("current_mA %04x \n",di->current_mA);
		di->charge_status = POWER_SUPPLY_STATUS_FULL;
	}

	if (di->charge_status != old_charge_status) {
		power_supply_changed(&di->bat);
		cancel_delayed_work(&di->monitor_work);
		queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ/10);
	}

	dev_dbg(di->dev, "Charge_status = %d\n", di->charge_status);
	dev_dbg(di->dev, "temp_tK = %d\n", di->temp_tK);
	dev_dbg(di->dev, "current_mV = %d\n", di->current_mV);
	dev_dbg(di->dev, "current_mA = %d\n", di->current_mA);
	dev_dbg(di->dev, "average_mA = %d\n", di->average_mA);
	dev_dbg(di->dev, "capacity_P = %d\n", di->capacity_P);
	dev_dbg(di->dev, "current_mAh = %d\n", di->current_mAh);
	dev_dbg(di->dev, "absolute_mAh = %d\n", di->absolute_mAh);
	dev_dbg(di->dev, "current_empty_time_Min = %d\n", di->current_empty_time_Min);
	dev_dbg(di->dev, "average_empty_time_Min = %d\n", di->average_empty_time_Min);
	dev_dbg(di->dev, "average_full_time_Min = %d\n", di->average_full_time_Min);
	dev_dbg(di->dev, "design_mAh = %d\n", di->design_mAh);
	return 0;
}

#if 0 // proc hosed
static struct proc_dir_entry *bat_proc_entry;
extern struct proc_dir_entry proc_root;
#endif

static int battery_proc_init(void)
{
#if 0 // proc hosed
	bat_proc_entry = create_proc_entry("battery", 0666, &proc_root);
	bat_proc_entry->read_proc = ene_bat_proc_read;
	bat_proc_entry->write_proc = ene_bat_proc_write;
#endif
	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////
// Utilities

static u8 ecram_val[256];

static int eneec_register_update(u8 addr)
{
	struct eneec *eneec = i2c_get_clientdata(ene_client);
	int err;
	u16 data_1;

	struct i2c_client *client = ene_client;

	if (eneec->serio_over_i2c)
		disable_irq_nosync(eneec->irq);

	data_1 = addr | (((u16) 0x0) << 8);
	err = i2c_smbus_write_word_data(client, PORT_ECRAM|0x01, data_1);
	if (err) {
		if (eneec->serio_over_i2c)
			enable_irq(eneec->irq);

		pr_err("kb3930 i2c bus error(0x%04x)",err);
		return 1;
	}

//        ENE_DEBUG("kb3930 Read_Write: id:%02x reg:%02x \n", (PORT_ECRAM|0x01), addr );
	data_1 = i2c_smbus_read_word_data(client, 0);
//        ENE_DEBUG("kb3930 Read_Resp: id:%02x reg:%02x data:%02x\n", (data_1&0xFF), addr, (data_1>>8));
	ecram_val[addr]=(u8)(data_1>>8);

	if (eneec->serio_over_i2c)
		enable_irq(eneec->irq);
	return 0;

}
static int eneec_register_update_all(void)
{
	struct eneec *eneec = i2c_get_clientdata(ene_client);
	int i,err;
	u16 data_1;

	struct i2c_client *client = ene_client;

	if (eneec->serio_over_i2c)
		disable_irq_nosync(eneec->irq);

	for (i=0; i < eneec->ram_reg_size; i++) {
		u8 reg = eneec->ram_reg[i];
		data_1 = reg | (((u16) 0x0) << 8);
		err = i2c_smbus_write_word_data(client, PORT_ECRAM|0x01, data_1);
		if (err) {
			if (eneec->serio_over_i2c)
				enable_irq(eneec->irq);

			pr_err("kb3930 i2c bus error(0x%04x)",err);
			return 1;
		}
		data_1 = i2c_smbus_read_word_data(client, 0);
		ecram_val[reg]=(u8)(data_1>>8);
	}

	if (machine_is_titan()) {
		u16 tmp;
		eneec->current_mV	= ecram_val[VOLTAGE_L]+ecram_val[VOLTAGE_H]*256;
		tmp=ecram_val[CURRENT_H];
		tmp= (tmp<<8)+ecram_val[CURRENT_L];
		eneec->current_mA	= (short)tmp;
		tmp=ecram_val[AVG_CURRENT_H];
		tmp= (tmp<<8)+ecram_val[AVG_CURRENT_L];
		eneec->average_mA       = (short)tmp;
		eneec->charge_mV	= ecram_val[CHARGE_V_L]+ecram_val[CHARGE_V_H]*256;
		tmp=ecram_val[CHARGE_C_H];
		tmp= (tmp<<8)+ecram_val[CHARGE_C_L];
		eneec->charge_mA	= (short)tmp;
		eneec->capacity_P	= ecram_val[RSOC];
		eneec->current_mAh	= ecram_val[CAP_REMAIN_L]+ecram_val[CAP_REMAIN_H]*256;
		eneec->absolute_mAh	= ecram_val[CAP_FULL_L]+ecram_val[CAP_FULL_H]*256;
		eneec->Cell_1_volt	= ecram_val[CELL_1_V_L]+ecram_val[CELL_1_V_H]*256;
		eneec->Cell_2_volt	= ecram_val[CELL_2_V_L]+ecram_val[CELL_2_V_H]*256;
		eneec->Cell_3_volt	= ecram_val[CELL_3_V_L]+ecram_val[CELL_3_V_H]*256;
		eneec->Cell_4_volt	= ecram_val[CELL_4_V_L]+ecram_val[CELL_4_V_H]*256;
		eneec->Max_Error	= ecram_val[MAX_ERROR_L]+ecram_val[MAX_ERROR_H]*256;
		eneec->temp_tC		= ecram_val[TEMP_C];
		eneec->Manufacture      = ecram_val[MANUFACTURE_L]+ecram_val[MANUFACTURE_H]*256;
		eneec->Battery_mode     = ecram_val[BATTERY_MODE_L]+ecram_val[BATTERY_MODE_H]*256;
		eneec->abs_capacity_P	= ecram_val[ASOC];
		eneec->design_mAh       = ecram_val[CAP_DESIGN_L]+ecram_val[CAP_DESIGN_H]*256;
	}

	if (eneec->serio_over_i2c)
		enable_irq(eneec->irq);
	return 0;
}

//
// Read XBI register through i2c bus.
static int eneec_read_reg(struct eneec *eneec, unsigned short reg, unsigned char *val)
{
	s32 data;

	if (eneec->is_37xx)
		return 0;

	if ((reg & 0xFFF0) != 0) { // e.g., for XBI reg 0xFEAB, caller should pass in 0xB.
	    pr_err("eneec_read_reg(reg), reg[8:15] is not zeros.\n");
	    return -EINVAL;
	}

	//                                                                                   low       high
	// Read XBI through smbus: {slave_adr | XBI_ID + XBI_ofs[3:0] | slave_adr in | XBI_ID + 0x02 | data}
	data = i2c_smbus_read_word_data(eneec->client, PORT_XBI | reg);
	if ((u8) data != (PORT_XBI | 0x02)) { // low byte should be (XBI_ID | 0x02).
		pr_err("eneec_read_xbi() does not get XBI_ID 0x%X from EC.\n", PORT_XBI);
		return -EIDRM;
	}

	*val = data >> 8; // high byte is the data from reg.
	return 0;
}

//
// Write XBI register through i2c bus.
static int eneec_write_reg(struct eneec *eneec, unsigned short reg, unsigned char val)
{
	int err = 0;
	u16 data;

	if (eneec->is_37xx)
		return 0;

	if ((reg & 0xFFF0) != 0) { // e.g., for XBI reg 0xFEAB, caller should pass in 0xB.
		pr_err("eneec_read_reg(reg), reg[8:15] is not zeros.\n");
		return -EINVAL;
	}

	//                                                     low        high
	// Write XBI through smbus: { slave_adr | XBI_ID | XBI_ofs[3:0] | val}
	data = reg | (((u16) val) << 8);
	err = i2c_smbus_write_word_data(eneec->client, PORT_XBI, data);

	if (err < 0)
	    pr_err("i2c_smbus_write_word_data(client, 0x%X, 0x%X) failed (%d)\n", PORT_XBI, data, err);

	return err;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Codes for interfacing with user AP.

//
// Handle the get-chipid ioctl call.
// sizeof(*arg) = 4.
// *arg=output chipIdL-chipIdH-revId.
static int ioctl_get_chipid(struct eneec *eneec, unsigned long arg)
{
	unsigned long id_rev = eneec->chip_id | (eneec->rev_id << 16);
	return __put_user(id_rev, (unsigned long __user *) arg);
}

static int ioctl_enter_code_in_ram(struct eneec *eneec)
{
	if (eneec->is_37xx)
		return 0;

	return i2c_smbus_write_word_data(eneec->client, PORT_XBI, 0x10);
}

static int ioctl_exit_code_in_ram(struct eneec *eneec)
{
	if (eneec->is_37xx)
		return 0;

	return i2c_smbus_write_word_data(eneec->client, PORT_XBI, 0xF0);
}

//
// Handle the read-reg ioctl call.
// sizeof(*arg) = 2.
// *arg=input regL-regH, also, =output byte.
static int ioctl_read_reg(struct eneec *eneec, unsigned long arg)
{
	unsigned short reg;
	unsigned char val;
	int retval;

	// Get register to read.
	val = 0;
	retval = __get_user(reg, (unsigned short __user *) arg);
	if (retval < 0)
		return retval;

	if (!eneec->is_37xx) {
		if (reg < 0xFEA0 || reg > 0xFEAD) { // only for XBI if non-37XX.
			pr_warn("ioctl_read_reg(reg), reg is not between 0xFEA0~0xFEAD for non-37XX\n");
			return -EINVAL;
		}
		reg &= 0x0F; // we don't need 0xFEA
	}

	retval = eneec_read_reg(eneec, reg, &val);
	if (retval < 0)
		return retval;

	return __put_user(val, (unsigned char __user *) arg);
}

//
// Handle the write-reg ioctl call.
// sizeof(*arg) = 4.
// *arg=input regL-regH-byte
static int ioctl_write_reg(struct eneec *eneec, unsigned long arg)
{
	unsigned long reg_data;
	unsigned short reg;
	unsigned char byte;
	int retval;

	// Get register and data to write.
	retval = __get_user(reg_data, (unsigned long __user *) arg);

	if (retval < 0) {
		pr_warn("get_user %d\n", retval);
		return retval;
	}

	// [15:0] is reg, [23:16] is data.
	reg = (unsigned short) reg_data;
	byte = (unsigned char) (reg_data >> 16);

	if (!eneec->is_37xx) {
		if (reg < 0xFEA0 || reg > 0xFEAD) { // only for XBI if non-37XX.
			pr_warn("ioctl_write_reg(reg), reg is not between 0xFEA0~0xFEAD for non-37XX\n");
			return -EINVAL;
		}
		reg &= 0x0F; // we don't need 0xFEA
	}

	retval = eneec_write_reg(eneec, reg, byte);

	return retval;
}

static long eneec_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct eneec *eneec = filp->private_data;
	int retval = 0;


	// Check type and command number
	if (_IOC_TYPE(cmd) != ENEEC_IOC_MAGIC) {
		pr_debug("Not ENEEC_IOC_MAGIC\n");
		return -ENOTTY;
	}

	switch (cmd)
	{
	case ENEEC_IOC_READ_REG:
		retval = ioctl_read_reg(eneec, arg);
		break;
	case ENEEC_IOC_WRITE_REG:
		retval = ioctl_write_reg(eneec, arg);
		break;
	case ENEEC_IOC_GET_CHIPID:
		retval = ioctl_get_chipid(eneec, arg);
		break;
	case ENEEC_IOC_ENTER_CODE_IN_RAM:
		retval = ioctl_enter_code_in_ram(eneec);
		break;
	case ENEEC_IOC_EXIT_CODE_IN_RAM:
		retval = ioctl_exit_code_in_ram(eneec);
		break;
	default:
		pr_warn("Unsupported ioctl\n");
		retval = -ENOTTY;
		break;
	}

	return retval;
}

static int eneec_open(struct inode *inode, struct file *filp)
{
	struct eneec *eneec = container_of(inode->i_cdev, struct eneec, cdev);

	pr_debug("eneec_open()\n");

	filp->private_data = eneec;

	return 0;
}

static int eneec_release(struct inode *inode, struct file *filp)
{
	pr_debug("eneec_release()\n");
	return 0;
}

static const struct file_operations eneec_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= eneec_ioctl,
	.open		= eneec_open,
	.release	= eneec_release,
};

//
// Undo eneec_create_cdev_node().
// Call this only if the char device node was ever created successfully.
void eneec_destroy_cdev_node(struct eneec *eneec)
{
	device_destroy(eneec->eneec_class, eneec->devt);
	cdev_del(&eneec->cdev);
	unregister_chrdev_region(eneec->devt, 1);
	class_destroy(eneec->eneec_class);
}

int eneec_create_cdev_node(struct eneec *eneec)
{
	int status;
	dev_t devt;
	struct device *dev;
	struct class *eneec_class;
	bool is_class_created = false, is_region_allocated = false, is_cdev_added = false, is_device_created = false;

	// Create class
	eneec_class = class_create(THIS_MODULE, "eneec");
	status = IS_ERR(eneec_class) ? PTR_ERR(eneec_class) : 0;
	if (status < 0) {
		pr_err("[eneec] class_create() failed -- %d\n", status);
		goto error_exit;
	}
	is_class_created = true;

	// Alloc chrdev region.
	status = alloc_chrdev_region(&devt, 0, 1, "eneec");
	if (status < 0) {
		pr_err("[eneec] alloc_chrdev_region() failed -- %d\n", status);
		goto error_exit;
	}
	is_region_allocated = true;

	// Add cdev.
	cdev_init(&eneec->cdev, &eneec_fops);
	status = cdev_add(&eneec->cdev, devt, 1);
	if (status < 0) {
		pr_err("[eneec] cdev_add() failed -- %d\n", status);
		goto error_exit;
	}
	is_cdev_added = true;

	// Create device
	dev = device_create(
		eneec_class,
		&eneec->client->dev,	// parent device (struct device *)
		devt,
		eneec,			// caller's context
		"eneec");

	status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	if (status < 0) {
		pr_err("[eneec] device_create() failed -- %d\n", status);
		goto error_exit;
	}
	is_device_created = true;

	// Succeed.
	eneec->eneec_class = eneec_class;
	eneec->devt = devt;

	return 0;

error_exit:

	if (is_device_created)
		device_destroy(eneec_class, devt);
	if (is_cdev_added)
		cdev_del(&eneec->cdev);
	if (is_region_allocated)
		unregister_chrdev_region(devt, 1);
	if (is_class_created)
		class_destroy(eneec_class);

	return status;
}

static void ennec_interrupt(u8 reg,u8 stat)
{
	switch (reg) {
		/*LID Sitch*/
		case 0x23 :
			pr_debug("LID reg = 0x%2x , stat = 0x%2x",reg,stat);
			break;
		case 0x24 :
			pr_debug("AC IN reg = 0x%2x , stat = 0x%2x",reg,stat);
			break;
		default	:
			return ;
	}
	return ;
}
///////////////////////////////////////////////////////////////////////////////////////////////////
// Codes for interfacing with kernel kbd, mouse.
#ifdef EC_BATTERY_SUPPORT
static void eneec_battery_work(struct work_struct *work)
{
	struct eneec_device_info *di = container_of(work,
	struct eneec_device_info, monitor_work.work);
	const int interval = HZ * 60;

	dev_dbg(di->dev, "%s\n", __func__);

//        eneec_battery_read_status(di);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

#define to_eneec_device_info(x) container_of((x), struct eneec_device_info, \
	                              bat);
#endif

static void eneec_read_work(struct work_struct *work)
{
#ifdef POLLING
	struct eneec *eneec = container_of(work, struct eneec, read_work.work);
#else
	struct eneec *eneec = container_of(work, struct eneec, read_work);
#endif
	struct i2c_client *client = eneec->client;
	s32 data;
	u8 port;
	int retry;

	data = i2c_smbus_read_word_data(client, 0);

#ifdef POLLING
	if (eneec->toggle != (data & 1)) // has toggle bit: valid data
		ENE_DEBUG("IN :       0x%04x\n", (u16) data);
	else
		goto queue_next_work;
#else
	// ENE_DEBUG("IN :       0x%04x\n", (u16) data);

	if (unlikely((eneec->toggle ^ (data & 1)) == 0)) {
		printk("WARNING: invalid toggle bit. Interrupts might be missed !\n");
	}
#endif

	eneec->toggle = data & 1;

	for (retry = 0; retry < 3; retry++) {
		// e.g., data=0xfa31, 0xfa=ack, 0x30=kbd, 1=toggle bit.
		port = DATA_TO_PORT(data);

		if (eneec->serio_over_i2c) {
			if (port == PORT_KBD) {
				serio_interrupt(eneec->kbd_port.serio,
						(u8) (data >> 8), 0);
				printk("Invalid data: 0x%08x\n", data);
				break;
			}

			if (port == PORT_MOU) {
				serio_interrupt(eneec->mou_port.serio,
						(u8) (data >> 8), 0);
				break;
			}
		}

		if (port == PORT_ECRAM) {
			ennec_interrupt((u8) (data >> 8),(u8) (data &0xff));
			break;
		} else {
			ENE_DEBUG("Invalid data: 0x%08x\n", data);
			data = i2c_smbus_read_word_data(client, 0);
			ENE_DEBUG("IN :       0x%04x (retry)\n", (u16) data);
		}
	}

#ifdef POLLING
queue_next_work:
	if (!eneec->stop_polling)
		queue_delayed_work(eneec->work_queue, &eneec->read_work, msecs_to_jiffies(POLL_INTERVAL));
#else
	if (eneec->serio_over_i2c)
		enable_irq(eneec->irq);
#endif
}

#ifndef POLLING
static irqreturn_t eneec_interrupt(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct eneec *eneec = i2c_get_clientdata(client);

	printk("eneec_interrupt\n");

	if (eneec->serio_over_i2c)
		disable_irq_nosync(irq);

	queue_work(eneec->work_queue, &eneec->read_work);
//	lpm_mfpr_edge_detect_clear_config(MFP_PIN_GPIO60);
	return IRQ_HANDLED;
}
#endif

static int eneec_write(struct serio *serio, unsigned char byte)
{
	int err;
	struct eneec_port *port = serio->port_data;
	struct eneec *eneec = port->eneec;

	//ENE_DEBUG("OUT: 0x%02x%02x\n", byte, port->port_no);

	err = i2c_smbus_write_word_data(eneec->client, port->port_no, byte);

	return err;
}

static int eneec_create_kbd_port(struct eneec *eneec)
{
	struct i2c_client *client = eneec->client;
	struct serio *serio;

	serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
	if (!serio)
		return -ENOMEM;

	serio->id.type      = SERIO_8042_XL;
	serio->write        = eneec_write;
	serio->port_data    = &eneec->kbd_port;
	serio->dev.parent   = &client->dev;
	strlcpy(serio->name, "eneec kbd port", sizeof(serio->name));
	strlcpy(serio->phys, "eneec/serio0", sizeof(serio->phys));

	eneec->kbd_port.serio = serio;
	eneec->kbd_port.port_no = PORT_KBD;
	eneec->kbd_port.eneec = eneec;

	return 0;
}

static int eneec_create_mouse_port(struct eneec *eneec)
{
	struct i2c_client *client = eneec->client;
	struct serio *serio;

	serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
	if (!serio)
		return -ENOMEM;

	serio->id.type      = SERIO_8042;
	serio->write        = eneec_write;
	serio->port_data    = &eneec->mou_port;
	serio->dev.parent   = &client->dev;
	strlcpy(serio->name, "eneec mouse port", sizeof(serio->name));
	strlcpy(serio->phys, "eneec/serio1", sizeof(serio->phys));

	eneec->mou_port.serio = serio;
	eneec->mou_port.port_no = PORT_MOU;
	eneec->mou_port.eneec = eneec;

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

	if (machine_is_titan()) {
		count += sprintf(buf + count,"Manufacturer Access: 0x%04X\n",eneec->Manufacture);
		count += sprintf(buf + count,"Battery Mode: 0x%04X\n",eneec->Battery_mode);
		count += sprintf(buf + count,"Temp : %d (C)\n",eneec->temp_tC);
		count += sprintf(buf + count,"Voltage(mV): %d\n",eneec->current_mV);
		count += sprintf(buf + count,"Current(mA): %d\n",eneec->current_mA);
		count += sprintf(buf + count,"Average Current(mA): %d\n",eneec->average_mA);
		count += sprintf(buf + count,"Max Error : %d\n",eneec->Max_Error);
		count += sprintf(buf + count,"RSOC(%%): %d %%\n",eneec->capacity_P);
		count += sprintf(buf + count,"ASOC(%%): %d %%\n",eneec->abs_capacity_P);
		count += sprintf(buf + count,"Remaining Capacity(mAh): %d\n",eneec->current_mAh);
		count += sprintf(buf + count,"Full Capacity(mAh): %d\n",eneec->absolute_mAh);
		count += sprintf(buf + count,"Charging Voltage(mV): %d\n",eneec->charge_mV);
		count += sprintf(buf + count,"charging Current(mA): %d\n",eneec->charge_mA);
		count += sprintf(buf + count,"Cell 1 voltage (mV): %d\n",eneec->Cell_1_volt);
		count += sprintf(buf + count,"Cell 2 voltage (mV): %d\n",eneec->Cell_2_volt);
		count += sprintf(buf + count,"Cell 3 voltage (mV): %d\n",eneec->Cell_3_volt);
		count += sprintf(buf + count,"Cell 4 voltage (mV): %d\n",eneec->Cell_4_volt);

		eneec->temp_tC          = ecram_val[TEMP_C];
		eneec->Manufacture      = ecram_val[MANUFACTURE_L]+ecram_val[MANUFACTURE_H]*256;
		eneec->Battery_mode     = ecram_val[BATTERY_MODE_L]+ecram_val[BATTERY_MODE_H]*256;
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

static ssize_t battery_RSOC(struct device *dev, struct device_attribute
	          *devattr, char *buf) {
	struct eneec *eneec = i2c_get_clientdata(ene_client);
	eneec_register_update(RSOC);
	eneec->capacity_P       = ecram_val[RSOC];
	return sprintf(buf, "%d %%\n", eneec->capacity_P);
}
static ssize_t battery_remain_cap(struct device *dev, struct device_attribute
	          *devattr, char *buf) {
	struct eneec *eneec = i2c_get_clientdata(ene_client);
	eneec_register_update(CAP_REMAIN_L);
	eneec_register_update(CAP_REMAIN_H);
	eneec->current_mAh      = ecram_val[CAP_REMAIN_L]+ecram_val[CAP_REMAIN_H]*256;
	return sprintf(buf, "%d mAh\n", eneec->current_mAh);
}
static ssize_t battery_full_cap(struct device *dev, struct device_attribute
	          *devattr, char *buf) {
	struct eneec *eneec = i2c_get_clientdata(ene_client);
	eneec_register_update(CAP_FULL_L);
	eneec_register_update(CAP_FULL_H);
	eneec->absolute_mAh     = ecram_val[CAP_FULL_L]+ecram_val[CAP_FULL_H]*256;
	return sprintf(buf, "%d mAh\n", eneec->absolute_mAh);
}
static ssize_t battery_design_cap(struct device *dev, struct device_attribute
	          *devattr, char *buf) {
	struct eneec *eneec = i2c_get_clientdata(ene_client);
	return sprintf(buf, "%d mAh\n", eneec->design_mAh);
}
static ssize_t ec_LID(struct device *dev, struct device_attribute
	          *devattr, char *buf) {
	struct eneec *eneec = i2c_get_clientdata(ene_client);
	eneec_register_update(LID_SWITCH);
	return sprintf(buf, "LID switch %s (%d)\n", (ecram_val[LID_SWITCH]!=0)? "ON":"OFF",ecram_val[LID_SWITCH]);
}
static ssize_t ec_ac_line(struct device *dev, struct device_attribute
	          *devattr, char *buf) {
	struct eneec *eneec = i2c_get_clientdata(ene_client);
	eneec_register_update(AC_IN);
	return sprintf(buf, "AC line status : %s AC IN (%d)\n", (ecram_val[AC_IN]!=0)? "":"NO",ecram_val[AC_IN]);
}

static ssize_t ec_version(struct device *dev, struct device_attribute
	          *devattr, char *buf) {
	int count=0;
	struct eneec *eneec = i2c_get_clientdata(ene_client);
	eneec_register_update(MB_TYPE);
	eneec_register_update(VERSION_1);
	eneec_register_update(VERSION_2);

	switch (ecram_val[MB_TYPE]){
	case 'M':
		count = sprintf(buf, "MIMAS Board ");
		break;
	case 'T':
		count = sprintf(buf, "TITAN+ Board ");
		break;
	case 'I':
		count = sprintf(buf, "TITAN12 Board ");
                break;
	case 'A':
                count = sprintf(buf, "ARIEL Board ");
                break;
	default:
		count = sprintf(buf, "UNKNOW Board ");
	}

	switch (ecram_val[VERSION_1]& 0x0f) {
	case 1:
		count += sprintf(buf + count, "EVT Build ");
		break;
	case 2:
	        count += sprintf(buf + count, "DVT Build ");
	        break;
	case 3:
	        count += sprintf(buf + count, "PVT Build ");
	        break;
	case 4:
	        count += sprintf(buf + count, "MP Build ");
	        break;
	}
	count += sprintf(buf + count, "Ver: %d.%d \n",((ecram_val[VERSION_2]& 0xf0)>>4),(ecram_val[VERSION_2]& 0x0f));

	return count;
}

BATTERY_DEVICE_ATTR(capacity_P, S_IRUGO, battery_RSOC, NULL,0);
BATTERY_DEVICE_ATTR(capacity_remain, S_IRUGO, battery_remain_cap, NULL,0);
BATTERY_DEVICE_ATTR(capacity_full, S_IRUGO, battery_full_cap, NULL,0);
BATTERY_DEVICE_ATTR(capacity_design, S_IRUGO, battery_design_cap, NULL,0);
BATTERY_DEVICE_ATTR(LID, S_IRUGO, ec_LID, NULL,0);
BATTERY_DEVICE_ATTR(ac_line, S_IRUGO, ec_ac_line, NULL,0);
//BATTERY_DEVICE_ATTR(ec_ram, S_IRWXUGO, read_ec_ram_value, set_ec_ram_value ,0);
BATTERY_DEVICE_ATTR(ec_ram, S_IRUGO | S_IWUSR, read_ec_ram_value, set_ec_ram_value ,0);
BATTERY_DEVICE_ATTR(ec_version, S_IRUGO, ec_version, NULL ,0);

static struct attribute *titan_ec_attributes[] = {
	&battery_dev_attr_capacity_P.dev_attr.attr,
	&battery_dev_attr_capacity_remain.dev_attr.attr,
	&battery_dev_attr_capacity_full.dev_attr.attr,
	&battery_dev_attr_capacity_design.dev_attr.attr,
	&battery_dev_attr_LID.dev_attr.attr,
	&battery_dev_attr_ac_line.dev_attr.attr,
	&battery_dev_attr_ec_ram.dev_attr.attr,
	&battery_dev_attr_ec_version.dev_attr.attr,
	NULL
};

static const struct attribute_group titan_ec_group = {
	.attrs = titan_ec_attributes,
};

static struct attribute *mimas_ec_attributes[] = {
	&battery_dev_attr_ec_ram.dev_attr.attr,
	&battery_dev_attr_ec_version.dev_attr.attr,
	NULL
};

static const struct attribute_group mimas_ec_group = {
	.attrs = mimas_ec_attributes,
};

static struct attribute *ariel_ec_attributes[] = {
        &battery_dev_attr_ec_ram.dev_attr.attr,
        &battery_dev_attr_ec_version.dev_attr.attr,
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

	if (machine_is_mimas()) {
		eneec->ram_reg = mimas_ram_reg;
		eneec->ram_reg_size = ARRAY_SIZE(mimas_ram_reg);
		eneec->serio_over_i2c = 1;
	}else if (machine_is_titan()){
		eneec->ram_reg = titan_ram_reg;
		eneec->ram_reg_size = ARRAY_SIZE(titan_ram_reg);
		eneec->serio_over_i2c = 0;
	}else if (machine_is_qseven()){
                eneec->ram_reg = ariel_ram_reg;
                eneec->ram_reg_size = ARRAY_SIZE(ariel_ram_reg);
                eneec->serio_over_i2c = 0;
	}

	eneec->chip_id = id->driver_data >> 8;
	eneec->rev_id = (unsigned char) id->driver_data;
	eneec->is_37xx = ((eneec->chip_id & 0xFF00) == 0x3700);
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

	if ((eneec->work_queue = create_singlethread_workqueue("eneec"))) {
#ifdef POLLING
		INIT_DELAYED_WORK(&eneec->read_work, eneec_read_work);
		queue_delayed_work(eneec->work_queue, &eneec->read_work,
					msecs_to_jiffies(POLL_INTERVAL));
#else
		INIT_WORK(&eneec->read_work, eneec_read_work);
#endif
	} else
		goto error_exit;

#ifndef POLLING
	if (eneec->serio_over_i2c) {
		unsigned long val = 0;
		err = request_irq(client->irq, eneec_interrupt,
					IRQF_TRIGGER_RISING, "eneec", client );
		if (err) {
			pr_err("Failed to request IRQ %d -- %d\n", client->irq, err);
			goto error_exit;
		}
		printk ("IRQ reuest finish\n");
//        	val = MFPR_EDGE_RISE_EN ;
//	        lpm_mfpr_edge_detect_config(MFP_PIN_GPIO60, val);
	}

	eneec->irq = client->irq;
#endif

	if (eneec->serio_over_i2c) {
		if ((err = eneec_create_kbd_port(eneec)))
			goto error_exit;

		serio_register_port(eneec->kbd_port.serio);

		/*
		if ((err = eneec_create_mouse_port(eneec)))
			goto error_exit;

		serio_register_port(eneec->mou_port.serio);
		*/
	}

	if ((err = eneec_create_cdev_node(eneec)))
		goto error_exit;

	/* create the sysfs file for ec_ram_write */
	if (machine_is_mimas())
		err = sysfs_create_group(&client->dev.kobj, &mimas_ec_group);

	if (machine_is_titan()) {
		eneec_probe_battery(eneec);
		err = sysfs_create_group(&client->dev.kobj, &titan_ec_group);
	}

        if (machine_is_qseven())
                err = sysfs_create_group(&client->dev.kobj, &ariel_ec_group);

	return err;
error_exit:
	if (eneec->serio_over_i2c) {
		if (eneec->mou_port.serio)
			serio_unregister_port(eneec->mou_port.serio);
		if (eneec->kbd_port.serio)
			serio_unregister_port(eneec->kbd_port.serio);
		if (eneec->irq)
			free_irq(eneec->irq, client);
	}

	if (eneec && eneec->work_queue) {
#ifdef POLLING
		eneec->stop_polling = true;
		cancel_delayed_work_sync(&eneec->read_work);
#endif
		destroy_workqueue(eneec->work_queue);
	}

	if (eneec)
		kfree(eneec);

	return err;
}

static int eneec_remove(struct i2c_client *client)
{
	struct eneec *eneec = i2c_get_clientdata(client);

	eneec_destroy_cdev_node(eneec);
	eneec_remove_battery(eneec);

	if (machine_is_mimas())
		sysfs_remove_group(&client->dev.kobj, &mimas_ec_group);
	if (machine_is_titan())
		sysfs_remove_group(&client->dev.kobj, &titan_ec_group);
        if (machine_is_qseven())
                sysfs_remove_group(&client->dev.kobj, &ariel_ec_group);

	if (eneec->serio_over_i2c) {
		if (eneec->mou_port.serio)
			serio_unregister_port(eneec->mou_port.serio);

		if (eneec->kbd_port.serio)
			serio_unregister_port(eneec->kbd_port.serio);

		if (eneec->irq)
			free_irq(eneec->irq, client);
	}

	if (eneec->work_queue) {
#ifdef POLLING
		eneec->stop_polling = true;
		cancel_delayed_work_sync(&eneec->read_work);
#endif
		destroy_workqueue(eneec->work_queue);
	}

	kfree(eneec);
	return 0;
}

#ifdef SUSPEND_RESUME
static int eneec_suspend(struct i2c_client *client)
{
	struct eneec *eneec = i2c_get_clientdata(client);
	
	if (enable_irq_wake(eneec->irq/*273*/) == 0) {
	unsigned long val = 0;
	val = MFPR_SLEEP_OE_N
				| MFPR_EDGE_FALL_EN | MFPR_SLEEP_SEL;
			lpm_mfpr_edge_detect_config(MFP_PIN_GPIO60, val);
	}
	return 0;
#if 0 
	struct eneec *eneec = i2c_get_clientdata(ene_client);
	struct eneec_device_info *di = eneec->battery_info;
	s32 data;
	int retry;

	data = i2c_smbus_read_word_data(client, 0);
	ENE_DEBUG("Drain data : 0x%04x\n", (u16) data);
	for (retry = 0; retry < 9; retry++) {
		eneec->toggle = i2c_smbus_read_word_data(client , 0);
		if (eneec->toggle == data) // no new data
			break;
		else {
			ENE_DEBUG("Drain data : 0x%04x\n", (u16) eneec->toggle);
			data = eneec->toggle;
		}
	}
	eneec->toggle = data & 1;

	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	return 0;
#endif
}

static int eneec_resume(struct i2c_client *client)
{
	//if (enable_irq_wake(ts->irq) == 0) {

	lpm_mfpr_edge_detect_clear_config(MFP_PIN_GPIO60);
	return 0;

}
#endif


#ifdef EC_BATTERY_SUPPORT
static void eneec_battery_external_power_changed(struct power_supply *psy)
{
	struct eneec_device_info *di = to_eneec_device_info(psy);

	dev_dbg(di->dev, "%s\n", __func__);

	cancel_delayed_work(&di->monitor_work);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ/10);
}

static int eneec_battery_get_property(struct power_supply *psy,
	                       enum power_supply_property psp,
	                       union power_supply_propval *val)
{
	struct eneec_device_info *di = to_eneec_device_info(psy);

	eneec_battery_read_status(di);

	/*
	 * All voltages, currents, charges, energies, time and temperatures in uV,
	 * µA, µAh, µWh, seconds and tenths of degree Celsius
	 */

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->charge_status;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		/* convert Kelvin to tenths of degree Celsius */
		val->intval = (di->temp_tK - 2730);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = (di->current_mV * 1000);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = (di->current_mA * 1000);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = (di->average_mA * 1000);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = di->capacity_P;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = (di->current_mAh * 1000);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = (di->absolute_mAh * 1000);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		val->intval = (di->current_empty_time_Min * 60);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		val->intval = (di->average_empty_time_Min * 60);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		val->intval = (di->average_full_time_Min * 60);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = (di->design_mAh * 1000);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property eneec_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static int eneec_probe_battery(struct eneec *eneec)
{
	int retval = 0;
	struct eneec_device_info *di;

	di = kzalloc(sizeof(struct eneec_device_info), GFP_KERNEL);
	if (!di) {
		retval = -ENOMEM;
		goto di_alloc_failed;
	}

	eneec->battery_info = di;

	di->dev                 = &eneec->client->dev;
	di->update_time         = jiffies;
	di->bat.name            = "eneec-battery";
	di->bat.type            = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties      = eneec_battery_props;
	di->bat.num_properties  = ARRAY_SIZE(eneec_battery_props);
	di->bat.get_property    = eneec_battery_get_property;
	di->bat.external_power_changed = eneec_battery_external_power_changed;

	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;

	retval = power_supply_register(&eneec->client->dev, &di->bat);
	if (retval) {
		dev_err(di->dev, "failed to register battery\n");
		goto batt_failed;
	}

	INIT_DELAYED_WORK(&di->monitor_work, eneec_battery_work);
	di->monitor_wqueue = create_singlethread_workqueue("eneec-battery");
	if (!di->monitor_wqueue) {
		retval = -ESRCH;
		goto workqueue_failed;
	}
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ * 1);

	battery_proc_init();	

	di_proc = di;

	pr_info("Eneec Power Supply driver loaded successfully. \n");
	return retval;

workqueue_failed:
	power_supply_unregister(&di->bat);
batt_failed:
	kfree(di);
di_alloc_failed:
success:
	return retval;
}

static int eneec_remove_battery(struct eneec *eneec)
{
	struct eneec_device_info *di = eneec->battery_info;

	cancel_rearming_delayed_workqueue(di->monitor_wqueue,
	                          &di->monitor_work);
	destroy_workqueue(di->monitor_wqueue);
	power_supply_unregister(&di->bat);

	return 0;
}
#endif /*EC_BATTERY_SUPPORT */

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
#ifdef SUSPEND_RESUME
	.suspend	= eneec_suspend,
	.resume		= eneec_resume,
#endif
};

module_i2c_driver(eneec_driver);
