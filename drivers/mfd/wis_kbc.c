#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <asm/uaccess.h>
#include <mach/wistron.h>
#include <mach/camera.h>
#include "wis_kbc.h"

static unsigned long kbc_firmware_status = KBC_FIRMWARE_UPDATE_NORMAL;
extern int mk2_bsp_interface_set_flash(enum FLASH_TYPE state);

struct wiskbc_info {
	
	struct i2c_client *i2c;
	struct device *dev;

	int device_open;
	struct mutex work_lock;
	struct mutex io_lock;
}*info;

/* I2C operations */
s32 wiskbc_reg_read(u8 reg)
{
	return i2c_smbus_read_byte_data(info->i2c, reg);
}
EXPORT_SYMBOL(wiskbc_reg_read);

s32 wiskbc_reg_write(u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(info->i2c, reg, data);
}
EXPORT_SYMBOL(wiskbc_reg_write);

int wiskbc_firmware_updating(void)
{
	return 	mutex_is_locked(&info->work_lock);
}
EXPORT_SYMBOL(wiskbc_firmware_updating);

static ssize_t kbc_version_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	s32 major, minor;

	if(off > 0){
		*eof = 1;
		return 0;
	}

	major = wiskbc_reg_read(0x06);
	minor = wiskbc_reg_read(0x07);	//test version

	if(minor == 0)
		return sprintf(page, "0x%x\n", major) + 1;
	else
		return sprintf(page, "0x%x%x\n", major, minor) + 1;

}

static void create_kbc_version_proc_file(void)
{
	struct proc_dir_entry *proc_file =
		create_proc_entry(PROJ_PROC_KBC_VER, 0444, NULL);

	if (proc_file)
		proc_file->read_proc = kbc_version_read_proc;
	else
		printk(KERN_ERR "proc file create failed!\n");
}

#define ENTER_FLASH_MODE_RETRY_CNT 5
static void update_usercode(const struct firmware *firmware, void *context)
{
	unsigned char pPage[EPF021J_PAGE_SIZE + 1] = {0};
	unsigned char pBlock[EPF021J_BLOCK_SIZE];
	unsigned char blockCount, page, pageCount;
	s32 i, block, value;
	int ret = -EINVAL;

	kbc_firmware_status = KBC_FIRMWARE_UPDATE_NORMAL;

	if(!firmware){
		kbc_firmware_status = KBC_FIRMWARE_UPDATE_USERCODE_NOT_FOUND;
		return;
	}

	mutex_lock(&info->work_lock);
	msleep(100);

	printk(KERN_INFO "<%d><%s> Check firmware size\n", __LINE__, __func__);
	//check size first(KBC announced fixed sized always)
	if(firmware->size != SIZE_USERCODE){ 
		kbc_firmware_status = KBC_FIRMWARE_UPDATE_USERCODE_NOT_FOUND;
		goto exit_user;
	}

	printk(KERN_INFO "<%d><%s> Check firmware signature\n", __LINE__, __func__);
	//check signature
	ret = memcmp((firmware->data + SIZE_USERCODE - 32), Signature, sizeof(Signature));
	if(!ret){
		kbc_firmware_status = KBC_FIRMWARE_UPDATE_USERCODE_NOT_FOUND;
		goto exit_user;
	}

#if 1
	printk(KERN_INFO "<%d><%s> Start update mode stage 1\n", __LINE__, __func__);
	for(i=0; i< ENTER_FLASH_MODE_RETRY_CNT; i++){
		//check_version
		//set kbc as flash-update mode
		wiskbc_reg_write(KBC_ENTER_FLASH, 0x1); 
		msleep(50);
	
		value = wiskbc_reg_read(KBC_READ_LOCATION); 
		if(value == 0xBB){
			kbc_firmware_status = KBC_FIRMWARE_UPDATE_USERCODE_FAIL;
			break;
		}
	}

	printk(KERN_INFO "<%d><%s> Start update mode stage 2\n", __LINE__, __func__);
	value = wiskbc_reg_read(KBC_READ_LOCATION); 
	if(value != 0xBB){
		printk(KERN_ERR "==>0x%x %d %s %s\n", value, __LINE__, __func__, __FILE__);
		kbc_firmware_status = KBC_FIRMWARE_UPDATE_USERCODE_FAIL;
		goto exit_user_kbc_fail;
	}
#endif

	msleep(10);

	blockCount = SIZE_USERCODE / EPF021J_BLOCK_SIZE + ((BUFFER_SIZE % EPF021J_BLOCK_SIZE == 0) ? 0 : 1);
	pageCount = EPF021J_BLOCK_SIZE / EPF021J_PAGE_SIZE;

	for (block = 0; block < blockCount; block++)
	{
		printk(KERN_INFO"  --> Writing blocks %02d \n", block);

		// Read firmware to buffer
		memcpy(pBlock, (firmware->data + (block * EPF021J_BLOCK_SIZE)), EPF021J_BLOCK_SIZE);

		// Set block number
		ret = wiskbc_reg_write(KBC_SET_BLK_NUM, block);

		// Write block to flash
		for (page = 0; page < pageCount; page++) 
		{
			pPage[0] = 0x10;		// Lead byte of each page
			memcpy(pPage + 1, pBlock + (page * EPF021J_PAGE_SIZE), EPF021J_PAGE_SIZE);

			i2c_smbus_write_i2c_block_data(info->i2c, KBC_PAGE_PROGRAM, sizeof(pPage), pPage);
		}

		msleep(55);

#if 1		//double check. request by Boyce
		//after finished write one block, read out and compare.
		for (page = 0; page < pageCount; page++)
		{ 
			i2c_smbus_read_i2c_block_data(info->i2c, KBC_READ_PAGE, sizeof(pPage), pPage);
		
			ret = memcmp((pBlock + (EPF021J_PAGE_SIZE * page)), pPage+1, sizeof(pPage)-1);
                        if(ret){
                                //add by Ares for bad block rewrite.+++++++
                                memcpy(pPage + 1, pBlock + (page * EPF021J_PAGE_SIZE), EPF021J_PAGE_SIZE);
                                i2c_smbus_write_i2c_block_data(info->i2c, KBC_PAGE_PROGRAM, sizeof(pPage), pPage);
                                printk(".........rewrite\n");
                                msleep(55);

                                i2c_smbus_read_i2c_block_data(info->i2c, KBC_READ_PAGE, sizeof(pPage), pPage);
                                ret = memcmp((pBlock + (EPF021J_PAGE_SIZE * page)), pPage+1, sizeof(pPage)-1);
                                if(ret){
                        	        //add by Ares for bad block rewrite.-------
                                	printk(KERN_ERR "==>%d %s %s\n", __LINE__, __func__, __FILE__);
					kbc_firmware_status = KBC_FIRMWARE_UPDATE_USERCODE_FAIL;
					goto exit_user_kbc_fail;
                                }
                        }
		}
#endif
	}

	printk(KERN_INFO "<%d><%s> Start checksum/xor ... \n", __LINE__, __func__);
	value = wiskbc_reg_read(KBC_CALC_CHKSUM);
	msleep(500);

	value = (unsigned char)wiskbc_reg_read(KBC_READ_CHK_SUM);
	if(value != firmware->data[SIZE_USERCODE - sizeof(Signature) - 11]){
		printk(KERN_ERR "==><0x%x><0x%x> %d %s %s\n", value, firmware->data[SIZE_USERCODE - sizeof(Signature) - 11], __LINE__, __func__, __FILE__);
		kbc_firmware_status = KBC_FIRMWARE_UPDATE_USERCODE_FAIL;
		goto exit_user_kbc_fail;
	}
	printk(KERN_NOTICE "  -->WisKBC firmware update: CheckSum PASS.\n");

	value = wiskbc_reg_read(KBC_READ_CHK_XOR);
	if(value != firmware->data[SIZE_USERCODE - sizeof(Signature) - 10]){
		printk(KERN_ERR "==> %d %s %s\n", __LINE__, __func__, __FILE__);
		kbc_firmware_status = KBC_FIRMWARE_UPDATE_USERCODE_FAIL;
		goto exit_user_kbc_fail;
	}
	printk(KERN_NOTICE "  -->WisKBC firmware update: CheckXor PASS.\n");
	kbc_firmware_status = KBC_FIRMWARE_UPDATE_USERCODE_OK;

exit_user_kbc_fail:
	wiskbc_reg_write(KBC_ENTER_USR_MODE, 0x0);
	mutex_unlock(&info->work_lock);
	release_firmware(firmware);
	return;
exit_user:
	mutex_unlock(&info->work_lock);
	release_firmware(firmware);
}

static void update_bootblock(const struct firmware *firmware, void *context)
{
	unsigned char pPage[EPF021J_PAGE_SIZE + 1] = {0};
	unsigned char pBlock[EPF021J_BLOCK_SIZE];
	unsigned char blockCount, page, pageCount;
	s32 i, block, value;
	int ret = -EINVAL;

	kbc_firmware_status = KBC_FIRMWARE_UPDATE_NORMAL;

	if(!firmware){
		kbc_firmware_status = KBC_FIRMWARE_UPDATE_BOOTBLOCK_NOT_FOUND;
		return;
	}

	mutex_lock(&info->work_lock);
	msleep(100);

	printk(KERN_INFO "<%d><%s> Check firmware size\n", __LINE__, __func__);
	//check size first(KBC announced fixed sized always)
	if(firmware->size != SIZE_BOOTBLOCK){ 
		kbc_firmware_status = KBC_FIRMWARE_UPDATE_BOOTBLOCK_NOT_FOUND;
		goto exit_boot_block;
	}

	printk(KERN_INFO "<%d><%s> Check firmware signature\n", __LINE__, __func__);
	//check signature
	ret = memcmp((firmware->data + SIZE_BOOTBLOCK - 32), Signature, sizeof(Signature));
	if(!ret){
		kbc_firmware_status = KBC_FIRMWARE_UPDATE_BOOTBLOCK_NOT_FOUND;
		goto exit_boot_block;
	}

#if 1
	printk(KERN_INFO "<%d><%s> Start update mode stage 1\n", __LINE__, __func__);
	for(i=0; i< ENTER_FLASH_MODE_RETRY_CNT; i++){
		//check_version
		//set kbc as flash-update mode
		wiskbc_reg_write(KBC_ENTER_FLASH, 0x2); 
		msleep(50);

		value = wiskbc_reg_read(KBC_READ_LOCATION); 
		if(value == 0xEE){
			kbc_firmware_status = KBC_FIRMWARE_UPDATE_BOOTBLOCK_FAIL;
			break;
		}
	}

	printk(KERN_INFO "<%d><%s> Start update mode stage 2\n", __LINE__, __func__);
	value = wiskbc_reg_read(KBC_READ_LOCATION); 
	if(value != 0xEE){
		printk(KERN_ERR "==>0x%x %d %s %s\n", value, __LINE__, __func__, __FILE__);
		kbc_firmware_status = KBC_FIRMWARE_UPDATE_BOOTBLOCK_FAIL;
		goto exit_boot_kbc_fail;
	}
#endif

	msleep(10);

	blockCount = SIZE_BOOTBLOCK / EPF021J_BLOCK_SIZE + ((BUFFER_SIZE % EPF021J_BLOCK_SIZE == 0) ? 0 : 1);
	pageCount = EPF021J_BLOCK_SIZE / EPF021J_PAGE_SIZE;

	for (block = 0; block < blockCount; block++)
	{
		printk(KERN_INFO"  --> Writing blocks %02d \n", block);

		// Read firmware to buffer
		memcpy(pBlock, (firmware->data + (block * EPF021J_BLOCK_SIZE)), EPF021J_BLOCK_SIZE);

		// Set block number
		ret = wiskbc_reg_write(KBC_SET_BLK_NUM, block);

		// Write block to flash
		for (page = 0; page < pageCount; page++) 
		{
			pPage[0] = 0x10;		// Lead byte of each page
			memcpy(pPage + 1, pBlock + (page * EPF021J_PAGE_SIZE), EPF021J_PAGE_SIZE);

			i2c_smbus_write_i2c_block_data(info->i2c, KBC_PAGE_PROGRAM, sizeof(pPage), pPage);
		}

		msleep(55);

#if 1		//double check. request by Boyce
		//after finished write one block, read out and compare.
		for (page = 0; page < pageCount; page++)
		{ 
			i2c_smbus_read_i2c_block_data(info->i2c, KBC_READ_PAGE, sizeof(pPage), pPage);
		
			ret = memcmp((pBlock + (EPF021J_PAGE_SIZE * page)), pPage+1, sizeof(pPage)-1);
                        if(ret){
                                //add by Ares for bad block rewrite.+++++++
                                memcpy(pPage + 1, pBlock + (page * EPF021J_PAGE_SIZE), EPF021J_PAGE_SIZE);
                                i2c_smbus_write_i2c_block_data(info->i2c, KBC_PAGE_PROGRAM, sizeof(pPage), pPage);
                                printk(".........rewrite\n");
                                msleep(55);

                                i2c_smbus_read_i2c_block_data(info->i2c, KBC_READ_PAGE, sizeof(pPage), pPage);
                                ret = memcmp((pBlock + (EPF021J_PAGE_SIZE * page)), pPage+1, sizeof(pPage)-1);
                                if(ret){
                        	        //add by Ares for bad block rewrite.-------
                                	printk(KERN_ERR "==>%d %s %s\n", __LINE__, __func__, __FILE__);
					kbc_firmware_status = KBC_FIRMWARE_UPDATE_BOOTBLOCK_FAIL;
					goto exit_boot_kbc_fail;
                                }
                        }
		}
#endif
	}

	printk(KERN_INFO "<%d><%s> Start checksum/xor ... \n", __LINE__, __func__);
	value = wiskbc_reg_read(KBC_CALC_CHKSUM);
	msleep(500);

	value = (unsigned char)wiskbc_reg_read(KBC_READ_CHK_SUM);
	if(value != firmware->data[SIZE_BOOTBLOCK - sizeof(Signature) - 11]){
		printk(KERN_ERR "==><0x%x><0x%x> %d %s %s\n", value, firmware->data[SIZE_BOOTBLOCK - sizeof(Signature) - 11], __LINE__, __func__, __FILE__);
		kbc_firmware_status = KBC_FIRMWARE_UPDATE_BOOTBLOCK_FAIL;
		goto exit_boot_kbc_fail;
	}
	printk(KERN_NOTICE "  -->WisKBC firmware update: CheckSum PASS.\n");

	value = wiskbc_reg_read(KBC_READ_CHK_XOR);
	if(value != firmware->data[SIZE_BOOTBLOCK - sizeof(Signature) - 10]){
		printk(KERN_ERR "==> %d %s %s\n", __LINE__, __func__, __FILE__);
		kbc_firmware_status = KBC_FIRMWARE_UPDATE_BOOTBLOCK_FAIL;
		goto exit_boot_kbc_fail;
	}
	printk(KERN_NOTICE "  -->WisKBC firmware update: CheckXor PASS.\n");
	kbc_firmware_status = KBC_FIRMWARE_UPDATE_BOOTBLOCK_OK;

exit_boot_kbc_fail:
	wiskbc_reg_write(KBC_ENTER_USR_MODE, 0x0);
	mutex_unlock(&info->work_lock);
	release_firmware(firmware);
	return;
exit_boot_block:
	mutex_unlock(&info->work_lock);
	release_firmware(firmware);
}

static ssize_t kbc_firmware_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[9];
	int ret;

	if(len > sizeof(messages)){
		if (copy_from_user(messages, buff, sizeof(messages)))
			return -EFAULT;
	}
	else{
		if (copy_from_user(messages, buff, len))
			return -EFAULT;
	}
			
	if (strncmp(messages, "usercode", strlen("usercode")) == 0){ 
			ret = request_firmware_nowait(
				THIS_MODULE, FW_ACTION_HOTPLUG, "FLASH.bin", info->dev, GFP_KERNEL, "user code", update_usercode); 
			if(!ret)
				printk(KERN_ERR "===> %d %s %s\n", __LINE__, __func__, __FILE__);
	}
	else if (strncmp(messages, "bootblock", strlen("bootblock")) == 0){
			ret = request_firmware_nowait(
				THIS_MODULE, FW_ACTION_HOTPLUG, "WISBB.bin", info->dev, GFP_KERNEL, "boot block", update_bootblock); 
			if(!ret)
				printk(KERN_ERR "===> %d %s %s\n", __LINE__, __func__, __FILE__);
	}
	else
		printk("usage: {usercode/bootblock}\n");

	return len;
}

static ssize_t kbc_firmware_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	if(off > 0){
		*eof = 1;
		return 0;
	}

	return sprintf(page, "0x%x\n", (unsigned int)kbc_firmware_status);
}

static void create_kbc_firmware_proc_file(void)
{
	struct proc_dir_entry *proc_file =
		create_proc_entry(PROJ_PROC_KBC_FIRMWARE, 0644, NULL);

	if (proc_file){
		proc_file->write_proc = (write_proc_t *)kbc_firmware_write_proc;
		proc_file->read_proc = kbc_firmware_read_proc;
	}
	else
		printk(KERN_ERR "proc file create failed!\n");
}

static ssize_t pcb_version_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	if(off > 0){
		*eof = 1;
		return 0;
	}

	return sprintf(page, "0x%x\n", wiskbc_reg_read(0x08)) + 1;
}

static void create_pcb_version_proc_file(void)
{
	struct proc_dir_entry *proc_file =
		create_proc_entry(PROJ_PROC_PCB_VER, 0444, NULL);

	if (proc_file)
		proc_file->read_proc = pcb_version_read_proc;
	else
		printk(KERN_ERR "proc file create failed!\n");
}
static ssize_t disable_charging_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	s32 value;

	if (strncmp(buff, "0", 1) == 0){
		value = (u32)wiskbc_reg_read(0x40);
		value &= ~(1<<3);
		wiskbc_reg_write(0x40, value);
	}
	else if (strncmp(buff, "1", 1) == 0)
		wiskbc_reg_write(0x40, 1<<3);
	else
		printk("usage: {0/1}\n");

	return len+1;
}

static void create_disable_charging_proc_file(void)
{
	struct proc_dir_entry *proc_file =
		create_proc_entry(PROJ_PROC_CHARGING_STOP, 0644, NULL);

	if (proc_file)
		proc_file->write_proc = (write_proc_t *)disable_charging_write_proc;
	else
		printk(KERN_INFO "proc file create failed!\n");
}

static ssize_t led_blink_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[2];
	size_t len2;

	s32 time;

	if (len > 2)
		len2 = strlen(messages);
	else
		len2 = len;

	if (copy_from_user(messages, buff, len2))
		return -EFAULT;

	time = simple_strtol(messages, NULL, 0);
	wiskbc_reg_write(0x42, time);
	wiskbc_reg_write(0x40, 0x1<<1);

	return len;
}

static void create_led_blink_proc_file(void)
{
	struct proc_dir_entry *proc_file =
		create_proc_entry(PROJ_PROC_LEDON, 0644, NULL);

	if (proc_file){
		proc_file->write_proc = (write_proc_t *)led_blink_write_proc;
	}
	else
		printk(KERN_ERR "proc file create failed!\n");
}

static ssize_t ms_read_proc(char *page, char **start, off_t off,
                int count, int *eof, void *data)
{
	printk(KERN_ERR "ms read\n");
        return sprintf(page, "%x\n", wiskbc_reg_read(0x43)) + 1;
}

static ssize_t ms_write_proc(struct file *filp,
                const char *buff, size_t len, loff_t *off)
{
	printk(KERN_ERR "ms write\n");
        if (strncmp(buff, "0", 1) == 0)
                wiskbc_reg_write(0x43, 1);
        else if (strncmp(buff, "1", 1) == 0)
                wiskbc_reg_write(0x43, 0);
        else
                printk("usage: {0/1}\n");

        return len+1;
}

static void create_ms_proc_file(void)  //magnetic switch
{
        struct proc_dir_entry *proc_file =
                create_proc_entry(PROJ_PROC_MS, 0600, NULL);

        if (proc_file){
                proc_file->read_proc = ms_read_proc;
                proc_file->write_proc = (write_proc_t *)ms_write_proc;
        }
        else
                printk(KERN_ERR "proc file create failed!\n");
}


static int __devinit wiskbc_probe(struct i2c_client *client,
					     const struct i2c_device_id *id)
{
	u32 value;

	info = kzalloc(sizeof(struct wiskbc_info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "No enough memory\n");
		return -ENOMEM;
	}

	info->i2c = client;
	info->dev = &client->dev;
	i2c_set_clientdata(client, info);
	create_kbc_version_proc_file();
	create_pcb_version_proc_file();
	create_disable_charging_proc_file();
	create_led_blink_proc_file();
	create_kbc_firmware_proc_file();
        create_ms_proc_file();
  
	info->device_open = 0;
	mutex_init(&info->work_lock);
	mutex_init(&info->io_lock);

	mk2_bsp_interface_set_flash(FLASH_TYPE_OFF);

	value = (u32)wiskbc_reg_read(0x40);
	value |= 1<<4;
	wiskbc_reg_write(0x40, value);

	return 0;
}

static int __devexit wiskbc_remove(struct i2c_client *client)
{
	struct wiskbc_info *info = i2c_get_clientdata(client);

	if (info) 
		kfree(info);
	
	i2c_set_clientdata(client, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int wiskbc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	u32 value;

	value = (u32)wiskbc_reg_read(0x40);
	value &= ~(1<<4);
	wiskbc_reg_write(0x40, value);

	return 0;
}

static int wiskbc_resume(struct i2c_client *client)
{
	u32 value;

	value = (u32)wiskbc_reg_read(0x40);
	value |= 1<<4;
	wiskbc_reg_write(0x40, value);

	return 0;
}

#else
#define wiskbc_suspend NULL
#define wiskbc_resume NULL
#endif /* CONFIG_PM */

static void wiskbc_shutdown(struct i2c_client *client)
{
//Only keep the command in mk2.c for synchronize TWSI1 with PMIC
#if 0
 	u32 value;
 	value = (u32)wiskbc_reg_read(0x40);
 	value &= ~(1<<4);
 	wiskbc_reg_write(0x40, value);
#endif
}

static const struct i2c_device_id wiskbc_id[] = {
	{ "wis-kbc", -1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wiskbc_id);

static struct i2c_driver wiskbc_driver = {
	.probe		= wiskbc_probe,
	.remove		= __devexit_p(wiskbc_remove),
	.suspend        = wiskbc_suspend,
	.resume         = wiskbc_resume,
	.shutdown	= wiskbc_shutdown,
	.driver		= {
		.name	= "wis kbc",
	},
	.id_table	= wiskbc_id,
};

static int __init wiskbc_init(void)
{
	return i2c_add_driver(&wiskbc_driver);
}
module_init(wiskbc_init);

static void __exit wiskbc_exit(void)
{
	i2c_del_driver(&wiskbc_driver);
}
module_exit(wiskbc_exit);

/* Module information */
MODULE_DESCRIPTION("Wistron KBC driver");
MODULE_AUTHOR("Wistron");
MODULE_LICENSE("GPL");

