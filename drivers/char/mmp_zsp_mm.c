/*
 *
 * Marvell MMP ZSP buffer driver.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
#include <linux/syscalls.h>
#include <linux/ipc.h>
#include <linux/ipc.h>
#include <linux/ioctl.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <mach/dma.h>
#include <mach/irqs.h>

enum
{
	ZSP_MM_DEV_IOC_MAGIC		= 100,
	ZSP_MM_IOC_GET_PHYADDR		= _IOW(ZSP_MM_DEV_IOC_MAGIC, 1, int ),
};
#define ZMQ_IOCTL_ENTER(param, ret)					\
	ret = copy_from_user(&(param), (void*)arg, sizeof(param));	\
	if (ret != 0) {							\
		printk(KERN_ERR "%s: %d: read %s failed, len %d/%d\n",	\
			__func__,__LINE__,#param, ret, sizeof(param));	\
		return -EFAULT;						\
	}
#define ZMQ_IOCTL_LEAVE(param, ret)					\
	ret = copy_to_user((void*)arg, &(param), sizeof(param));	\
	if (ret != 0) {							\
		printk(KERN_ERR "%s: %d: read %s failed, len %d/%d\n",	\
			__func__,__LINE__,#param, ret, sizeof(param));	\
		return -EFAULT;						\
	}


dma_addr_t p_zsp_buff_p;
void *p_zsp_buff_v;
#define ZSP_MM_SIZE	(512*1024)

static int pxa688_zsp_mm_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	vma->vm_flags |= VM_RESERVED;	/* Don't swap */
	vma->vm_flags |= VM_DONTEXPAND; /* Don't remap */
	vma->vm_flags |= VM_DONTCOPY;	/* Don't fork */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	ret =  remap_pfn_range(vma, vma->vm_start, (p_zsp_buff_p >> PAGE_SHIFT), (vma->vm_end - vma->vm_start),
		vma->vm_page_prot);
	return ret;
}

static long pxa688_zsp_mm_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int ret;
	unsigned long param;

	ret = 0;
	switch (cmd) {
	case ZSP_MM_IOC_GET_PHYADDR:
		ZMQ_IOCTL_ENTER(param, ret);
		param =(unsigned long)p_zsp_buff_p;
		ZMQ_IOCTL_LEAVE(param, ret);

		break;
	default:
		break;
	}
	return ret;
}



static struct file_operations pxa688_zsp_mm_fops = {
	.mmap			= pxa688_zsp_mm_mmap,
	.unlocked_ioctl		= pxa688_zsp_mm_ioctl,
	.owner			= THIS_MODULE
};


static struct miscdevice pxa688_zsp_mm_miscdev = {
	.minor			= MISC_DYNAMIC_MINOR,
	.name			= "zsp_mm",
	.fops			= &pxa688_zsp_mm_fops
};

static int __init pxa688_zsp_mm_init(void)
{
	int ret;
	ret = misc_register(&pxa688_zsp_mm_miscdev);

	pxa688_zsp_mm_miscdev.this_device->coherent_dma_mask = DMA_BIT_MASK(32);
	if (ret) {
		return ret;
	}
	p_zsp_buff_v = dmam_alloc_coherent(pxa688_zsp_mm_miscdev.this_device,
			ZSP_MM_SIZE, &p_zsp_buff_p, GFP_KERNEL);

	return ret;
}
static void __exit pxa688_zsp_mm_exit(void)
{
	dma_free_coherent(pxa688_zsp_mm_miscdev.this_device, ZSP_MM_SIZE, p_zsp_buff_v, p_zsp_buff_p);
	misc_deregister(&pxa688_zsp_mm_miscdev);

}

late_initcall(pxa688_zsp_mm_init);
module_exit(pxa688_zsp_mm_exit);
