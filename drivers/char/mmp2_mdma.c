/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2010 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/mmp2_mdma.h>

#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/mmp_dma.h>
#include <mach/pxa910-squ.h>

#include <linux/wakelock.h>

#define MAX_DESC_NUM		0x1000
#define SINGLE_DESC_TRANS_MAX  	0xff80

#define MMP2_FILL_DATA(base)		__DMA_REG(base, 0xa8)

static void *dma_desc_array_ch1 = NULL;
static void *dma_desc_array_ch2 = NULL;
static dma_addr_t dma_desc_array_phys_ch1;
static dma_addr_t dma_desc_array_phys_ch2;
static struct completion complete_ch1;
static struct completion complete_ch2;

static unsigned long uva_to_pa(struct mm_struct *mm, unsigned long addr)
{
	unsigned long ret = 0UL;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	pgd = pgd_offset(mm, addr);
	if (!pgd_none(*pgd)) {
		pud = pud_offset(pgd, addr);
		if (!pud_none(*pud)) {
			pmd = pmd_offset(pud, addr);
			if (!pmd_none(*pmd)) {
				pte = pte_offset_map(pmd, addr);
				if (!pte_none(*pte) && pte_present(*pte)) {
					struct page *page = pte_page(*pte);
					if(page) {
						ret = page_to_phys(page);
						ret |= (addr & (PAGE_SIZE-1));
					}
				}
			}
		}
	}
	return ret;
}

static void mdma_irq(int channel, void *data)
{
	unsigned int base_register = mmp_get_dma_reg_base(channel);

	if(base_register){
		if (TDISR(base_register) & 0x1) {
			if(channel == MDMA1_CH0)
				complete(&complete_ch1);
			else
				complete(&complete_ch2);
			if(TDISR(base_register) & 0xc)
				printk(KERN_ERR "MDMA error 0x%x on channel %d \n",
					TDISR(base_register), channel);
		} else {
			printk(KERN_ERR "MDMA error on channel %d \n", channel);
		}
		TDISR(base_register) = 0;
	}

	return;
}

/*
 * len requires at least 8bytes alignment. 128bytes alignment will get better performance.
 * Alignment: 8, 16, 32, 64, 128. The bigger, the better.
 * source and destination address aslo need alignment with burst size.
 */
unsigned long mdma_pmemcpy(unsigned long pdst, unsigned long psrc, unsigned int len)
{
	int ret, dma_ch;
	unsigned int base_register;
	unsigned int dcmd, mdma_dcr;
	unsigned long len_tmp, len_total;
	struct pxa910_squ_desc *dma_desc_tmp;
	dma_addr_t dma_desc_p_tmp;
	unsigned long srcphyaddr = psrc;
	unsigned long dstphyaddr = pdst;

	if (srcphyaddr == 0 || dstphyaddr == 0)
		return -1;

	if (len & 0x7) {
		printk(KERN_ERR "size is not 8 bytes aligned\n");
		return -1;
	}

	if (srcphyaddr & 0x7) {
		printk(KERN_ERR "srcphyaddr is not 8 bytes aligned\n");
		return -1;
	}

	if (dstphyaddr & 0x7) {
		printk(KERN_ERR "dstphyaddr is not 8 bytes aligned\n");
		return -1;
	}

	ret = mmp_request_dma("mdma_memcpy", MDMA1_CH0,
					mdma_irq, NULL);
	if (ret < 0){
		ret = mmp_request_dma("mdma_memcpy", MDMA1_CH1,
						mdma_irq, NULL);
		if (ret < 0){
			printk(KERN_ERR	"Can't request MDMA for memcpy\n");
			return -ENODEV;
		}else{
			dma_ch = MDMA1_CH1;
		}
	}else{
		dma_ch = MDMA1_CH0;
	}

	if(dma_ch == MDMA1_CH0){
		dma_desc_tmp = dma_desc_array_ch1;
		dma_desc_p_tmp = dma_desc_array_phys_ch1;
	}else{
		dma_desc_tmp = dma_desc_array_ch2;
		dma_desc_p_tmp = dma_desc_array_phys_ch2;
	}

	len_total = len;
	while(len_total) {
		len_tmp = len_total>SINGLE_DESC_TRANS_MAX ?
				SINGLE_DESC_TRANS_MAX : len_total;
		dma_desc_tmp->nxt_desc = dma_desc_p_tmp + sizeof(struct pxa910_squ_desc);
		dma_desc_tmp->src_addr = srcphyaddr;
		dma_desc_tmp->dst_addr = dstphyaddr;
		dma_desc_tmp->byte_cnt = len_tmp;
		if (len_total <= SINGLE_DESC_TRANS_MAX) {
			dma_desc_tmp->nxt_desc = 0;
			break;
		}
		len_total -= len_tmp;
		dma_desc_tmp ++;
		dma_desc_p_tmp += sizeof(struct pxa910_squ_desc);
		srcphyaddr += len_tmp;
		dstphyaddr += len_tmp;
	}

	if (!((len & 0x7f) || (srcphyaddr & 0x7f) || (dstphyaddr & 0x7f)))
		dcmd = MDCR_DST_ADDR_INC | MDCR_SRC_ADDR_INC | MDCR_TRANSMOD |
			MDCR_DMA_BURST_128B | SDCR_FETCHND;
	else if (!((len & 0x3f) || (srcphyaddr & 0x3f) || (dstphyaddr & 0x3f)))
		dcmd = MDCR_DST_ADDR_INC | MDCR_SRC_ADDR_INC | MDCR_TRANSMOD |
			MDCR_DMA_BURST_64B | SDCR_FETCHND;
	else if (!((len & 0x1f) || (srcphyaddr & 0x1f) || (dstphyaddr & 0x1f)))
		dcmd = MDCR_DST_ADDR_INC | MDCR_SRC_ADDR_INC | MDCR_TRANSMOD |
			MDCR_DMA_BURST_32B | SDCR_FETCHND;
	else if (!((len & 0xf) || (srcphyaddr & 0xf) || (dstphyaddr & 0xf)))
		dcmd = MDCR_DST_ADDR_INC | MDCR_SRC_ADDR_INC | MDCR_TRANSMOD |
			MDCR_DMA_BURST_16B | SDCR_FETCHND;
	else
		dcmd = MDCR_DST_ADDR_INC | MDCR_SRC_ADDR_INC | MDCR_TRANSMOD |
			MDCR_DMA_BURST_8B | SDCR_FETCHND;

	base_register = mmp_get_dma_reg_base(dma_ch);
	TDCR(base_register)= (dcmd) & (~MDCR_CHANEN);
	TDIMR(base_register) = MDIMR_COMP;
	if(dma_ch == MDMA1_CH0)
		TDNDPR(base_register) = dma_desc_array_phys_ch1;
	else
		TDNDPR(base_register) = dma_desc_array_phys_ch2;
	if(dma_ch == MDMA1_CH0){
		init_completion(&complete_ch1);
	}else{
		init_completion(&complete_ch2);
	}


	TDCR(base_register)= dcmd | MDCR_CHANEN;

	if(dma_ch == MDMA1_CH0){
		ret = wait_for_completion_timeout(&complete_ch1, 10*HZ);
	}else{
		ret = wait_for_completion_timeout(&complete_ch2, 10*HZ);
	}

	TDIMR(base_register) = 0;
	mdma_dcr = TDCR(base_register);
	if (mdma_dcr & TDCR_CHANACT) {
		TDCR(base_register) = mdma_dcr | TDCR_ABR;
	}
	while (mdma_dcr & TDCR_CHANACT) {
		mdma_dcr = TDCR(base_register);
	}
	mmp_free_dma(dma_ch);

	if (ret)
		return len;
	else
		return -ERESTARTSYS;
}
EXPORT_SYMBOL(mdma_pmemcpy);

/*
 * len requires at least 8bytes alignment. 128bytes alignment will get better performance.
 * Alignment: 8, 16, 32, 64, 128. The bigger, the better.
 * destination address aslo need alignment with burst size.
 * data requires 32bits.
 */
unsigned long mdma_pmemset(unsigned long paddr, unsigned long c, unsigned int len)
{
	int ret, dma_ch;
	unsigned int base_register;
	unsigned int dcmd, mdma_dcr;
	unsigned long len_tmp, len_total;
	struct pxa910_squ_desc *dma_desc_tmp;
	dma_addr_t dma_desc_p_tmp;
	unsigned long dstphyaddr = paddr;

	if (dstphyaddr == 0)
		return -1;

	if (len & 0x7) {
		printk(KERN_ERR "size is not 8 bytes aligned\n");
		return -1;
	}

	if (dstphyaddr & 0x7) {
		printk(KERN_ERR "dstphyaddr is not 8 bytes aligned\n");
		return -1;
	}

	ret = mmp_request_dma("mdma_memcpy", MDMA1_CH0,
					mdma_irq, NULL);
	if (ret < 0){
		ret = mmp_request_dma("mdma_memcpy", MDMA1_CH1,
						mdma_irq, NULL);
		if (ret < 0){
			printk(KERN_ERR	"Can't request MDMA for memcpy\n");
			return -ENODEV;
		}else{
			dma_ch = MDMA1_CH1;
		}
	}else{
		dma_ch = MDMA1_CH0;
	}

	if(dma_ch == MDMA1_CH0){
		dma_desc_tmp = dma_desc_array_ch1;
		dma_desc_p_tmp = dma_desc_array_phys_ch1;
	}else{
		dma_desc_tmp = dma_desc_array_ch2;
		dma_desc_p_tmp = dma_desc_array_phys_ch2;
	}

	len_total = len;
	while(len_total) {
		len_tmp = len_total>SINGLE_DESC_TRANS_MAX ?
				SINGLE_DESC_TRANS_MAX : len_total;
		dma_desc_tmp->nxt_desc = dma_desc_p_tmp + sizeof(struct pxa910_squ_desc);
		dma_desc_tmp->src_addr = 0;
		dma_desc_tmp->dst_addr = dstphyaddr;
		dma_desc_tmp->byte_cnt = len_tmp;
		if (len_total <= SINGLE_DESC_TRANS_MAX) {
			dma_desc_tmp->nxt_desc = 0;
			break;
		}
		len_total -= len_tmp;
		dma_desc_tmp ++;
		dma_desc_p_tmp += sizeof(struct pxa910_squ_desc);
		dstphyaddr += len_tmp;
	}

	if (!((len & 0x7f) || (dstphyaddr & 0x7f)))
		dcmd = MDCR_DST_ADDR_INC | MDCR_SRC_ADDR_HOLD | MDCR_TRANSMOD |
			MDCR_FILLMOD | MDCR_DMA_BURST_128B | SDCR_FETCHND;
	else if (!((len & 0x3f) || (dstphyaddr & 0x3f)))
		dcmd = MDCR_DST_ADDR_INC | MDCR_SRC_ADDR_HOLD | MDCR_TRANSMOD |
			MDCR_FILLMOD | MDCR_DMA_BURST_64B | SDCR_FETCHND;
	else if (!((len & 0x1f) || (dstphyaddr & 0x1f)))
		dcmd = MDCR_DST_ADDR_INC | MDCR_SRC_ADDR_HOLD | MDCR_TRANSMOD |
			MDCR_FILLMOD | MDCR_DMA_BURST_32B | SDCR_FETCHND;
	else if (!((len & 0xf) || (dstphyaddr & 0xf)))
		dcmd = MDCR_DST_ADDR_INC | MDCR_SRC_ADDR_HOLD | MDCR_TRANSMOD |
			MDCR_FILLMOD | MDCR_DMA_BURST_16B | SDCR_FETCHND;
	else
		dcmd = MDCR_DST_ADDR_INC | MDCR_SRC_ADDR_HOLD | MDCR_TRANSMOD |
			MDCR_FILLMOD | MDCR_DMA_BURST_8B | SDCR_FETCHND;

	base_register = mmp_get_dma_reg_base(dma_ch);
	TDCR(base_register)= (dcmd) & (~MDCR_CHANEN);
	TDIMR(base_register) = MDIMR_COMP;
	if(dma_ch == MDMA1_CH0)
		TDNDPR(base_register) = dma_desc_array_phys_ch1;
	else
		TDNDPR(base_register) = dma_desc_array_phys_ch2;
	MMP2_FILL_DATA(base_register) = c;

	if(dma_ch == MDMA1_CH0){
		init_completion(&complete_ch1);
	}else{
		init_completion(&complete_ch2);
	}

	TDCR(base_register)= dcmd | MDCR_CHANEN;

	if(dma_ch == MDMA1_CH0){
		ret = wait_for_completion_timeout(&complete_ch1, 10*HZ);
	}else{
		ret = wait_for_completion_timeout(&complete_ch2, 10*HZ);
	}

	TDIMR(base_register) = 0;
	mdma_dcr = TDCR(base_register);
	if (mdma_dcr & TDCR_CHANACT) {
		TDCR(base_register) = mdma_dcr | TDCR_ABR;
	}
	while (mdma_dcr & TDCR_CHANACT) {
		mdma_dcr = TDCR(base_register);
	}
	mmp_free_dma(dma_ch);

	if (ret)
		return len;
	else
		return -ERESTARTSYS;
}
EXPORT_SYMBOL(mdma_pmemset);

static unsigned long mdma_uvmemcpy(unsigned long uvdst, unsigned long uvsrc, unsigned int len)
{
	unsigned long srcphyaddr, dstphyaddr;
	struct mm_struct *mm = current->mm;
	unsigned long ret;

	spin_lock(&mm->page_table_lock);
	srcphyaddr = uva_to_pa(mm, uvsrc);
	dstphyaddr = uva_to_pa(mm, uvdst);
	spin_unlock(&mm->page_table_lock);

	ret = mdma_pmemcpy(dstphyaddr, srcphyaddr, len);

	return ret;
}

static unsigned long mdma_uvmemset(unsigned long uvaddr, unsigned long c, unsigned int len)
{
	unsigned long phyaddr;
	struct mm_struct *mm = current->mm;
	unsigned long ret;

	spin_lock(&mm->page_table_lock);
	phyaddr = uva_to_pa(mm, uvaddr);
	spin_unlock(&mm->page_table_lock);

	ret = mdma_pmemset(phyaddr, c, len);

	return ret;
}

#define MAX_TEST_SIZE	(16*1024*1024)
void mdma_test(void)
{
	int ret, i, j;
	dma_addr_t test_temp_p_src, test_temp_p_dst;
	void *test_temp_src, *test_temp_dest;
	unsigned long t1, t2, diff;

	test_temp_src = (void *)__get_free_pages(GFP_DMA | GFP_KERNEL,
						get_order(MAX_TEST_SIZE));
	test_temp_p_src = (dma_addr_t)__virt_to_phys(test_temp_src);
	test_temp_dest = (void *)__get_free_pages(GFP_DMA | GFP_KERNEL,
						get_order(MAX_TEST_SIZE));
	test_temp_p_dst = (dma_addr_t)__virt_to_phys(test_temp_dest);

	if ((test_temp_src == NULL) || (test_temp_dest == NULL)) {
		printk("%s: no enough memory!\n", __func__);
		ret = -ENOMEM;
		return;
	}

	printk("mdma_pmemset test_temp_p_src=0x%x\n", test_temp_p_src);
	printk("mdma_pmemset test_temp_p_dst=0x%x\n", test_temp_p_dst);

	for(i = 1024; i <= MAX_TEST_SIZE; i=i*2){
		printk("\nlen=%d\n", i);

		t1 = jiffies;
		for(j = 0; j < 1000; j++)
			memset(test_temp_src, 0x98562378, i);
		t2 = jiffies;
		diff=(long)t2-(long)t1;
		printk("memset jiffies delta = %ld\n", diff);
		printk("memset ms delta = %ld\n", diff*1000/HZ);
		printk("memset ms/1000 delta = %ld\n", diff*1000/HZ/1000);

		t1 = jiffies;
		for(j = 0; j < 1000; j++)
			ret = mdma_pmemset(test_temp_p_src, 0x98562378, i);
		t2 = jiffies;
		diff=(long)t2-(long)t1;
		printk("mdma_pmemset jiffies delta = %ld\n", diff);
		printk("mdma_pmemset ms delta = %ld\n", diff*1000/HZ);
		printk("mdma_pmemset ms/1000 delta = %ld\n", diff*1000/HZ/1000);

		t1 = jiffies;
		for(j = 0; j < 1000; j++)
			memcpy(test_temp_dest, test_temp_src, i);
		t2 = jiffies;
		diff=(long)t2-(long)t1;
		printk("memcpy jiffies delta = %ld\n", diff);
		printk("memcpy ms delta = %ld\n", diff*1000/HZ);
		printk("memcpy ms/1000 delta = %ld\n", diff*1000/HZ/1000);

		t1 = jiffies;
		for(j = 0; j < 1000; j++)
			ret = mdma_pmemcpy(test_temp_p_dst, test_temp_p_src, i);
		t2 = jiffies;
		diff=(long)t2-(long)t1;
		printk(KERN_INFO "mdma_pmemcpy jiffies delta = %ld\n", diff);
		printk(KERN_INFO "mdma_pmemcpy ms delta = %ld\n", diff*1000/HZ);
		printk(KERN_INFO "mdma_pmemcpy ms/1000 delta = %ld\n",
				diff*1000/HZ/1000);
	}
}

static long mdma_ioctl(struct file * filp, unsigned int cmd, unsigned long arg)
{
	int ret;

	switch(cmd) {
	case MDMAIO_DMA_MEMCPY:
	{
		ioctl_mdma_memcpy mdma_memcpy;
		void __user *argp = (void *)arg;

		if (copy_from_user(&mdma_memcpy, argp, sizeof(mdma_memcpy)))
			return -EFAULT;

		ret = mdma_uvmemcpy(mdma_memcpy.dstaddr, mdma_memcpy.srcaddr, mdma_memcpy.length);
		break;
	}
	case MDMAIO_DMA_MEMSET:
	{
		ioctl_mdma_memset mdma_memset;
		void __user *argp = (void *)arg;

		if (copy_from_user(&mdma_memset, argp, sizeof(mdma_memset)))
			return -EFAULT;
		ret = mdma_uvmemset(mdma_memset.addr, mdma_memset.data, mdma_memset.length);
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static struct file_operations mdma_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl		= mdma_ioctl,
};

static struct miscdevice mdma_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "mdma",
	.fops	= &mdma_fops,
};

static int __init mdma_init(void)
{
	int ret = -EINVAL;

	dma_desc_array_ch1 = (void *)__get_free_pages(GFP_DMA | GFP_KERNEL,
					get_order(MAX_DESC_NUM * sizeof(struct pxa910_squ_desc)));
	dma_desc_array_phys_ch1 = (dma_addr_t)__virt_to_phys(dma_desc_array_ch1);

	if (!dma_desc_array_ch1) {
		printk(KERN_ERR "dma desc channel 1 allocate error!!\n");
		ret =  -ENOMEM;
		goto err_init;
	}

	dma_desc_array_ch2 = (void *)__get_free_pages(GFP_DMA | GFP_KERNEL,
					get_order(MAX_DESC_NUM * sizeof(struct pxa910_squ_desc)));
	dma_desc_array_phys_ch2 = (dma_addr_t)__virt_to_phys(dma_desc_array_ch2);
	if (!dma_desc_array_ch2) {
		printk(KERN_ERR "dma desc channel 2 allocate error!!\n");
		ret =  -ENOMEM;
		goto err_init;
	}
	printk(KERN_DEBUG "dma_desc_array_ch1 = 0x%lx\n", (unsigned long)dma_desc_array_ch1);
	printk(KERN_DEBUG "dma_desc_array_ch2 = 0x%lx\n", (unsigned long)dma_desc_array_ch2);
	printk(KERN_DEBUG "dma_desc_array_phys_ch1 = 0x%lx\n", (unsigned long)dma_desc_array_phys_ch1);
	printk(KERN_DEBUG "dma_desc_array_phys_ch2 = 0x%lx\n", (unsigned long)dma_desc_array_phys_ch2);

	ret = misc_register(&mdma_miscdev);
	if (ret < 0){
		printk(KERN_ERR "Error registering device %s\n", mdma_miscdev.name);
		goto err_init;
	}

	printk(KERN_INFO "memcpy and memset by mdma was loaded!\n");
	return 0;
err_init:
	if (!dma_desc_array_ch1) {
		free_pages((unsigned long)dma_desc_array_ch1,
			get_order(MAX_DESC_NUM * sizeof(struct pxa910_squ_desc)));
	}
	if (!dma_desc_array_ch2) {
		free_pages((unsigned long)dma_desc_array_ch2,
			get_order(MAX_DESC_NUM * sizeof(struct pxa910_squ_desc)));
	}
	return ret;
}

static void __exit mdma_exit(void)
{
	misc_deregister(&mdma_miscdev);
	if(dma_desc_array_ch1)
		free_pages((unsigned long)dma_desc_array_ch1, get_order(MAX_DESC_NUM * sizeof(struct pxa910_squ_desc)));
	if(dma_desc_array_ch2)
		free_pages((unsigned long)dma_desc_array_ch2, get_order(MAX_DESC_NUM * sizeof(struct pxa910_squ_desc)));
	return;
}

module_init(mdma_init);
module_exit(mdma_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MDMA Engine");

