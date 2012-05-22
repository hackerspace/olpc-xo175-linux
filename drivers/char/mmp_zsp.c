/*
 *
 * Marvell MMP ZSP Host driver.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */


#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/string.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/notifier.h>
#include <linux/relay.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/mtd/super.h>
#include <linux/version.h>
#include <linux/mmp2_mdma.h>
#include <linux/proc_fs.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <mach/hardware.h>
#include <mach/regs-zsp.h>
#include <mach/regs-apmu.h>
#include <mach/regs-icu.h>
#include <mach/mmp-zmq.h>
#include <mach/mmp-zsp.h>
#include <mach/cputype.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <linux/wakelock.h>
#include <mach/sram.h>

#define SRAM_OFFSET_FOR_DTCM 0x6400

/* mmp PZIPC registers */
#define IPC_ISRW	0x0008
#define IPC_ICR		0x000C
#define IPC_IIR		0x0010


#define pzipc_readl(pszp, off) \
	__raw_readl(pzsp->_ipc_regs_base + (off))
#define pzipc_writel(pzsp, off, v) \
	__raw_writel((v), pzsp->_ipc_regs_base + (off))

/* read IIR */
#define PZIPC_IIR_READ(pzsp, IIR_val)			\
{							\
	/* dummy write to latch the IIR value */	\
	pzipc_writel(pzsp, IPC_IIR, 0x0);		\
	barrier();					\
	(IIR_val) = pzipc_readl(pzsp, IPC_IIR);		\
}

typedef struct {
	enum ipc_client_id	logic_id;	/* ACIPC Client Logical Id */
	u32			ipc_irq_bit;	/* The interrupt type bit*/
	irq_handler_t		isr_handler;	/* The ISR handler for client */
} zsp_ipc_cft_t;

typedef struct {
	struct platform_device *	_dev;
	struct mmp_zsp_platform_device *_devop;
	bool				_pending_clkcfg;
	void				*_regs_base;
	void				*_ipc_regs_base;
	void				*_sram_base;
	phys_addr_t			_sram_pa;
	resource_size_t			_sram_sz;
	resource_size_t			_sram_sz_used;
	phys_addr_t			_itcm_pa;
	void				*_itcm_va;
	resource_size_t			_itcm_sz;
	resource_size_t			_itcm_sz_used;
	phys_addr_t			_dtcm_pa;
	void				*_dtcm_va;
	resource_size_t			_dtcm_sz;
	resource_size_t			_dtcm_sz_used;
	unsigned int			_irq;
	atomic_t			_ccnt;
	struct mutex			_floadlock;
	volatile bool			_ready;
	volatile bool			_zspup;
	zsp_ipc_cft_t			_ipcmap[IPC_MAX_NUM];
	void				*pbuffer;
	void				*pbuffer_aligned;
	dma_addr_t 			dma_handle;
	dma_addr_t 			dma_handle_aligned;
	bool				firmware_loaded;
} zsp_device_t;

static zsp_device_t gs_zsp = {
	NULL, NULL, false, NULL, NULL, NULL,
	0, 0, 0, 0, NULL, 0, 0, 0, NULL, 0, 0, -1, {0}, {{1},}, false, false,
	{
		/*client id */		/* interrupt bit*/	/* handler */
		{IPC_HANDSHAKE_ID,		IPC_INTERRUPT_BIT_0_0,	NULL},
		{IPC_MSG_TRANSFER_ID,		IPC_INTERRUPT_BIT_0_1,	NULL},
		{IPC_DMEM_REQ_ID,		IPC_INTERRUPT_BIT_0_2,	NULL},
		{IPC_DMEM_RLS_ID,		IPC_INTERRUPT_BIT_0_3,	NULL},
		{IPC_PORT_FLOWCONTROL_ID,	IPC_INTERRUPT_BIT_0_4,	NULL},
		{IPC_TEST_ID,			IPC_INTERRUPT_BIT_0_5,	NULL},
		{IPC_IPM_ID,			IPC_INTERRUPT_BIT_2_9,	NULL},
		/* TBD */
	},
	NULL, NULL, 0, 0, false
};

struct completion zsp_halt_completion;

static irqreturn_t pzipc_interrupt_handler(int irq, void *dev_id)
{
	zsp_device_t * pzsp = (zsp_device_t *)dev_id;
	u32 i, iir_value = 0;

	/* read the IIR*/
	PZIPC_IIR_READ(pzsp, iir_value);
	/* clear status */
	pzipc_writel(pzsp, IPC_ICR, iir_value);

	/* call ISR for detail interrupt  */
	for (i = 0; i < IPC_MAX_NUM; i++) {
		if (((iir_value & pzsp->_ipcmap[i].ipc_irq_bit) != 0)
			&& (pzsp->_ipcmap[i].isr_handler != NULL)) {
			pzsp->_ipcmap[i].isr_handler(irq,dev_id);
		}
	}
	return IRQ_HANDLED;
}

enum pzipc_return_code pzipc_set_interrupt(enum ipc_client_id client_id)
{
	zsp_device_t * pzsp = &gs_zsp;

	if (client_id >= IPC_MAX_NUM)
		return PZIPC_RC_WRONG_PARAM;

	pzipc_writel(pzsp, IPC_ISRW, pzsp->_ipcmap[client_id].ipc_irq_bit);
	return PZIPC_RC_OK;
}
EXPORT_SYMBOL(pzipc_set_interrupt);

enum pzipc_return_code pzipc_add_isr(enum ipc_client_id client_id,
	irq_handler_t isr_handler)
{
	if (client_id >= IPC_MAX_NUM)
		return PZIPC_RC_WRONG_PARAM;
	gs_zsp._ipcmap[client_id].isr_handler = isr_handler;
	return PZIPC_RC_OK;
}
EXPORT_SYMBOL(pzipc_add_isr);

enum pzipc_return_code pzipc_remove_isr(enum ipc_client_id client_id)
{
	if (client_id >= IPC_MAX_NUM)
		return PZIPC_RC_WRONG_PARAM;
	gs_zsp._ipcmap[client_id].isr_handler = NULL;
	return PZIPC_RC_OK;
}
EXPORT_SYMBOL(pzipc_remove_isr);


#define _ZALIGN(x, a, t) ((t)(((u32)(x)+((u32)(a)-1))&(~((u32)(a)-1))))
#define ZALIGN256(x, t) _ZALIGN(x,256,t)

static irqreturn_t zsp_ap_handshake_isr(int irq, void *dev_id)
{
	zsp_device_t * pzsp = (zsp_device_t *)dev_id;

	pzsp->_zspup = true;

	return IRQ_HANDLED;
}
static irqreturn_t zsp_dmem_req_isr(int irq, void *dev_id)
{
	zsp_device_t * pzsp;
	pzsp = (zsp_device_t *)dev_id;
	/**************  add pm constraint here later  ************/
	pzipc_set_interrupt(IPC_DMEM_REQ_ID);

	return IRQ_HANDLED;
}
static irqreturn_t zsp_dmem_rls_isr(int irq, void *dev_id)
{
	zsp_device_t * pzsp;
	pzsp = (zsp_device_t *)dev_id;
	/**************  remove pm constraint here later  ************/

	pzipc_set_interrupt(IPC_DMEM_RLS_ID);
	return IRQ_HANDLED;
}

static void zsp_halt(zsp_device_t * pzsp)
{
	if ((pzsp == NULL)
		|| (pzsp->_devop == NULL)
		|| (pzsp->_devop->domain_halt == NULL)) return;

	pzsp->_devop->domain_halt();
}

static void zsp_on(zsp_device_t * pzsp)
{
	if ((pzsp == NULL)
		|| (pzsp->_devop == NULL)
		|| (pzsp->_devop->domain_on == NULL)) return;

	pzsp->_devop->domain_on(pzsp->_devop->clkcfg.spd,
			pzsp->_devop->clkcfg.src,
			pzsp->_devop->clkcfg.asclk);
}

static void zsp_start_core(zsp_device_t * pzsp)
{
	if ((pzsp == NULL)
		|| (pzsp->_devop == NULL)
		|| (pzsp->_devop->start_core == NULL)) return;

	pzsp->_devop->start_core();
}
static int zsp_loadbin(const char *pname, u32 offset, u32 targetphyaddr,
	void *targetvirtaddr, struct platform_device *pdev) {

	zsp_device_t * pzsp;
	int ret = -1;
	const struct firmware *fw;
	char *fw_ptr;
	u32 fw_size, adjusted_size;

	pzsp = (zsp_device_t *)platform_get_drvdata(pdev);

	ret = request_firmware(&fw, pname, &pdev->dev);
	if (ret){
		printk(KERN_ERR "%s:load firmware %s fail -- ret %d\n", __func__
			, pname, ret);
		return ret;
	}

	fw_ptr		=(char *)fw->data;
	fw_size		= (u32)fw->size;
	adjusted_size	= ZALIGN256(fw_size, u32);

	memcpy(pzsp->pbuffer_aligned + offset, fw_ptr, fw_size);

	if (memcmp(fw_ptr, pzsp->pbuffer_aligned + offset, fw_size) != 0) {
		printk(KERN_ERR "%s: memory content doesnot match\n", __func__);
		ret = -1;
		goto cleanup;
	}
	if (pzsp->_devop->hw_memcpy != NULL) {
		ret = pzsp->_devop->hw_memcpy(targetphyaddr,
			pzsp->dma_handle_aligned + offset, adjusted_size);
	} else {
		memcpy(targetvirtaddr,
			pzsp->pbuffer_aligned + offset, adjusted_size);
		ret = adjusted_size;
	}

	if (memcmp(targetvirtaddr, pzsp->pbuffer_aligned + offset,
		adjusted_size) != 0) {
		printk(KERN_ERR "%s: memory content doesnot match\n", __func__);
		ret = -1;
		goto cleanup;
	}
	if ((targetphyaddr >= pzsp->_itcm_pa) && (targetphyaddr < pzsp->_itcm_pa + pzsp->_itcm_sz)) {
		if (targetphyaddr < pzsp->_itcm_pa + 0x400)
			pzsp->_itcm_sz_used += 0x400;
		else
			pzsp->_itcm_sz_used += adjusted_size;
	} else if ((targetphyaddr >= pzsp->_dtcm_pa) &&
		(targetphyaddr < pzsp->_dtcm_pa + pzsp->_dtcm_sz)) {
		pzsp->_dtcm_sz_used += adjusted_size;
	} else if ((targetphyaddr >= pzsp->_sram_pa) &&
		(targetphyaddr < pzsp->_sram_pa + pzsp->_sram_sz)) {
		pzsp->_sram_sz_used += adjusted_size;
	}

	ret = 0;

cleanup:
	if (ret == -1) {
		pzsp->_itcm_sz_used = 0;
		pzsp->_dtcm_sz_used = 0;
		pzsp->_sram_sz_used = 0;
	}
	release_firmware(fw);
	return ret;
}

static int zsp_doreset(struct platform_device *pdev, int timeout_ms) {

	zsp_device_t * pzsp;
	int loop;
	int intervalms;
	int ret = -EINVAL;

	pzsp = (zsp_device_t *)platform_get_drvdata(pdev);
	zsp_on(pzsp);

	memset(pzsp->_sram_base, 0x00, pzsp->_sram_sz);

	if (!pzsp->firmware_loaded) {
		ret = zsp_loadbin("zsp_ints.bin",
			0, pzsp->_itcm_pa, pzsp->_itcm_va, pdev);
		if (ret != 0) {
			printk(KERN_ERR "%s: load ints.bin failed\n", __func__);
			goto cleanup;
		}
		ret = zsp_loadbin("zsp_text.bin",
			0x400, pzsp->_itcm_pa + 0x400,
			pzsp->_itcm_va + 0x400, pdev);
		if (ret != 0) {
			printk(KERN_ERR "%s: load text.bin failed\n", __func__);
			goto cleanup;
		}
		if (pzsp->_devop->hw_memcpy != NULL) {
			ret = zsp_loadbin("zsp_data.bin",
				ZALIGN256(pzsp->_itcm_sz + 255, u32),
				pzsp->_dtcm_pa, pzsp->_dtcm_va, pdev);
		} else {
			ret = zsp_loadbin("zsp_data.bin",
				ZALIGN256(pzsp->_itcm_sz + 255, u32),
				pzsp->_sram_pa + SRAM_OFFSET_FOR_DTCM,
				pzsp->_sram_base + SRAM_OFFSET_FOR_DTCM, pdev);
		}
		if (ret != 0) {
			printk(KERN_ERR "%s: load data.bin failed\n", __func__);
			goto cleanup;
		}
		pzsp->firmware_loaded = true;
	} else {
		if (pzsp->_devop->hw_memcpy != NULL)
			ret = pzsp->_devop->hw_memcpy(pzsp->_itcm_pa,
				pzsp->dma_handle_aligned, pzsp->_itcm_sz_used);
		else {
			memcpy(pzsp->_itcm_va,
				pzsp->pbuffer_aligned, pzsp->_itcm_sz_used);
			ret = pzsp->_itcm_sz_used;
		}
		if (ret != pzsp->_itcm_sz_used) {
			printk(KERN_ERR "%s: fill itcm failed\n", __func__);
			goto cleanup;
		}
		if (pzsp->_devop->hw_memcpy != NULL) {
			ret = pzsp->_devop->hw_memcpy(pzsp->_dtcm_pa,
				pzsp->dma_handle_aligned +
				ZALIGN256(pzsp->_itcm_sz + 255, u32),
				pzsp->_dtcm_sz_used);
			if (ret != pzsp->_dtcm_sz_used) {
				printk(KERN_ERR "%s: fill dtcm failed\n", __func__);
				goto cleanup;
			}
		} else {
			memcpy(pzsp->_sram_base + SRAM_OFFSET_FOR_DTCM,
				pzsp->pbuffer_aligned +
				ZALIGN256(pzsp->_itcm_sz + 255, u32),
				pzsp->_sram_sz_used);
			ret = pzsp->_sram_sz_used;
			if (ret != pzsp->_sram_sz_used) {
				printk(KERN_ERR "%s: fill sram failed\n", __func__);
				goto cleanup;
			}
		}
	}

	ret = pzipc_add_isr(IPC_HANDSHAKE_ID, zsp_ap_handshake_isr);
	if (PZIPC_RC_OK != ret) {
		printk(KERN_ERR "%s: cannot hand shake with ZSP\n", __func__);
		return -EINVAL;
	}

	pzsp->_zspup = false;

	zsp_start_core(pzsp);

	intervalms = 50;
	loop = timeout_ms/intervalms + 1;

	while (loop-- > 0) {
		mdelay(intervalms);
		if(pzsp->_zspup) {
			pzipc_set_interrupt(IPC_HANDSHAKE_ID);
			ret = 0;

			goto done;
		}
	}

	printk(KERN_ERR "%s: ZSP initialize timeout\n", __func__);
	ret = -EINVAL;

cleanup:
	zsp_halt(pzsp);
done:
	pzipc_remove_isr(IPC_HANDSHAKE_ID);
	return ret;
}

#define ZSP_HALT_WHEN_NO_REFERENCE
static void zsp_start(int timeout_ms, int retry) {
	zsp_device_t * pzsp = &gs_zsp;
	if (pzsp->_dev != NULL) {
		if (atomic_add_return(1, &(pzsp->_ccnt)) == 1) {
#ifdef ZSP_HALT_WHEN_NO_REFERENCE
			while (retry -- > 0) {
				if (!zsp_doreset(pzsp->_dev, timeout_ms)) {
					printk(KERN_INFO "%s: ZSP ready\n",
						__func__);
					init_completion(&zsp_halt_completion);
					break;
				}
			}
#else
			static bool s_zsp_loaded = false;
			if (s_zsp_loaded) return;
			while (retry -- > 0) {
				if (!zsp_doreset(pzsp->_dev, timeout_ms)) {
					printk(KERN_INFO "%s: ZSP ready\n",
						__func__);
					s_zsp_loaded = true;
					break;
				}
			}
#endif
		}
	}
}
static void zsp_stop(void) {
	zsp_device_t * pzsp = &gs_zsp;
	if (pzsp->_dev != NULL) {
		if (atomic_sub_return(1, &(pzsp->_ccnt)) == 0) {
#ifdef ZSP_HALT_WHEN_NO_REFERENCE
			printk(KERN_INFO "%s: ZSP halt\n", __func__);
			zsp_halt(pzsp);
			complete(&zsp_halt_completion);
#endif
		}
	}
}

void zsp_local_start(atomic_t *pcounter, int timeout_ms, int retry) {
	zsp_device_t * pzsp = &gs_zsp;
	mutex_lock(&(pzsp->_floadlock));
	if (atomic_add_return(1, pcounter) == 1) {
		zsp_start(timeout_ms,retry);
		pzsp->_pending_clkcfg = false; /* clear the flag */
	}
	mutex_unlock(&(pzsp->_floadlock));
}
EXPORT_SYMBOL(zsp_local_start);

void zsp_local_stop(atomic_t *pcounter) {
	zsp_device_t * pzsp = &gs_zsp;
	mutex_lock(&(pzsp->_floadlock));
	if (atomic_sub_return(1, pcounter) == 0) {
		zsp_stop();
	}
	mutex_unlock(&(pzsp->_floadlock));
}
EXPORT_SYMBOL(zsp_local_stop);

void* zsp_get_datawnd(void)
{
	zsp_device_t *pzsp = &gs_zsp;
	return pzsp->_sram_base;
}
EXPORT_SYMBOL(zsp_get_datawnd);

struct mmp_zsp_platform_device *zsp_get_devop()
{
	zsp_device_t *pzsp = &gs_zsp;
	return pzsp->_devop;
}
EXPORT_SYMBOL(zsp_get_devop);

int zsp_set_clock_preference(u32 opmask, struct mmp_zsp_clkcfg *pcfg)
{
	int ret, bit;
	int *pparam;
	int *plocal;
	zsp_device_t *pzsp = &gs_zsp;

	ret = -EFAULT;
	mutex_lock(&(pzsp->_floadlock));
	if (pzsp->_devop != NULL) {
		pparam = (int*)pcfg;
		plocal = (int*)(&(pzsp->_devop->clkcfg));
		opmask = opmask & MMP_ZSP_FLAGS_MSK;
		ret = 0;
		if ((atomic_read(&(pzsp->_ccnt)) == 0)
			&& (!pzsp->_pending_clkcfg)) {
			/* the ZSP is in halt, we can change the clock source */
			bit = 0;
			while ((1u<<bit) & MMP_ZSP_FLAGS_MSK) {
				if (opmask & (1u<<bit)) {
					plocal[bit] = pparam[bit];
				}
				bit++;
			}
			/* disable pending clkcfg check, until better ways */
			/* pzsp->_pending_clkcfg = true; */
		} else { /* busy let's check if target is the same */
			bit = 0;
			while ((1u<<bit) & MMP_ZSP_FLAGS_MSK) {
				if ((opmask & (1u<<bit))
					&& (plocal[bit] != pparam[bit])) {
					ret = -EBUSY;
				}
				bit++;
			}
		}
		/* copy new values back */
		memcpy(pparam, plocal, sizeof(*pcfg));
	}
	mutex_unlock(&(pzsp->_floadlock));
	return ret;
}
EXPORT_SYMBOL(zsp_set_clock_preference);

int zsp_mmap_datawnd(struct vm_area_struct *vma)
{
	int ret;
	vma->vm_flags |= VM_RESERVED;	/* Don't swap */
	vma->vm_flags |= VM_DONTEXPAND; /* Don't remap */
	vma->vm_flags |= VM_DONTCOPY;	/* Don't fork */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	ret =  remap_pfn_range(vma, vma->vm_start,
			 ((unsigned long)gs_zsp._sram_pa >> PAGE_SHIFT),
			 (vma->vm_end - vma->vm_start), vma->vm_page_prot);
	return ret;

}
EXPORT_SYMBOL(zsp_mmap_datawnd);

static ssize_t zsp_pm_stat(struct device *dev, struct device_attribute *attr,
						   char *buf)
{
	zsp_device_t *pzsp = &gs_zsp;
	return sprintf(buf, "%u\n", atomic_read(&(pzsp->_ccnt)));
}

static DEVICE_ATTR(zsp_run, S_IRUGO | S_IWUSR, zsp_pm_stat, NULL);

static int __devinit zsp_probe(struct platform_device *pdev)
{
	zsp_device_t *		pzsp;
	struct resource *	resource;
	int			ret;
	int			ret_hook;
	int			irq;

	pzsp		= &gs_zsp;
	ret		= -EBUSY;
	ret_hook	= -EBUSY;

	if ( pzsp->_ready ) {
		printk(KERN_WARNING "ZSP already running\n");
		return -EBUSY;
	}

	pzsp->_ready	= false;

	mutex_init(&(pzsp->_floadlock));

	pzsp->_devop = pdev->dev.platform_data;
	if (pzsp->_devop == NULL) {
		printk(KERN_ERR "failed to get zsp platform ops\n");
		ret = -EBUSY;
		goto cleanup;
	}

	/* map ZSP register area */
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (resource == NULL) {
		printk(KERN_ERR "resource 0 for %s not exist\n", pdev->name);
		ret = -ENXIO;
		goto cleanup;
	}
	if ((pzsp->_regs_base = ioremap_nocache(resource->start,
			resource->end - resource->start + 1)) == NULL) {
		printk(KERN_ERR "failed to map IO resource 0\n");
		ret = -EBUSY;
		goto cleanup;
	}

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (resource == NULL) {
		printk(KERN_ERR "resource 0 for %s not exist\n", pdev->name);
		ret = -ENXIO;
		goto cleanup;
	}
	if (resource->start == resource->end) {
		/* mmp2 case */
		pzsp->_ipc_regs_base = (void *) resource->start;
		pzsp->_sram_sz = pzsp->_devop->sram_size;
		pzsp->_sram_base = sram_alloc("audio sram",
			pzsp->_sram_sz, &pzsp->_sram_pa);
		if (pzsp->_sram_base == NULL) {
			printk(KERN_ERR "failed to map audio sram\n");
			ret = -EBUSY;
			goto cleanup;
		}
	} else {
		/* mmp3 case */
		pzsp->_ipc_regs_base = ioremap_nocache(resource->start,
				resource->end - resource->start + 1);
		if (pzsp->_ipc_regs_base == NULL) {
			printk(KERN_ERR "failed to map IPC resource 0\n");
			ret = -EBUSY;
			goto cleanup;
		}

		pzsp->_sram_pa = 0xD1030000;
		pzsp->_sram_sz = pzsp->_devop->sram_size;
		pzsp->_sram_base = ioremap_nocache(0xD1030000,
					pzsp->_sram_sz);
		if (pzsp->_sram_base == NULL) {
			printk(KERN_ERR "failed to map audio sram\n");
			ret = -EBUSY;
			goto cleanup;
		}
	}


	/* get tcm sram area */
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (resource == NULL) {
		printk(KERN_ERR "resource 1 for %s not exist\n", pdev->name);
		ret = -ENXIO;
		goto cleanup;
	}
	pzsp->_itcm_pa = resource->start;
	pzsp->_itcm_sz = resource->end - resource->start + 1;
	pzsp->_itcm_va = ioremap_nocache(resource->start,
		resource->end - resource->start + 1);
	if (pzsp->_itcm_va == NULL) {
		printk(KERN_ERR "failed to map itcm resource 0\n");
		ret = -EBUSY;
		goto cleanup;
	}

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (resource == NULL) {
		printk(KERN_ERR "resource 2 for %s not exist\n", pdev->name);
		ret = -ENXIO;
		goto cleanup;
	}
	pzsp->_dtcm_pa = resource->start;
	pzsp->_dtcm_sz = resource->end - resource->start + 1;
	pzsp->_dtcm_va = ioremap_nocache(resource->start,
		resource->end - resource->start + 1);
	if (pzsp->_dtcm_va == NULL) {
		printk(KERN_ERR "failed to map dtcm resource 0\n");
		ret = -EBUSY;
		goto cleanup;
	}


	/* get IRQ */
	if ((irq = platform_get_irq(pdev, 0)) < 0 ) {
		printk(KERN_ERR "failed to get irq\n");
		ret = -EBUSY;
		goto cleanup;
	}
	pzsp->_irq = (unsigned int)irq;

	ret_hook = request_irq(pzsp->_irq, pzipc_interrupt_handler,
			0, "pzipc", pzsp);
	if (ret_hook != 0) {
		printk(KERN_ERR "failed to request irq\n");
		ret = -EBUSY;
		goto cleanup;
	}


	platform_set_drvdata(pdev, pzsp);

	pdev->dev.coherent_dma_mask = 0xffffffff;
	pzsp->_ready	= true;
	pzsp->_dev	= pdev;
	pzsp->_ccnt.counter = 0;

	pzsp->pbuffer	= dmam_alloc_coherent(&pdev->dev, pzsp->_itcm_sz + pzsp->_dtcm_sz + 512,
				&pzsp->dma_handle, GFP_KERNEL);
	if ((pzsp->pbuffer == NULL)) {
		printk(KERN_ERR "%s:allocate buffer fail\n", __func__);
		goto cleanup;
	}
	pzsp->pbuffer_aligned = ZALIGN256(pzsp->pbuffer,void*);
	pzsp->dma_handle_aligned = ZALIGN256(pzsp->dma_handle, u32);


	if (device_create_file(&pdev->dev, &dev_attr_zsp_run))
		printk(KERN_WARNING "zsp: creating sys file failed");

	printk(KERN_INFO "ZSP driver started\n");

	return 0;

cleanup:
	if (ret_hook == 0)
		free_irq(pzsp->_irq, pzsp);

	if (pzsp->_regs_base != NULL)
		iounmap(pzsp->_regs_base);

	if (pzsp->_sram_base != NULL)
		iounmap(pzsp->_sram_base);

	if (pzsp->_itcm_va != NULL)
		iounmap(pzsp->_itcm_va);

	if (pzsp->_dtcm_va != NULL)
		iounmap(pzsp->_dtcm_va);

	pzsp->_irq = -1;
	pzsp->_regs_base = NULL;
	pzsp->_ipc_regs_base = NULL;
	pzsp->_sram_base = NULL;
	pzsp->_sram_pa	= 0;
	pzsp->_itcm_pa	= 0;
	pzsp->_dtcm_pa	= 0;
	pzsp->_dev	= NULL;
	pzsp->_devop	= NULL;
	pzsp->_pending_clkcfg = false;
	pzsp->_ready	= false;
	pzsp->_zspup	= false;

	return ret;
}

static int __devexit zsp_remove(struct platform_device *pdev)
{
	zsp_device_t *pzsp = (zsp_device_t *)platform_get_drvdata(pdev);
	sram_free("audio sram", (void *)pzsp->_sram_base, pzsp->_sram_sz);

	if (pzsp->_ready) {
		disable_irq(pzsp->_irq);
		zsp_halt(pzsp);
		free_irq(pzsp->_irq, pzsp);
		iounmap(pzsp->_regs_base);
		if (pzsp->pbuffer != NULL) {
			dmam_free_coherent(&pdev->dev, pzsp->_itcm_sz + pzsp->_dtcm_sz + 512,
				pzsp->pbuffer, pzsp->dma_handle);
			pzsp->pbuffer = NULL;
			pzsp->pbuffer_aligned = NULL;
			pzsp->dma_handle = 0;
			pzsp->dma_handle_aligned = 0;
		}
		pzsp->firmware_loaded = false;
		pzsp->_irq = -1;
		pzsp->_regs_base = NULL;
		pzsp->_ipc_regs_base = NULL;
		pzsp->_sram_base = NULL;
		pzsp->_sram_pa	= 0;
		pzsp->_sram_sz	= 0;
		pzsp->_sram_sz_used = 0;
		pzsp->_itcm_pa	= 0;
		pzsp->_itcm_sz	= 0;
		pzsp->_itcm_sz_used	= 0;
		pzsp->_dtcm_pa	= 0;
		pzsp->_dtcm_sz	= 0;
		pzsp->_dtcm_sz_used	= 0;
		pzsp->_dev	= NULL;
		pzsp->_devop	= NULL;
		pzsp->_pending_clkcfg = false;
		pzsp->_ready	= false;
		pzsp->_zspup	= false;
		pzsp->_ccnt.counter = 0;
		mutex_destroy(&gs_zsp._floadlock);
		printk(KERN_INFO "ZSP driver stopped\n");
	}

	return 0;
}

static struct platform_driver gs_zsp_driver = {
	.driver		= {
		.name	= "mmp-zsp",
	},
	.probe		= zsp_probe,
	.remove		= __devexit_p(zsp_remove)
};


static int __init zsp_init(void)
{
	return platform_driver_register(&gs_zsp_driver);
}

static void __exit zsp_exit(void)
{
	platform_driver_unregister(&gs_zsp_driver);
}


module_init(zsp_init);
module_exit(zsp_exit);

MODULE_AUTHOR("Marvell Technology");
MODULE_DESCRIPTION("MMP ZSP Host Driver");
MODULE_LICENSE("GPL");
