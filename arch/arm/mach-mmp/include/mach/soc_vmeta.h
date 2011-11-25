#ifndef _SOC_VMETA_H_
#define _SOC_VMETA_H_

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#include <linux/uio_driver.h>
#include <linux/uio_vmeta.h>
#include <linux/vmeta.h>

#if defined(CONFIG_PXA3xx_DVFM)
#define VMETA_DVFM_ENABLE 1
#else
#define VMETA_DVFM_ENABLE 0
#endif

#if VMETA_DVFM_ENABLE
#include <linux/notifier.h>
#include <linux/timer.h>
#include <mach/dvfm.h>
#endif

#define UIO_VMETA_NAME		"mmp-vmeta"
#define UIO_VMETA_BUS_IRQ_NAME  UIO_VMETA_NAME"-bus"

#define VMETA_PWR_ENABLE 0x1
#define VMETA_PWR_DISABLE 0x0

void vmeta_pwr(unsigned int enableDisable);
irqreturn_t mmp_vmeta_bus_irq_handler(int irq, void *dev_id);
int mmp_vmeta_set_dvfm_constraint(int idx);
int mmp_vmeta_unset_dvfm_constraint(int idx);
void __init mmp_set_vmeta_info(void *info);
int mmp_update_vmeta_clk(struct vmeta_instance *vi);
#endif
