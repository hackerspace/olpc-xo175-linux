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

#include <mach/pxa3xx-regs.h>
#include <linux/uio_driver.h>
#include <linux/uio_vmeta.h>
#include <linux/vmeta.h>

#include <linux/notifier.h>
#include <linux/timer.h>
#include <mach/dvfm.h>

#define UIO_VMETA_NAME		"pxa955-vmeta"
#define UIO_VMETA_BUS_IRQ_NAME  UIO_VMETA_NAME"-bus"

#define VMETA_PWR_ENABLE	0x1
#define VMETA_PWR_DISABLE	0x0

struct vmeta_plat_data {
	irqreturn_t (*bus_irq_handler)(int irq, void *dev_id);
	int axi_clk_available;
	int power_down_ms;
};

void vmeta_pwr(unsigned int enableDisable);
irqreturn_t pxa95x_vmeta_bus_irq_handler(int irq, void *dev_id);
void __init pxa95x_set_vmeta_info(void *info);
#endif
