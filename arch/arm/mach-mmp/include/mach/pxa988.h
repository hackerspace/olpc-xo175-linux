#ifndef __ASM_CPU_PXA988_H
#define __ASM_CPU_PXA988_H

struct sys_timer;

extern struct sys_timer pxa988_timer;
extern void __init pxa988_init_gic(void);
extern void __init pxa988_init_irq(void);
extern void __init pxa988_reserve(void);

#include <mach/devices.h>
#include <mach/cputype.h>
#include <mach/regs-apbc.h>

extern struct pxa_device_desc pxa988_device_uart1;
extern struct pxa_device_desc pxa988_device_uart2;
extern struct pxa_device_desc pxa988_device_uart3;

static inline int pxa988_add_uart(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1:
		d = &pxa988_device_uart1;
		break;
	case 2:
		d = &pxa988_device_uart2;
		break;
	case 3:
		d = &pxa988_device_uart3;
		break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

#endif /* __ASM_CPU_PXA988_H */

