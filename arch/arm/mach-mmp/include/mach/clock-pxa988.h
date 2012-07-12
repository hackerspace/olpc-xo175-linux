#ifndef __MACH_CLK_PXA988_H
#define __MACH_CLK_PXA988_H

#include <linux/clk.h>
#include <plat/clock.h>

extern void pxa988_init_one_clock(struct clk *c);

/* Interface used to get components avaliable rates, unit Khz */
extern unsigned int pxa988_get_vpu_op_num(void);
extern unsigned int pxa988_get_vpu_op_rate(unsigned int index);

extern unsigned int pxa988_get_ddr_op_num(void);
extern unsigned int pxa988_get_ddr_op_rate(unsigned int index);

#endif /* __MACH_CLK_PXA988_H */
