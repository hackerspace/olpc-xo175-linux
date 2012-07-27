/*
 * ddr calibration Driver
 *
 * Copyright (C) 2007 Marvell Corporation
 * Idan Bartura <ibartura@marvell.com>
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <mach/hardware.h>
#include <mach/pxa95x_pm.h>
#include <asm/io.h>

#define RCI_BIT			0x80000000

#define RCOMP_UPDATE_BIT	0x40000000

#define RCOMP_PCODE_MASK	0x1FC00000	/* Rcomp PCODE */
#define RCOMP_NCODE_MASK	0x003F8000	/* Rcomp NCODE */
#define XCODE_MASK		0x0000000F	/* XCODE from DMCISR2 register */



static unsigned int dmc_base;
static unsigned int is_pxa930;
static unsigned int ddr_performance_counter_old[4];
struct ddr_cycle_type ddr_ticks_array[OP_NUM];
spinlock_t ddr_performance_data_lock;
unsigned long cpu_flag;
unsigned int is_ddr_statics_enabled;
static unsigned long delta_time[4];

static irqreturn_t ddr_calibration_handler(int irq, void *dev_id)
{
	unsigned int dmcier, dmcisr, rcomp;
	unsigned int ncode, pcode, pad_vals;

	dmcisr = __raw_readl(dmc_base + DMCISR_OFF);

	if (dmcisr & RCI_BIT) {	/* this is a RCOMP interrupt */

		/* disabling the rcomp interrupt */
		dmcier = __raw_readl(dmc_base + DMCIER_OFF);
		dmcier &= ~RCI_BIT;
		__raw_writel(dmcier, dmc_base + DMCIER_OFF);

		/* calculating and updating pads */
		ncode = __raw_readl(dmc_base + DMCISR_OFF);
		ncode &= RCOMP_NCODE_MASK;
		ncode = ncode >> 15;	/* shift to rightmost 7 bits */
		pcode = __raw_readl(dmc_base + DMCISR_OFF);
		pcode &= RCOMP_PCODE_MASK;
		pcode = pcode >> 22;	/* shift to rightmost 7 bits */

		if (!is_pxa930) {
			unsigned int xcode, sr = 0x1;

			xcode = __raw_readl(dmc_base + DMCISR2_OFF);
			if ((xcode & XCODE_MASK) == 0)
				xcode = 0x6;

			/* NCODE and PCODE are inputs from SV */
			if (pcode == 0x0)
				pcode = 0x10;

			ncode = pcode;

			/* move values to correct position */
			pcode = pcode << 24;
			ncode = ncode << 16;
			xcode = xcode << 8;

			/* put values in one 32bit result */
			pad_vals = pcode | ncode | xcode | sr;

			/* send result to pad registers */
			__raw_writel(pad_vals, dmc_base + PAD_MA_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_MDLSB_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_SDCLK_OFF);
			/*check with ASI!!!!!!!!!!!!!!!!!! if ( CalCond.isDDR_MDPAD_WA ) */
			__raw_writel(pad_vals, dmc_base + PAD_MDMSB_OFF);

		} else {	/* TAVOR P */

			unsigned int nslew, pslew;

			/*The calculation that is done (new formula) here is:
			 *PSLEW= 1.9473 -  0.051*Pcode + 0.1914*Ncode
			 *NSLEW=-2.0786+ 0.2739*Pcode+ 0.0279*Ncode
			 *To gain the desired accuracy, the constant values above are multiplied
			 *by 10,000 (so 0.0435 will become 435) and the math is done as integer math
			 *rather than floating point. When the calculations are done, the result
			 *is divided by 10,000 and rounded up/down to the closest integer value
			 *for PSLEW and NSLEW.
			 */

			pslew = 19473 - 510 * pcode + 1914 * ncode;
			pslew = (unsigned int) (pslew / 10000);
			nslew = 2739 * pcode + 279 * ncode - 20786;
			nslew = (unsigned int) (nslew / 10000);

			/* clear irrelevant bits */
			pslew = pslew & 0x0F;
			nslew = nslew & 0x0F;

			/* move values to correct position */
			pcode = pcode << 24;
			ncode = ncode << 16;
			pslew = pslew << 8;

			/* put values in one 32bit result */
			pad_vals = pcode | ncode | pslew | nslew;

			/* send result to pad registers */
			__raw_writel(pad_vals, dmc_base + PAD_MA_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_MDMSB_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_MDLSB_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_SDRAM_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_SDCLK_OFF);
			/*check with ASI!!!!!!!!!!!!!!!!!! if ( CalCond.isDDR_MDPAD_WA ) */
			__raw_writel(pad_vals, dmc_base + PAD_SDCS_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_SMEM_OFF);
			__raw_writel(pad_vals, dmc_base + PAD_SCLK_OFF);
		}

		/* enabling the rcomp interrupt. */
		dmcier = __raw_readl(dmc_base + DMCIER_OFF);
		dmcier |= RCI_BIT;
		__raw_writel(dmcier, dmc_base + DMCIER_OFF);

		/* set the RCOMP update bit (30) so DMEMC initiates EMPI NOP cycle and
		   programs EMPI pads with values in PAD_XX regs. */
		rcomp = __raw_readl(dmc_base + RCOMP_OFF);
		rcomp |= RCOMP_UPDATE_BIT;
		__raw_writel(rcomp, dmc_base + RCOMP_OFF);

		/* clear the interrupt status */
		dmcisr = __raw_readl(dmc_base + DMCISR_OFF);
		dmcisr |= RCI_BIT;
		__raw_writel(dmcisr, dmc_base + DMCISR_OFF);

		pr_debug("DDR Rcomp calibration\n");

	}

	return IRQ_HANDLED;

}

extern unsigned int cur_op;
/* Dynamic Memory Controller IRQ handler */
static irqreturn_t ddr_MC_irq_handler(int irq, void *dev_id)
{
	if (cpu_is_pxa978()) {
		if ((__raw_readl(dmc_base + PERF_STATUS_OFF) >> 16) & 0xf) {
			/* update data and clean irq flags automaticly */
			update_ddr_performance_data(cur_op);
		} else {
			printk(KERN_ERR "%s: unhandled DDR controller IRQ!\n",
				__func__);
			printk(KERN_ERR "DDR_ERR_STATUS=%08x"
				" DDR_ERR_ID_OFF=%08x DDR_ERR_ADDR=%08x\n",
				__raw_readl(dmc_base + DDR_ERR_STATUS_OFF),
				__raw_readl(dmc_base + DDR_ERR_ID_OFF),
				__raw_readl(dmc_base + DDR_ERR_ADDR_OFF));

			/* clear the Error to prevent endless interrupt */
			__raw_writel(
				__raw_readl(dmc_base + DDR_ERR_STATUS_OFF),
				dmc_base + DDR_ERR_STATUS_OFF);
			__raw_writel(__raw_readl(dmc_base + DDR_ERR_ID_OFF),
				dmc_base + DDR_ERR_ID_OFF);
			WARN_ON(1);
		}

	} else {
		printk(KERN_ERR "%s: unknow ddr controller interrupt!\n",
			__func__);
		BUG_ON(1);
	}

	return IRQ_HANDLED;
}

void init_ddr_performance_counter(void)
{
	unsigned int i;

	if (!dmc_base)
		dmc_base = (unsigned int) ioremap(0x7ff00000, 0x1000);

	/* reference: Nevo_C0_IAS_Vol2 chapter 14.5.20 Performance Counters */
	/* disable interrupt and counter */
	__raw_writel((0xf << 12) | (0xf << 4),
				dmc_base + PERF_CTRL_0_OFF);

	/* div clock by 1 & Stop on any counter overflow
	 * & enable counter begin couting */
	__raw_writel((0x0 << 16) | (0 << 4),
				dmc_base + PERF_CTRL_1_OFF);

	/* counter0: 0x0 = Clock (divided by pc_clk_div) */
	__raw_writel((0x1 << 31) | (0x0 << 4) | (0x0 << 0),
				dmc_base + PERF_SELECT_OFF);
	__raw_writel(0x0, dmc_base + PERF_COUNTER_OFF);

	/* counter1: 0x1 = MC idle cycles (MC pipeline empty) */
	__raw_writel((0x1 << 31) | (0x1 << 4) | (0x1 << 0),
				dmc_base + PERF_SELECT_OFF);
	__raw_writel(0x0, dmc_base + PERF_COUNTER_OFF);

	/* counter2: 0x4 = MC busy cycles with no data bus utilization */
	__raw_writel((0x1 << 31) | (0x4 << 4) | (0x2 << 0),
				dmc_base + PERF_SELECT_OFF);
	__raw_writel(0x0, dmc_base + PERF_COUNTER_OFF);

	/* counter3: 0x18 = All AXI Read/Write data request */
	__raw_writel((0x1 << 31) | (0x18 << 4) | (0x3 << 0),
				dmc_base + PERF_SELECT_OFF);
	__raw_writel(0x0, dmc_base + PERF_COUNTER_OFF);

	spin_lock_irqsave(&ddr_performance_data_lock, cpu_flag);
	for (i = 0; i < 4; i++)
		ddr_performance_counter_old[i] = 0;
	spin_unlock_irqrestore(&ddr_performance_data_lock, cpu_flag);

	/* clear performance counter IRQ flag */
	__raw_writel((0xf << 16), dmc_base + PERF_STATUS_OFF);

	/* enable counter 0/1/2/3, enable counter0/1/2/3 interrupt */
	__raw_writel((0xf << 0) | (0xf << 8), dmc_base + PERF_CTRL_0_OFF);
}

void stop_ddr_performance_counter(void)
{
	unsigned int i;

	if (!dmc_base)
		dmc_base = (unsigned int) ioremap(0x7ff00000, 0x1000);

	/* reference: Nevo_C0_IAS_Vol2 chapter 14.5.20 Performance Counters */
	/* disable interrupt and counter */
	__raw_writel((0xf << 12) | (0xf << 4),
				dmc_base + PERF_CTRL_0_OFF);

	/* div clock by 1 & Stop on any counter overflow
	 * & enable counter begin couting */
	__raw_writel((0x0 << 16) | (0 << 4),
				dmc_base + PERF_CTRL_1_OFF);

	/* counter0: 0x0 = Clock (divided by pc_clk_div) */
	__raw_writel((0x1 << 31) | (0x0 << 4) | (0x0 << 0),
				dmc_base + PERF_SELECT_OFF);
	__raw_writel(0x0, dmc_base + PERF_COUNTER_OFF);

	/* counter1: 0x1 = MC idle cycles (MC pipeline empty) */
	__raw_writel((0x1 << 31) | (0x1 << 4) | (0x1 << 0),
				dmc_base + PERF_SELECT_OFF);
	__raw_writel(0x0, dmc_base + PERF_COUNTER_OFF);

	/* counter2: 0x4 = MC busy cycles with no data bus utilization */
	__raw_writel((0x1 << 31) | (0x4 << 4) | (0x2 << 0),
				dmc_base + PERF_SELECT_OFF);
	__raw_writel(0x0, dmc_base + PERF_COUNTER_OFF);

	/* counter3: 0x18 = All AXI Read/Write data request */
	__raw_writel((0x1 << 31) | (0x18 << 4) | (0x3 << 0),
				dmc_base + PERF_SELECT_OFF);
	__raw_writel(0x0, dmc_base + PERF_COUNTER_OFF);

	spin_lock_irqsave(&ddr_performance_data_lock, cpu_flag);
	for (i = 0; i < 4; i++)
		ddr_performance_counter_old[i] = 0;
	spin_unlock_irqrestore(&ddr_performance_data_lock, cpu_flag);

	/* clear performance counter IRQ flag */
	__raw_writel((0xf << 16), dmc_base + PERF_STATUS_OFF);
}

void get_ddr_count(unsigned long *total, unsigned long *busy)
{
	update_ddr_performance_data(0);
	*total = delta_time[0];
	*busy = delta_time[0] - delta_time[1];
}

void update_ddr_performance_data(int op_idx)
{
	unsigned int reg[4], i, overflow_flag;

	if (!dmc_base)
		dmc_base = (unsigned int) ioremap(0x7ff00000, 0x1000);

	/* stop counters, to keep data synchronized */
	__raw_writel((0xf << 4), dmc_base + PERF_CTRL_0_OFF);

	overflow_flag = __raw_readl(dmc_base + PERF_STATUS_OFF);
	overflow_flag = (overflow_flag >> 16) & 0xf;

	spin_lock_irqsave(&ddr_performance_data_lock, cpu_flag);
	for (i = 0; i < 4; i++) {
		__raw_writel((i << 0), dmc_base + PERF_SELECT_OFF);
		reg[i] = __raw_readl(dmc_base + PERF_COUNTER_OFF);
		if (overflow_flag & (1 << i))
			delta_time[i] = 0x100000000LLU
				+ reg[i] - ddr_performance_counter_old[i];
		else
			delta_time[i] = reg[i] - ddr_performance_counter_old[i];

		ddr_ticks_array[cur_op].reg[i] += delta_time[i];

		ddr_performance_counter_old[i] = reg[i];
	}

	spin_unlock_irqrestore(&ddr_performance_data_lock, cpu_flag);

	/* if there is overflow, clean interrupt flags and re-init counters */
	if (overflow_flag) {
		__raw_writel((0xf << 16), dmc_base + PERF_STATUS_OFF);
		init_ddr_performance_counter();
	}

	/* enable counter 0/1/2/3, enable counter0/1/2/3 interrupt */
	__raw_writel((0xf << 0) | (0xf << 8), dmc_base + PERF_CTRL_0_OFF);
}

static int __init ddr_init(void)
{
	unsigned int dmcier, dmcisr, temp;

	if ((cpu_is_pxa95x() && !cpu_is_pxa978()) || cpu_is_pxa935())
		is_pxa930 = 0;
	else if (cpu_is_pxa930())
		is_pxa930 = 1;
	else if (cpu_is_pxa978()) {
		spin_lock_init(&ddr_performance_data_lock);
		if (!dmc_base)
			dmc_base = (unsigned int) ioremap(0x7ff00000, 0x1000);

		temp = request_irq(IRQ_DMEMC, ddr_MC_irq_handler,
					IRQF_DISABLED, "pxa9xx-dmemc", NULL);
		if (temp) {
			printk(KERN_ERR "can't assign IRQ_DMEMC!\n");
			return -EAGAIN;
		}
		printk(KERN_INFO"pxa978 ddr_init OK\n");
		return 0;
	}
	else {
		pr_err("DDR calibration is only for pxa93x, pxa955 and pxa968\n");
		return 0;
	}

	/* signing up to the interupt controler */
	temp = request_irq(IRQ_DMEMC, ddr_calibration_handler,
			   IRQF_DISABLED, "pxa9xx-dmemc", NULL);

	dmc_base =
	    (unsigned int) ioremap(DMC_START, DMC_END - DMC_START + 1);

	/* clearing both RCOMP interupt from the status register. */
	dmcisr = __raw_readl(dmc_base + DMCISR_OFF);
	dmcisr |= RCI_BIT;
	__raw_writel(dmcisr, dmc_base + DMCISR_OFF);

	/* enabling RCOMP interupt from dmemc. */
	dmcier = __raw_readl(dmc_base + DMCIER_OFF);
	dmcier |= RCI_BIT;
	__raw_writel(dmcier, dmc_base + DMCIER_OFF);

	return 0;
}

module_init(ddr_init);
