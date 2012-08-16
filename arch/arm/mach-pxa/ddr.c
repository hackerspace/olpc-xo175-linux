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
#include <linux/clk.h>
#include <plat/clock.h>

#define RCI_BIT			0x80000000

#define RCOMP_UPDATE_BIT	0x40000000

#define RCOMP_PCODE_MASK	0x1FC00000	/* Rcomp PCODE */
#define RCOMP_NCODE_MASK	0x003F8000	/* Rcomp NCODE */
#define XCODE_MASK		0x0000000F	/* XCODE from DMCISR2 register */

static unsigned int dmc_base;
static unsigned int is_pxa930;
static unsigned int ddr_perf_cnt_old[4];
struct ddr_cycle_type ddr_ticks_array[DDR_OP_NUM];
spinlock_t ddr_performance_data_lock;
unsigned long cpu_flag;
static u64 delta_time[4];
static struct clk *ddr_clk;

extern int ddr_get_table_size(void);

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

/* Dynamic Memory Controller IRQ handler */
static irqreturn_t ddr_MC_irq_handler(int irq, void *dev_id)
{
	if (cpu_is_pxa978()) {
		if ((__raw_readl(dmc_base + PERF_STATUS_OFF) >> 16) & 0xf) {
			/* update data and clean irq flags automaticly */
			update_ddr_performance_data();
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
		ddr_perf_cnt_old[i] = 0;
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
		ddr_perf_cnt_old[i] = 0;
	spin_unlock_irqrestore(&ddr_performance_data_lock, cpu_flag);

	/* clear performance counter IRQ flag */
	__raw_writel((0xf << 16), dmc_base + PERF_STATUS_OFF);
}

void get_ddr_count(unsigned long *total, unsigned long *busy)
{
	int i = 0;
	update_ddr_performance_data();
	while ((delta_time[0] >> i) > 0xFFFFFFLLU)
		i++;
	*total = (unsigned long)(delta_time[0] >> i);
	*busy = (unsigned long)((delta_time[0] - delta_time[1]) >> i);
}

extern int ddr_freq_to_table_idx(unsigned long ddr_freq);

void update_ddr_performance_data()
{
	unsigned int reg[4], i, overflow_flag;
	int ddr_idx = -1;

	if (!dmc_base)
		dmc_base = (unsigned int) ioremap(0x7ff00000, 0x1000);

	/* stop counters, to keep data synchronized */
	__raw_writel((0xf << 4), dmc_base + PERF_CTRL_0_OFF);

	overflow_flag = __raw_readl(dmc_base + PERF_STATUS_OFF);
	overflow_flag = (overflow_flag >> 16) & 0xf;

	spin_lock_irqsave(&ddr_performance_data_lock, cpu_flag);

	if (ddr_clk)
		ddr_idx = ddr_freq_to_table_idx(ddr_clk->rate);
	else
		printk(KERN_ERR "ddr_clk not inited!\n");

	if ((ddr_idx >= 0) && (ddr_idx < ddr_get_table_size())) {
		for (i = 0; i < 4; i++) {
			__raw_writel((i << 0), dmc_base + PERF_SELECT_OFF);
			reg[i] = __raw_readl(dmc_base + PERF_COUNTER_OFF);

			if (overflow_flag & (1 << i))
				delta_time[i] = 0x100000000LLU
					+ reg[i] - ddr_perf_cnt_old[i];
			else
				delta_time[i] = reg[i] - ddr_perf_cnt_old[i];

			ddr_ticks_array[ddr_idx].reg[i] += delta_time[i];
			ddr_perf_cnt_old[i] = reg[i];
		}
	} else
		printk(KERN_ERR"%s: invalid ddr_idx!\n", __func__);

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

		ddr_clk = clk_get(NULL, "DDR");
		if (ddr_clk == NULL) {
			printk(KERN_ERR "can't get ddr clock!\n");
			return -EAGAIN;
		}

		if (ARRAY_SIZE(ddr_ticks_array) < ddr_get_table_size())
			printk(KERN_ERR "ddr_ticks_array is not big enough!\n");

		init_ddr_performance_counter();
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

arch_initcall(ddr_init);
