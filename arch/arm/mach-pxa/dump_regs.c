/*
 * arch/arm/mach-pxa/dump_regs.c
 *
 * PXA78 Dump Soc registers and stack driver
 *
 * Copyright (C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include <asm/io.h>
#include <asm/current.h>
#include <mach/dump_regs.h>

struct reg_map pxa_reg_map[] =
{
	{0x40F50000, 0x40F50100, 0, "Main_PMU"}, /*include Main_CCU_1 PWR_I2C*/
/*	{0x40F50024, 0x40F50028, 0, "Main_CCU_1"}, already in "Main_PMU" */
	{0x41350000, 0x41350008, 0, "Main_CCU_2"},
	{0x42404008, 0x42404088, 0, "Main_CCU_3"},
	{0x40F40000, 0x40F400E4, 0, "Apps_PMU"},
	{0x41340000, 0x41340064, 0, "Apps_CCU"},
/*	{0x40A000E4, 0x40A000E4, 0, "Apps_CCU2"}, already in "Main_PMU" */
/*	{0xD404001C, 0xD4040040, 0, "PWR_CLK_Com_1"}, some issue on D0 */
/*	{0xD4041148, 0xD40412CC, 0, "PWR_CLK_Com_2"}, some issue on D0 */
/*	{0xD4044004, 0xD404414C, 0, "PWR_CLK_Com_3"}, some issue on D0 */
/*	{0xD4044804, 0xD40448C0, 0, "PWR_CLK_Com_4"}, some issue on D0 */
/*	{0xD41000A4, 0xD41000A4, 0, "PWR_CLK_Com_5"}, some issue on D0 */
/*	{0xF00C0020, 0xF00C0060, 0, "PWR_CLK_Com_6"}, if read, will hang on C0*/
/*	{0xFFB00030, 0xFFB00030, 0, "PWR_CLK_Com_7"}, if read, will hang on C0*/
	{0x42404060, 0x424040A4, 0, "App_Reg_BPB_1"},
	{0x42404104, 0x42404150, 0, "App_Reg_BPB_2"},
	{0x42404200, 0x42404218, 0, "App_Reg_BPB_3"},
	{0x42405320, 0x4240532C, 0, "App_Reg_BPB_4"},
	{0x42440154, 0x42440160, 0, "App_Reg_BPB_5"},
	{0x4600FF80, 0x4600FF80, 0, "App_Reg_BPB_6"},
/*	{0xD4041150, 0xD4041154, 0, "App_Reg_BPB_7"}, some issue on D0 */
/*	{0xFFA60020, 0xFFA60024, 0, "App_Reg_BPB_8"}, if read, will hang on C0 */
	{0x40E101E4, 0x40E101FC, 0, "AIB_1"}, /* include "Apps_Regs_AIB_2" */
	{0x40E10204, 0x40E1059C, 0, "AIB_MFPRX"},
	{0x40E107B0, 0x40E107FC, 0, "AIB_3"}, /* include "Comm_CCU_AIB_2" */
	{0x4600FE00, 0x4600FE08, 0, "Apps_Switch_1"}, /* include "SGPR" */
	{0x55510000, 0x555100A0, 0, "Apps_Switch_2"},
	{0x55D00000, 0x55D00000, 0, "Apps_Switch_3"},
	{0x55D10000, 0x55D10060, 0, "Apps_Switch_4"},
/*	{0x4600FE08, 0x4600FE08, 0, "SGPR"}, already in "Apps_Switch_1" */
	{0x58110008, 0x581100B0, 0, "SCU"},
/*	{0x40A000E4, 0x40A000E4, 0, "SCU2"}, already in "Main_PMU" */
	{0x40D00000, 0x40D001C0, 0, "INTC"},
	{0x40900000, 0x4090003C, 0, "RTC"},
	{0x40A00000, 0x40A000E4, 0, "TMR"}, /* include "SCU2" "Apps_CCU2" */
	{0x40000000, 0x400003FC, 0, "DMAC_1"},
	{0x40001108, 0x4000118C, 0, "DMAC_2"},
	{0x56100000, 0x5610000C, 0, "HiDMA_1"},
	{0x561000A0, 0x561000A4, 0, "HiDMA_2"},
	{0x561000F0, 0x56100114, 0, "HiDMA_3"},
	{0x56100200, 0x5610023C, 0, "HiDMA_4"},
/*	{0x40F500C0, 0x40F500D0, 0, "PWR_I2C"}, already in "Main_PMU" */
	{0x43100000, 0x4310007C, 0, "NAND"},
/*	{0x4A000008, 0x4A00001C, 0, "SMC_1"}, if read, will hang C0*/
/*	{0x4A000068, 0x4A0000B4, 0, "SMC_2"}, if read, will hang C0*/
	{0x7FF00000, 0x7FF002B0, 0, "DDR_MC_1"},
	{0x7FF00380, 0x7FF005B0, 0, "DDR_MC_2"},
	{0x7FF00650, 0x7FF007E0, 0, "DDR_MC_3"},
	{0x7FF00B40, 0x7FF00B60, 0, "DDR_MC_4"},
	{0x7FF00C00, 0x7FF00C30, 0, "DDR_MC_5"},
	{0x7FF00E00, 0x7FF00E90, 0, "DDR_MC_6"},
	{0x7FF00F00, 0x7FF00F50, 0, "DDR_MC_7"},
	{0x40E00000, 0x40E00050, 0, "GPIO_1"},
	{0x40E00100, 0x40E00150, 0, "GPIO_2"},
	{0x40E00400, 0x40E004B4, 0, "GPIO_3"},
	{0x41500000, 0x41500048, 0, "Keypad"},
	{0x4240400C, 0x42404014, 0, "TrackBall"},
	{0x41000000, 0x41000040, 0, "SSP1"},
	{0x41700000, 0x41700040, 0, "SSP2"},
	{0x41A00000, 0x41A00090, 0, "ABU"}, /* include "ABU_SSP" */
	{0x40100000, 0x4010002C, 0, "UART"},
	{0x41B00000, 0x41B00010, 0, "1Wire"},
	{0x41D00000, 0x41D00024, 0, "CIR"},
	{0x40B00000, 0x40B00018, 0, "PWM_1"},
	{0x40C00000, 0x40C00018, 0, "PWM_2"},
	{0x42404020, 0x4240406C, 0, "PWM_3"},
	{0x40301680, 0x403016E0, 0, "I2C_1"},
	{0x40401680, 0x404016E0, 0, "I2C_2"},
	{0x40801680, 0x408016E0, 0, "I2C_3"},
	{0x55000000, 0x55000118, 0, "SD1"},
	{0x55100000, 0x55100118, 0, "SD2"},
	{0x55200000, 0x55200118, 0, "SD3"},
	{0x55300000, 0x55300118, 0, "SD4"},
	{0x55502000, 0x555021FC, 0, "USB_OTG_1"},
	{0x55502300, 0x55502308, 0, "USB_OTG_2"},
	{0x55503000, 0x555031FC, 0, "USB_HOST_1"},
	{0x55503300, 0x55503308, 0, "USB_HOST_2"},
	{0x55509F00, 0x55509F00, 0, "USB_OTG_IPD"},
	{0x5550AF00, 0x5550AF00, 0, "USB_HST_IPD"},
	{0x42405E00, 0x42405E7C, 0, "USB_PHY"},
	{0x50000000, 0x5000000C, 0, "SCI0_1"},
	{0x500000F8, 0x500000FC, 0, "SCI0_2"},
	{0x50000200, 0x5000022C, 0, "SCI0_3"},
	{0x50000300, 0x50000324, 0, "SCI0_4"},
	{0x50010000, 0x5001000C, 0, "SCI1_1"},
	{0x500100F8, 0x500100FC, 0, "SCI1_2"},
	{0x50010200, 0x5001022C, 0, "SCI1_3"},
	{0x50010300, 0x50010324, 0, "SCI1_4"},
	{0x50020000, 0x5002003C, 0, "CSI_0"},
	{0x50022000, 0x5002203C, 0, "CSI_1"},
	{0x44100000, 0x44100014, 0, "Display_1"},
	{0x44100100, 0x44100110, 0, "Display_2"},
	{0x44100200, 0x441003A0, 0, "Display_3"},
	{0x44100580, 0x441007DC, 0, "Display_4"},
	{0x44100800, 0x44100848, 0, "Display_5"},
	{0x44100900, 0x44100948, 0, "Display_6"},
	{0x44100A00, 0x44100A48, 0, "Display_7"},
	{0x44101000, 0x44101030, 0, "Display_8"},
	{0x44103000, 0x44103168, 0, "DSI"},
	{0x42404160, 0x4240416C, 0, "DSI_PLL"},
	{0x44108000, 0x44108150, 0, "HDMI_Genl_1"},
	{0x44108200, 0x44108208, 0, "HDMI_Genl_2"},
	{0x44108340, 0x4410839c, 0, "HDMI_Genl_3"},
	{0x44104000, 0x44104064, 0, "HDMI_Conv"},
	{0x4410E200, 0x4410E288, 0, "HDMI_CEC"},
	{0x4411A060, 0x4411A0E0, 0, "HDMI_CMU"},
	{0x4410C000, 0x4410C02C, 0, "HDMI_Intl"},
	{0x424041B0, 0x424041DC, 0, "HDMI_PLL"},
	{0x41900000, 0x419000BC, 0, "HDMI_SSP"},
	{0         , 0         , 0, NULL   }
};

static void ioremap_reg_map(struct reg_map map[])
{
	unsigned int i;
	for (i = 0; map[i].reg_name != NULL; i++) {
		if ((map[i].vir_addr == 0) && (map[i].end_addr >= map[i].beg_addr)) {

			map[i].vir_addr = (unsigned int)ioremap(map[i].beg_addr
					, map[i].end_addr + 4 - map[i].beg_addr);

			if (map[i].vir_addr == 0)
				printk("ioremap(0x%08x, %d) failed!\n", map[i].beg_addr,
					map[i].end_addr + 4 - map[i].beg_addr);
		}
	}
}

static void iounmap_reg_map(struct reg_map map[])
{
	unsigned int i;
	for (i = 0; map[i].reg_name != NULL; i++) {
		if ((map[i].vir_addr != 0)) {
			iounmap((volatile void *)map[i].vir_addr);
		}
	}
}

void dump_soc_regs(struct reg_map map[])
{
	unsigned int i, j, k;

	unsigned int total = 0;
	for (i = 0; map[i].reg_name != NULL; i++) {

		printk("\n%-15s\t start:%08x end:%08x size=%d\n",
			map[i].reg_name, map[i].beg_addr, map[i].end_addr,
			map[i].end_addr - map[i].beg_addr + 4);

		total += (map[i].end_addr - map[i].beg_addr + 4);

		if (unlikely(map[i].vir_addr == 0)) {
			printk("%s: NULL virtual address!\n",map[i].reg_name);
			continue;
		}

		for(j = map[i].beg_addr; j <= map[i].end_addr; j += 32) {

			printk("%08x:", j);

			for (k=j; (k <= map[i].end_addr) && (k < j + 32); k += 4)
				printk(" %08x", __raw_readl(k - map[i].beg_addr + map[i].vir_addr));

			printk("\n");
		}
	}

	printk("total dumped reg size = %d\n", total);
}



void dump_stack_and_func(void)
{
	register unsigned long current_sp asm ("sp");
	unsigned long flag;
	unsigned long *cur_stack;
	unsigned long *copy_stack;
	unsigned long *top_stack;
	extern unsigned int _stext, _etext;
	char *thread_name;
	pid_t cur_pid;

	local_irq_save(flag);
	cur_stack = (unsigned long *)current_sp;
	copy_stack = (unsigned long *)current_sp;
	thread_name = current->comm;
	cur_pid = current->pid;
	top_stack = (unsigned long *)((current_sp
			& (~(THREAD_SIZE - 1))) + THREAD_SIZE);
	local_irq_restore(flag);

	printk("\nStart to print kernel stack and func in stack!\n");
	printk("Thread_name:%s PID:%d sp=%lx, Kernel_stack@%lx-to-%lx\n",
		thread_name, cur_pid, (unsigned long)cur_stack,
		(unsigned long)cur_stack & (~(THREAD_SIZE - 1)),
		(unsigned long )top_stack);

	cur_stack = (unsigned long *)((unsigned long)cur_stack & (~31));
	while(cur_stack < top_stack) {
		printk("%08lx: %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n",
			(unsigned long)cur_stack, *cur_stack, *(cur_stack + 1),
			*(cur_stack + 2), *(cur_stack + 3), *(cur_stack + 4),
			*(cur_stack + 5),*(cur_stack + 6),*(cur_stack + 7));

		cur_stack += 8;
	}

	printk("\n_stext:0x%x, _etext:0x%x\n", (unsigned int)&_stext, (unsigned int)&_etext);
	printk("\nPossible func in current stack, please check them manually\n");

	while(copy_stack <= top_stack) {
		if ((*copy_stack < (unsigned int)&_etext) && (*copy_stack > (unsigned int)&_stext))
		printk("[<%08lx>] (%pS)\n", (*copy_stack), (void *)(*copy_stack));
		copy_stack += 1;
	}
	printk("\nEnd of %s()\n", __func__);
}

static int __init dump_regs_init(void)
{
	printk(KERN_INFO "dump_regs_init()!\n");
	ioremap_reg_map(pxa_reg_map);
	return 0;
}

static void __exit dump_regs_exit(void)
{
	iounmap_reg_map(pxa_reg_map);
}

arch_initcall(dump_regs_init);
__exitcall(dump_regs_exit);
