/*
 * MMP2 Power Management Routines
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2010 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __MMP2_PM_H__
#define __MMP2_PM_H__

#include <mach/regs-mpmu.h>
#include <mach/regs-apmu.h>

#define	MICRON_100		0
#define	MICRON_200		1
#define	MICRON_400		2
#define	ELPIDA_100		3
#define	ELPIDA_200		4
#define	ELPIDA_400		5
#define	ELPIDA_1G_100		6
#define	ELPIDA_1G_200		7
#define	ELPIDA_1G_400		8
#define ELPIDA_512M_100		9
#define ELPIDA_512M_200		10
#define ELPIDA_512M_400		11

#define BIT_0 (1 << 0)
#define BIT_1 (1 << 1)
#define BIT_2 (1 << 2)
#define BIT_3 (1 << 3)
#define BIT_4 (1 << 4)
#define BIT_5 (1 << 5)
#define BIT_6 (1 << 6)
#define BIT_7 (1 << 7)
#define BIT_8 (1 << 8)
#define BIT_9 (1 << 9)
#define BIT_10 (1 << 10)
#define BIT_11 (1 << 11)
#define BIT_12 (1 << 12)
#define BIT_13 (1 << 13)
#define BIT_14 (1 << 14)
#define BIT_15 (1 << 15)
#define BIT_16 (1 << 16)
#define BIT_17 (1 << 17)
#define BIT_18 (1 << 18)
#define BIT_19 (1 << 19)
#define BIT_20 (1 << 20)
#define BIT_21 (1 << 21)
#define BIT_22 (1 << 22)
#define BIT_23 (1 << 23)
#define BIT_24 (1 << 24)
#define BIT_25 (1 << 25)
#define BIT_26 (1 << 26)
#define BIT_27 (1 << 27)
#define BIT_28 (1 << 28)
#define BIT_29 (1 << 29)
#define BIT_30 (1 << 30)
#define BIT_31 ((unsigned)1 << 31)

#define SHIFT0(Val)  (Val)
#define SHIFT1(Val)  ((Val) << 1)
#define SHIFT2(Val)  ((Val) << 2)
#define SHIFT3(Val)  ((Val) << 3)
#define SHIFT4(Val)  ((Val) << 4)
#define SHIFT5(Val)  ((Val) << 5)
#define SHIFT6(Val)  ((Val) << 6)
#define SHIFT7(Val)  ((Val) << 7)
#define SHIFT8(Val)  ((Val) << 8)
#define SHIFT9(Val)  ((Val) << 9)
#define SHIFT10(Val) ((Val) << 10)
#define SHIFT11(Val) ((Val) << 11)
#define SHIFT12(Val) ((Val) << 12)
#define SHIFT13(Val) ((Val) << 13)
#define SHIFT14(Val) ((Val) << 14)
#define SHIFT15(Val) ((Val) << 15)
#define SHIFT16(Val) ((Val) << 16)
#define SHIFT17(Val) ((Val) << 17)
#define SHIFT18(Val) ((Val) << 18)
#define SHIFT19(Val) ((Val) << 19)
#define SHIFT20(Val) ((Val) << 20)
#define SHIFT21(Val) ((Val) << 21)
#define SHIFT22(Val) ((Val) << 22)
#define SHIFT23(Val) ((Val) << 23)
#define SHIFT24(Val) ((Val) << 24)
#define SHIFT25(Val) ((Val) << 25)
#define SHIFT26(Val) ((Val) << 26)
#define SHIFT27(Val) ((Val) << 27)
#define SHIFT28(Val) ((Val) << 28)
#define SHIFT29(Val) ((Val) << 29)
#define SHIFT30(Val) ((Val) << 30)
#define SHIFT31(Val) ((Val) << 31)

/*
 * pmua registers and bits definition
 */
#define	CC_SEA_OFF				0x0000
#define	CC_MOH_OFF				0x0004
#define	DM_CC_SEA_OFF				0x0008
#define	DM_CC_MOH_OFF				0x000C
#define	MOH_IMR_OFF				0x0098
#define	MOH_ISR_OFF				0x00A0
#define MC_SLP_REQ_OFF				0x00B4
#define PLL_SEL_OFF				0x00C4
#define	DEBUG_REG				0x0088

#define	PMUA_CC_SEA_SEA_RD_ST_CLEAR		BIT_31
#define	PMUA_CC_SEA_ACLK_DYN_FC			BIT_30
#define	PMUA_CC_SEA_DCLK_DYN_FC			BIT_29
#define	PMUA_CC_SEA_CORE_DYN_FC			BIT_28
#define	PMUA_CC_SEA_SEA_ALLOW_SPD_CHG		BIT_27
#define	PMUA_CC_SEA_BUS_FREQ_CHG_REQ		BIT_26
#define	PMUA_CC_SEA_DDR_FREQ_CHG_REQ		BIT_25
#define	PMUA_CC_SEA_SEA_FREQ_CHG_REQ		BIT_24
#define	PMUA_CC_SEA_ASYNC5			BIT_23
#define	PMUA_CC_SEA_ASYNC4			BIT_22
#define	PMUA_CC_SEA_ASYNC3_1			BIT_21
#define	PMUA_CC_SEA_ASYNC3			BIT_20
#define	PMUA_CC_SEA_ASYNC2			BIT_19
#define	PMUA_CC_SEA_ASYNC1			BIT_18
#define	PMUA_CC_SEA_BUS_CLK_DIV_MSK		SHIFT15(0x7)
#define	PMUA_CC_SEA_BUS_CLK_DIV_BASE		15
#define	PMUA_CC_SEA_DDR_CLK_DIV_MSK		SHIFT12(0x7)
#define	PMUA_CC_SEA_DDR_CLK_DIV_BASE		12
#define	PMUA_CC_SEA_XP_CLK_DIV_MSK		SHIFT9(0x7)
#define	PMUA_CC_SEA_XP_CLK_DIV_BASE		9
#define	PMUA_CC_SEA_BIU_CLK_DIV_MSK		SHIFT6(0x7)
#define	PMUA_CC_SEA_BIU_CLK_DIV_BASE		6
#define	PMUA_CC_SEA_BUS_MC_CLK_DIV_MSK		SHIFT3(0x7)
#define	PMUA_CC_SEA_BUS_MC_CLK_DIV_BASE		3
#define	PMUA_CC_SEA_CORE_CLK_DIV_MSK		SHIFT0(0x7)
#define	PMUA_CC_SEA_CORE_CLK_DIV_BASE		0

#define	PMUA_CC_MOH_MOH_RD_ST_CLEAR		BIT_31
#define	PMUA_CC_MOH_ACLK_DYN_FC			BIT_30
#define	PMUA_CC_MOH_DCLK_DYN_FC			BIT_29
#define	PMUA_CC_MOH_CORE_DYN_FC			BIT_28
#define	PMUA_CC_MOH_MOH_ALLOW_SPD_CHG		BIT_27
#define	PMUA_CC_MOH_BUS_FREQ_CHG_REQ		BIT_26
#define	PMUA_CC_MOH_DDR_FREQ_CHG_REQ		BIT_25
#define	PMUA_CC_MOH_MOH_FREQ_CHG_REQ		BIT_24
#define	PMUA_CC_MOH_ASYNC5			BIT_23
#define	PMUA_CC_MOH_ASYNC4			BIT_22
#define	PMUA_CC_MOH_ASYNC3_1			BIT_21
#define	PMUA_CC_MOH_ASYNC3			BIT_20
#define	PMUA_CC_MOH_ASYNC2			BIT_19
#define	PMUA_CC_MOH_ASYNC1			BIT_18
#define	PMUA_CC_MOH_BUS_2_CLK_DIV_BASE		18
#define	PMUA_CC_MOH_BUS_CLK_DIV_MSK		SHIFT15(0x7)
#define	PMUA_CC_MOH_BUS_CLK_DIV_BASE		15
#define	PMUA_CC_MOH_DDR_CLK_DIV_MSK		SHIFT12(0x7)
#define	PMUA_CC_MOH_DDR_CLK_DIV_BASE		12
#define	PMUA_CC_MOH_XP_CLK_DIV_MSK		SHIFT9(0x7)
#define	PMUA_CC_MOH_XP_CLK_DIV_BASE		9
#define	PMUA_CC_MOH_BIU_CLK_DIV_MSK		SHIFT6(0x7)
#define	PMUA_CC_MOH_BIU_CLK_DIV_BASE		6
#define	PMUA_CC_MOH_BUS_MC_CLK_DIV_MSK		SHIFT3(0x7)
#define	PMUA_CC_MOH_BUS_MC_CLK_DIV_BASE		3
#define	PMUA_CC_MOH_CORE_CLK_DIV_MSK		SHIFT0(0x7)
#define	PMUA_CC_MOH_CORE_CLK_DIV_BASE		0

#define	PMUA_DM_CC_SEA_MOH_RD_STATUS		BIT_25
#define	PMUA_DM_CC_SEA_SEA_RD_STATUS		BIT_24
#define	PMUA_DM_CC_SEA_ASYNC5			BIT_23
#define	PMUA_DM_CC_SEA_ASYNC4			BIT_22
#define	PMUA_DM_CC_SEA_ASYNC3_1			BIT_21
#define	PMUA_DM_CC_SEA_ASYNC3			BIT_20
#define	PMUA_DM_CC_SEA_ASYNC2			BIT_19
#define	PMUA_DM_CC_SEA_ASYNC1			BIT_18
#define	PMUA_DM_CC_SEA_BUS_CLK_DIV_MSK		SHIFT15(0x7)
#define	PMUA_DM_CC_SEA_BUS_CLK_DIV_BASE		15
#define	PMUA_DM_CC_SEA_DDR_CLK_DIV_MSK		SHIFT12(0x7)
#define	PMUA_DM_CC_SEA_DDR_CLK_DIV_BASE		12
#define	PMUA_DM_CC_SEA_XP_CLK_DIV_MSK		SHIFT9(0x7)
#define	PMUA_DM_CC_SEA_XP_CLK_DIV_BASE		9
#define	PMUA_DM_CC_SEA_BIU_CLK_DIV_MSK		SHIFT6(0x7)
#define	PMUA_DM_CC_SEA_BIU_CLK_DIV_BASE		6
#define	PMUA_DM_CC_SEA_BUS_MC_CLK_DIV_MSK	SHIFT3(0x7)
#define	PMUA_DM_CC_SEA_BUS_MC_CLK_DIV_BASE	3
#define	PMUA_DM_CC_SEA_CORE_CLK_DIV_MSK		SHIFT0(0x7)
#define	PMUA_DM_CC_SEA_CORE_CLK_DIV_BASE	0

#define	PMUA_DM_CC_MOH_MOH_RD_STATUS		BIT_25
#define	PMUA_DM_CC_MOH_SEA_RD_STATUS		BIT_24
#define	PMUA_DM_CC_MOH_ASYNC5			BIT_23
#define	PMUA_DM_CC_MOH_ASYNC4			BIT_22
#define	PMUA_DM_CC_MOH_ASYNC3_1			BIT_21
#define	PMUA_DM_CC_MOH_ASYNC3			BIT_20
#define	PMUA_DM_CC_MOH_ASYNC2			BIT_19
#define	PMUA_DM_CC_MOH_ASYNC1			BIT_18
#define	PMUA_DM_CC_MOH_BUS_CLK_DIV_MSK		SHIFT15(0x7)
#define	PMUA_DM_CC_MOH_BUS_CLK_DIV_BASE		15
#define	PMUA_DM_CC_MOH_DDR_CLK_DIV_MSK		SHIFT12(0x7)
#define	PMUA_DM_CC_MOH_DDR_CLK_DIV_BASE		12
#define	PMUA_DM_CC_MOH_XP_CLK_DIV_MSK		SHIFT9(0x7)
#define	PMUA_DM_CC_MOH_XP_CLK_DIV_BASE		9
#define	PMUA_DM_CC_MOH_BIU_CLK_DIV_MSK		SHIFT6(0x7)
#define	PMUA_DM_CC_MOH_BIU_CLK_DIV_BASE		6
#define	PMUA_DM_CC_MOH_BUS_MC_CLK_DIV_MSK	SHIFT3(0x7)
#define	PMUA_DM_CC_MOH_BUS_MC_CLK_DIV_BASE	3
#define	PMUA_DM_CC_MOH_CORE_CLK_DIV_MSK		SHIFT0(0x7)
#define	PMUA_DM_CC_MOH_CORE_CLK_DIV_BASE	0

#define	PMUA_MOH_IMR_MOH_FC_INTR_MASK		BIT_1
#define	PMUA_MOH_IMR_SEA_FC_INTR_MASK		BIT_0

#define	PMUA_MOH_ISR_MOH_FC_ISR			BIT_1
#define	PMUA_MOH_ISR_SEA_FC_ISR			BIT_0

#define PMUA_PLL_SEL_CORE_PLL_SEL		SHIFT2(0x3)
#define PMUA_PLL_SEL_CORE_PLL_BASE		2
#define PMUA_PLL_SEL_DDR_PLL_SEL		SHIFT4(0x3)
#define PMUA_PLL_SEL_DDR_PLL_BASE		4
#define PMUA_PLL_SEL_BUS_PLL_SEL		SHIFT6(0x3)
#define PMUA_PLL_SEL_BUS_PLL_BASE		6

#define PMUA_MOH_DIS_MC_SW_REQ                 BIT_21
#define PMUA_MOH_MC_WAKE_EN                    BIT_20
#define PMUA_MOH_SRAM_PWRDWN                   BIT_6
#define PMUA_MOH_PWRDWN                                BIT_5
#define PMUA_MOH_IDLE                          BIT_1

/*
 * pmum registers and bits definition
 */
#define	FCCR_OFF				0x0008
#define	PLL2CR_OFF				0x0034
#define PLL2_CTRL1				0x0414
#define PLL1_CTRL				0x0418
#define ACGR_OFF				0x1024

#define	PMUM_FCCR_MOHCLKSEL_MSK			SHIFT29(0x7)
#define	PMUM_FCCR_MOHCLKSEL_BASE		29
#define	PMUM_FCCR_SEAGCLKSEL_MSK		SHIFT26(0x7)
#define	PMUM_FCCR_SEAGCLKSEL_BASE		26
#define	PMUM_FCCR_AXICLKSEL_MSK			SHIFT23(0x7)
#define	PMUM_FCCR_AXICLKSEL_BASE		23
#define	PMUM_FCCR_MFC				BIT_15
#define	PMUM_FCCR_PLL1CEN			BIT_14
#define	PMUM_FCCR_PLL1REFD_MSK			SHIFT9(0x1f)
#define	PMUM_FCCR_PLL1REFD_BASE			9
#define	PMUM_FCCR_PLL1FBD_MSK			SHIFT0(0x1ff)
#define	PMUM_FCCR_PLL1FBD_BASE			0

#define	PMUM_PLL2CR_FBDIV_MSK			SHIFT10(0x1ff)
#define PMUM_PLL2CR_FBDIV_BASE			10
#define	PMUM_PLL2CR_REFDIV_MSK			SHIFT19(0x1f)
#define PMUM_PLL2CR_REFDIV_BASE			19
#define PMUM_PLL2CR_CTRL			BIT_9;
#define PMUM_PLL2CR_PLL2_SW_EN			BIT_8;

#define PMUM_ACGR_PLL1				BIT_15
#define PMUM_ACGR_PLL2EN			BIT_14
#define PMUM_ACGR_PLL1_2			BIT_13

#define PMUM_AXISD                             BIT_31
#define PMUM_DSPSD                             BIT_30
#define PMUM_SLPEN                             BIT_29
#define PMUM_DTCMSD                            BIT_28
#define PMUM_DDRCORSD                          BIT_27
#define PMUM_APBSD                             BIT_26
#define PMUM_BBSD                              BIT_25
#define PMUM_INTCLR                            BIT_24
#define PMUM_SLPWP0                            BIT_23
#define PMUM_SLPWP1                            BIT_22
#define PMUM_SLPWP2                            BIT_21
#define PMUM_SLPWP3                            BIT_20
#define PMUM_VCTCXOSD                          BIT_19
#define PMUM_SLPWP4                            BIT_18
#define PMUM_SLPWP5                            BIT_17
#define PMUM_SLPWP6                            BIT_16
#define PMUM_SLPWP7                            BIT_15
#define PMUM_MSASLPEN                          BIT_14

#define PMUM_GSM_WAKEUPWMX                     BIT_29
#define PMUM_WCDMA_WAKEUPX                     BIT_28
#define PMUM_GSM_WAKEUPWM                      BIT_27
#define PMUM_WCDMA_WAKEUPWM                    BIT_26
#define PMUM_AP_ASYNC_INT                      BIT_25
#define PMUM_AP_FULL_IDLE                      BIT_24
#define PMUM_SDH1                              BIT_23
#define PMUM_SDH2                              BIT_22
#define PMUM_KEYPRESS                          BIT_21
#define PMUM_TRACKBALL                         BIT_20
#define PMUM_NEWROTARY                         BIT_19
#define PMUM_WDT                               BIT_18
#define PMUM_RTC_ALARM                         BIT_17
#define PMUM_AP2_TIMER_3                       BIT_16
#define PMUM_AP2_TIMER_2                       BIT_15
#define PMUM_AP2_TIMER_1                       BIT_14
#define PMUM_AP1_TIMER_3                       BIT_10
#define PMUM_AP1_TIMER_2                       BIT_9
#define PMUM_AP1_TIMER_1                       BIT_8
#define PMUM_WAKEUP7                           BIT_7
#define PMUM_WAKEUP6                           BIT_6
#define PMUM_WAKEUP5                           BIT_5
#define PMUM_WAKEUP4                           BIT_4
#define PMUM_WAKEUP3                           BIT_3
#define PMUM_WAKEUP2                           BIT_2
#define PMUM_WAKEUP1                           BIT_1
#define PMUM_WAKEUP0                           BIT_0

#define DMCU_CONFIG_DECODE_ADDR_OFFSET		0x0010
#define DMCU_MMAP0_OFFSET                       0x0100
#define DMCU_MMAP1_OFFSET                       0x0110
#define DMCU_MMAP2_OFFSET                       0x0130
#define DMCU_MMAP3_OFFSET                       0x0A30
#define DMCU_SDRAM_CONFIG_TYPE1_CS0_OFFSET      0x0020
#define DMCU_SDRAM_CONFIG_TYPE1_CS1_OFFSET      0x0030
#define DMCU_SDRAM_CONFIG_TYPE2_CS0_OFFSET      0x0B40
#define DMCU_SDRAM_CONFIG_TYPE2_CS1_OFFSET      0x0B50
#define DMCU_SDRAM_TIMING1_OFFSET               0x0050
#define DMCU_SDRAM_TIMING2_OFFSET               0x0060
#define DMCU_SDRAM_TIMING3_OFFSET               0x0190
#define DMCU_SDRAM_TIMING4_OFFSET               0x01C0
#define DMCU_SDRAM_TIMING5_OFFSET               0x0650
#define DMCU_SDRAM_TIMING6_OFFSET               0x0660
#define DMCU_SDRAM_CTRL1_OFFSET                 0x0080
#define DMCU_SDRAM_CTRL2_OFFSET                 0x0090
#define DMCU_SDRAM_CTRL3_OFFSET                 0x00F0
#define DMCU_SDRAM_CTRL4_OFFSET                 0x01A0
#define DMCU_SDRAM_CTRL5_ARB_WEIGHTS_OFFSET     0x0280
#define DMCU_SDRAM_CTRL6_SDRAM_ODT_CTRL_OFFSET  0x0760
#define DMCU_SDRAM_CTRL7_SDRAM_ODT_CTRL2_OFFSET 0x0770
#define DMCU_SDRAM_CTRL8_SDRAM_ODT_CTRL2_OFFSET 0x0780
#define DMCU_SDRAM_CTRL11_ARB_WEIGTHS_FAST_QUEUE_OFFSET	0x07B0
#define DMCU_SDRAM_CTRL13_OFFSET                0x07D0
#define DMCU_SDRAM_CTRL14_OFFSET                0x07E0
#define DMCU_MCB_CTRL4_OFFSET                   0x0540
#define DMCU_MCB_SLFST_SEL_OFFSET               0x0570
#define DMCU_MCB_SLFST_CTRL0_OFFSET             0x0580
#define DMCU_MCB_SLFST_CTRL1_OFFSET             0x0590
#define DMCU_MCB_SLFST_CTRL2_OFFSET             0x05A0
#define DMCU_MCB_SLFST_CTRL3_OFFSET             0x05B0
#define DMCU_PHY_CTRL3_OFFSET                   0x0140
#define DMCU_CM_WRITE_PROTECTION_OFFSET         0x0180
#define DMCU_PHY_CTRL7_OFFSET                   0x01D0
#define DMCU_PHY_CTRL8_OFFSET                   0x01E0
#define DMCU_PHY_CTRL9_OFFSET                   0x01F0
#define DMCU_PHY_CTRL10_OFFSET                  0x0200
#define DMCU_PHY_CTRL11_OFFSET                  0x0210
#define DMCU_PHY_CTRL13_OFFSET                  0x0230
#define DMCU_PHY_CTRL14_OFFSET                  0x0240
#define DMCU_PHY_CTRL15_OFFSET                  0x0250
#define DMCU_PHY_DLL_CTRL1_OFFSET               0x0E10
#define DMCU_PHY_DLL_CTRL2_OFFSET               0x0E20
#define DMCU_PHY_DLL_CTRL3_OFFSET               0x0E30
#define DMCU_PHY_CTRL_WL_SELECT_OFFSET          0x0E40
#define DMCU_PHY_CTRL_WL_CTRL0_OFFSET           0x0E50
#define DMCU_PHY_CTRL_TESTMODE_OFFSET           0x0E80
#define DMCU_USER_INITIATED_COMMAND0_OFFSET     0x0120
#define DMCU_USER_INITIATED_COMMAND1_OFFSET     0x0410
#define DMCU_MODE_RD_DATA_OFFSET                0x0440

#define DDR_SETTING_ENTRY_NUM			14
#define DDR_CONFIG_ENTRY_SIZE                   (4*5)
#define DDR_CONFIG_ENTRY_NUM                    54
#define DDR_CONFIG_MAX                          4

/* TODO
 */
#define DDR_CONFIG_TABLE_SETTING_SIZE		(1080 - 4)

#define EXIT_LATENCY_CORE_EXTIDLE		1
#define EXIT_LATENCY_APPS_IDLE			10
#define EXIT_LATENCY_APPS_SLEEP			20
#define EXIT_LATENCY_CHIP_SLEEP			100

enum {
	POWER_MODE_ACTIVE = 0,
	POWER_MODE_CORE_INTIDLE,
	POWER_MODE_CORE_EXTIDLE,
	POWER_MODE_APPS_IDLE,
	POWER_MODE_APPS_SLEEP,
	POWER_MODE_CHIP_SLEEP,
	POWER_MODE_SYS_SLEEP,
};

extern void mmp2_pm_enter_lowpower_mode(int state);

#endif
