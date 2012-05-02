/*
 * linux/include/asm-arm/arch-pxa/pxa_u2o.h
 *
 * This supports machine-specific differences in how the PXA
 * USB 2.0 Device Controller (U2O) is wired.
 *
 * (C) Copyright 2008 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __ASM_ARCH_PXA_U2O_H
#define __ASM_ARCH_PXA_U2O_H

/* PHY registers */
#define U2PPLL		(0x000)       /* U2O PHY PLL Control */
#define U2PTX		(0x004)       /* U2O PHY TX Control */
#define U2PRX		(0x008)       /* U2O PHY RX Control */
#define U2IVREF		(0x00C)       /* U2O PHY IVREF Control */
#define U2PT0		(0x010)       /* U2O PHY Test 0 Control */
#define U2PT1		(0x014)       /* U2O PHY Test 1 Control */
#define U2PT2		(0x018)       /* U2O PHY Test 2 Control */
#define U2PT3		(0x01C)       /* U2O PHY Test 3 Control */
#define U2PT4		(0x020)       /* U2O PHY Test 4 Control */
#define U2PT5		(0x024)       /* U2O PHY Test 5 Control */
#define U2PID		(0x028)       /* U2O PHY ID Register */
#define U2PRS		(0x02C)       /* U2O PHY Reserve Register */
#define U2PMN		(0x030)       /* U2O PHY Monitor Register */
#define U2OCG		(0x108)       /* U2O Clock Gate Register */

#define PXA935_U2O_REGBASE  (0x55502000)
#define PXA935_U2O_PHYBASE  (0x5550a000)

#define USB_REG_RANGE		(0x1ff)
#define USB_PHY_RANGE		(0xff)

#define USB_PHY_MODULE_BASE_ADDR     (0x42405E00)

/* usb_phy_module bits */
#define USB2_PLL_FBDIV_SHIFT                           0
#define USB2_PLL_REFDIV_SHIFT                          8
#define USB2_PLL_REFDIV_SHIFT_NEW_PHY                  9

#define USB2_PLL_VDD12_SHIFT                           12
#define USB2_PLL_VDD18_SHIFT                           14


#define USB2_PLL_FBDIV_MASK                            0x00FF
#define USB2_PLL_REFDIV_MASK                           0x0F00


#define USB2_PLL_FBDIV_MASK_NEW_PHY                    0x01FF
#define USB2_PLL_REFDIV_MASK_NEW_PHY                   0x3E00


#define USB2_PLL_PLLVDD12_MASK                         (0x3 << 12)
#define USB2_PLL_PLLVDD18_MASK                         (0x3 << 14)

#define USB2_PLL_CAL12_SHIFT                            0
#define USB2_PLL_VCOCALL_START                          2
#define USB2_PLL_CLK_BLK_EN_SHIFT                       3
#define USB2_PLL_KVCO_SHIFT                             4
#define USB2_PLL_KVCO_EXE_SHIFT                         7
#define USB2_PLL_ICP_SHIFT                              8
#define USB2_PLL_LOCKDET_ISEL_SHIFT                     11
#define USB2_PLL_LOCK_BYPASS_SHIFT                      12
#define USB2_PLL_PU_PLL_SHIFT                           13
#define USB2_PLL_CONTROL_BY_PIN_SHIFT                   14
#define USB2_PLL_READY_SHIFT                            15
#define USB2_PLL_CALI12_MASK                            (0x3)
#define USB2_PLL_KVCO_MASK                              (0x7 << 4)
#define USB2_PLL_ICP_MASK                               (0x7 << 8)
#define USB2_PLL_LOCK_BYPASS_MASK                       (0x1 << 12)
#define USB2_PLL_PU_PLL_MASK                            (0x1 << 13)
#define USB2_PLL_CONTROL_BY_PIN_MASK                    (0x1 << 14)
#define USB2_PLL_READY_MASK                             (0x1 << 15)

#define USB2_TX_EXT_FS_RCAL_SHIFT                       0
#define USB2_TX_HS_RCAL_SHIFT                           4
#define USB2_TX_IMPCAL_VTH_SHIFT                        8
#define USB2_TX_EXT_FS_RCAL_EN                          11
#define USB2_TX_EXT_HS_RCAL_EN_SHIFT                    12
#define USB2_TX_RCAL_START_SHIFT                        13
#define USB2_TX_DATA_BLOCK_EN                           14
#define USB2_TX_EXT_FS_RCAL_MASK                        (0xf)
#define USB2_TX_HS_RCAL_MASK                            (0xf << 4)
#define USB2_TX_IMPCAL_VTH_MASK                         (0x7 << 8)
#define USB2_TX_EXT_FS_RCAL_EN_MASK                     (0x1 << 11)
#define USB2_TX_EXT_HS_RCAL_EN_MASK                     (0x1 << 12)
#define USB2_TX_RCAL_START_MASK                         (0x1 << 13)
#define USB2_TX_DATA_BLOCK_EN_MASK                      (0x1 << 14)

#define USB2_TX_CK60_PHSEL_SHIFT                         0
#define USB2_TX_AMP_SHIFT                                4
#define USB2_TX_LOW_VDD_EN_SHIFT                         7
#define USB2_TX_VDD12_SHIFT                              8
#define USB2_TX_VDD15_SHIFT                              10
#define USB2_TX_CK60_PHSEL_MASK                          (0xf)
#define USB2_TX_AMP_MASK                                 (0x7 << 4)
#define USB2_TX_LOWVDD_MASK                              (0x1 << 7)
#define USB2_TX_VDD12_MASK                               (0x3 << 8)
#define USB2_TX_VDD15_MASK                               (0x3 << 10)

#define USB2_RX_INTP_SHIFT                               0
#define USB2_RX_LPF_COEF_SHIFT                           2
#define USB2_RX_SQ_THRESH_SHIFT                          4
#define USB2_RX_DISCON_THRESH_SHITF                      8
#define USB2_RX_SQ_LENGTH_SHIFT                          10
#define USB2_RX_ACQ_LENGTH_SHIFT                         12
#define USB2_RX_USQ_LENGTH_SHIFT                         14
#define USB2_RX_PHASE_FREEZE_DLY_SHIFT                   15
#define USB2_RX_INTP_MASK                                (0x3)
#define USB2_RX_LPF_COEF_MAKS                            (0x3 << 2)
#define USB2_RX_SQ_THRESH_MASK                           (0xf << 4)
#define USB2_RX_DISCON_THREASH_MASK                      (0x3 << 8)
#define USB2_RX_SQ_LENGTH_MASK                           (0x3 << 10)
#define USB2_RX_ACQ_LENGTH_MASK                          (0x3 << 12)
#define USB2_RX_USQ_LENGTH_MASK                          (0x1 << 14)
#define USB2_RX_PHASE_FREEZE_DLY_MASK                    (0x1 << 15)

#define USB2_RX_S2T03_DLY_SEL_SHITF                      0
#define USB2_RX_CDR_FASTLOCK_EN_SHIFT                    2
#define USB2_RX_CDR_COEF_SEL_SHIFT                       3
#define USB2_RX_EDGE_DET_SEL_SHIFT                       4
#define USB2_RX_DATA_BLOCK_LENGTH_SHIFT                  6
#define USB2_RX_CAP_SEL_SHIFT                            8
#define USB2_RX_EDGE_DET_EN_SHIFT                        11
#define USB2_RX_DATA_BLOCK_EN_SHIFT                      12
#define USB2_RX_EARLY_VOS_ON_EN_SHIFT                    13
#define USB2_RX_S2T03_DLY_SEL_MASK                       (0x3)
#define USB2_RX_CDR_FASTLOCK_EN_MASK                     (0x1 << 2)
#define USB2_RX_CDR_COEF_SEL_MASK                        (0x1 << 3)
#define USB2_RX_EDGE_DET_SEL_MASK                        (0x3 << 4)
#define USB2_RX_DATA_BLOCK_LENGTH_MASK                   (0x3 << 6)
#define USB2_RX_CAP_SEL_MASK                             (0x7 << 8)
#define USB2_RX_EDGE_DET_EN_MASK                         (0x1 << 11)
#define USB2_RX_DATA_BLOCK_EN_MASK                       (0x1 << 12)
#define USB2_RX_EARLY_VOS_ON_EN_MASK                     (0x1 << 13)

#define USB2_RX_VDD12_SHIFT                              (0)
#define USB2_RX_VDD18_SHIFT                              (2)
#define USB2_RX_SQ_ALWAYS_ON_SHIFT                       (4)
#define USB2_RX_SQ_BUFFER_EN_SHIFT                       (5)
#define USB2_RX_SAMPLER_CTRL_SHIFT                       (6)
#define USB2_RX_SQ_CM_SHIFT                              (7)
#define USB2_RX_USQ_FILTER_SHIFT                         (8)
#define USB2_RX_VDD12_MASK                               (0x3)
#define USB2_RX_VDD18_MASK                               (0x3 << 2)
#define USB2_RX_SQ_ALWAYS_ON_MASK                        (0x1 << 4)
#define USB2_RX_SQ_BUFFER_EN_MASK                        (0x1 << 5)
#define USB2_RX_SAMPLER_CTRL_MASK                        (0x1 << 6)
#define USB2_RX_SQ_CM_MASK                               (0x1 << 7)
#define USB2_RX_USQ_FILTER_MASK                          (0x1 << 8)

#define USB2_ANA_PU_ANA_SHIFT                            14
#define USB2_R_ROTATE_SEL                                7
#define USB2_R_ROTATE_SELMASK                            (0x1 << 7)
#define USB2_OTG_PU_OTG_SHIFT                            3

#define USB2_CTRL_PWR_UP_SHIFT                           0
#define USB2_CTRL_PLL_PWR_UP_SHIFT                       1
#define USB2_CTRL_PU_REF_SHIFT                           20

typedef struct _usb_phy_module
{
    unsigned int   PHYID;            //0x00 - PHY Identification Register
    unsigned int   PLL_CTRL_0;       //0x04 - PLL Control Register0
    unsigned int   PLL_CTRL_1;       //0x08 - PLL Control Register1
    unsigned int   PLL_CTRL_2;       //0x0C - PLL Control Register2
    unsigned int   TX_CH_CTRL_0;     //0x10 - TX Channel Control register0
    unsigned int   TX_CH_CTRL_1;     //0x14 - TX Channel Control register1
    unsigned int   TX_CH_CTRL_2;     //0x18 - TX Channel Control register2
    unsigned int   TX_CH_CTRL_3;     //0x1C - TX Channel Control register3
    unsigned int   RX_CH_CTRL_0;     //0x20 - RX Channel Control register0
    unsigned int   RX_CH_CTRL_1;     //0x24 - RX Channel Control register0
    unsigned int   RX_CH_CTRL_2;     //0x28 - RX Channel Control register0
    unsigned int   RX_CH_CTRL_3;     //0x2C - RX Channel Control register0
    unsigned int   ANALOG_CTRL_0;    //0x30 - Analog Top Control register0
    unsigned int   ANALOG_CTRL_1;    //0x34 - Analog Top Control register1
    unsigned int   ANALOG_CTRL_2;    //0x38 - Analog Top Control register2
    unsigned int   DIGITAL_CTRL_0;   //0x3C - Digital Control register0
    unsigned int   DIGITAL_CTRL_1;   //0x40 - Digital Control register1
    unsigned int   DIGITAL_CTRL_2;   //0x44 - Digital Control register2
    unsigned int   DIGITAL_CTRL_3;   //0x48 - Digital Control register3
    unsigned int   TEST_CTRL_0 ;     //0x4C - Test Control and status register0
    unsigned int   TEST_CTRL_1 ;     //0x50 - Test Control and status register1
    unsigned int   TEST_CTRL_2 ;     //0x54 - Test Control and status register2
    unsigned int   CHARGERDTCT_CTRL; //0x58 - Charger detector control register
    unsigned int   OTG_CTRL_0;       //0x5C - OTG control register0
    unsigned int   PHY_MON_STATUS;   //0x60 - Read PHY_MON[15:0]
    unsigned int   DIGITAL_CTRL_4;   //0x64 - Digital Control register4
    unsigned int   reserved1[3];     //0x68 - 0x74 - reserved
    unsigned int   INTERNALCID_0;    //0x78 - usb2_icid_reg0
    unsigned int   INTERNALCID_1;    //0x7C - usb2_icid_reg1
} usb_phy_module;

#endif /* __ASM_ARCH_PXA_U2O_H */
