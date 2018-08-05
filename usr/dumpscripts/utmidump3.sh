usbphy ()
{
	echo === $1 ===
	while read a n; do
		p=$(( $a + $2 ))
		printf "%-30s 0x%08x %s\n" $1_$n $p "$(busybox devmem $p 2>&1)"
	done <<EOF
0x04   USB2_PLL_REG0
0x08   USB2_PLL_REG1
0x10   USB2_TX_REG0
0x14   USB2_TX_REG1
0x18   USB2_TX_REG2
0x20   USB2_RX_REG0
0x24   USB2_RX_REG1
0x28   USB2_RX_REG2
0x30   USB2_ANA_REG0
0x34   USB2_ANA_REG1
0x38   USB2_ANA_REG2
0x3C   USB2_DIG_REG0
0x40   USB2_DIG_REG1
0x44   USB2_DIG_REG2
0x48   USB2_DIG_REG3
0x4C   USB2_TEST_REG0
0x50   USB2_TEST_REG1
0x54   USB2_TEST_REG2
0x58   USB2_CHARGER_REG0
0x5C   USB2_OTG_REG0
0x60   USB2_PHY_MON0
0x64   USB2_RESETVE_REG0
0x78   USB2_ICID_REG0
0x7C   USB2_ICID_REG1
EOF
	echo
}

hsic ()
{
	echo === $1 ===
	while read a n; do
		p=$(( $a + $2 ))
		printf "%-30s 0x%08x %s\n" $1_$n $p "$(busybox devmem $p 2>&1)"
	done <<EOF
0x4	HSIC_PAD_CTRL
0x8	HSIC_CTRL
0xc	TEST_GRP_0
0x10	TEST_GRP_1
0x14	HSIC_INT
0x18	HSIC_CONFIG
0x20	USBHSIC_CTRL
0x28	HSIC_USB_CTRL
EOF
	echo
}

fsic ()
{
	echo === $1 ===
	while read a n; do
		p=$(( $a + $2 ))
		printf "%-30s 0x%08x %s\n" $1_$n $p "$(busybox devmem $p 2>&1)"
	done <<EOF
0x4	FSIC_MISC
0x28	FSIC_INT
0x30	FSIC_CTRL
EOF
	echo
}

utmi ()
{
	echo === $1 ===
	while read a n; do
		p=$(( $a + $2 ))
		printf "%-26s 0x%08x %s\n" $1_$n $p "$(busybox devmem $p 2>&1)"
	done <<EOF
0x00   UTMI_REVISION
0x04   UTMI_CTRL
0x08   UTMI_PLL
0x0c   UTMI_TX
0x10   UTMI_RX
0x14   UTMI_IVREF
0x18   UTMI_T0
0x1c   UTMI_T1
0x20   UTMI_T2
0x24   UTMI_T3
0x28   UTMI_T4
0x2c   UTMI_T5
0x30   UTMI_RESERVE
0x34   UTMI_USB_INT
0x38   UTMI_DBG_CTL
0x3c   UTMI_OTG_ADDON
EOF
	echo
}

usbphy PXA168_U2O 0xd4207000
#utmi   PXA168_U2H 0xd4206000
hsic   MMP3_HSIC1 0xf0001800
#hsic   MMP3_HSIC2 0xf0002800
fsic   MMP3_FSIC  0xf0003800
