usbphy ()
{
	echo === $1 ===
	while read a n; do
		p=$(( $a + $2 ))
		printf "%-26s 0x%08x %s\n" $1_$n $p "$(busybox devmem $p 2>&1)"
		#printf "%-26s 0x%08x %s\n" $1_$n $p 666
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
#usbphy PXA168_U2H 0xd4206000 # unused anywhere?
