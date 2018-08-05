twsi ()
{
	echo === twsi$1 ===
	while read a n; do
		p=$(( $a + $2 ))
		printf "%-12s 0x%08x %s\n" $n$1 $p $(busybox devmem $p)
	done <<EOF
0x0000 TWSI_BMR
0x0008 TWSI_DBR
0x0010 TWSI_CR
0x0018 TWSI_SR
0x0020 TWSI_SAR
0x0028 TWSI_LCR
0x0030 TWSI_WCR
0x0040 TWSI_WFIFO
0x0044 WFIFO_WPTR
0x0048 WFIFO_RPTR
0x0050 TWSI_RFIFO
0x0054 RFIFO_WPTR
0x0058 RFIFO_WPTR
EOF
	echo
}

twsi 0 0xd4011000
twsi 1 0xd4031000
twsi 3 0xd4033000
twsi 5 0xd4034000
