gic ()
{
	if [ "$3" ]; then
		for i in $(seq 0 $(( $3 - 1 ))); do
			p=$(( $i * 4 + $2 ))
			printf "%-30s 0x%08x %s\n" $1[$i] $p "$(busybox devmem $p 2>&1)"
		done
	else
		printf "%-30s 0x%08x %s\n" $1 $2 "$(busybox devmem $2 2>&1)"
	fi
}


#echo === gic_cpu ===

#0xe0000000
#define GIC_CPU_VIRT_BASE       (PGU_VIRT_BASE + 0x0100)

#echo "GIC_CPU_CTRL            0x80000100 $(busybox devmem 0x80000100)"
#echo "GIC_CPU_PRIMASK         0x80000104 $(busybox devmem 0x80000104)"
#echo "GIC_CPU_BINPOINT        0x80000108 $(busybox devmem 0x80000108)"
#echo "GIC_CPU_INTACK          0x8000010c $(busybox devmem 0x8000010c)"
#echo "GIC_CPU_EOI             0x80000110 $(busybox devmem 0x80000110)"
#echo "GIC_CPU_RUNNINGPRI      0x80000114 $(busybox devmem 0x80000114)"
#echo "GIC_CPU_HIGHPRI         0x80000118 $(busybox devmem 0x80000118)"
#echo "GIC_CPU_ALIAS_BINPOINT  0x8000011c $(busybox devmem 0x8000011c)"
#echo "GIC_CPU_ACTIVEPRIO      0x800001d0 $(busybox devmem 0x800001d0)"
#echo "GIC_CPU_IDENT           0x800001fc $(busybox devmem 0x800001fc)"
#echo

echo === gic_dist ===
gic GIC_DIST_CTRL               0xe0001000
gic GIC_DIST_CTR                0xe0001004
gic GIC_DIST_IIDR               0xe0001008
gic GIC_DIST_IGROUP             0xe0001080 32
gic GIC_DIST_ENABLE_SET         0xe0001100 32
gic GIC_DIST_ENABLE_CLEAR       0xe0001180 32
gic GIC_DIST_PENDING_SET        0xe0001200 32
gic GIC_DIST_PENDING_CLEAR      0xe0001280 32
gic GIC_DIST_ACTIVE_SET         0xe0001300 32
gic GIC_DIST_ACTIVE_CLEAR       0xe0001380 32
gic GIC_DIST_PRI                0xe0001400 32
gic GIC_DIST_TARGET             0xe0001800 32 
gic GIC_DIST_CONFIG             0xe0001c00 32
gic GIC_DIST_SOFTINT            0xe0001f00
gic GIC_DIST_SGI_PENDING_CLEAR  0xe0001f10 4
gic GIC_DIST_SGI_PENDING_SET    0xe0001f20 4
echo
