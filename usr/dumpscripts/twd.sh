echo === twd ===
echo "TWD_TIMER_LOAD      0xe0000600 $(busybox devmem 0xe0000600)"
echo "TWD_TIMER_COUNTER   0xe0000604 $(busybox devmem 0xe0000604)"
echo "TWD_TIMER_CONTROL   0xe0000608 $(busybox devmem 0xe0000608)"
echo "TWD_TIMER_INTSTAT   0xe000060C $(busybox devmem 0xe000060C)"
echo
