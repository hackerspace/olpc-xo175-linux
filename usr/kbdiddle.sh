echo === kbd ===

for i in $(busybox seq 0xd4290040 4 0xd42900cf); do printf '%x: %x\n' $i $(busybox devmem $i); done |
sed '
s,\(d4290080\):,\1: COMMAND_RETURN_STATUS,;
s,\(d42900c4\):,\1: COMMAND_FIFO_STATUS,;
s,\(d42900c8\):,\1: PJ_RST_INTERRUPT,;
s,\(d42900cc\):,\1: PJ_INTERRUPT_MASK,;
';
echo

# Write 1 to PJ_RST_INTERRUPT to acknowledge and clear the interrupt
busybox devmem 0xd42900c8 32 0x1
# Write 0xff00 to SECURE_PROCESSOR_COMMAND.
busybox devmem 0xd4290040 32 0xff00
