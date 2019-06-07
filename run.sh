# CROSS_COMPILE=arm-linux-gnu- ARCH=arm make -j32 vexpress_defconfig
# sed s/=m/=y/ -i .config
# CROSS_COMPILE=arm-linux-gnu- ARCH=arm make -j32
# http://dl-cdn.alpinelinux.org/alpine/v3.9/releases/armhf/alpine-minirootfs-3.9.4-armhf.tar.gz


# dd if=/dev/zero bs=1k count=8192 of=disk.img
# mkfs.ext4 disk.img 
# mount disk.img /mnt
# tar xzf alpine-minirootfs-3.9.4-armhf.tar.gz -C /mnt
#	-serial stdio \
#	-nographic \
#	-append 'init=/bin/sh root=/dev/mmcblk0 console=ttyAMA0 earlyprintk rw panic=1'
# umount /mnt

:<<:
=== set_handle_irq ===
------------[ cut here ]------------
WARNING: CPU: 0 PID: 0 at kernel/irq/handle.c:217 set_handle_irq+0x24/0x4c
Modules linked in:
CPU: 0 PID: 0 Comm: swapper/0 Not tainted 5.2.0-rc3+ #5
Hardware name: ARM-Versatile Express
[<80110af4>] (unwind_backtrace) from [<8010c750>] (show_stack+0x10/0x14)
[<8010c750>] (show_stack) from [<8071d8d4>] (dump_stack+0x88/0x9c)
[<8071d8d4>] (dump_stack) from [<80121194>] (__warn.part.3+0xb8/0xd4)
[<80121194>] (__warn.part.3) from [<80121314>] (warn_slowpath_null+0x44/0x4c)
[<80121314>] (warn_slowpath_null) from [<80a0d6f8>] (set_handle_irq+0x24/0x4c)
[<80a0d6f8>] (set_handle_irq) from [<80a29550>] (__gic_init_bases+0x94/0x210)
[<80a29550>] (__gic_init_bases) from [<80a299e8>] (gic_of_init+0x2ec/0x3dc)
[<80a299e8>] (gic_of_init) from [<80a36d24>] (of_irq_init+0x194/0x2b8)
[<80a36d24>] (of_irq_init) from [<80a02c80>] (init_IRQ+0x24/0x78)
[<80a02c80>] (init_IRQ) from [<80a00ce4>] (start_kernel+0x2bc/0x498)
[<80a00ce4>] (start_kernel) from [<00000000>] (0x0)
random: get_random_bytes called from print_oops_end_marker+0x24/0x48 with crng_init=0
---[ end trace 0000000000000000 ]---
=== set_handle_irq ===
:

set -e

CROSS_COMPILE=arm-linux-gnu- ARCH=arm make -j32 &&
qemu-system-arm \
	-M vexpress-a9 \
	-nographic \
	-sd disk.img \
	-dtb arch/arm/boot/dts/vexpress-v2p-ca9.dtb \
	-kernel arch/arm/boot/zImage \
	-no-reboot \
	-append 'init=/bin/dmesg root=/dev/mmcblk0 console=ttyAMA0 earlyprintk=ttyAMA0 ro panic=1' 2>&1 |
	awk '/as init process/ {e=1} {if(e)print}' |tee LOG
