if [ -z "$ARCH" ]; then
	grep -q CONFIG_ARM=y .config && ARCH=arm
	grep -q CONFIG_X86=y .config && ARCH=x86
fi

if [ "$ARCH" = arm ]; then
	[ -n "$CROSS_COMPILE" ]	|| CROSS_COMPILE=arm-none-eabi-
	[ -n "$DEFCONFIG" ]	|| DEFCONFIG=dell_ariel_defconfig
	[ -n "$DIR" ]		|| DIR=XO175
	[ -n "$IMAGE" ]		|| IMAGE=zImage
	[ -n "$DTB_DT" ]	|| DTB_DT=mmp3-dell-ariel.dtb
	[ -n "$DTB_XO" ]	|| DTB_XO=mmp2-olpc-xo-1-75.dtb
	[ -n "$XO" ]		|| XO=xo.local
elif [ "$ARCH" = x86 ]; then
	[ -n "$CROSS_COMPILE" ]	|| CROSS_COMPILE=
	[ -n "$DEFCONFIG" ]	|| DEFCONFIG=olpc_xo1_defconfig
	[ -n "$DIR" ]		|| DIR=XO1
	[ -n "$IMAGE" ]		|| IMAGE=bzImage
	if [ "$HOSTNAME" = belphegor ]; then
		[ -n "$XO" ]	|| XO=xo2.local
	else
		[ -n "$XO" ]	|| XO=xo1.local
		[ -n "$INST" ]	|| INST=/versions/running/boot/vmlinuz
	fi
else
	echo Bad ARCH >&2
	exit
fi
XO=

[ -n "$INST" ]	|| INST=/boot/$IMAGE

#make ARCH=arm CROSS_COMPILE=arm-none-eabi- mmp2_defconfig oldconfig
#time make ARCH=arm CROSS_COMPILE=arm-none-eabi- -j16
#perl append.pl ./arch/arm/boot/zImage ./arch/arm/boot/dts/mmp2-brownstone.dtb >/run/media/lkundrak/19DE-9DE4/boot/zImage && umount /run/media/lkundrak/19DE-9DE4
#make ARCH=arm CROSS_COMPILE=arm-none-eabi- mmp2_defconfig oldconfig
#make ARCH=arm CROSS_COMPILE=arm-none-eabi- olpc_xo175_defconfig savedefconfig; cp defconfig ./arch/arm/configs/olpc_xo175_defconfig
#[ -e /dev/xoec ] && stty -F /dev/xoec 406:0:18b2:8a30:3:1c:7f:15:4:2:64:0:11:13:1a:0:12:f:17:16:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0 && echo P0 >/dev/xoec 

[ -f .config ] || make ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE $DEFCONFIG
time make ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE savedefconfig
cp defconfig ./arch/$ARCH/configs/$DEFCONFIG
time make ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE -j$(awk '/^processor/ {n++} END {print n*4}'  /proc/cpuinfo) $IMAGE $DTB_DT $DTB_XO || exit 1

if [ "$DTB_DT" ]; then
	cat arch/$ARCH/boot/{$IMAGE,dts/$DTB_DT} >arch/$ARCH/boot/$IMAGE.dt
fi

if [ "$DTB_XO" ]; then
	cat arch/$ARCH/boot/{$IMAGE,dts/$DTB_XO} >arch/$ARCH/boot/$IMAGE.xo
fi

if [ -z "$XO" ]; then
	XO=butt.lan PART=/boot
	XO=t00d.lan PART=/boot
	XO=butt.lan PART=/
	XO=xo.local PART=/
	XO=t00d.local PART=/boot/efi
	mkimage -A arm -O linux -C none  -T kernel -a 0x00008000 -e 0x00008000 -n "Linux-$(git describe --abbrev=7)-dajia" -d \
		arch/$ARCH/boot/$IMAGE.dt arch/$ARCH/boot/uImage || exit 1
	if [ -d /run/media/lkundrak/DELLWYSEIMG ]; then
		/bin/cp arch/$ARCH/boot/uImage /run/media/lkundrak/DELLWYSEIMG/kernel/TX0D/uImage-new &&
		/bin/cp arch/$ARCH/boot/$IMAGE /run/media/lkundrak/DELLWYSEIMG/boot/$IMAGE &&
		/bin/cp arch/$ARCH/boot/$IMAGE.dt /run/media/lkundrak/DELLWYSEIMG/boot/$IMAGE.dt &&
		/bin/cp arch/$ARCH/boot/$IMAGE.xo /run/media/lkundrak/DELLWYSEIMG/boot/$IMAGE.xo &&
		umount /run/media/lkundrak/*
	fi
	su -c "/bin/cp arch/$ARCH/boot/$IMAGE.dt /var/www/html/$IMAGE.dt" &&
	scp arch/$ARCH/boot/$IMAGE root@$XO:$PART/boot/$IMAGE &&
	scp arch/$ARCH/boot/$IMAGE.dt root@$XO:$PART/boot/$IMAGE.dt &&
	scp arch/$ARCH/boot/$IMAGE.xo root@$XO:$PART/boot/$IMAGE.xo &&
	(scp arch/$ARCH/boot/uImage root@$XO:$PART/kernel/TX0D/uImage-new || :) &&
	ssh root@$XO reboot
	exit 0
fi

[ -d /run/media/lkundrak/19DE-9DE4 ] && /bin/cp arch/$ARCH/boot/$IMAGE /run/media/lkundrak/19DE-9DE4/boot/$IMAGE && umount /run/media/lkundrak/*
[ -d /run/media/lkundrak/$DIR ] && su -c "/bin/cp arch/$ARCH/boot/$IMAGE /run/media/lkundrak/$DIR/boot/$IMAGE" && umount /run/media/lkundrak/*
if [ "$(git symbolic-ref --short HEAD)" == lr/olpc-xo175 ]; then
	[ "$ARCH" == "arm" ] && rsync -avr arch/$ARCH/boot/$IMAGE v3.sk:public_html/xo175/vmlinuz &
else
	true &
fi

ssh root@$XO "[ ! -f /etc/olpc-release ] && cp $INST $INST.old"

#if [ "$APPEND_DTB" ]; then
#	scp arch/$ARCH/boot/$IMAGE-dt root@$XO:$INST
#	scp arch/$ARCH/boot/$IMAGE-dt root@$XO:$INST-olpc
#	scp arch/$ARCH/boot/$IMAGE root@$XO:$INST-bare
#else
#	scp arch/$ARCH/boot/$IMAGE root@$XO:$INST
#fi

ssh root@$XO reboot
wait

#[ -e /dev/xoec ] && echo P1 >/dev/xoec 
