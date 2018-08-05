if [ -z "$ARCH" ]; then
	grep -q CONFIG_ARM=y .config && ARCH=arm
	grep -q CONFIG_X86=y .config && ARCH=x86
fi

if [ "$ARCH" = arm ]; then
	[ -n "$CROSS_COMPILE" ]	|| CROSS_COMPILE=arm-linux-gnu-
	[ -n "$DEFCONFIG" ]	|| DEFCONFIG=olpc_xo175_defconfig
	[ -n "$DIR" ]		|| DIR=XO175
	[ -n "$IMAGE" ]		|| IMAGE=zImage
	[ -n "$APPEND_DTB" ]	|| APPEND_DTB=mmp2-olpc-xo-1-75.dtb
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

[ -n "$INST" ]	|| INST=/boot/$IMAGE

#make ARCH=arm CROSS_COMPILE=arm-linux-gnu- mmp2_defconfig oldconfig
#time make ARCH=arm CROSS_COMPILE=arm-linux-gnu- -j16
#perl append.pl ./arch/arm/boot/zImage ./arch/arm/boot/dts/mmp2-brownstone.dtb >/run/media/lkundrak/19DE-9DE4/boot/zImage && umount /run/media/lkundrak/19DE-9DE4
#make ARCH=arm CROSS_COMPILE=arm-linux-gnu- mmp2_defconfig oldconfig
#make ARCH=arm CROSS_COMPILE=arm-linux-gnu- olpc_xo175_defconfig savedefconfig; cp defconfig ./arch/arm/configs/olpc_xo175_defconfig
#[ -e /dev/xoec ] && stty -F /dev/xoec 406:0:18b2:8a30:3:1c:7f:15:4:2:64:0:11:13:1a:0:12:f:17:16:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0:0 && echo P0 >/dev/xoec 

[ -f .config ] || make ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE $DEFCONFIG
time make ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE savedefconfig
cp defconfig ./arch/$ARCH/configs/$DEFCONFIG
time make ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE -j$(awk '/^processor/ {n++} END {print n*2}'  /proc/cpuinfo) $IMAGE $APPEND_DTB || exit 1
[ -d /run/media/lkundrak/19DE-9DE4 ] && /bin/cp arch/$ARCH/boot/$IMAGE /run/media/lkundrak/19DE-9DE4/boot/$IMAGE && umount /run/media/lkundrak/*
[ -d /run/media/lkundrak/$DIR ] && su -c "/bin/cp arch/$ARCH/boot/$IMAGE /run/media/lkundrak/$DIR/boot/$IMAGE" && umount /run/media/lkundrak/*
if [ "$(git symbolic-ref --short HEAD)" == lr/olpc-xo175 ]; then
	[ "$ARCH" == "arm" ] && rsync -avr arch/$ARCH/boot/$IMAGE v3.sk:public_html/xo175/vmlinuz &
else
	true &
fi

ssh root@$XO "[ ! -f /etc/olpc-release ] && cp $INST $INST.old"

if [ "$APPEND_DTB" ]; then
	cat arch/$ARCH/boot/{$IMAGE,dts/$APPEND_DTB} >arch/$ARCH$INST-olpc
	scp arch/$ARCH/boot/$IMAGE-olpc root@$XO:$INST
	scp arch/$ARCH/boot/$IMAGE-olpc root@$XO:$INST-olpc
	scp arch/$ARCH/boot/$IMAGE root@$XO:$INST-bare
else
	scp arch/$ARCH/boot/$IMAGE root@$XO:$INST
fi

ssh root@$XO reboot
wait

#[ -e /dev/xoec ] && echo P1 >/dev/xoec 
