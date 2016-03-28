# We have to override the new %%install behavior because, well... the kernel is special.
%global __spec_install_pre %{___build_pre}

Summary: The Linux kernel (the core of the Linux operating system)

# kernel-firmware
%define with_firmware 0

# set to 1 to build the initramfs during kernel-build time
# set to 0 to build the initramfs during %post on the host
%define buildinitramfs 1

# Versions of various parts

%define flavor _xo%{xoversion}
%define kversion 4.%{patchlevel}.%{sublevel}%{?extraversion}
%define rpmversion 4.%{patchlevel}.%{sublevel}%{?extraversion}%{flavor}
%define release %{timestamp}.olpc.%{?head}
%define image_install_path boot

%define KVERREL %{version}-%{release}
%define hdrarch %_target_cpu
%define asmarch %_target_cpu

%ifarch %{ix86}
%define hdrarch i386
%define asmarch x86
%define _arch x86
%define make_target bzImage
%define kernel_image arch/x86/boot/bzImage
%endif

%ifarch %{arm}
%define asmarch arm
%define hdrarch arm
%define _arch arm
%define make_target zImage
%define kernel_image arch/arm/boot/zImage
%endif

#
# Three sets of minimum package version requirements in the form of Conflicts:
# to versions below the minimum
#

#
# First the general kernel 2.6 required versions as per
# Documentation/Changes
#
%define kernel_dot_org_conflicts  ppp < 2.4.3-3, isdn4k-utils < 3.2-32, nfs-utils < 1.0.7-12, e2fsprogs < 1.37-4, util-linux < 2.12, jfsutils < 1.1.7-2, reiserfs-utils < 3.6.19-2, xfsprogs < 2.6.13-4, procps < 3.2.5-6.3, oprofile < 0.9.1-2

#
# Then a series of requirements that are distribution specific, either
# because we add patches for something, or the older versions have
# problems with the newer kernel or lack certain things that make
# integration in the distro harder than needed.
#
%define package_conflicts kudzu < 1.2.5, initscripts < 7.23, udev < 063-6, iptables < 1.3.2-1, ipw2200-firmware < 2.4, selinux-policy-targeted < 1.25.3-14

#
# The ld.so.conf.d file we install uses syntax older ldconfig's don't grok.
#
%define xen_conflicts glibc < 2.3.5-1, xen < 3.0.1

#
# Packages that need to be installed before the kernel is, because the %post
# scripts use them.
%define kernel_prereq  fileutils, module-init-tools, initscripts >= 8.11.1-1, hostname

Name: kernel
Group: System Environment/Kernel
License: GPLv2
Version: %{rpmversion}
Release: %{release}
ExclusiveOS: Linux
Provides: kernel = %{version}
Provides: kernel-drm = 4.3.0
Provides: kernel-%{_target_cpu} = %{rpmversion}-%{release}
BuildRequires: %{kernel_prereq}
Conflicts: %{kernel_dot_org_conflicts}
Conflicts: %{package_conflicts}
# We can't let RPM do the dependencies automatic because it'll then pick up
# a correct but undesirable perl dependency from the module headers which
# isn't required for the kernel proper to function
AutoReq: no
AutoProv: yes

BuildRequires: libertas-usb8388-olpc-firmware libertas-sd8686-firmware libertas-sd8787-firmware

BuildRequires: module-init-tools, patch >= 2.5.4, bash >= 2.03, sh-utils, tar
BuildRequires: bzip2, findutils, gzip, m4, perl, make >= 3.78, diffutils
BuildRequires: gcc >= 3.4.2, binutils >= 2.12, redhat-rpm-config
BuildRequires: unifdef bc
BuildConflicts: rhbuildsys(DiskFree) < 500Mb

BuildRequires: dracut, dracut-modules-olpc
Requires(post): coreutils, module-init-tools

Source0: olpc-kernel-%{head}.tar.bz2

Source10: COPYING.modules

BuildRoot: %{_tmppath}/kernel-%{KVERREL}-root

%description
The kernel package contains the Linux kernel (vmlinuz), the core of any
Linux operating system.  The kernel handles the basic functions
of the operating system:  memory allocation, process allocation, device
input and output, etc.

%package devel
Summary: Development package for building kernel modules to match the kernel.
Group: System Environment/Kernel
AutoReqProv: no
Provides: kernel-devel-%{_target_cpu} = %{rpmversion}-%{release}
Requires(pre): /usr/bin/find

%description devel
This package provides kernel headers and makefiles sufficient to build modules
against the kernel package.


%package headers
Summary: Header files for the Linux kernel for use by glibc
Group: Development/System
Obsoletes: glibc-kernheaders
Provides: glibc-kernheaders = 3.0-46

%description headers
Kernel-headers includes the C header files that specify the interface
between the Linux kernel and userspace libraries and programs.  The
header files define structures and constants that are needed for
building most standard programs and are also needed for rebuilding the
glibc package.


%package firmware
Summary: Firmware files used by the Linux kernel
Group: Development/System
# This is... complicated.
# Look at the WHENCE file.
License: GPL+ and GPLv2+ and MIT and Redistributable, no modification permitted
%if "x%{?variant}" != "x"
Provides: kernel-firmware = %{rpmversion}-%{release}
%endif
%description firmware
Kernel-firmware includes firmware files required for some devices to
operate.


%prep
%setup -q -n olpc-kernel
cp %{SOURCE10} Documentation/

# make sure the kernel has the sublevel we know it has. 
perl -p -i -e "s/^SUBLEVEL.*/SUBLEVEL = %{sublevel}/" Makefile
perl -p -i -e "s/^EXTRAVERSION.*/EXTRAVERSION = %{?extraversion}%{flavor}-%{release}/" Makefile

%build

BuildKernel() {
    MakeTarget=$1
    KernelImage=$2

    Config=kernel-%{kversion}-%{_target_cpu}.config
    DevelDir=/usr/src/kernels/%{KVERREL}-%{_target_cpu}

    KernelVer=%{version}-%{release}
    echo BUILDING A KERNEL FOR %{_arch}: %{_target_cpu}...

    cp %{defconfig} .config
    make -s ARCH=%{_arch} olddefconfig
    make -s ARCH=%{_arch} %{?_smp_mflags} $MakeTarget
    make -s ARCH=%{_arch} %{?_smp_mflags} modules || exit 1

    # Start installing the results

    mkdir -p $RPM_BUILD_ROOT/%{image_install_path}

    # remove blank lines and comments from .config to save space, and compress
    grep "^[^#]" .config | xz -9 > $RPM_BUILD_ROOT/boot/config-$KernelVer.xz

    cp $KernelImage $RPM_BUILD_ROOT/%{image_install_path}/vmlinuz-$KernelVer
    if [ -f arch/$Arch/boot/zImage.stub ]; then
      cp arch/$Arch/boot/zImage.stub $RPM_BUILD_ROOT/%{image_install_path}/zImage.stub-$KernelVer || :
    fi

    mkdir -p $RPM_BUILD_ROOT/lib/modules/$KernelVer
    # Override $(mod-fw) because we don't want it to install any firmware
    # We'll do that ourselves with 'make firmware_install'
    make -s ARCH=%{_arch} INSTALL_MOD_PATH=$RPM_BUILD_ROOT modules_install KERNELRELEASE=$KernelVer mod-fw=

    # And save the headers/makefiles etc for building modules against
    #
    # This all looks scary, but the end result is supposed to be:
    # * all arch relevant include/ files
    # * all Makefile/Kconfig files
    # * all script/ files

    rm -f $RPM_BUILD_ROOT/lib/modules/$KernelVer/build
    rm -f $RPM_BUILD_ROOT/lib/modules/$KernelVer/source
    mkdir -p $RPM_BUILD_ROOT/lib/modules/$KernelVer/build
    (cd $RPM_BUILD_ROOT/lib/modules/$KernelVer ; ln -s build source)
    # dirs for additional modules per module-init-tools, kbuild/modules.txt
    mkdir -p $RPM_BUILD_ROOT/lib/modules/$KernelVer/extra
    mkdir -p $RPM_BUILD_ROOT/lib/modules/$KernelVer/updates
    mkdir -p $RPM_BUILD_ROOT/lib/modules/$KernelVer/weak-updates
    # first copy everything
    cp --parents `find  -type f -name "Makefile*" -o -name "Kconfig*"` $RPM_BUILD_ROOT/lib/modules/$KernelVer/build
    cp Module.symvers $RPM_BUILD_ROOT/lib/modules/$KernelVer/build
    # then drop all but the needed Makefiles/Kconfig files
    rm -rf $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/Documentation
    rm -rf $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/scripts
    rm -rf $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/include
    cp .config $RPM_BUILD_ROOT/lib/modules/$KernelVer/build
    cp -a scripts $RPM_BUILD_ROOT/lib/modules/$KernelVer/build
    if [ -d arch/%{_arch}/scripts ]; then
      cp -a arch/%{_arch}/scripts $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/arch/%{_arch} || :
    fi
    if [ -f arch/%{_arch}/*lds ]; then
      cp -a arch/%{_arch}/*lds $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/arch/%{_arch}/ || :
    fi
    rm -f $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/scripts/*.o
    rm -f $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/scripts/*/*.o
     if [ -d arch/%{asmarch}/include ]; then
       cp -a --parents arch/%{asmarch}/include $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/
     fi
    cp -a include $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/include
    # Make sure the Makefile and version.h have a matching timestamp so that
    # external modules can be built
    touch -r $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/Makefile $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/include/linux/version.h
    touch -r $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/.config $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/include/linux/autoconf.h
    # Copy .config to include/config/auto.conf so "make prepare" is unnecessary.
    cp $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/.config $RPM_BUILD_ROOT/lib/modules/$KernelVer/build/include/config/auto.conf

    find $RPM_BUILD_ROOT/lib/modules/$KernelVer -name "*.ko" -type f >modnames

    # mark modules executable so that strip-to-file can strip them
    cat modnames | xargs chmod u+x

    # detect missing or incorrect license tags
    for i in `cat modnames`
    do
      echo -n "$i "
      /sbin/modinfo -l $i >> modinfo
    done
    cat modinfo |\
      grep -v "^GPL" |
      grep -v "^Dual BSD/GPL" |\
      grep -v "^Dual MPL/GPL" |\
      grep -v "^GPL and additional rights" |\
      grep -v "^GPL v2" && exit 1
    rm -f modinfo
    rm -f modnames

%if "%{buildinitramfs}" == "1"
	export OLPC_WIFI_FW_SELECT=1

case %{xoversion} in
	1)
		export OLPC_WIFI_FW_8388=1 ;;
	1.5)
		export OLPC_WIFI_FW_8686=1 ;;
	1.75)
		export OLPC_WIFI_FW_8686=1 ;;
	4)
		export OLPC_WIFI_FW_8686=1
		export OLPC_WIFI_FW_8787=1 ;;
esac

    /sbin/dracut --conf /etc/dracut-olpc-runrd.conf --force --kmoddir $RPM_BUILD_ROOT/lib/modules/$KernelVer/ $RPM_BUILD_ROOT/boot/initrd-%{KVERREL}.img %{KVERREL} || exit $?
    /sbin/dracut --conf /etc/dracut-olpc-actrd.conf --force --kmoddir $RPM_BUILD_ROOT/lib/modules/$KernelVer/ $RPM_BUILD_ROOT/boot/actrd-%{KVERREL}.img %{KVERREL} || exit $?
%endif

    # remove files that will be auto generated by depmod at rpm -i time
    rm -f $RPM_BUILD_ROOT/lib/modules/$KernelVer/modules.*

    # Move the devel headers out of the root file system
    mkdir -p $RPM_BUILD_ROOT/usr/src/kernels
    mv $RPM_BUILD_ROOT/lib/modules/$KernelVer/build $RPM_BUILD_ROOT/$DevelDir
    ln -sf ../../..$DevelDir $RPM_BUILD_ROOT/lib/modules/$KernelVer/build
}

# prepare directories
rm -rf $RPM_BUILD_ROOT
mkdir -p $RPM_BUILD_ROOT/boot

BuildKernel %make_target %kernel_image


%install
# Install kernel headers
make ARCH=%{hdrarch} INSTALL_HDR_PATH=$RPM_BUILD_ROOT/usr headers_install

find $RPM_BUILD_ROOT/usr/include -type f -name "*install*" -delete

# glibc provides scsi headers for itself, for now
rm -rf $RPM_BUILD_ROOT/usr/include/scsi
rm -f $RPM_BUILD_ROOT/usr/include/asm*/atomic.h
rm -f $RPM_BUILD_ROOT/usr/include/asm*/io.h
rm -f $RPM_BUILD_ROOT/usr/include/asm*/irq.h

%if %{with_firmware}
make INSTALL_FW_PATH=$RPM_BUILD_ROOT/lib/firmware firmware_install
%endif

install -m0644 olpc/olpc.fth $RPM_BUILD_ROOT/%{image_install_path}


%clean
rm -rf $RPM_BUILD_ROOT


%post
if [ -x /usr/sbin/module_upgrade ]
then
    /usr/sbin/module_upgrade %{rpmversion}-%{release} || exit $?
fi
depmod -a %{KVERREL} || exit $?
%if "%{buildinitramfs}" == "0"
/sbin/dracut --conf /etc/dracut-olpc-runrd.conf --force /boot/initrd-%{KVERREL}.img %{KVERREL} || exit $?
/sbin/dracut --conf /etc/dracut-olpc-actrd.conf --force /boot/actrd-%{KVERREL}.img %{KVERREL} || exit $?
%endif

# if running live on XO hardware, we also want to install the kernel to
# the boot partition. We detect this case by the simple presence of the
# directory structure.
DuplicateInstall()
{
	local tgt=$1
	[ -d "$tgt" ] || return 0
	[ -f /boot/olpc.fth ] && cp -a /boot/olpc.fth $tgt
	[ -f /boot/vmlinuz-%{KVERREL} ] && cp -a /boot/vmlinuz-%{KVERREL} $tgt
	[ -f /boot/initrd-%{KVERREL}.img ] && cp -a /boot/initrd-%{KVERREL}.img $tgt
	[ -f /boot/actrd-%{KVERREL}.img ] && cp -a /boot/actrd-%{KVERREL}.img $tgt
}

DuplicateInstall /bootpart/boot
DuplicateInstall /versions/boot/current/boot

UpdateSymlinks()
{
	local tgt=$1
	[ -d "$tgt" ] || return 0
	[ -f /boot/vmlinuz-%{KVERREL} ] && ln -sf vmlinuz-%{KVERREL} $tgt/vmlinuz
	[ -f /boot/initrd-%{KVERREL}.img ] && ln -sf initrd-%{KVERREL}.img $tgt/initrd.img
	[ -f /boot/actrd-%{KVERREL}.img ] && ln -sf actrd-%{KVERREL}.img $tgt/actrd.img
}

UpdateSymlinks /boot
UpdateSymlinks /bootpart/boot
UpdateSymlinks /versions/boot/current/boot


%post devel
if [ -f /etc/sysconfig/kernel ]
then
    . /etc/sysconfig/kernel || exit $?
fi
if [ "$HARDLINK" != "no" -a -x /usr/sbin/hardlink ] ; then
  pushd /usr/src/kernels/%{KVERREL}-%{_target_cpu} > /dev/null
  /usr/bin/find . -type f | while read f; do hardlink -c /usr/src/kernels/*FC*/$f $f ; done
  popd > /dev/null
fi


%postun
# If running on XO, perform the equivalent uninstall from the boot partition.
DuplicateUninstall()
{
	local tgt=$1
	rm -f $tgt/vmlinuz-%{KVERREL} $tgt/initrd-%{KVERREL}.img $tgt/actrd-%{KVERREL}.img
}
DuplicateUninstall /bootpart/boot
DuplicateUninstall /versions/boot/current/boot


%files
%defattr(-,root,root)
/%{image_install_path}/vmlinuz-%{KVERREL}
/boot/config-%{KVERREL}.xz
/boot/olpc.fth
%dir /lib/modules/%{KVERREL}
/lib/modules/%{KVERREL}/kernel
/lib/modules/%{KVERREL}/build
/lib/modules/%{KVERREL}/source
/lib/modules/%{KVERREL}/extra
/lib/modules/%{KVERREL}/updates
/lib/modules/%{KVERREL}/weak-updates

%if %{with_firmware}
/lib/firmware/*
%endif
%if %{buildinitramfs}
/boot/initrd-%{KVERREL}.img
/boot/actrd-%{KVERREL}.img
%else
%ghost /boot/initrd-%{KVERREL}.img
%ghost /boot/actrd-%{KVERREL}.img
%endif

%files devel
%defattr(-,root,root)
%verify(not mtime) /usr/src/kernels/%{KVERREL}-%{_target_cpu}

%files headers
%defattr(-,root,root)
/usr/include/*

%if %{with_firmware}
%files firmware
%defattr(-,root,root)
/lib/firmware/*
%endif

%changelog

