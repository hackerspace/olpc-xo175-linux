set -x
set -e

# curl -L https://releases.linaro.org/components/toolchain/binaries/4.9-2016.02/arm-linux-gnueabihf/gcc-linaro-4.9-2016.02-x86_64_arm-linux-gnueabihf.tar.xz |tar xJf - -C /opt
CROSS_COMPILE=/opt/gcc-linaro-4.9-2016.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf- ARCH=arm make mmp3_ariel_defconfig
CROSS_COMPILE=/opt/gcc-linaro-4.9-2016.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf- ARCH=arm make -j$(nproc) -k uImage
#CROSS_COMPILE=/opt/gcc-linaro-4.9-2016.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf- ARCH=arm make -k uImage V=1
