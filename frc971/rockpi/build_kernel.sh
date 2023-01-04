#!/bin/bash

set -eux
set -o pipefail

if [[ ! -e linux ]]; then
  git clone --branch 6.0.8-rt14-rockpi4b https://github.com/frc971/linux
  ln -s ../.config linux/.config
fi

(
cd linux

export CC=aarch64-linux-gnu-
export CROSS_COMPILE=aarch64-linux-gnu-
export LOCALVERSION=-rockpi4b

make ARCH=arm64 CROSS_COMPILE="${CROSS_COMPILE}" oldconfig
make ARCH=arm64 CROSS_COMPILE="${CROSS_COMPILE}" menuconfig
make -j40 ARCH=arm64 LOCALVERSION="${LOCALVERSION}" \
  CROSS_COMPILE="${CROSS_COMPILE}" Image modules
make -j40 ARCH=arm64 LOCALVERSION="${LOCALVERSION}" \
  CROSS_COMPILE="${CROSS_COMPILE}" dtbs

rm -rf ../kernel-install
mkdir -p ../kernel-install

VERSION="$(cat include/config/kernel.release)"

make -s ARCH=arm64 LOCALVERSION="${LOCALVERSION}" \
  CROSS_COMPILE="${CROSS_COMPILE}" \
  modules_install INSTALL_MOD_PATH="$(realpath ../kernel-install)"
make -s ARCH=arm64 LOCALVERSION="${LOCALVERSION}" \
  CROSS_COMPILE="${CROSS_COMPILE}" \
  dtbs_install INSTALL_DTBS_PATH="$(realpath ../kernel-install)/boot/dtbs/${VERSION}"
make -s ARCH=arm64 LOCALVERSION="${LOCALVERSION}" CROSS_COMPILE="${CROSS_COMPILE}" \
  install INSTALL_PATH="$(realpath ../kernel-install)/boot/"
)

VERSION="$(cat linux/include/config/kernel.release)"

(
  cd ../../y2022/localizer/kernel/
  make rockpi
)

cp ../../y2022/localizer/kernel/adis16505.ko "kernel-install/lib/modules/${VERSION}/kernel/"

/sbin/depmod -b ./kernel-install ${VERSION}

tar -cvf "linux-kernel-${VERSION}.tar.xz" -C kernel-install .
