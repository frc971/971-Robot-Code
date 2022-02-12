#!/bin/bash
# This script builds the kernel for the raspberry pi.
#
# git@github.com:frc971/linux.git
#
# We are using the frc971-5.10-pi4-rt branch
#
# Point it at the version of that repo you want to build, and it'll generate a
# .tar.gz of the bits to install.


set -ex

echo $#
if [[ $# -lt 2 ]];
then
  echo "Pass in the kernel directory and then the destination .tar.gz"
  exit 1
fi

export OUTPUT="$(realpath $(dirname $2))/$(basename ${2})"
export TMPDIR="${OUTPUT}_tmpdir"

mkdir -p "${TMPDIR}"
mkdir -p "${TMPDIR}/fat32/overlays"
mkdir -p "${TMPDIR}/ext4"

pushd "${1}"

export KERNEL=kernel8

make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image modules dtbs -j 60

cp arch/arm64/boot/Image "${TMPDIR}/fat32/$KERNEL.img"
cp arch/arm64/boot/dts/broadcom/*.dtb "${TMPDIR}/fat32/"
cp arch/arm64/boot/dts/overlays/*.dtb* "${TMPDIR}/fat32/overlays/"
cp arch/arm64/boot/dts/overlays/README "${TMPDIR}/fat32/overlays/"

make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- INSTALL_MOD_PATH="${TMPDIR}/ext4" modules_install

cd "${TMPDIR}"
tar cvzf "${OUTPUT}" --owner=0 --group=0 "./"

popd

rm -rf "${TMPDIR}"

echo "Kernel is now available at: ${OUTPUT}"
