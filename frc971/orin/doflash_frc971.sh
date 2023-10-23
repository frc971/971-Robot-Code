#!/bin/bash
set -ex

TMPDIR=$(mktemp -d /tmp/yoctoflash.XXXXXXXXXX)

# Cleanup on exit.
function finish {
  sudo rm -rf "${TMPDIR}"
}
trap finish EXIT

# Assumes that the image has been copied into ./
tar xf frc971-image-orin-nx-8g.tegraflash.tar.gz -C "${TMPDIR}"

# Replace the rootfs with our new image.
cp --sparse=always arm64_bookworm_debian_yocto.img "${TMPDIR}/frc971-image.ext4"

cd ${TMPDIR}

sudo ./initrd-flash
