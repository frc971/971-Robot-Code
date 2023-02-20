#!/bin/bash

set -e

# Disk image to use for creating SD card
# NOTE: You MUST run modify_rootfs.sh on this image BEFORE running make_sd.sh
ORIG_IMAGE="arm64_bullseye_debian.img"
IMAGE=`echo ${ORIG_IMAGE} | sed s/.img/-frc-mods.img/`
DEVICE="/dev/sda"

if mount | grep "${DEVICE}" >/dev/null ;
then
  echo "Overwriting a mounted partition, is ${DEVICE} the sd card?"
  exit 1
fi

sudo dd if=${IMAGE} of=${DEVICE} bs=1M status=progress

PARTITION="${IMAGE}.partition"

mkdir -p "${PARTITION}"
sudo mount "${DEVICE}2" "${PARTITION}"

function target() {
  HOME=/root/ USER=root sudo proot -0 -q qemu-aarch64-static -w / -r "${PARTITION}" "$@"
}

if [ "${1}" == "" ]; then
  echo "No hostname specified, so skipping setting it."
  echo "You do this manually on the pi by running /root/bin/change_hostname.sh PI_NAME"
else
  target /root/bin/change_hostname.sh "${1}"
fi

# Put a timestamp on when this card got created and by whom
TIMESTAMP_FILE="${PARTITION}/home/pi/.DiskFlashedDate.txt"
echo "Date Imaged: "`date` > "${TIMESTAMP_FILE}"
echo "Image file: ${IMAGE}"  >> "${TIMESTAMP_FILE}"
echo "Git tag: "`git rev-parse HEAD` >> "${TIMESTAMP_FILE}"
echo "User: "`whoami` >> "${TIMESTAMP_FILE}"

echo "Starting a shell for any manual configuration"
target /bin/bash --rcfile /root/.bashrc

# Found I had to do a lazy force unmount ("-l" flag) to make it work reliably
sudo umount -l "${PARTITION}"
rmdir "${PARTITION}"
