#!/bin/bash

### Helper script to mount a Raspberry Pi SD card
### Optionally run change_hostname if given pi name as argument

set -e

MOUNT_PT="tmp_mount"
DEVICE="/dev/sda"

if mount | grep "${DEVICE}" >/dev/null ;
then
  echo "Overwriting a mounted partition, is ${DEVICE} the sd card?"
  exit 1
fi

PARTITION="${MOUNT_PT}.partition"

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


echo "Starting a shell for any manual configuration"
target /bin/bash --rcfile /root/.bashrc

# Found I had to do a lazy force unmount ("-l" flag) to make it work reliably
sudo umount -l "${PARTITION}"
rmdir "${PARTITION}"
