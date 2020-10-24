#!/bin/bash

set -e

IMAGE="2020-08-20-raspios-buster-armhf-lite.img"
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
  HOME=/root/ USER=root sudo proot -0 -q qemu-arm-static -w / -r "${PARTITION}" "$@"
}

target /root/bin/change_hostname.sh "${1}"

echo "Starting a shell for any manual configuration"
target /bin/bash --rcfile /root/.bashrc

sudo umount "${PARTITION}"
