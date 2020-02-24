#!/bin/bash

set -xe

IMAGE="2020-02-13-raspbian-buster-lite.img"
PARTITION="2020-02-13-raspbian-buster-lite.img.partition"
HOSTNAME="pi-8971-1"

function target() {
  HOME=/root/ USER=root sudo proot -0 -q qemu-arm-static -w / -r "${PARTITION}" "$@"
}

function user_pi_target() {
  USER=root sudo proot -0 -q qemu-arm-static -w / -r "${PARTITION}" sudo -h 127.0.0.1 -u pi "$@"
}

OFFSET="$(fdisk -lu "${IMAGE}" | grep "${IMAGE}2" | awk '{print 512*$2}')"
mkdir -p "${PARTITION}"

if mount | grep "${PARTITION}" >/dev/null ;
then
  echo "Already mounted"
else
  echo "Mounting"
  sudo mount -o loop,offset=${OFFSET} "${IMAGE}" "${PARTITION}"
fi

sudo cp target_configure.sh "${PARTITION}/tmp/"
sudo cp dhcpcd.conf "${PARTITION}/tmp/dhcpcd.conf"
sudo cp change_hostname.sh "${PARTITION}/tmp/change_hostname.sh"

target /bin/mkdir -p /home/pi/.ssh/
cat ~/.ssh/id_rsa.pub | target tee /home/pi/.ssh/authorized_keys

target /bin/bash /tmp/target_configure.sh

# Run a prompt as root inside the target to poke around and check things.
target /bin/bash --rcfile /root/.bashrc

sudo umount "${PARTITION}"
rmdir "${PARTITION}"
