#!/bin/bash

set -xe

# Full path to Raspberry Pi Buster disk image
IMAGE="2020-08-20-raspios-buster-armhf-lite.img"
BOOT_PARTITION="${IMAGE}.boot_partition"
PARTITION="${IMAGE}.partition"

function target() {
  HOME=/root/ USER=root sudo proot -0 -q qemu-arm-static -w / -r "${PARTITION}" "$@"
}

function user_pi_target() {
  USER=root sudo proot -0 -q qemu-arm-static -w / -r "${PARTITION}" sudo -h 127.0.0.1 -u pi "$@"
}


mkdir -p "${PARTITION}"
mkdir -p "${BOOT_PARTITION}"

if mount | grep "${BOOT_PARTITION}" >/dev/null ;
then
  echo "Already mounted"
else
  OFFSET="$(fdisk -lu "${IMAGE}" | grep "${IMAGE}1" | awk '{print 512*$2}')"
  sudo mount -o loop,offset=${OFFSET} "${IMAGE}" "${BOOT_PARTITION}"
fi

# Enable the camera on boot.
if ! grep "start_x=1" "${BOOT_PARTITION}/config.txt"; then
  echo "start_x=1" | sudo tee -a "${BOOT_PARTITION}/config.txt"
fi
if ! grep "gpu_mem=128" "${BOOT_PARTITION}/config.txt"; then
  echo "gpu_mem=128" | sudo tee -a "${BOOT_PARTITION}/config.txt"
fi

sudo umount "${BOOT_PARTITION}"
rmdir "${BOOT_PARTITION}"

if mount | grep "${PARTITION}" >/dev/null ;
then
  echo "Already mounted"
else
  OFFSET="$(fdisk -lu "${IMAGE}" | grep "${IMAGE}2" | awk '{print 512*$2}')"

  if [[ "$(stat -c %s "${IMAGE}")" < 3000000000 ]]; then
    echo "Growing image"
    dd if=/dev/zero bs=1G count=1 >> "${IMAGE}"
    START="$(fdisk -lu "${IMAGE}" | grep "${IMAGE}2" | awk '{print $2}')"

    sed -e 's/\s*\([\+0-9a-zA-Z]*\).*/\1/' << EOF | fdisk "${IMAGE}"
  d # remove old partition
  2
  n # new partition
  p # primary partition
  2 # partion number 2
  532480 # start where the old one starts
    # To the end
  p # print the in-memory partition table
  w # Flush
  q # and we're done
EOF

    sudo losetup -o "${OFFSET}" -f "${IMAGE}"
    LOOPBACK="$(sudo losetup --list | grep "${IMAGE}" | awk '{print $1}')"
    sudo e2fsck -f "${LOOPBACK}"
    sudo resize2fs "${LOOPBACK}"
    sudo losetup -d "${LOOPBACK}"
  fi

  echo "Mounting"
  sudo mount -o loop,offset=${OFFSET} "${IMAGE}" "${PARTITION}"
fi

sudo cp target_configure.sh "${PARTITION}/tmp/"
sudo cp dhcpcd.conf "${PARTITION}/tmp/dhcpcd.conf"
sudo cp sctp.conf "${PARTITION}/etc/sysctl.d/sctp.conf"
sudo cp change_hostname.sh "${PARTITION}/tmp/change_hostname.sh"
sudo cp frc971.service "${PARTITION}/etc/systemd/system/frc971.service"
sudo cp rt.conf "${PARTITION}/etc/security/limits.d/rt.conf"

target /bin/mkdir -p /home/pi/.ssh/
cat ~/.ssh/id_rsa.pub | target tee /home/pi/.ssh/authorized_keys

target /bin/bash /tmp/target_configure.sh

# Run a prompt as root inside the target to poke around and check things.
target /bin/bash --rcfile /root/.bashrc

sudo umount "${PARTITION}"
rmdir "${PARTITION}"
