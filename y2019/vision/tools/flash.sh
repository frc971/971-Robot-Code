#!/bin/sh

if [ "$(id -u)" != "0" ]
then
  echo "Please run as root: $(id -u)"
  exit
fi

DEV_BASE=/dev/mmcblk0p

echo "Please check that ${DEV_BASE}3 is the correct drive to format. This is a destructive command so please be sure."
read -p "Enter \"yes\" when you are ready: " input

if [ "$input" != "yes" ]
then
  echo "Format aborted."
  exit -1
fi

# need to disable some new features on 17.x
sudo mkfs.ext4 -L JEVOIS -O ^64bit,uninit_bg,^metadata_csum ${DEV_BASE}3

echo "Mounting JEVOIS."
mkdir -p /tmp/JEVOIS
sudo mount ${DEV_BASE}3 /tmp/JEVOIS

echo "Mounting LINUX."
mkdir -p /tmp/LINUX
sudo mount ${DEV_BASE}2 /tmp/LINUX

echo "Make JEVOIS directories."
mkdir -p /tmp/JEVOIS/packages
mkdir -p /tmp/JEVOIS/modules
mkdir -p /tmp/JEVOIS/config
mkdir -p /tmp/JEVOIS/lib

echo "Copy configs."
cp ./austin_cam.sh /tmp/JEVOIS
cp ./videomappings.cfg /tmp/JEVOIS/config/
cp ./rcS_script.txt /tmp/LINUX/etc/init.d/rcS
cp Jevois_fstab /tmp/LINUX/etc/fstab

echo "Un-mounting JEVOIS"
sudo umount /tmp/JEVOIS

echo "Un-mounting LINUX"
sudo umount /tmp/LINUX


