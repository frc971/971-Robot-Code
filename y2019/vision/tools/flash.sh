#!/bin/sh

if [ "$(id -u)" != "0" ]
then
  echo "Please run as root: $(id -u)"
  exit
fi

echo "Pulling Binaries ..."
if [ ! -f "./libjevoisbase.so" ]
then
  wget http://www.frc971.org/Build-Dependencies/libjevoisbase.so
fi
echo "Got libjevoisbase.so"

if [ ! -f "./PassThrough.so" ]
then
  wget http://www.frc971.org/Build-Dependencies/PassThrough.so
fi
echo "Got PassThrough.so"

DEV_BASE=/dev/mmcblk0p

echo "Please check that ${DEV_BASE}3 is the correct drive to format. This is a destructive command so please be sure."
read -p "Enter \"yes\" when you are ready: " input

if [ "$input" != "yes" ]
then
  echo "Format aborted."
  exit -1
fi

# need to disable some new features on 17.x
sudo mkfs.ext3 -L JEVOIS ${DEV_BASE}3

echo "Mounting JEVOIS."
mkdir -p /tmp/JEVOIS
sudo mount ${DEV_BASE}3 /tmp/JEVOIS

echo "Mounting LINUX."
mkdir -p /tmp/LINUX
sudo mount ${DEV_BASE}2 /tmp/LINUX

echo "Make JEVOIS directories."
mkdir -p /tmp/JEVOIS/packages
mkdir -p /tmp/JEVOIS/modules/JeVois/PassThrough/
mkdir -p /tmp/JEVOIS/config
mkdir -p /tmp/JEVOIS/lib/JeVois/
mkdir -p /tmp/JEVOIS/deploy/
mkdir -p /tmp/JEVOIS/data/

echo "Copy configs."
cp ./austin_cam.sh /tmp/JEVOIS
cp ./videomappings.cfg /tmp/JEVOIS/config/
cp ./rcS_script.txt /tmp/LINUX/etc/init.d/rcS
cp Jevois_fstab /tmp/LINUX/etc/fstab
cp ./PassThrough.so /tmp/JEVOIS/modules/JeVois/PassThrough/
cp ./libjevoisbase.so /tmp/JEVOIS/lib/JeVois/
cp ./launch.sh /tmp/JEVOIS/deploy/

echo "Un-mounting JEVOIS"
sudo umount /tmp/JEVOIS

echo "Un-mounting LINUX"
sudo umount /tmp/LINUX


