#!/bin/sh
echo "Mount jevois ..."
./jevois-cmd usbsd

echo "Waiting for fs ..."
while [ ! -d /media/$USER/JEVOIS ]
do
  sleep 1
done
echo "OK"

echo "Copying files ..."
cp ./austin_cam.sh /media/$USER/JEVOIS/

echo "Unmount sd card ..."
umount /media/$USER/JEVOIS
echo "OK"

echo "Rebooting Jevois."
./jevois-cmd restart
