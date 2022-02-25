#!/bin/bash

CONFIG=/boot/config.txt

if grep -q adis16505 "${CONFIG}"; then
  echo "Already enabled"
  exit 0;
fi

sed -i '1h;1!H;$!d;x;s/.*dtparam=spi[^\n]*/&\n\n# Enable the IMU\ndtoverlay=adis16505/' "${CONFIG}"

echo "Enabled 16505"
