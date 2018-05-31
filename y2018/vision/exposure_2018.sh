#!/bin/bash

set -e

EXPOSURE='100'

echo $SHELL


# Michael added this one to try and have the exposer set correctly sooner.
sleep 1
echo Setting exposure again after 1 seconds

for CAMERA in /dev/video0 /dev/video1
do
  echo "${CAMERA}"
  v4l2-ctl --set-ctrl="exposure_absolute=$EXPOSURE" -d "${CAMERA}"
done

echo Done setting exposure again after 1 seconds


# Michael added this one to try and have the exposer set correctly sooner.
sleep 5
echo Setting exposure again after 5 seconds
for CAMERA in /dev/video0 /dev/video1
do
  echo "${CAMERA}"
  v4l2-ctl --set-ctrl="exposure_absolute=$EXPOSURE" -d "${CAMERA}"
done

echo Done setting exposure again after 5 seconds

#sleep 15
#
#echo Setting exposure again after 20 seconds
#
#v4l2-ctl --set-ctrl="exposure_absolute=$EXPOSURE" -d $A
#
#echo Done setting exposure again after 20 seconds
