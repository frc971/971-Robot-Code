#!/bin/bash

set -e

A=`ls /dev/video*`
EXPOSURE='2000'

echo $SHELL

echo $A

# Michael added this one to try and have the exposer set correctly sooner.
sleep 1
echo Setting exposure again after 1 seconds

v4l2-ctl --set-ctrl="exposure_absolute=$EXPOSURE" -d $A

echo Done setting exposure again after 1 seconds


# Michael added this one to try and have the exposer set correctly sooner.
sleep 5
echo Setting exposure again after 5 seconds
v4l2-ctl --set-ctrl="exposure_absolute=$EXPOSURE" -d $A


echo Done setting exposure again after 5 seconds

#sleep 15
#
#echo Setting exposure again after 20 seconds
#
#v4l2-ctl --set-ctrl="exposure_absolute=$EXPOSURE" -d $A
#
#echo Done setting exposure again after 20 seconds
