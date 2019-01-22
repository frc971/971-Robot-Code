#!/bin/bash

while true
do
  v4l2-ctl --set-ctrl="exposure_absolute=300" -d /dev/video0
  v4l2-ctl --set-ctrl="exposure_absolute=300" -d /dev/video1
  sleep 10
done
