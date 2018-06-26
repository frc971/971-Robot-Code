#!/bin/bash

set -e

cd /root/

sleep 1

echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

sleep 1


# Comp Bot
# while [ ! -e /dev/v4l/by-id/usb-046d_0825_A17C8DE0-video-index0 ] ; do echo no camera1 && sleep 1 ; done
# while [ ! -e /dev/v4l/by-id/usb-046d_0825_B914CDE0-video-index0 ] ; do echo no camera2 && sleep 1 ; done
# Practice Bot
# while [ ! -e /dev/v4l/by-id/usb-046d_0825_9224CDE0-video-index0 ] ; do echo no camera1 && sleep 1 ; done
# while [ ! -e /dev/v4l/by-id/usb-046d_0825_B07B8DE0-video-index0 ] ; do echo no camera2 && sleep 1 ; done

# v4l2-ctl --set-ctrl="exposure_auto=1" -d /dev/video0
# sleep 0.5
# v4l2-ctl --set-ctrl="exposure_auto=1" -d /dev/video1
# sleep 2
# echo All done disabling auto-exposure
# v4l2-ctl --set-ctrl="exposure_absolute=20" -d /dev/video0
# sleep 0.5
# v4l2-ctl --set-ctrl="exposure_absolute=20" -d /dev/video1
# sleep 1
# echo All done setting exposure

# echo "Starting target sender now."

for CAMERA in /dev/video0 /dev/video1
do
  echo $CAMERA

  v4l2-ctl --set-ctrl="exposure_auto=1" -d $CAMERA
  sleep 0.5
  v4l2-ctl --set-ctrl="exposure_absolute=100" -d $CAMERA
  sleep 0.5

  PATH="./;$PATH"

  /root/camera_primer $CAMERA
done

# Run a script to reset the exposure a few times and exit.
/root/exposure_2018.sh &

exec /root/image_streamer --single_camera=false
#exec ./target_sender Practice
#exec ./target_sender Comp
#exec ./target_sender Spare
