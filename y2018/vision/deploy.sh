#!/bin/bash

set -e

JETSON="root@$1"

# To build for the Jetson, use
bazel build -c opt //y2018/vision:image_streamer --cpu=armhf-debian

# Copy files to Jetson
rsync -av --progress bazel-bin/y2018/vision/image_streamer y2018/vision/exposure_loop.sh "${JETSON}":.
rsync -av --progress y2018/vision/exposure_loop.conf y2018/vision/vision.conf "${JETSON}":/etc/supervisor/conf.d/

ssh "${JETSON}" sync

# Can't restart with supervisorctl because the USB devices don't come up reliably...
echo "Restart the Jetson now" >&2
