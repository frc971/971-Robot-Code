#!/bin/bash

set -e

JETSON="root@$1"

# To build for the Jetson, use
bazel build -c opt //y2018/vision:image_streamer \
    //aos/vision/tools:camera_primer --cpu=armhf-debian

# Remove files before copying them.
# Doing so avoids problems with trying to copy over a file in use.
ssh "${JETSON}" rm -f image_streamer camera_primer exposure_2018.sh startup.sh

# Copy files to Jetson
scp bazel-bin/y2018/vision/image_streamer "${JETSON}":.
scp bazel-bin/aos/vision/tools/camera_primer "${JETSON}":.
scp y2018/vision/exposure_2018.sh "${JETSON}":.
scp y2018/vision/startup.sh "${JETSON}":.

ssh "${JETSON}" sync

# Can't restart with supervisorctl because the USB devices don't come up reliably...
echo "You just restart the Jetson now" >&2