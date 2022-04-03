#!/bin/bash

# Some configurations to avoid dropping frames
# 640x480@30fps, 400x300@60fps.
# Bitrate 500000-1500000
DEVICE=/dev/video0
WIDTH=640
HEIGHT=480
BITRATE=1500000
FRAMERATE=30
EXPOSURE=200

# Handle weirdness with openssl and gstreamer
export OPENSSL_CONF=""

# Enable for verbose logging
#export GST_DEBUG=4

export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0

exec ./image_streamer --device=$DEVICE --width=$WIDTH --height=$HEIGHT --framerate=$FRAMERATE --bitrate=$BITRATE --exposure=$EXPOSURE --config=$HOME/bin/aos_config.json

