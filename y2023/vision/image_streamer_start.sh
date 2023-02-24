#!/bin/bash

# Some configurations to avoid dropping frames
# 640x480@30fps, 400x300@60fps.
# Bitrate 500000-1500000
WIDTH=640
HEIGHT=480
BITRATE=1500000
LISTEN_ON="/camera/downsized"
# Don't interfere with field webpage
STREAMING_PORT=1181

# Handle weirdness with openssl and gstreamer
export OPENSSL_CONF=""

# Enable for verbose logging
#export GST_DEBUG=4

export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0

exec ./image_streamer --width=$WIDTH --height=$HEIGHT --bitrate=$BITRATE --listen_on=$LISTEN_ON --config=aos_config.json --streaming_port=$STREAMING_PORT

