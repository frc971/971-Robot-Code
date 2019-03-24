#!/bin/bash

set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

# We need to rebuild to save the old constants since the .cc file is baked into
# the executable.
calibration() {
  bazel build -c opt //y2019/vision:global_calibration

  ../../bazel-bin/y2019/vision/global_calibration "$@"
}

# Calibrate the comp robot.  This is assuming the data file captured has been
# extracted into the users home directory.
calibration \
    --camera_id 1 \
    --tape_start_x=-12.5 \
    --tape_start_y=-1.0 \
    --tape_direction_x=-1.0 \
    --tape_direction_y=0.0 \
    --beginning_tape_measure_reading=16 \
    --image_count=45 \
    --constants=constants.cc \
    --prefix ~/cam1/debug_viewer_jpeg_

calibration \
    --camera_id 14 \
    --tape_start_x=12.5 \
    --tape_start_y=0.0 \
    --tape_direction_x=1.0 \
    --tape_direction_y=0.0 \
    --beginning_tape_measure_reading=16 \
    --image_count=71 \
    --constants=constants.cc \
    --prefix ~/cam14/debug_viewer_jpeg_

calibration \
    --camera_id 15 \
    --tape_start_x=12.5 \
    --tape_start_y=0.0 \
    --tape_direction_x=1.0 \
    --tape_direction_y=0.0 \
    --beginning_tape_measure_reading=25 \
    --image_count=62 \
    --constants=constants.cc \
    --prefix ~/cam15/debug_viewer_jpeg_

calibration \
    --camera_id 17 \
    --tape_start_x=-6.5 \
    --tape_start_y=11.0 \
    --tape_direction_x=0.0 \
    --tape_direction_y=-1.0 \
    --beginning_tape_measure_reading=29 \
    --image_count=58 \
    --constants=constants.cc \
    --prefix ~/cam17/debug_viewer_jpeg_

calibration \
    --camera_id 18 \
    --tape_start_x=6.5 \
    --tape_start_y=-11.0 \
    --tape_direction_x=0.0 \
    --tape_direction_y=1.0 \
    --beginning_tape_measure_reading=27 \
    --image_count=60 \
    --constants=constants.cc \
    --prefix ~/cam18/debug_viewer_jpeg_
