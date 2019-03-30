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
# extracted into the data folder in //y2019/vision/
calibration \
    --camera_id 10 \
    --tape_start_x=-12.5 \
    --tape_start_y=0.0 \
    --tape_direction_x=-1.0 \
    --tape_direction_y=0.0 \
    --beginning_tape_measure_reading=11 \
    --image_count=69 \
    --constants=constants.cc \
    --prefix data/cam10_1/debug_viewer_jpeg_

calibration \
    --camera_id 6 \
    --tape_start_x=12.5 \
    --tape_start_y=0.0 \
    --tape_direction_x=1.0 \
    --tape_direction_y=0.0 \
    --beginning_tape_measure_reading=11 \
    --image_count=75 \
    --constants=constants.cc \
    --prefix data/cam6_0/debug_viewer_jpeg_

calibration \
    --camera_id 7 \
    --tape_start_x=12.5 \
    --tape_start_y=0.0 \
    --tape_direction_x=1.0 \
    --tape_direction_y=0.0 \
    --beginning_tape_measure_reading=21 \
    --image_count=65 \
    --constants=constants.cc \
    --prefix data/cam7_0/debug_viewer_jpeg_

calibration \
    --camera_id 9 \
    --tape_start_x=-6.5 \
    --tape_start_y=11.0 \
    --tape_direction_x=0.0 \
    --tape_direction_y=-1.0 \
    --beginning_tape_measure_reading=30 \
    --image_count=56 \
    --constants=constants.cc \
    --prefix data/cam9_0/debug_viewer_jpeg_

calibration \
    --camera_id 8 \
    --tape_start_x=6.5 \
    --tape_start_y=-11.0 \
    --tape_direction_x=0.0 \
    --tape_direction_y=1.0 \
    --beginning_tape_measure_reading=25 \
    --image_count=61 \
    --constants=constants.cc \
    --prefix data/cam8_0/debug_viewer_jpeg_
