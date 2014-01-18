#!/bin/bash
#
# Updates the wrist controllers.

cd $(dirname $0)

./python/wrists.py wrists/top_wrist_motor_plant.h \
    wrists/top_wrist_motor_plant.cc \
    wrists/bottom_wrist_motor_plant.h \
    wrists/bottom_wrist_motor_plant.cc
