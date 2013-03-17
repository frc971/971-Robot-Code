#!/bin/bash
#
# Updates the angle adjust controller.

./python/angle_adjust.py angle_adjust/angle_adjust_motor_plant.h \
    angle_adjust/angle_adjust_motor_plant.cc \
    angle_adjust/unaugmented_angle_adjust_motor_plant.h \
    angle_adjust/unaugmented_angle_adjust_motor_plant.cc \
