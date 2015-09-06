#!/bin/bash
#
# Updates the arm controllers (for both robots).

cd $(dirname $0)

./python/arm.py fridge/arm_motor_plant.cc \
    fridge/arm_motor_plant.h \
    fridge/integral_arm_plant.cc \
    fridge/integral_arm_plant.h
