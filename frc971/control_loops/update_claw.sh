#!/bin/bash
#
# Updates the claw controllers.

cd $(dirname $0)

./python/claw.py claw/unaugmented_top_claw_motor_plant.h \
    claw/unaugmented_top_claw_motor_plant.cc \
    claw/top_claw_motor_plant.h \
    claw/top_claw_motor_plant.cc \
    claw/unaugmented_bottom_claw_motor_plant.h \
    claw/unaugmented_bottom_claw_motor_plant.cc \
    claw/bottom_claw_motor_plant.h \
    claw/bottom_claw_motor_plant.cc
