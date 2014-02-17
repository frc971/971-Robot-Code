#!/bin/bash
#
# Updates the claw controllers.

cd $(dirname $0)

./python/claw.py claw/claw_motor_plant.h \
    claw/claw_motor_plant.cc
