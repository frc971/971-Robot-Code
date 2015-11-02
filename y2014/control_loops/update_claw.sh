#!/bin/bash
#
# Updates the claw controllers.

export PYTHONPATH=../../frc971/control_loops/python

cd $(dirname $0)

./python/claw.py claw/claw_motor_plant.h \
    claw/claw_motor_plant.cc
