#!/bin/bash
#
# Updates the claw controllers (for both robots).

cd $(dirname $0)

export PYTHONPATH=../../frc971/control_loops/python

./python/claw.py claw/claw_motor_plant.cc \
    claw/claw_motor_plant.h
