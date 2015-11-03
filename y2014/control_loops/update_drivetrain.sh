#!/bin/bash
#
# Updates the drivetrain controllers (for both robots).

export PYTHONPATH=../../frc971/control_loops/python

cd $(dirname $0)

./python/drivetrain.py drivetrain/drivetrain_dog_motor_plant.h \
    drivetrain/drivetrain_dog_motor_plant.cc \
    drivetrain/drivetrain_clutch_motor_plant.h \
    drivetrain/drivetrain_clutch_motor_plant.cc
