#!/bin/bash
#
# Updates the drivetrain controllers (for both robots).

cd $(dirname $0)

export PYTHONPATH=../../frc971/control_loops/python

./python/drivetrain.py drivetrain/drivetrain_dog_motor_plant.h \
    drivetrain/drivetrain_dog_motor_plant.cc \
    drivetrain/drivetrain_clutch_motor_plant.h \
    drivetrain/drivetrain_clutch_motor_plant.cc
