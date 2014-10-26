#!/bin/bash
#
# Updates the drivetrain controllers for the third robot.

cd $(dirname $0)

PYTHONPATH=$PYTHONPATH:../../frc971/control_loops/python \
./python/drivetrain.py drivetrain/drivetrain_dog_motor_plant.h \
    drivetrain/drivetrain_dog_motor_plant.cc
