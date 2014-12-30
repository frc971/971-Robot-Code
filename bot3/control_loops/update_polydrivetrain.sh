#!/bin/bash
#
# Updates the polydrivetrain controllers and CIM models for the third robot.

cd $(dirname $0)

PYTHONPATH=$PYTHONPATH:../../frc971/control_loops/python \
./python/polydrivetrain.py drivetrain/polydrivetrain_dog_motor_plant.h \
    drivetrain/polydrivetrain_dog_motor_plant.cc \
    drivetrain/polydrivetrain_cim_plant.h \
    drivetrain/polydrivetrain_cim_plant.cc
