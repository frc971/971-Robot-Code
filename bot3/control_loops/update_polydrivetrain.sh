#!/bin/bash
#
# Updates the polydrivetrain controllers and CIM models.

cd $(dirname $0)

export PYTHONPATH=../../frc971/control_loops/python

./python/polydrivetrain.py drivetrain/polydrivetrain_dog_motor_plant.h \
    drivetrain/polydrivetrain_dog_motor_plant.cc \
    drivetrain/polydrivetrain_clutch_motor_plant.h \
    drivetrain/polydrivetrain_clutch_motor_plant.cc \
    drivetrain/polydrivetrain_cim_plant.h \
    drivetrain/polydrivetrain_cim_plant.cc
