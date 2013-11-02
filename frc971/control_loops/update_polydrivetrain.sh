#!/bin/bash
#
# Updates the polydrivetrain controller and CIM models.

./python/polydrivetrain.py drivetrain/polydrivetrain_motor_plant.h \
    drivetrain/polydrivetrain_motor_plant.cc \
    drivetrain/polydrivetrain_cim_plant.h \
    drivetrain/polydrivetrain_cim_plant.cc
