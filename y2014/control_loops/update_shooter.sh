#!/bin/bash
#
# Updates the shooter controller.

cd $(dirname $0)

./python/shooter.py shooter/shooter_motor_plant.h \
    shooter/shooter_motor_plant.cc \
    shooter/unaugmented_shooter_motor_plant.h \
    shooter/unaugmented_shooter_motor_plant.cc
