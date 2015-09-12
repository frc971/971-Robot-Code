#!/bin/bash
#
# Updates the elevator controllers.

cd $(dirname $0)

./python/elevator3.py elevator/elevator_motor_plant.h \
    elevator/elevator_motor_plant.cc
