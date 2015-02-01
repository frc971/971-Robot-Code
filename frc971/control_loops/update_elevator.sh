#!/bin/bash
#
# Updates the elevator controllers (for both robots).

cd $(dirname $0)

./python/elevator.py fridge/elevator_motor_plant.cc \
    fridge/elevator_motor_plant.h
