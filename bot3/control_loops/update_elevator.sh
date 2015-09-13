#!/bin/bash
#
# Updates the elevator controllers.

cd $(dirname $0)

export PYTHONPATH=../../frc971/control_loops/python

./python/elevator3.py elevator/elevator_motor_plant.h \
    elevator/elevator_motor_plant.cc elevator/integral_elevator_motor_plant.cc elevator/integral_elevator_motor_plant.h
