#!/usr/bin/python3

from frc971.control_loops.python import control_loop
from y2020.control_loops.python import flywheel
import numpy

import sys

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

# Gear ratio to the final wheel.
# 40 tooth on the flywheel
# 48 for the falcon.
# 60 tooth on the outer wheel.
G = 44.0 / 40.0
# Overall flywheel inertia.
J = 0.00507464
J = 0.008

def AddResistance(motor, resistance):
    motor.resistance += resistance
    return motor

def ScaleKv(motor, scale):
    motor.Kv *= scale
    return motor

# The position and velocity are measured for the final wheel.
kFinisher = flywheel.FlywheelParams(
    name='Finisher',
    motor=AddResistance(control_loop.NMotor(control_loop.Falcon(), 2), 0.03),
    G=G,
    J=J,
    q_pos=0.01,
    q_vel=10.0,
    q_voltage=4.0,
    r_pos=0.01,
    controller_poles=[.89])


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[0.0], [500.0], [0.0]])
        flywheel.PlotSpinup(params=kFinisher, goal=R, iterations=400)
        return 0

    if len(argv) != 5:
        glog.fatal('Expected .h file name and .cc file name')
    else:
        namespaces = ['y2020', 'control_loops', 'superstructure', 'finisher']
        flywheel.WriteFlywheel(kFinisher, argv[1:3], argv[3:5], namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    sys.exit(main(argv))
