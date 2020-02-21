#!/usr/bin/python

from frc971.control_loops.python import control_loop
from y2020.control_loops.python import flywheel
import numpy

import sys

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

# Inertia for a single 4" diameter, 2" wide neopreme wheel.
J_wheel = 0.000319 * 2.0
# Gear ratio to the final wheel.
# 40 tooth on the flywheel
# 48 for the falcon.
# 60 tooth on the outer wheel.
G = 48.0 / 40.0
# Overall flywheel inertia.
J = J_wheel * (1.0 + (40.0 / 60.0)**2.0)

# The position and velocity are measured for the final wheel.
kFinisher = flywheel.FlywheelParams(
    name='Finisher',
    motor=control_loop.Falcon(),
    G=G,
    J=J,
    q_pos=0.08,
    q_vel=4.00,
    q_voltage=0.3,
    r_pos=0.05,
    controller_poles=[.87])


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[0.0], [500.0], [0.0]])
        flywheel.PlotSpinup(params=kFinisher, goal=R, iterations=200)
        return 0

    if len(argv) != 5:
        glog.fatal('Expected .h file name and .cc file name')
    else:
        namespaces = ['y2020', 'control_loops', 'superstructure', 'finisher']
        flywheel.WriteFlywheel(kFinisher, argv[1:3], argv[3:5], namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    sys.exit(main(argv))
