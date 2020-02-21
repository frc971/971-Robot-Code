#!/usr/bin/python

from frc971.control_loops.python import control_loop
from y2020.control_loops.python import flywheel
import numpy

import sys

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

# Inertia for a single 4" diameter, 1" wide neopreme wheel.
J_wheel = 0.000319
# Gear ratio between wheels (speed up!)
G_per_wheel = 1.2
# Gear ratio to the final wheel.
G = (30.0 / 40.0) * numpy.power(G_per_wheel, 3.0)
# Overall flywheel inertia.
J = J_wheel * (
    1.0 + numpy.power(G, -2.0) + numpy.power(G, -4.0) + numpy.power(G, -6.0))

# The position and velocity are measured for the final wheel.
kAccelerator = flywheel.FlywheelParams(
    name='Accelerator',
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
        R = numpy.matrix([[0.0], [100.0], [0.0]])
        flywheel.PlotSpinup(kAccelerator, goal=R, iterations=200)
        return 0

    if len(argv) != 5:
        glog.fatal('Expected .h file name and .cc file name')
    else:
        namespaces = [
            'y2020', 'control_loops', 'superstructure', 'accelerator'
        ]
        flywheel.WriteFlywheel(kAccelerator, argv[1:3], argv[3:5], namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    sys.exit(main(argv))
