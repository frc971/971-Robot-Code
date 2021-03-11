#!/usr/bin/python3

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
    1.0 + numpy.power(G_per_wheel, -2.0) + numpy.power(G_per_wheel, -4.0) + numpy.power(G_per_wheel, -6.0))

# The position and velocity are measured for the final wheel.
kAccelerator = flywheel.FlywheelParams(
    name='Accelerator',
    motor=control_loop.Falcon(),
    G=G,
    J=J * 1.3,
    q_pos=0.01,
    q_vel=40.0,
    q_voltage=1.0,
    r_pos=0.03,
    controller_poles=[.89])


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[0.0], [500.0], [0.0]])
        flywheel.PlotSpinup(kAccelerator, goal=R, iterations=400)
        return 0

    glog.debug("J is %f" % J)

    if len(argv) != 5:
        glog.fatal('Expected .h file name and .cc file name')
    else:
        namespaces = [
            'y2020', 'control_loops', 'superstructure', 'accelerator'
        ]
        flywheel.WriteFlywheel(kAccelerator, argv[1:3], argv[3:5], namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
