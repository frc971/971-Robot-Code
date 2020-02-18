#!/usr/bin/python

from frc971.control_loops.python import control_loop
from y2020.control_loops.python import flywheel
import numpy

import sys

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

kFinisher = flywheel.FlywheelParams(
    name='Finisher',
    motor=control_loop.Falcon(),
    G=1.0,
    J=0.006,
    q_pos=0.08,
    q_vel=4.00,
    q_voltage=0.3,
    r_pos=0.05,
    controller_poles=[.87],
    dt=0.00505)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[0.0], [100.0], [0.0]])
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
