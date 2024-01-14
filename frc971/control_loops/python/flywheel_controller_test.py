#!/usr/bin/python3

# Generates a test flywheel for flywheel_controller_test

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import flywheel

import numpy
import sys
import gflags
import glog

FLAGS = gflags.FLAGS

try:
    gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
    pass

kFlywheel = flywheel.FlywheelParams(name='FlywheelTest',
                                    motor=control_loop.Falcon(),
                                    G=(60.0 / 48.0),
                                    J=0.0035,
                                    q_pos=0.01,
                                    q_vel=10.0,
                                    q_voltage=4.0,
                                    r_pos=0.01,
                                    controller_poles=[.95])


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[0.0], [500.0], [0.0]])
        flywheel.PlotSpinup(params=kFlywheel, goal=R, iterations=400)
        return 0

    # Write the generated constants out to a file.
    if len(argv) != 5:
        glog.fatal('Expected .h file name and .cc file name')
    else:
        namespaces = ['frc971', 'control_loops', 'flywheel']
        flywheel.WriteFlywheel(kFlywheel, argv[1:3], argv[3:5], namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
