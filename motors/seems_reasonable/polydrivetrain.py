#!/usr/bin/python

import sys
from motors.seems_reasonable import drivetrain
from frc971.control_loops.python import polydrivetrain

import gflags
import glog

__author__ = 'Austin Schuh (austin.linux@gmail.com)'

FLAGS = gflags.FLAGS

try:
    gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
    pass

def main(argv):
    if FLAGS.plot:
        polydrivetrain.PlotPolyDrivetrainMotions(drivetrain.kDrivetrain)
    elif len(argv) != 7:
        glog.fatal('Expected .h file name and .cc file name')
    else:
        polydrivetrain.WritePolyDrivetrain(
            argv[1:3],
            argv[3:5],
            argv[5:7],
            ['motors', 'seems_reasonable'],
            drivetrain.kDrivetrain,
            scalar_type='float')


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
