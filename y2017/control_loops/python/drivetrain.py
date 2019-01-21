#!/usr/bin/python

from frc971.control_loops.python import drivetrain
import sys

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

kDrivetrain = drivetrain.DrivetrainParams(
    J=6.0,
    mass=52,
    robot_radius=0.59055 / 2.0,
    wheel_radius=0.08255 / 2.0,
    G=11.0 / 60.0,
    q_pos_low=0.12,
    q_pos_high=0.14,
    q_vel_low=1.0,
    q_vel_high=0.95,
    has_imu=False)

def main(argv):
    argv = FLAGS(argv)
    glog.init()

    if FLAGS.plot:
        drivetrain.PlotDrivetrainMotions(kDrivetrain)
    elif len(argv) != 5:
        print "Expected .h file name and .cc file name"
    else:
        # Write the generated constants out to a file.
        drivetrain.WriteDrivetrain(argv[1:3], argv[3:5], 'y2017', kDrivetrain)

if __name__ == '__main__':
    sys.exit(main(sys.argv))
