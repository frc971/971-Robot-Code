#!/usr/bin/python3

from __future__ import print_function
from frc971.control_loops.python import drivetrain
from frc971.control_loops.python import control_loop
import sys

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

# TODO(max): Change constants based on robot / CAD
kDrivetrain = drivetrain.DrivetrainParams(
    J=2.2,
    mass=44.7,
    # TODO(austin): Measure radius a bit better.
    robot_radius=0.25,
    wheel_radius=2.5 * 0.0254,
    motor_type=control_loop.Falcon(),
    num_motors=2,
    G=(14.0 / 68.0) * (30.0 / 54.0),
    q_pos=0.24,
    q_vel=2.5,
    efficiency=0.92,
    has_imu=False,
    force=True,
    kf_q_voltage=1.0,
    controller_poles=[0.82, 0.82])


def main(argv):
    argv = FLAGS(argv)
    glog.init()

    if FLAGS.plot:
        drivetrain.PlotDrivetrainMotions(kDrivetrain)
    elif len(argv) != 5:
        print("Expected .h file name and .cc file name")
    else:
        # Write the generated constants out to a file.
        drivetrain.WriteDrivetrain(argv[1:3], argv[3:5], 'y2023_bot3',
                                   kDrivetrain)


if __name__ == '__main__':
    sys.exit(main(sys.argv))
