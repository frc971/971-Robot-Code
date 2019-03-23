#!/usr/bin/python

from frc971.control_loops.python import drivetrain
import sys

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

#update mass (120lbs right now)
#robot radius needs confirming(set as distance of center wheels from each other)
#J needs updating

# TODO(austin): wpilib
# Encoder is on a 24 tooth gear attached to the output gear.

kDrivetrain = drivetrain.DrivetrainParams(
    J=1.5,
    mass=38.5,
    # TODO(austin): Measure.
    robot_radius=0.45 / 2.0,
    wheel_radius=4.0 * 0.0254 / 2.0,
    G=9.0 / 52.0,
    q_pos=0.14,
    q_vel=1.30,
    efficiency=0.80,
    has_imu=True,
    force=True,
    kf_q_voltage=13.0,
    controller_poles=[0.82, 0.82],
    robot_cg_offset=0.0)


def main(argv):
    argv = FLAGS(argv)
    glog.init()

    if FLAGS.plot:
        drivetrain.PlotDrivetrainMotions(kDrivetrain)
    elif len(argv) != 5:
        print "Expected .h file name and .cc file name"
    else:
        # Write the generated constants out to a file.
        drivetrain.WriteDrivetrain(argv[1:3], argv[3:5], 'y2019', kDrivetrain)

if __name__ == '__main__':
    sys.exit(main(sys.argv))
