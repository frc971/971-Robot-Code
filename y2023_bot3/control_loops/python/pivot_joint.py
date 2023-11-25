#!/usr/bin/python3

from aos.util.trapezoid_profile import TrapezoidProfile
from frc971.control_loops.python import control_loop
from frc971.control_loops.python import angular_system
from frc971.control_loops.python import controls
import numpy
import sys
from matplotlib import pylab
import gflags
import glog

FLAGS = gflags.FLAGS

try:
    gflags.DEFINE_bool("plot", False, "If true, plot the loop response.")
except gflags.DuplicateFlagError:
    pass

kPivotJoint = angular_system.AngularSystemParams(
    name="PivotJoint",
    motor=control_loop.Falcon(),
    G=(14 / 50) * (24 / 64) * (24 / 64) * (15 / 60),
    # Use parallel axis theorem to get the moment of inertia around
    # the joint (I = I_cm + mh^2 = 0.001877 + 0.8332 * 0.0407162^2)
    J=(0.13372 * 2.00),
    q_pos=0.80,
    q_vel=80.0,
    kalman_q_pos=0.12,
    kalman_q_vel=2.0,
    kalman_q_voltage=1.5,
    kalman_r_position=0.05,
    radius=5.71 * 0.0254,
)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[numpy.pi / 2.0], [0.0]])
        angular_system.PlotKick(kPivotJoint, R)
        angular_system.PlotMotion(kPivotJoint, R)
        return

    # Write the generated constants out to a file.
    if len(argv) != 5:
        glog.fatal(
            "Expected .h file name and .cc file name for the pivot joint and integral pivot joint."
        )
    else:
        namespaces = [
            "y2023_bot3", "control_loops", "superstructure", "pivot_joint"
        ]
        angular_system.WriteAngularSystem(kPivotJoint, argv[1:3], argv[3:5],
                                          namespaces)


if __name__ == "__main__":
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
