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

kGroundIntakePivot = angular_system.AngularSystemParams(
    name="GroundIntakePivot",
    motor=control_loop.KrakenX44Trapezoidal(),
    G=(12 / 60) * (15 / 60),
    J=0.01685,  # 57.6 in^2 lb
    q_pos=1.0,
    q_vel=800.0,
    kalman_q_pos=0.12,
    kalman_q_vel=2.0,
    kalman_q_voltage=1.0,
    kalman_r_position=0.05,
    radius=12.12 * 0.0254,
    enable_voltage_error=False,
    dt=0.005,
)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[numpy.pi / 2.0], [0.0]])
        angular_system.PlotKick(kGroundIntakePivot, R)
        angular_system.PlotMotion(kGroundIntakePivot, R)
        return
    if len(argv) != 7:
        glog.fatal(
            "Expected .h file name and .cc file name for the intake pivot and integral intake pivot."
        )
    else:
        namespaces = [
            "y2025", "control_loops", "superstructure", "ground_intake_pivot"
        ]
        angular_system.WriteAngularSystem(kGroundIntakePivot, argv[1:4],
                                          argv[4:7], namespaces)


if __name__ == "__main__":
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
