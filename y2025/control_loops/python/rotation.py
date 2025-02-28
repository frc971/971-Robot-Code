#!/usr/bin/python3
from aos.util.trapezoid_profile import TrapezoidProfile
from frc971.control_loops.python import control_loop
from frc971.control_loops.python import angular_system_current
from frc971.control_loops.python import controls
import numpy
import sys
from matplotlib import pylab
import gflags
import glog
import matplotlib

FLAGS = gflags.FLAGS
try:
    gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
    pass
kRotation = angular_system_current.AngularSystemCurrentParams(
    name='Rotation',
    motor=control_loop.KrakenX60FOC(),
    G=9.0 / 24.0 * 14.0 / 72.0,
    J=3.1 / 1000.0,
    q_pos=0.05,
    q_vel=20.0,
    kalman_q_pos=0.12,
    kalman_q_vel=2.0,
    kalman_q_voltage=4.0,
    kalman_r_position=0.05,
    radius=25 * 0.0254,
    wrap_point=2.0 * numpy.pi)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[numpy.pi / 2.0], [0.0]])
        angular_system_current.PlotKick(kRotation, R)
        angular_system_current.PlotMotion(kRotation, R)
        return
    # Write the generated constants out to a file.
    if len(argv) != 7:
        glog.fatal(
            'Expected .h file name and .cc file name for the wrist and integral wrist.'
        )
    else:
        namespaces = ['y2025', 'control_loops', 'drivetrain']
        angular_system_current.WriteAngularSystemCurrent(
            kRotation, argv[1:4], argv[4:7], namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
