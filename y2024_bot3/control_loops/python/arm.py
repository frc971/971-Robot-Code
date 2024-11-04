#!/usr/bin/python3

from aos.util.trapezoid_profile import TrapezoidProfile
from frc971.control_loops.python import control_loop
from frc971.control_loops.python import linear_system
from frc971.control_loops.python import controls
import numpy
import sys
from matplotlib import pylab
import gflags
import glog

FLAGS = gflags.FLAGS

try:
    gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
    pass

gflags.DEFINE_bool('hybrid', False, 'If true, make it hybrid.')

kArm = linear_system.LinearSystemParams(
    name='Arm',
    motor=control_loop.KrakenFOC(),
    G=(14. / 60.) * (32. / 48.),
    radius=36 * 0.005 / numpy.pi / 2.0,
    mass=5.0,
    q_pos=0.20,
    q_vel=80.0,
    kalman_q_pos=0.12,
    kalman_q_vel=2.0,
    kalman_q_voltage=8.0,
    kalman_r_position=0.05,
    dt=0.005,
)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[0.4], [0.0]])
        linear_system.PlotKick(kArm, R)
        linear_system.PlotMotion(kArm,
                                 R,
                                 max_velocity=2.0,
                                 max_acceleration=15.0)
        return

    # Write the generated constants out to a file.
    if len(argv) != 7:
        glog.fatal(
            'Expected .h file name and .cc file name for the arm pivot and integral arm pivot.'
        )
    else:
        namespaces = ['y2024_bot3', 'control_loops', 'superstructure', 'arm']
        linear_system.WriteLinearSystem(kArm, argv[1:4], argv[4:7], namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
