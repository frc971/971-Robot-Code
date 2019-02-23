#!/usr/bin/python

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
    gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
    pass

kIntake = angular_system.AngularSystemParams(
    name='Intake',
    motor=control_loop.Vex775Pro(),
    # (1 / 35.0) * (20.0 / 40.0) -> 16 tooth sprocket on #25 chain
    G=(12.0 / 56.0) * (14.0 / 54.0) * (18.0 / 64.0) * (16.0 / 48.0),
    J=0.34 - 0.03757568,
    q_pos=0.20,
    q_vel=5.0,
    dt=0.005,
    kalman_q_pos=0.12,
    kalman_q_vel=2.0,
    kalman_q_voltage=4.0,
    kalman_r_position=0.05)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[numpy.pi / 2.0], [0.0]])
        angular_system.PlotMotion(kIntake, R)

    # Write the generated constants out to a file.
    if len(argv) != 5:
        glog.fatal(
            'Expected .h file name and .cc file name for the intake and integral intake.'
        )
    else:
        namespaces = ['y2016', 'control_loops', 'superstructure']
        angular_system.WriteAngularSystem(kIntake, argv[1:3], argv[3:5],
                                          namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
