#!/usr/bin/python

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import linear_system
import numpy
import sys
import gflags
import glog

FLAGS = gflags.FLAGS

try:
    gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
    pass

kIntake = linear_system.LinearSystemParams(
    name='Intake',
    motor=control_loop.Vex775Pro(),
    # (1 / 35.0) * (20.0 / 40.0) -> 16 tooth sprocket on #25 chain
    G=(1.0 / 35.0) * (20.0 / 40.0),
    radius=16.0 * 0.25 / (2.0 * numpy.pi) * 0.0254,
    mass=5.4,
    q_pos=0.015,
    q_vel=0.3,
    kalman_q_pos=0.12,
    kalman_q_vel=2.00,
    kalman_q_voltage=40.0,
    kalman_r_position=0.05)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[0.1], [0.0]])
        linear_system.PlotMotion(kIntake, R)

    # Write the generated constants out to a file.
    if len(argv) != 5:
        glog.fatal(
            'Expected .h file name and .cc file name for the intake and integral intake.'
        )
    else:
        namespaces = ['y2017', 'control_loops', 'superstructure', 'intake']
        linear_system.WriteLinearSystem(kIntake, argv[1:3], argv[3:5],
                                        namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
