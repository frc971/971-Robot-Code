#!/usr/bin/python3

# Generates profiled subsystem for use in
# static_zeroing_single_dof_profiled_subsystem_test

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import angular_system
import numpy
import sys
import gflags
import glog

FLAGS = gflags.FLAGS

try:
    gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
    pass

kWrappedSystem = angular_system.AngularSystemParams(
    name='TestWrappedSystem',
    motor=control_loop.Vex775Pro(),
    G=(1.0 / 35.0) * (20.0 / 40.0),
    radius=16.0 * 0.25 / (2.0 * numpy.pi) * 0.0254,
    J=5.4,
    q_pos=0.015,
    q_vel=0.3,
    kalman_q_pos=0.12,
    kalman_q_vel=2.00,
    kalman_q_voltage=40.0,
    kalman_r_position=0.05,
    wrap_point=2.0 * numpy.pi)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[0.1], [0.0]])
        angular_system.PlotMotion(kWrappedSystem, R)

    # Write the generated constants out to a file.
    if len(argv) != 7:
        glog.fatal(
            'Expected .h, .cc, and .json filenames and .json file name for the \
            static_zeroing_single_dof_profiled_subsystem_test and integral \
            static_zeroing_single_dof_profiled_subsystem_test.')
    else:
        namespaces = ['frc971', 'control_loops']
        angular_system.WriteAngularSystem(kWrappedSystem, argv[1:4], argv[4:7],
                                          namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
