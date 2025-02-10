#!/usr/bin/python3

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import linear_system
import copy
import numpy
import sys
import gflags
import glog

FLAGS = gflags.FLAGS

try:
    gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
    pass

kElevator = linear_system.LinearSystemParams(
    name='Elevator',
    motor=control_loop.KrakenFOC(),
    G=(8.0 / 62.0),
    radius=(1.751 / 2.0) * 0.0254,
    mass=(5.554 / 2.205) + ((11.665 * 2) / 2.205),
    q_pos=0.0070,
    q_vel=1.35,
    kalman_q_pos=0.12,
    kalman_q_vel=0.20,
    kalman_q_voltage=35.0,
    kalman_r_position=0.05,
    dt=0.005,
)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[1.5], [0.0]])
        linear_system.PlotKick(kElevator, R)
        linear_system.PlotMotion(kElevator,
                                 R,
                                 max_velocity=2.0,
                                 max_acceleration=15.0)

    # Write the generated constants out to a file.
    if len(argv) != 7:
        glog.fatal(
            'Expected .h file name and .cc file name for the elevator and integral elevator.'
        )
    else:
        namespaces = ['y2025', 'control_loops', 'superstructure', 'elevator']
        linear_system.WriteLinearSystem(kElevator, argv[1:4], argv[4:7],
                                        namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
