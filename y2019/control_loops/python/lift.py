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

kLift = linear_system.LinearSystemParams(
    name='Lift',
    motor=control_loop.Vex775Pro(),
    G=(12.0 / 100.0) * (14.0 / 52.0),
    # 5mm pitch, 18 tooth
    radius=0.005 * 18.0 / (2.0 * numpy.pi),
    # Or, 2.34 lb * 2 (2.1 kg) when lifting back up
    mass=40.0,
    q_pos=0.070,
    q_vel=1.2,
    kalman_q_pos=0.12,
    kalman_q_vel=2.00,
    kalman_q_voltage=35.0,
    kalman_r_position=0.05,
    dt=0.00505)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[1.5], [0.0]])
        linear_system.PlotKick(kLift, R)
        linear_system.PlotMotion(kLift, R, max_velocity=5.0)

    # Write the generated constants out to a file.
    if len(argv) != 5:
        glog.fatal(
            'Expected .h file name and .cc file name for the elevator and integral elevator.'
        )
    else:
        namespaces = ['y2019', 'control_loops', 'superstructure', 'lift']
        linear_system.WriteLinearSystem(kLift, argv[1:3], argv[3:5],
                                        namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
