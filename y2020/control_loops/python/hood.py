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
    gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
    pass

# Hood is an angular subsystem due to the mounting of the encoder on the hood
# joint.  We are currently electing to ignore potential non-linearity.

range_of_travel_radians = (38.0 * numpy.pi / 180.0)
# 0.5 inches/turn
# 6.7725 inches of travel
turns_of_leadscrew_per_range_of_travel = 6.7725 / 0.5

radians_per_turn = range_of_travel_radians / turns_of_leadscrew_per_range_of_travel

radians_per_turn_of_motor = 12.0 / 60.0 * radians_per_turn

kHood = angular_system.AngularSystemParams(
    name='Hood',
    motor=control_loop.BAG(),
    G=radians_per_turn_of_motor / (2.0 * numpy.pi),
    J=0.1,
    q_pos=0.15,
    q_vel=10.0,
    kalman_q_pos=0.12,
    kalman_q_vel=10.0,
    kalman_q_voltage=30.0,
    kalman_r_position=0.02)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[numpy.pi / 4.0], [0.0]])
        angular_system.PlotKick(kHood, R)
        angular_system.PlotMotion(kHood, R)

    glog.debug("Radians per turn: %f\n", radians_per_turn)

    # Write the generated constants out to a file.
    if len(argv) != 5:
        glog.fatal(
            'Expected .h file name and .cc file name for the hood and integral hood.'
        )
    else:
        namespaces = ['y2020', 'control_loops', 'superstructure', 'hood']
        angular_system.WriteAngularSystem(kHood, argv[1:3], argv[3:5],
                                          namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
