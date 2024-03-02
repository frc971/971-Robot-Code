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

gflags.DEFINE_bool('hybrid', False, 'If true, make it hybrid.')


def AddResistance(motor, resistance):
    motor.resistance += resistance
    return motor


kCatapultWithGamePiece = angular_system.AngularSystemParams(
    name='Catapult',
    # Add the battery series resistance to make it better match.
    motor=AddResistance(control_loop.NMotor(control_loop.KrakenFOC(), 2),
                        0.00),
    G=(14.0 / 60.0) * (12.0 / 24.0),
    # 208.7328 in^2 lb
    J=0.065 + 0.04,
    q_pos=0.80,
    q_vel=15.0,
    kalman_q_pos=0.12,
    kalman_q_vel=2.0,
    kalman_q_voltage=0.7,
    kalman_r_position=0.05,
    radius=12 * 0.0254,
    delayed_u=1)

kCatapultWithoutGamePiece = angular_system.AngularSystemParams(
    name='Catapult',
    # Add the battery series resistance to make it better match.
    motor=AddResistance(control_loop.NMotor(control_loop.KrakenFOC(), 2),
                        0.00),
    G=(14.0 / 60.0) * (12.0 / 24.0),
    # 135.2928 in^2 lb
    J=0.06,
    q_pos=0.80,
    q_vel=15.0,
    kalman_q_pos=0.12,
    kalman_q_vel=2.0,
    kalman_q_voltage=0.7,
    kalman_r_position=0.05,
    radius=12 * 0.0254,
    delayed_u=1)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[numpy.pi / 2.0], [0.0]])
        angular_system.PlotKick(kCatapultWithGamePiece, R)
        angular_system.PlotMotion(kCatapultWithGamePiece, R)
        return

    # Write the generated constants out to a file.
    if len(argv) != 7:
        glog.fatal(
            'Expected .h file name and .cc file name for the intake and integral intake.'
        )
    else:
        namespaces = ['y2024', 'control_loops', 'superstructure', 'catapult']
        angular_system.WriteAngularSystem(
            [kCatapultWithoutGamePiece, kCatapultWithGamePiece], argv[1:4],
            argv[4:7], namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
