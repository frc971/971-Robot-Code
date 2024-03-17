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

kTurret = angular_system.AngularSystemParams(
    name='Turret',
    motor=control_loop.KrakenFOC(),
    G=(14.0 / 60.0) * (28.0 / 48.0) * (22.0 / 100.0),
    # 1305 in^2 lb
    J=0.4,
    q_pos=0.60,
    q_vel=10.0,
    kalman_q_pos=0.12,
    kalman_q_vel=2.0,
    kalman_q_voltage=2.0,
    kalman_r_position=0.05,
    radius=24 * 0.0254,
    dt=0.005)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[numpy.pi], [0.0]])
        angular_system.PlotKick(kTurret, R)
        angular_system.PlotMotion(kTurret, R)

    # Write the generated constants out to a file.
    if len(argv) != 7:
        glog.fatal(
            'Expected .h file name and .cc file name for the turret and integral turret.'
        )
    else:
        namespaces = ['y2024', 'control_loops', 'superstructure', 'turret']
        angular_system.WriteAngularSystem(kTurret, argv[1:4], argv[4:7],
                                          namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
