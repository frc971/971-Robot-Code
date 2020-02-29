#!/usr/bin/python

from aos.util.trapezoid_profile import TrapezoidProfile
from frc971.control_loops.python import control_loop
from frc971.control_loops.python import angular_system
from frc971.control_loops.python import controls
import copy
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
    motor=control_loop.Vex775Pro(),
    G=(6.0 / 60.0) * (26.0 / 150.0),
    J=0.20,
    q_pos=0.30,
    q_vel=4.5,
    kalman_q_pos=0.12,
    kalman_q_vel=10.0,
    kalman_q_voltage=12.0,
    kalman_r_position=0.05)

def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[numpy.pi], [0.0]])
        angular_system.PlotKick(kTurret, R)
        angular_system.PlotMotion(kTurret, R)

    # Write the generated constants out to a file.
    if len(argv) != 5:
        glog.fatal(
            'Expected .h file name and .cc file name for the turret.'
        )
    else:
        namespaces = ['y2020', 'control_loops', 'superstructure', 'turret']
        angular_system.WriteAngularSystem([kTurret],
                                          argv[1:3], argv[3:5], namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
