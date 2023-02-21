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

kWrist = angular_system.AngularSystemParams(
    name='Wrist',
    motor=control_loop.BAG(),
    G=(6.0 / 48.0) * (20.0 / 100.0) * (24.0 / 36.0) * (36.0 / 60.0),
    # Use parallel axis theorem to get the moment of inertia around
    # the joint (I = I_cm + mh^2 = 0.001877 + 0.8332 * 0.0407162^2)
    J=0.003258,
    q_pos=0.40,
    q_vel=20.0,
    kalman_q_pos=0.12,
    kalman_q_vel=2.0,
    kalman_q_voltage=4.0,
    kalman_r_position=0.05,
    radius=5.71 * 0.0254)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[numpy.pi / 2.0], [0.0]])
        angular_system.PlotKick(kWrist, R)
        angular_system.PlotMotion(kWrist, R)
        return

    # Write the generated constants out to a file.
    if len(argv) != 5:
        glog.fatal(
            'Expected .h file name and .cc file name for the wrist and integral wrist.'
        )
    else:
        namespaces = ['y2023', 'control_loops', 'superstructure', 'wrist']
        angular_system.WriteAngularSystem(kWrist, argv[1:3], argv[3:5],
                                          namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
