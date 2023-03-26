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

kRoll = angular_system.AngularSystemParams(
    name='Roll',
    motor=control_loop.BAG(),
    G=18.0 / 48.0 * 1.0 / 36.0,
    # 598.006 in^2 lb
    J=0.175 * 3.0,
    q_pos=0.40,
    q_vel=5.0,
    kalman_q_pos=0.12,
    kalman_q_vel=2.0,
    kalman_q_voltage=4.0,
    kalman_r_position=0.05,
    radius=13 * 0.0254)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[numpy.pi / 2.0], [0.0]])
        angular_system.PlotKick(kRoll, R)
        angular_system.PlotMotion(kRoll, R)
        return

    # Write the generated constants out to a file.
    if len(argv) != 5:
        glog.fatal(
            'Expected .h file name and .cc file name for the intake and integral intake.'
        )
    else:
        namespaces = ['y2023', 'control_loops', 'superstructure', 'roll']
        if FLAGS.hybrid:
            kRoll.name = 'HybridRoll'
            angular_system.WriteAngularSystem(
                kRoll,
                argv[1:3],
                argv[3:5],
                namespaces,
                plant_type='StateFeedbackHybridPlant',
                observer_type='HybridKalman')
        else:
            angular_system.WriteAngularSystem(kRoll, argv[1:3], argv[3:5],
                                              namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
