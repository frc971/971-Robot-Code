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

# Wrist alone
#  0.1348
# Wrist with ball
#  0.3007
# Wrist with hatch
#  0.446

kWrist = angular_system.AngularSystemParams(
    name='Wrist',
    motor=control_loop.BAG(),
    G=(6.0 / 60.0) * (20.0 / 100.0) * (24.0 / 84.0),
    J=0.30,
    q_pos=0.20,
    q_vel=5.0,
    kalman_q_pos=0.12,
    kalman_q_vel=2.0,
    kalman_q_voltage=4.0,
    kalman_r_position=0.05)

kWristBall = copy.copy(kWrist)
kWristBall.J = 0.4007
kWristBall.q_pos = 0.55
kWristBall.q_vel = 5.0

kWristPanel = copy.copy(kWrist)
kWristPanel.J = 0.446

kWristModel = copy.copy(kWrist)
kWristModel.J = 0.1348


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[numpy.pi / 2.0], [0.0]])
        angular_system.PlotKick(kWristBall, R, plant_params=kWristBall)
        angular_system.PlotMotion(kWristBall, R, plant_params=kWristBall)

    # Write the generated constants out to a file.
    if len(argv) != 5:
        glog.fatal(
            'Expected .h file name and .cc file name for the wrist and integral wrist.'
        )
    else:
        namespaces = ['y2019', 'control_loops', 'superstructure', 'wrist']
        angular_system.WriteAngularSystem([kWrist, kWristBall, kWristPanel],
                                          argv[1:3], argv[3:5], namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
