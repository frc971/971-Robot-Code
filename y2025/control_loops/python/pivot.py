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
#TODO replace constants for y2025 bot
kpivot = angular_system.AngularSystemParams(
    name='pivot',
    motor=control_loop.KrakenFOC(),
    G=(14. / 50.) * (24. / 64.) * (24. / 64.) * (12. / 48.),
    J=0.2,
    q_pos=0.20,
    q_vel=0.05,
    kalman_q_pos=0.12,
    kalman_q_vel=20.0,
    kalman_q_voltage=1.0,
    kalman_r_position=0.05,
    # radius is distance between pivot point and center of mass
    radius=0.0254 * ((5.955 - 3.099)**2 + (0.0 - 0.047)**2 +
                     (8.005 - 29.587)**2)**0.5,
    dt=0.005,
)


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[0.4], [0.0]])
        angular_system.PlotKick(kpivot, R)
        angular_system.PlotMotion(kpivot,
                                  R,
                                  max_velocity=2.0,
                                  max_acceleration=15.0)
        return
    # Write the generated constants out to a file.
    if len(argv) != 7:
        glog.fatal(
            'Expected .h file name and .cc file name for the pivot pivot and integral pivot pivot.'
        )
    else:
        namespaces = ['y2025', 'control_loops', 'superstructure', 'pivot']
        angular_system.WriteAngularSystem(kpivot, argv[1:4], argv[4:7],
                                          namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))