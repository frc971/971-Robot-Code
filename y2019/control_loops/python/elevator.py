#!/usr/bin/python

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

first_stage_mass = 0.7957
carriage_mass = 2.754

kElevator = linear_system.LinearSystemParams(
    name='Elevator',
    motor=control_loop.Vex775Pro(),
    G=(8.0 / 82.0),
    radius=2.25 * 0.0254 / 2.0,
    mass=first_stage_mass + carriage_mass,
    q_pos=0.070,
    q_vel=1.35,
    kalman_q_pos=0.12,
    kalman_q_vel=2.00,
    kalman_q_voltage=35.0,
    kalman_r_position=0.05)

kElevatorBall = copy.copy(kElevator)
kElevatorBall.q_pos = 0.15
kElevatorBall.q_vel = 1.5

kElevatorModel = copy.copy(kElevator)
kElevatorModel.mass = carriage_mass + first_stage_mass + 1.0


def main(argv):
    if FLAGS.plot:
        R = numpy.matrix([[1.5], [0.0]])
        linear_system.PlotKick(kElevatorBall, R, plant_params=kElevatorModel)
        linear_system.PlotMotion(
            kElevatorBall, R, max_velocity=5.0, plant_params=kElevatorModel)

    # Write the generated constants out to a file.
    if len(argv) != 5:
        glog.fatal(
            'Expected .h file name and .cc file name for the elevator and integral elevator.'
        )
    else:
        namespaces = ['y2019', 'control_loops', 'superstructure', 'elevator']
        linear_system.WriteLinearSystem([kElevator, kElevatorBall, kElevator],
                                        argv[1:3], argv[3:5], namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
