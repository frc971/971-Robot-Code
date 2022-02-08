#!/usr/bin/python3

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
import math
import sys
import math
from y2022.control_loops.python import catapult_lib
from matplotlib import pylab

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', True, 'If true, plot the loop response.')

ball_mass = 0.25
ball_diameter = 9.5 * 0.0254
lever = 17.5 * 0.0254

G = (14.0 / 72.0) * (12.0 / 33.0)


def AddResistance(motor, resistance):
    motor.resistance += resistance
    return motor

J_ball = 1.5 * ball_mass * lever * lever
# Assuming carbon fiber, calculate the mass of the bar.
M_bar = (1750 * lever * 0.0254 * 0.0254 * (1.0 - (1 - 0.07)**2.0))
# And the moment of inertia.
J_bar = 1.0 / 3.0 * M_bar * lever**2.0

# Do the same for a theoretical cup.  Assume a 40 thou thick carbon cup.
M_cup = (1750 * 0.0254 * 0.04 * 2 * math.pi * (ball_diameter / 2.)**2.0)
J_cup = M_cup * lever**2.0 + M_cup * (ball_diameter / 2.)**2.0

print("J ball", ball_mass * lever * lever)
print("J bar", J_bar)
print("bar mass", M_bar)
print("J cup", J_cup)
print("cup mass", M_cup)

J = (J_ball + J_bar + J_cup * 1.5)
print("J", J)

kFinisher = catapult_lib.CatapultParams(
    name='Finisher',
    motor=AddResistance(control_loop.NMotor(control_loop.Falcon(), 2), 0.03),
    G=G,
    J=J,
    lever=lever,
    q_pos=0.01,
    q_vel=10.0,
    q_voltage=4.0,
    r_pos=0.01,
    controller_poles=[.93],
    dt=0.0005)


def main(argv):
    # Do all our math with a lower voltage so we have headroom.
    U = numpy.matrix([[9.0]])
    print("For G:", G, " max speed ", catapult_lib.MaxSpeed(params=kFinisher, U=U, final_position = math.pi / 2.0))

    if FLAGS.plot:
        catapult_lib.PlotShot(kFinisher, U, final_position = math.pi / 4.0)

        gs = []
        speed = []
        for i in numpy.linspace(0.01, 0.15, 150):
            kFinisher.G = i
            gs.append(kFinisher.G)
            speed.append(catapult_lib.MaxSpeed(params=kFinisher, U=U, final_position = math.pi / 2.0))
        pylab.plot(gs, speed, label = "max_speed")
        pylab.show()
        return 0


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    sys.exit(main(argv))
