#!/usr/bin/python3

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
import math
import scipy.optimize
import sys
import math
from y2022.control_loops.python import catapult_lib
from matplotlib import pylab

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

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

kCatapult = catapult_lib.CatapultParams(
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
    dt=0.00505)

catapult = catapult_lib.Catapult(kCatapult)

# Ideas for adjusting the cost function:
#
# Penalize battery current?
# Penalize accel/rotor current?
# Penalize velocity error off destination?
# Penalize max u
#
# Ramp up U cost over time?
# Once moving, open up saturation bounds
#
# We really want our cost function to be robust so that we can tolerate the
# battery not delivering like we want at the end.


def mpc_cost(X_initial, X_final, u_matrix):
    X = X_initial.copy()
    cost = 0.0
    last_u = u_matrix[0]
    max_u = 0.0
    for count, u in enumerate(u_matrix):
        v_prior = X[1, 0]
        X = catapult.A * X + catapult.B * numpy.matrix([[u]])
        v = X[1, 0]

        # Smoothness cost on voltage change and voltage.
        #cost += (u - last_u) ** 2.0
        #cost += (u - 6.0) ** 2.0

        measured_a = (v - v_prior) / catapult.dt
        expected_a = 0.0

        # Our good cost!
        cost_scalar = 0.02 * max(0.0, (20 - (len(u_matrix) - count)) / 20.)
        cost += ((measured_a - expected_a) * cost_scalar)**2.0
        cost += (measured_a * 0.010)**2.0

        # Quadratic cost.  This delays as long as possible, but approximates a
        # LQR until saturation.
        #cost += (u - 0.0) ** 2.0
        #cost += (0.1 * (X_final[0, 0] - X[0, 0])) ** 2.0
        #cost += (0.5 * (X_final[1, 0] - X[1, 0])) ** 2.0

        max_u = max(u, max_u)
        last_u = u

    # Penalize max power usage.  This is hard to solve.
    #cost += max_u * 10

    terminal_cost = (X - X_final).transpose() * numpy.matrix(
        [[10000.0, 0.0], [0.0, 10000.0]]) * (X - X_final)
    cost += terminal_cost[0, 0]

    return cost


def SolveCatapult(X_initial, X_final, u):
    """ Solves for the optimal action given a seed, state, and target """
    def vbat_constraint(z, i):
        return 12.0 - z[i]

    def forward(z, i):
        return z[i]

    def mpc_cost2(u_matrix):
        return mpc_cost(X_initial, X_final, u_matrix)

    constraints = [{
        'type': 'ineq',
        'fun': vbat_constraint,
        'args': (i, )
    } for i in numpy.arange(len(u))]

    constraints += [{
        'type': 'ineq',
        'fun': forward,
        'args': (i, )
    } for i in numpy.arange(len(u))]

    result = scipy.optimize.minimize(mpc_cost2,
                                     u,
                                     method='SLSQP',
                                     constraints=constraints)
    print(result)

    return result.x


def CatapultProblem():
    c = catapult_lib.Catapult(kCatapult)

    kHorizon = 40

    u = [0.0] * kHorizon
    X_initial = numpy.matrix([[0.0], [0.0]])
    X_final = numpy.matrix([[2.0], [25.0]])


    X_initial = numpy.matrix([[0.0], [0.0]])
    X = X_initial.copy()

    t_samples = [0.0]
    x_samples = [0.0]
    v_samples = [0.0]
    a_samples = [0.0]

    # Iteratively solve our MPC and simulate it.
    u_samples = []
    for i in range(kHorizon):
        u_horizon = SolveCatapult(X, X_final, u)
        u_samples.append(u_horizon[0])
        v_prior = X[1, 0]
        X = c.A * X + c.B * numpy.matrix([[u_horizon[0]]])
        v = X[1, 0]
        t_samples.append(t_samples[-1] + c.dt)
        x_samples.append(X[0, 0])
        v_samples.append(X[1, 0])
        a_samples.append((v - v_prior) / c.dt)

        u = u_horizon[1:]

    print('Final state', X.transpose())
    print('Final velocity', X[1, 0] * lever)
    pylab.subplot(2, 1, 1)
    pylab.plot(t_samples, x_samples, label="x")
    pylab.plot(t_samples, v_samples, label="v")
    pylab.plot(t_samples[1:], u_samples, label="u")
    pylab.legend()
    pylab.subplot(2, 1, 2)
    pylab.plot(t_samples, a_samples, label="a")
    pylab.legend()

    pylab.show()


def main(argv):
    # Do all our math with a lower voltage so we have headroom.
    U = numpy.matrix([[9.0]])
    print(
        "For G:", G, " max speed ",
        catapult_lib.MaxSpeed(params=kCatapult,
                              U=U,
                              final_position=math.pi / 2.0))

    CatapultProblem()

    if FLAGS.plot:
        catapult_lib.PlotShot(kCatapult, U, final_position=math.pi / 4.0)

        gs = []
        speed = []
        for i in numpy.linspace(0.01, 0.15, 150):
            kCatapult.G = i
            gs.append(kCatapult.G)
            speed.append(
                catapult_lib.MaxSpeed(params=kCatapult,
                                      U=U,
                                      final_position=math.pi / 2.0))
        pylab.plot(gs, speed, label="max_speed")
        pylab.show()
        return 0


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    sys.exit(main(argv))
