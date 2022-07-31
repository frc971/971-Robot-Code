#!/usr/bin/python3

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
import osqp
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

J = (0.0 * J_ball + J_bar + J_cup * 0.0)
JEmpty = (J_bar + J_cup * 0.0)

kCatapultWithBall = catapult_lib.CatapultParams(
    name='Catapult',
    motor=AddResistance(control_loop.NMotor(control_loop.Falcon(), 2), 0.01),
    G=G,
    J=J,
    radius=lever,
    q_pos=2.8,
    q_vel=20.0,
    kalman_q_pos=0.01,
    kalman_q_vel=1.0,
    kalman_q_voltage=1.5,
    kalman_r_position=0.001)

kCatapultEmpty = catapult_lib.CatapultParams(
    name='Catapult',
    motor=AddResistance(control_loop.NMotor(control_loop.Falcon(), 2), 0.02),
    G=G,
    J=JEmpty,
    radius=lever,
    q_pos=2.8,
    q_vel=20.0,
    kalman_q_pos=0.12,
    kalman_q_vel=1.0,
    kalman_q_voltage=1.5,
    kalman_r_position=0.05)

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


def quadratic_cost(catapult, X_initial, X_final, horizon):
    Q_final = numpy.matrix([[10000.0, 0.0], [0.0, 10000.0]])

    As = numpy.vstack([catapult.A**(n + 1) for n in range(0, horizon)])
    Af = catapult.A**horizon

    Bs = numpy.matrix(numpy.zeros((2 * horizon, horizon)))
    for n in range(0, horizon):
        for m in range(0, n + 1):
            Bs[n * 2:(n * 2) + 2, m] = catapult.A**(n - m) * catapult.B

    Bf = Bs[horizon * 2 - 2:, :]

    P_final = 2.0 * Bf.transpose() * Q_final * Bf
    q_final = (2.0 * (Af * X_initial - X_final).transpose() * Q_final *
               Bf).transpose()

    constant_final = (Af * X_initial - X_final).transpose() * Q_final * (
        Af * X_initial - X_final)

    m = numpy.matrix([[catapult.A[1, 1]**(n + 1)] for n in range(horizon)])
    M = Bs[1:horizon * 2:2, :]

    W = numpy.matrix(
        numpy.identity(horizon) -
        numpy.eye(horizon, horizon, -1)) / catapult.dt
    w = -numpy.matrix(numpy.eye(horizon, 1, 0)) / catapult.dt

    Pi = numpy.diag([
        (0.01**2.0) + (0.02 * max(0.0, 20 - (horizon - n)) / 20.0)**2.0
        for n in range(horizon)
    ])

    P_accel = 2.0 * M.transpose() * W.transpose() * Pi * W * M
    q_accel = 2.0 * ((
        (W * m + w) * X_initial[1, 0]).transpose() * Pi * W * M).transpose()
    constant_accel = ((W * m + w) * X_initial[1, 0]).transpose() * Pi * (
        (W * m + w) * X_initial[1, 0])

    return ((P_accel + P_final), (q_accel + q_final),
            (constant_accel + constant_final))


def new_cost(catapult, X_initial, X_final, u):
    u_matrix = numpy.matrix(u).transpose()
    Q_final = numpy.matrix([[10000.0, 0.0], [0.0, 10000.0]])

    As = numpy.vstack([catapult.A**(n + 1) for n in range(0, len(u))])
    Af = catapult.A**len(u)

    Bs = numpy.matrix(numpy.zeros((2 * len(u), len(u))))
    for n in range(0, len(u)):
        for m in range(0, n + 1):
            Bs[n * 2:(n * 2) + 2, m] = catapult.A**(n - m) * catapult.B

    Bf = Bs[len(u) * 2 - 2:, :]

    P_final = 2.0 * Bf.transpose() * Q_final * Bf
    q_final = (2.0 * (Af * X_initial - X_final).transpose() * Q_final *
               Bf).transpose()

    constant_final = (Af * X_initial - X_final).transpose() * Q_final * (
        Af * X_initial - X_final)

    m = numpy.matrix([[catapult.A[1, 1]**(n + 1)] for n in range(len(u))])
    M = Bs[1:len(u) * 2:2, :]

    W = numpy.matrix(numpy.identity(len(u)) -
                     numpy.eye(len(u), len(u), -1)) / catapult.dt
    w = -numpy.matrix(numpy.eye(len(u), 1, 0)) * X_initial[1, 0] / catapult.dt

    accel = W * (M * u_matrix + m * X_initial[1, 0]) + w

    Pi = numpy.diag([
        (0.01**2.0) + (0.02 * max(0.0, 20 - (len(u) - n)) / 20.0)**2.0
        for n in range(len(u))
    ])

    P_accel = 2.0 * M.transpose() * W.transpose() * Pi * W * M
    q_accel = 2.0 * (
        (W * m * X_initial[1, 0] + w).transpose() * Pi * W * M).transpose()
    constant_accel = (W * m * X_initial[1, 0] +
                      w).transpose() * Pi * (W * m * X_initial[1, 0] + w)


def mpc_cost(catapult, X_initial, X_final, u_matrix):

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


def SolveCatapult(catapult, X_initial, X_final, u):
    """ Solves for the optimal action given a seed, state, and target """

    def vbat_constraint(z, i):
        return 12.0 - z[i]

    def forward(z, i):
        return z[i]

    P, q, c = quadratic_cost(catapult, X_initial, X_final, len(u))

    def mpc_cost2(u_solver):
        u_matrix = numpy.matrix(u_solver).transpose()
        cost = mpc_cost(catapult, X_initial, X_final, u_solver)
        return cost

    def mpc_cost3(u_solver):
        u_matrix = numpy.matrix(u_solver).transpose()
        return (0.5 * u_matrix.transpose() * P * u_matrix +
                q.transpose() * u_matrix + c)[0, 0]

    # If we provide scipy with the analytical jacobian and hessian, it solves
    # more accurately and a *lot* faster.
    def jacobian(u):
        u_matrix = numpy.matrix(u).transpose()
        return numpy.array(P * u_matrix + q)

    def hessian(u):
        return numpy.array(P)

    constraints = []
    constraints += [{
        'type': 'ineq',
        'fun': vbat_constraint,
        'args': (i, )
    } for i in numpy.arange(len(u))]

    constraints += [{
        'type': 'ineq',
        'fun': forward,
        'args': (i, )
    } for i in numpy.arange(len(u))]

    result = scipy.optimize.minimize(mpc_cost3,
                                     u,
                                     jac=jacobian,
                                     hess=hessian,
                                     method='SLSQP',
                                     tol=1e-12,
                                     constraints=constraints)
    print(result)

    return result.x


def CatapultProblem():
    c = catapult_lib.Catapult(kCatapultWithBall)

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
        u_horizon = SolveCatapult(c, X, X_final, u)

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
    if FLAGS.plot:
        # Do all our math with a lower voltage so we have headroom.
        U = numpy.matrix([[9.0]])

        prob = osqp.OSQP()

        kHorizon = 40
        catapult = catapult_lib.Catapult(kCatapultWithBall)
        X_initial = numpy.matrix([[0.0], [0.0]])
        X_final = numpy.matrix([[2.0], [25.0]])
        P, q, c = quadratic_cost(catapult, X_initial, X_final, kHorizon)
        A = numpy.identity(kHorizon)
        l = numpy.zeros((kHorizon, 1))
        u = numpy.ones((kHorizon, 1)) * 12.0

        prob.setup(scipy.sparse.csr_matrix(P),
                   q,
                   scipy.sparse.csr_matrix(A),
                   l,
                   u,
                   warm_start=True)

        result = prob.solve()
        # Check solver status
        if result.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')

        # Apply first control input to the plant
        print(result.x)

        glog.debug("J ball", ball_mass * lever * lever)
        glog.debug("J bar", J_bar)
        glog.debug("bar mass", M_bar)
        glog.debug("J cup", J_cup)
        glog.debug("cup mass", M_cup)
        glog.debug("J", J)
        glog.debug("J Empty", JEmpty)

        glog.debug(
            "For G:", G, " max speed ",
            catapult_lib.MaxSpeed(params=kCatapultWithBall,
                                  U=U,
                                  final_position=math.pi / 2.0))

        CatapultProblem()

        catapult_lib.PlotStep(params=kCatapultWithBall,
                              R=numpy.matrix([[1.0], [0.0]]))
        catapult_lib.PlotKick(params=kCatapultWithBall,
                              R=numpy.matrix([[1.0], [0.0]]))
        return 0

        catapult_lib.PlotShot(kCatapultWithBall,
                              U,
                              final_position=math.pi / 4.0)

        gs = []
        speed = []
        for i in numpy.linspace(0.01, 0.15, 150):
            kCatapultWithBall.G = i
            gs.append(kCatapultWithBall.G)
            speed.append(
                catapult_lib.MaxSpeed(params=kCatapultWithBall,
                                      U=U,
                                      final_position=math.pi / 2.0))
        pylab.plot(gs, speed, label="max_speed")
        pylab.show()

    if len(argv) != 5:
        glog.fatal(
            'Expected .h file name and .cc file name for the catapult and integral catapult.'
        )
    else:
        namespaces = ['y2022', 'control_loops', 'superstructure', 'catapult']
        catapult_lib.WriteCatapult([kCatapultWithBall, kCatapultEmpty],
                                   argv[1:3], argv[3:5], namespaces)
    return 0


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
