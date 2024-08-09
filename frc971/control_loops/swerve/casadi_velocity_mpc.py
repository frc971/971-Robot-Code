#!/usr/bin/python3

from frc971.control_loops.swerve import dynamics
import matplotlib.pyplot as pyplot
from matplotlib import pylab
import numpy
import time
import scipy
import casadi
import os, sys


class MPC(object):

    def __init__(self):
        self.fdot = dynamics.swerve_full_dynamics(
            casadi.SX.sym("X", dynamics.NUM_STATES, 1),
            casadi.SX.sym("U", 8, 1))
        self.velocity_fdot = dynamics.velocity_swerve_physics(
            casadi.SX.sym("X", dynamics.NUM_VELOCITY_STATES, 1),
            casadi.SX.sym("U", 8, 1))

        self.wrapped_swerve_physics = lambda X, U: numpy.array(self.fdot(X, U))

        self.dt = 0.005

        # TODO(austin): Do we need a disturbance torque per module to account for friction?
        # Do it only in the observer/post?

        self.next_X = self.make_physics()
        self.force = [
            dynamics.F(i, casadi.SX.sym("X", 25, 1), casadi.SX.sym("U", 8, 1))
            for i in range(4)
        ]
        self.slip_angle = [
            dynamics.slip_angle(i, casadi.SX.sym("X", 25, 1),
                                casadi.SX.sym("U", 8, 1)) for i in range(4)
        ]

        self.N = 2

        # TODO(austin): Can we approximate sin/cos/atan for the initial operating point to solve faster if we need it?

        # Start with an empty nonlinear program.
        self.w = []
        self.w0 = []
        self.lbw = []
        self.ubw = []
        J = 0
        self.g = []
        self.lbg = []
        self.ubg = []

        self.X0 = casadi.MX.sym('X0', dynamics.NUM_VELOCITY_STATES)

        # We care about the linear and angular velocities only.
        self.R = casadi.MX.sym('R', 3)

        Xk = self.X0
        # Instead of an equality constraint on the goal, what about:
        # self.w += [Xk]
        # lbw += [0, 1]
        # ubw += [0, 1]
        # w0 += [0, 1]

        for k in range(self.N):
            Uk = casadi.MX.sym('U_' + str(k), 8)
            # TODO(austin): Add a line to g here for a 12 volt battery and the breakers?
            self.w += [Uk]
            self.lbw += [-100] * 8
            self.ubw += [100] * 8
            self.w0 += [0] * 8

            # Integrate till the end of the interval
            Fk = self.next_X(x0=Xk, u=Uk)
            Xk_end = Fk['xf']

            # TODO(austin): tune cost fn?
            # Do we want to penalize slipping tires?
            # Do we want to penalize powers unevenly applied across the motors?
            # Need to do some simulations to see what works well.

            J += 1000 * (self.R[0] - Xk_end[dynamics.VELOCITY_STATE_VX])**2.0
            J += 1000 * (self.R[1] - Xk_end[dynamics.VELOCITY_STATE_VY])**2.0
            J += 1000 * (self.R[2] -
                         Xk_end[dynamics.VELOCITY_STATE_OMEGA])**2.0

            kSteerPositionGain = 0
            kSteerVelocityGain = 0.05
            J += kSteerPositionGain * (
                Xk_end[dynamics.VELOCITY_STATE_THETAS0])**2.0
            J += kSteerVelocityGain * (
                Xk_end[dynamics.VELOCITY_STATE_OMEGAS0])**2.0

            J += kSteerPositionGain * (
                Xk_end[dynamics.VELOCITY_STATE_THETAS1])**2.0
            J += kSteerVelocityGain * (
                Xk_end[dynamics.VELOCITY_STATE_OMEGAS1])**2.0

            J += kSteerPositionGain * (
                Xk_end[dynamics.VELOCITY_STATE_THETAS2])**2.0
            J += kSteerVelocityGain * (
                Xk_end[dynamics.VELOCITY_STATE_OMEGAS2])**2.0

            J += kSteerPositionGain * (
                Xk_end[dynamics.VELOCITY_STATE_THETAS3])**2.0
            J += kSteerVelocityGain * (
                Xk_end[dynamics.VELOCITY_STATE_OMEGAS3])**2.0

            #for i in range(4):
            #    sa = self.slip_angle[i](Xk_end, Uk)
            #    J += 10 * sa * sa

            for i in range(4):
                # Steer
                J += Uk[2 * i + 0] * Uk[2 * i + 0] / 100000.0
                # Drive
                J += Uk[2 * i + 1] * Uk[2 * i + 1] / 1000.0

            # New NLP variable for state at end of interval
            Xk = casadi.MX.sym('X_' + str(k + 1), dynamics.NUM_VELOCITY_STATES)
            self.w += [Xk]
            self.ubw += [casadi.inf] * dynamics.NUM_VELOCITY_STATES
            self.lbw += [-casadi.inf] * dynamics.NUM_VELOCITY_STATES

            # Add equality constraint by using lbg <= g <= ubg
            self.g += [Xk_end - Xk]
            self.lbg += [0] * dynamics.NUM_VELOCITY_STATES
            self.ubg += [0] * dynamics.NUM_VELOCITY_STATES

        prob = {
            'f': J,
            # lbx <= x <= ubx
            'x': casadi.vertcat(*self.w),
            # lbg <= g(x, p) <= ubg
            'g': casadi.vertcat(*self.g),
            # Input parameters (initial position + goal)
            'p': casadi.vertcat(self.X0, self.R),
        }
        print('J', J)
        self.solver = casadi.nlpsol('solver', 'ipopt', prob)

    def make_physics(self):
        X0 = casadi.MX.sym('X0', dynamics.NUM_VELOCITY_STATES)
        U = casadi.MX.sym('U', 8)

        X = X0
        M = 4  # RK4 steps per interval
        DT = self.dt / M

        for j in range(M):
            k1 = self.velocity_fdot(X, U)
            k2 = self.velocity_fdot(X + DT / 2 * k1, U)
            k3 = self.velocity_fdot(X + DT / 2 * k2, U)
            k4 = self.velocity_fdot(X + DT * k3, U)
            X = X + DT / 6 / M * (k1 + 2 * k2 + 2 * k3 + k4)

        return casadi.Function("next_X", [X0, U], [X], ['x0', 'u'], ['xf'])

    def solve(self, p, seed=None):
        w0 = []
        for k in range(self.N):
            w0 += [0, 0] * 4

            w_step = p[:dynamics.NUM_VELOCITY_STATES, 0]
            w0 += list(w_step)

        return self.solver(x0=w0,
                           lbx=self.lbw,
                           ubx=self.ubw,
                           lbg=self.lbg,
                           ubg=self.ubg,
                           p=casadi.DM(p))

    def unpack_u(self, sol, i):
        return sol['x'].full().flatten()[
            (8 + 1 * dynamics.NUM_VELOCITY_STATES) *
            i:(8 + 1 * dynamics.NUM_VELOCITY_STATES) * i + 8]

    def unpack_x(self, sol, i):
        return sol['x'].full().flatten()[
            (8 + 1 * dynamics.NUM_VELOCITY_STATES) * (i - 1) +
            8:(8 + 1 * dynamics.NUM_VELOCITY_STATES) * (i - 1) + 8 +
            dynamics.NUM_VELOCITY_STATES]


mpc = MPC()

R_goal = numpy.zeros((3, 1))
R_goal[0, 0] = 1.0
R_goal[1, 0] = 1.0

X_initial = numpy.zeros((25, 1))
# All the wheels are spinning at the speed needed to hit 1 m/s
X_initial[3, 0] = 0.0
X_initial[7, 0] = 0.0
X_initial[11, 0] = 0.0
X_initial[15, 0] = 0.0

# Robot is moving at 0 m/s
X_initial[19, 0] = 0.01
X_initial[20, 0] = 0.0
# No angular velocity
X_initial[21, 0] = 0.0

iterations = 200

X_plot = numpy.zeros((25, iterations))
U_plot = numpy.zeros((8, iterations))
t = []

X = X_initial.copy()

pyplot.ion()

fig0, axs0 = pylab.subplots(2)
fig1, axs1 = pylab.subplots(2)
last_time = time.time()

for i in range(iterations):
    t.append(i * mpc.dt)
    print("Current X at", i * mpc.dt, X.transpose())
    print("Goal R at", i * mpc.dt, R_goal)
    sol = mpc.solve(
        # TODO(austin): Is this better or worse than constraints on the initial state for convergence?
        p=numpy.vstack((dynamics.to_velocity_state(X), R_goal)))
    X_plot[:, i] = X[:, 0]

    U = mpc.unpack_u(sol, 0)
    U_plot[:, i] = U

    print('x(0):', X.transpose())
    for j in range(mpc.N):
        print(f'u({j}): ', mpc.unpack_u(sol, j))
        print(f'x({j+1}): ', mpc.unpack_x(sol, j + 1))

    result = scipy.integrate.solve_ivp(
        lambda t, x: mpc.wrapped_swerve_physics(x, U).flatten(), [0, mpc.dt],
        X.flatten())
    X[:, 0] = result.y[:, -1]

    if time.time() > last_time + 2 or i == iterations - 1:
        axs0[0].clear()
        axs0[1].clear()

        axs0[0].plot(t, X_plot[dynamics.STATE_VX, 0:i + 1], label="vx")
        axs0[0].plot(t, X_plot[dynamics.STATE_VY, 0:i + 1], label="vy")
        axs0[0].legend()

        axs0[1].plot(t, U_plot[0, 0:i + 1], label="Is0")
        axs0[1].plot(t, U_plot[1, 0:i + 1], label="Id0")
        axs0[1].legend()

        axs1[0].clear()
        axs1[1].clear()

        axs1[0].plot(t, X_plot[0, 0:i + 1], label='steer0')
        axs1[0].plot(t, X_plot[4, 0:i + 1], label='steer1')
        axs1[0].plot(t, X_plot[8, 0:i + 1], label='steer2')
        axs1[0].plot(t, X_plot[12, 0:i + 1], label='steer3')
        axs1[0].legend()
        axs1[1].plot(t, X_plot[2, 0:i + 1], label='steer_velocity0')
        axs1[1].plot(t, X_plot[6, 0:i + 1], label='steer_velocity1')
        axs1[1].plot(t, X_plot[10, 0:i + 1], label='steer_velocity2')
        axs1[1].plot(t, X_plot[14, 0:i + 1], label='steer_velocity3')
        axs1[1].legend()
        pyplot.pause(0.0001)
        last_time = time.time()

pyplot.pause(-1)
