#!/usr/bin/env python3

from frc971.control_loops.swerve import dynamics
import casadi
import numpy


class MPC(object):

    def __init__(self, solver='fatrop', jit=True):
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

        self.force = [
            dynamics.F(i, casadi.SX.sym("X", 25, 1), casadi.SX.sym("U", 8, 1))
            for i in range(4)
        ]
        self.force_vel = [
            dynamics.F_vel(i,
                           casadi.SX.sym("X", dynamics.NUM_VELOCITY_STATES, 1),
                           casadi.SX.sym("U", 8, 1)) for i in range(4)
        ]
        self.slip_angle = [
            dynamics.slip_angle(i, casadi.SX.sym("X", 25, 1),
                                casadi.SX.sym("U", 8, 1)) for i in range(4)
        ]
        self.rotated_mounting_location = [
            dynamics.rotated_mounting_location(
                i, casadi.SX.sym("X", dynamics.NUM_VELOCITY_STATES, 1),
                casadi.SX.sym("U", 8, 1)) for i in range(4)
        ]
        self.torque_cross = self.torque_cross_func(casadi.SX.sym("r", 2, 1),
                                                   casadi.SX.sym("F", 2, 1))
        self.force_cross = self.force_cross_func(casadi.SX.sym("Tau", 1, 1),
                                                 casadi.SX.sym("r", 2, 1))
        self.next_X = self.make_physics()
        self.cost = self.make_cost()

        self.N = 200

        # Start with an empty nonlinear program.
        self.w = []
        self.lbw = []
        self.ubw = []
        J = 0
        self.g = []
        self.lbg = []
        self.ubg = []

        self.X0 = casadi.MX.sym('X0', dynamics.NUM_VELOCITY_STATES)

        # We care about the linear and angular velocities only.
        self.R = casadi.MX.sym('R', 3)

        # Make Xn and U for each step.  fatrop wants us to interleave the control variables and
        # states so that it can produce a banded/structured problem which it can solve a lot
        # faster.
        Xn_variables = []
        U_variables = []
        for i in range(self.N):
            U_variables.append(casadi.MX.sym(f'U{i}', 8))

            if i == 0:
                continue

            Xn_variables.append(
                casadi.MX.sym(f'X{i}', dynamics.NUM_VELOCITY_STATES))

        Xn = casadi.horzcat(*Xn_variables)
        U = casadi.horzcat(*U_variables)

        # printme(number) is the debug.
        Xk_begin = casadi.horzcat(self.X0, Xn)
        Xk_end = self.next_X.map(self.N, "thread")(Xk_begin, U)
        J = casadi.sum2(self.cost.map(self.N, "thread")(Xk_end, U, self.R))

        # Put U and Xn interleaved into w to go fast.
        for i in range(self.N):
            self.w += [U_variables[i]]
            self.ubw += [100] * 8
            self.lbw += [-100] * 8

            if i == self.N - 1:
                continue

            self.w += [Xn_variables[i]]
            self.ubw += [casadi.inf] * dynamics.NUM_VELOCITY_STATES
            self.lbw += [-casadi.inf] * dynamics.NUM_VELOCITY_STATES

        self.g += [
            casadi.reshape(Xn - Xk_end[:, 0:(self.N - 1)],
                           dynamics.NUM_VELOCITY_STATES * (self.N - 1), 1)
        ]

        self.lbg += [0] * dynamics.NUM_VELOCITY_STATES * (self.N - 1)
        self.ubg += [0] * dynamics.NUM_VELOCITY_STATES * (self.N - 1)

        prob = {
            'f': J,
            # lbx <= x <= ubx
            'x': casadi.vertcat(*self.w),
            # lbg <= g(x, p) <= ubg
            'g': casadi.vertcat(*self.g),
            # Input parameters (initial position + goal)
            'p': casadi.vertcat(self.X0, self.R),
        }

        compiler = "ccache clang"
        flags = ["-O3"]
        jit_options = {
            "flags": flags,
            "verbose": True,
            "compiler": compiler,
            "temp_suffix": False,
        }

        if solver == 'fatrop':
            equality = [
                True
                for _ in range(dynamics.NUM_VELOCITY_STATES * (self.N - 1))
            ]
            options = {
                "jit": jit,
                "jit_cleanup": False,
                "jit_temp_suffix": False,
                "compiler": "shell",
                "jit_options": jit_options,
                "structure_detection": "auto",
                "fatrop": {
                    "tol": 1e-7
                },
                "debug": True,
                "equality": equality,
            }
        else:
            options = {
                "jit": jit,
                "jit_cleanup": False,
                "jit_temp_suffix": False,
                "compiler": "shell",
                "jit_options": jit_options,
            }
            if FLAGS.full_debug:
                options["jit"] = False
                options["ipopt"] = {
                    "print_level": 12,
                }

        self.solver = casadi.nlpsol('solver', solver, prob, options)

    # TODO(austin): Vary the number of sub steps to be more short term and fewer long term?
    def make_physics(self):
        X0 = casadi.MX.sym('X0', dynamics.NUM_VELOCITY_STATES)
        U = casadi.MX.sym('U', 8)

        X = X0
        M = 2  # RK4 steps per interval
        DT = self.dt / M

        for j in range(M):
            k1 = self.velocity_fdot(X, U)
            k2 = self.velocity_fdot(X + DT / 2 * k1, U)
            k3 = self.velocity_fdot(X + DT / 2 * k2, U)
            k4 = self.velocity_fdot(X + DT * k3, U)
            X = X + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        return casadi.Function("next_X", [X0, U], [X])

    def make_cost(self):
        # TODO(austin): tune cost fn?
        # Need to do some simulations to see what works well.

        X = casadi.MX.sym('X', dynamics.NUM_VELOCITY_STATES)
        U = casadi.MX.sym('U', 8)
        R = casadi.MX.sym('R', 3)

        J = 0
        vnorm = casadi.sqrt(R[0]**2.0 + R[1]**2.0)

        vnormx = casadi.if_else(vnorm > 0.0001, R[0] / vnorm, 1.0)
        vnormy = casadi.if_else(vnorm > 0.0001, R[1] / vnorm, 0.0)

        vperpx = casadi.if_else(vnorm > 0.0001, -vnormy, 0.0)
        vperpy = casadi.if_else(vnorm > 0.0001, vnormx, 1.0)

        # TODO(austin): Do we want to do something more special for 0?

        # cost velocity a lot more in the perpendicular direction to allow tire to spin up
        # we also only want to get moving in the correct direction as fast as possible
        J += 75 * ((R[0] - X[dynamics.VELOCITY_STATE_VX]) * vnormx +
                   (R[1] - X[dynamics.VELOCITY_STATE_VY]) * vnormy)**2.0

        J += 1500 * ((R[0] - X[dynamics.VELOCITY_STATE_VX]) * vperpx +
                     (R[1] - X[dynamics.VELOCITY_STATE_VY]) * vperpy)**2.0

        J += 1000 * (R[2] - X[dynamics.VELOCITY_STATE_OMEGA])**2.0

        kSteerVelocityGain = 0.10
        J += kSteerVelocityGain * (X[dynamics.VELOCITY_STATE_OMEGAS0])**2.0
        J += kSteerVelocityGain * (X[dynamics.VELOCITY_STATE_OMEGAS1])**2.0
        J += kSteerVelocityGain * (X[dynamics.VELOCITY_STATE_OMEGAS2])**2.0
        J += kSteerVelocityGain * (X[dynamics.VELOCITY_STATE_OMEGAS3])**2.0

        # cost variance of the force by a tire and the expected average force and torque on it
        total_force = self.force_vel[0](X, U)
        total_torque = self.torque_cross(
            self.rotated_mounting_location[0](X, U), self.force_vel[0](X, U))
        for i in range(3):
            total_force += self.force_vel[i + 1](X, U)
            total_torque += self.torque_cross(
                self.rotated_mounting_location[i + 1](X, U),
                self.force_vel[i + 1](X, U))

        total_force /= 4
        total_torque /= 4
        for i in range(4):
            f_diff = (total_force + self.force_cross(
                total_torque, self.rotated_mounting_location[i]
                (X, U))) - self.force_vel[i](X, U)
            J += 0.01 * (f_diff[0, 0]**2.0 + f_diff[1, 0]**2.0)

        # TODO(austin): Don't penalize torque steering current.
        for i in range(4):
            Is = U[2 * i + 0]
            Id = U[2 * i + 1]
            # Steer, cost it a lot less than drive to be more agressive in steering
            J += ((Is + dynamics.STEER_CURRENT_COUPLING_FACTOR * Id)**
                  2.0) / 100000.0
            # Drive
            J += Id * Id / 1000.0

        return casadi.Function("Jn", [X, U, R], [J])

    def torque_cross_func(self, r, F):
        result = casadi.SX.sym('Tau', 1, 1)
        result[0, 0] = r[0, 0] * F[1, 0] - r[1, 0] * F[0, 0]
        return casadi.Function('Tau', [r, F], [result])

    def force_cross_func(self, Tau, r):
        result = casadi.SX.sym('F', 2, 1)
        result[0, 0] = -r[1, 0] * Tau[0, 0] / casadi.norm_2(r)**2.0
        result[1, 0] = r[0, 0] * Tau[0, 0] / casadi.norm_2(r)**2.0
        return casadi.Function('F', [Tau, r], [result])

    def solve(self, p, seed=None):
        if seed is None:
            seed = []

            for i in range(self.N):
                seed += [0, 0] * 4
                if i < self.N - 1:
                    seed += list(p[:dynamics.NUM_VELOCITY_STATES, 0])

        return self.solver(x0=seed,
                           lbx=self.lbw,
                           ubx=self.ubw,
                           lbg=self.lbg,
                           ubg=self.ubg,
                           p=casadi.DM(p))

    def unpack_u(self, sol, i):
        return sol['x'].full().flatten()[
            (8 + dynamics.NUM_VELOCITY_STATES) *
            i:((8 + dynamics.NUM_VELOCITY_STATES) * i + 8)]

    def unpack_x(self, sol, i):
        return sol['x'].full().flatten(
        )[8 + (8 + dynamics.NUM_VELOCITY_STATES) *
          (i - 1):(8 + dynamics.NUM_VELOCITY_STATES) * i]
