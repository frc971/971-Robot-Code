#!/usr/bin/env python3

from frc971.control_loops.swerve import dynamics
from frc971.control_loops.swerve.casadi_velocity_mpc_lib import MPC
import pickle
import matplotlib.pyplot as pyplot
import matplotlib
from matplotlib import pylab
import numpy
import time
import scipy
import casadi
import os, sys
from absl import flags
from absl import app

FLAGS = flags.FLAGS
flags.DEFINE_bool('compileonly', False,
                  'If true, load casadi, don\'t compile it')
flags.DEFINE_float('vx', 1.0, 'Goal velocity in m/s in x')
flags.DEFINE_float('vy', 0.0, 'Goal velocity in m/s in y')
flags.DEFINE_float('omega', 0.0, 'Goal velocity in m/s in omega')
flags.DEFINE_float('duration', 0.5, 'Time to simulate in seconds.')
flags.DEFINE_bool('pickle', False, 'Write optimization results.')
flags.DEFINE_string('outputdir', None, 'Directory to write problem results to')

# Full print level on ipopt. Piping to a file and using a filter or search method is suggested
# grad_x prints out the gradient at each iteration in the following sequence: U0, X1, U1, etc.
flags.DEFINE_bool('full_debug', False,
                  'If true, turn on all the debugging in the solver.')


class Solver(object):

    def __init__(self):
        self.iterations = int(round(FLAGS.duration / 0.005))

        self.X_plot = numpy.zeros((25, self.iterations))
        self.U_plot = numpy.zeros((8, self.iterations))
        self.t = []

    def solve(self, mpc, X_initial, R_goal, debug=False):
        X = X_initial.copy()

        if debug:
            pyplot.ion()
            fig0, axs0 = pylab.subplots(2)
            fig1, axs1 = pylab.subplots(2)

        last_time = time.time()

        seed = ([0, 0] * 4 +
                list(dynamics.to_velocity_state(X))) * (mpc.N - 1) + [0, 0] * 4

        overall_time = 0
        for i in range(self.iterations):
            self.t.append(i * mpc.dt)
            print("Current X at", i * mpc.dt, X.transpose())
            print("Goal R at", i * mpc.dt, R_goal)
            start_time = time.time()
            sol = mpc.solve(
                # TODO(austin): Is this better or worse than constraints on the initial state for convergence?
                p=numpy.vstack((dynamics.to_velocity_state(X), R_goal)),
                seed=seed)
            end_time = time.time()
            print(f"Took {end_time - start_time} seconds to solve.")
            overall_time += end_time - start_time

            self.X_plot[:, i] = X[:, 0]

            U = mpc.unpack_u(sol, 0)
            seed = (list(
                sol['x'].full().flatten()[8 + dynamics.NUM_VELOCITY_STATES:]) +
                    list(sol['x'].full().flatten()
                         [-(8 + dynamics.NUM_VELOCITY_STATES):]))
            self.U_plot[:, i] = U

            print('x(0):', X.transpose())
            for j in range(mpc.N):
                print(f'u({j}): ', mpc.unpack_u(sol, j))
                print(f'x({j+1}): ', mpc.unpack_x(sol, j + 1))

            result = scipy.integrate.solve_ivp(
                lambda t, x: mpc.wrapped_swerve_physics(x, U).flatten(),
                [0, mpc.dt], X.flatten())
            X[:, 0] = result.y[:, -1]

            if time.time() > last_time + 2 or i == self.iterations - 1:
                if debug:
                    axs0[0].clear()
                    axs0[1].clear()

                    axs0[0].plot(self.t,
                                 self.X_plot[dynamics.STATE_VX, 0:i + 1],
                                 label="vx")
                    axs0[0].plot(self.t,
                                 self.X_plot[dynamics.STATE_OMEGA, 0:i + 1],
                                 label="omega")
                    axs0[0].plot(self.t,
                                 self.X_plot[dynamics.STATE_VY, 0:i + 1],
                                 label="vy")
                    axs0[0].legend()

                    axs0[1].plot(self.t, self.U_plot[0, 0:i + 1], label="Is0")
                    axs0[1].plot(self.t, self.U_plot[1, 0:i + 1], label="Id0")
                    axs0[1].legend()

                    axs1[0].clear()
                    axs1[1].clear()

                    axs1[0].plot(self.t,
                                 self.X_plot[dynamics.STATE_THETAS0, 0:i + 1],
                                 label='steer0')
                    axs1[0].plot(self.t,
                                 self.X_plot[dynamics.STATE_THETAS1, 0:i + 1],
                                 label='steer1')
                    axs1[0].plot(self.t,
                                 self.X_plot[dynamics.STATE_THETAS2, 0:i + 1],
                                 label='steer2')
                    axs1[0].plot(self.t,
                                 self.X_plot[dynamics.STATE_THETAS3, 0:i + 1],
                                 label='steer3')
                    axs1[0].legend()
                    axs1[1].plot(self.t,
                                 self.X_plot[dynamics.STATE_OMEGAS0, 0:i + 1],
                                 label='steer_velocity0')
                    axs1[1].plot(self.t,
                                 self.X_plot[dynamics.STATE_OMEGAS1, 0:i + 1],
                                 label='steer_velocity1')
                    axs1[1].plot(self.t,
                                 self.X_plot[dynamics.STATE_OMEGAS2, 0:i + 1],
                                 label='steer_velocity2')
                    axs1[1].plot(self.t,
                                 self.X_plot[dynamics.STATE_OMEGAS3, 0:i + 1],
                                 label='steer_velocity3')
                    axs1[1].legend()
                    pyplot.pause(0.0001)

                last_time = time.time()

        print(f"Took {overall_time} seconds overall to solve.")


def main(argv):
    matplotlib.use("GTK3Agg")

    if FLAGS.outputdir:
        os.chdir(FLAGS.outputdir)

    module_velocity = 0.0

    X_initial = numpy.zeros((25, 1))
    # All the wheels are spinning at the speed needed to hit 1 m/s
    X_initial[dynamics.STATE_THETAS0, 0] = 0.0
    X_initial[dynamics.STATE_OMEGAS0, 0] = module_velocity

    X_initial[dynamics.STATE_THETAS1, 0] = 0.0
    X_initial[dynamics.STATE_OMEGAS1, 0] = module_velocity

    X_initial[dynamics.STATE_THETAS2, 0] = 0.0
    X_initial[dynamics.STATE_OMEGAS2, 0] = module_velocity

    X_initial[dynamics.STATE_THETAS3, 0] = 0.0
    X_initial[dynamics.STATE_OMEGAS3, 0] = module_velocity

    # Robot is moving at 0 m/s
    X_initial[dynamics.STATE_VX, 0] = 0.0
    X_initial[dynamics.STATE_VY, 0] = 0.0
    # No angular velocity
    X_initial[dynamics.STATE_OMEGA, 0] = 0.0

    R_goal = numpy.zeros((3, 1))
    R_goal[0, 0] = FLAGS.vx
    R_goal[1, 0] = FLAGS.vy
    R_goal[2, 0] = FLAGS.omega

    mpc = MPC(solver='fatrop') if not FLAGS.full_debug else MPC(solver='ipopt')
    solver = Solver()
    if not FLAGS.compileonly:
        results = solver.solve(mpc=mpc,
                               X_initial=X_initial,
                               R_goal=R_goal,
                               debug=(FLAGS.pickle == False))
    else:
        return 0

    if FLAGS.pickle:
        with open('t.pkl', 'wb') as f:
            pickle.dump(solver.t, f)
        with open('X_plot.pkl', 'wb') as f:
            pickle.dump(solver.X_plot, f)
        with open('U_plot.pkl', 'wb') as f:
            pickle.dump(solver.U_plot, f)

        fig0, axs0 = pylab.subplots(2)
        fig1, axs1 = pylab.subplots(2)

        axs0[0].clear()
        axs0[1].clear()

        axs0[0].plot(solver.t, solver.X_plot[dynamics.STATE_VX, :], label="vx")
        axs0[0].plot(solver.t, solver.X_plot[dynamics.STATE_VY, :], label="vy")
        axs0[0].legend()

        axs0[1].plot(solver.t, solver.U_plot[0, :], label="Is0")
        axs0[1].plot(solver.t, solver.U_plot[1, :], label="Id0")
        axs0[1].legend()

        axs1[0].clear()
        axs1[1].clear()

        axs1[0].plot(solver.t,
                     solver.X_plot[dynamics.STATE_THETAS0, :],
                     label='steer0')
        axs1[0].plot(solver.t,
                     solver.X_plot[dynamics.STATE_THETAS1, :],
                     label='steer1')
        axs1[0].plot(solver.t,
                     solver.X_plot[dynamics.STATE_THETAS2, :],
                     label='steer2')
        axs1[0].plot(solver.t,
                     solver.X_plot[dynamics.STATE_THETAS3, :],
                     label='steer3')
        axs1[0].legend()
        axs1[1].plot(solver.t,
                     solver.X_plot[dynamics.STATE_OMEGAS0, :],
                     label='steer_velocity0')
        axs1[1].plot(solver.t,
                     solver.X_plot[dynamics.STATE_OMEGAS1, :],
                     label='steer_velocity1')
        axs1[1].plot(solver.t,
                     solver.X_plot[dynamics.STATE_OMEGAS2, :],
                     label='steer_velocity2')
        axs1[1].plot(solver.t,
                     solver.X_plot[dynamics.STATE_OMEGAS3, :],
                     label='steer_velocity3')
        axs1[1].legend()

        fig0.savefig('state.svg')
        fig1.savefig('steer.svg')


if __name__ == '__main__':
    app.run(main)
