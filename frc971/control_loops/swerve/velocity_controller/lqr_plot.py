#!/usr/bin/env python3

import os

# Setup JAX to run on the CPU
os.environ['XLA_FLAGS'] = ' '.join([
    # Teach it where to find CUDA
    '--xla_gpu_cuda_data_dir=/usr/lib/cuda',
    # Use up to 20 cores
    '--xla_force_host_platform_device_count=20',
    # Dump XLA to /tmp/foo to aid debugging
    '--xla_dump_to=/tmp/foo',
    '--xla_gpu_enable_command_buffer='
])

os.environ['JAX_PLATFORMS'] = 'cpu'
# Don't pre-allocate memory
os.environ['XLA_PYTHON_CLIENT_PREALLOCATE'] = 'false'

import absl
from absl import logging
from matplotlib.animation import FuncAnimation
import matplotlib
import numpy
import scipy
import time

matplotlib.use("gtk3agg")

from matplotlib import pylab
from matplotlib import pyplot
from flax.training import checkpoints
from frc971.control_loops.python import controls
import tensorflow as tf
import threading
import collections

import jax

jax._src.deprecations.accelerate('tracer-hash')
jax.config.update("jax_compilation_cache_dir", "/tmp/jax_cache")
jax.config.update("jax_persistent_cache_min_entry_size_bytes", -1)
jax.config.update("jax_persistent_cache_min_compile_time_secs", 0)

from frc971.control_loops.swerve.velocity_controller.model import *
from frc971.control_loops.swerve.velocity_controller.physics import *

# Container for the data.
Data = collections.namedtuple('Data', [
    't', 'X', 'X_lqr', 'U', 'U_lqr', 'cost', 'cost_lqr', 'q1_grid', 'q2_grid',
    'q_grid', 'target_q_grid', 'lqr_grid', 'pi_grid_U', 'lqr_grid_U', 'grid_X',
    'grid_Y', 'reward', 'reward_lqr', 'step'
])

FLAGS = absl.flags.FLAGS

absl.flags.DEFINE_string('workdir', None, 'Directory to store model data.')

absl.flags.DEFINE_integer(
    'horizon',
    default=100,
    help='Horizon to simulate',
)

absl.flags.DEFINE_float(
    'alpha',
    default=0.2,
    help='Entropy.  If negative, automatically solve for it.',
)

numpy.set_printoptions(linewidth=200, )


def restore_checkpoint(state: TrainState, workdir: str):
    return checkpoints.restore_checkpoint(workdir, state)


def generate_data(step=None):
    grid_X = numpy.arange(-1, 1, 0.1)
    grid_Y = numpy.arange(-10, 10, 0.1)
    grid_X, grid_Y = numpy.meshgrid(grid_X, grid_Y)
    grid_X = jax.numpy.array(grid_X)
    grid_Y = jax.numpy.array(grid_Y)
    # Load the training state.
    problem = physics.TurretProblem()

    rng = jax.random.key(0)
    rng, init_rng = jax.random.split(rng)

    state = create_train_state(
        init_rng,
        problem,
        q_learning_rate=FLAGS.q_learning_rate,
        pi_learning_rate=FLAGS.pi_learning_rate,
        alpha_learning_rate=FLAGS.alpha_learning_rate,
    )

    state = restore_checkpoint(state, FLAGS.workdir)
    if step is not None and state.step == step:
        return None

    X = jax.numpy.array([1.0, 0.0])
    X_lqr = X.copy()
    goal = jax.numpy.array([0.0, 0.0])

    logging.info('X: %s', X)
    logging.info('goal: %s', goal)
    logging.debug('params: %s', state.params)

    # Compute the various cost surfaces for plotting.
    def compute_q1(X, Y):
        return state.q1_apply(
            state.params,
            observation=state.problem.unwrap_angles(jax.numpy.array([X, Y])),
            R=goal,
            action=jax.numpy.array([0.]),
        )[0]

    def compute_q2(X, Y):
        return state.q2_apply(
            state.params,
            observation=state.problem.unwrap_angles(jax.numpy.array([X, Y])),
            R=goal,
            action=jax.numpy.array([0.]),
        )[0]

    def lqr_cost(X, Y):
        x = jax.numpy.array([X, Y]) - goal
        return -x.T @ jax.numpy.array(problem.P) @ x

    def compute_q(params, x, y):
        X = state.problem.unwrap_angles(jax.numpy.array([x, y]))

        return jax.numpy.minimum(
            state.q1_apply(
                params,
                observation=X,
                R=goal,
                action=jax.numpy.array([0.]),
            )[0],
            state.q2_apply(
                params,
                observation=X,
                R=goal,
                action=jax.numpy.array([0.]),
            )[0])

    cost_grid1 = jax.vmap(jax.vmap(compute_q1))(grid_X, grid_Y)
    cost_grid2 = jax.vmap(jax.vmap(compute_q2))(grid_X, grid_Y)
    cost_grid = jax.vmap(jax.vmap(lambda x, y: compute_q(state.params, x, y)))(
        grid_X, grid_Y)
    target_cost_grid = jax.vmap(
        jax.vmap(lambda x, y: compute_q(state.target_params, x, y)))(grid_X,
                                                                     grid_Y)
    lqr_cost_grid = jax.vmap(jax.vmap(lqr_cost))(grid_X, grid_Y)

    # Now compute the two controller surfaces.
    def compute_lqr_U(X, Y):
        x = jax.numpy.array([X, Y])
        return (-jax.numpy.array(problem.F.reshape((2, ))) @ x)[0]

    def compute_pi_U(X, Y):
        x = jax.numpy.array([X, Y])
        U, _, _ = state.pi_apply(rng,
                                 state.params,
                                 observation=state.problem.unwrap_angles(x),
                                 R=goal,
                                 deterministic=True)
        return U[0] * problem.action_limit

    lqr_cost_U = jax.vmap(jax.vmap(compute_lqr_U))(grid_X, grid_Y)
    pi_cost_U = jax.vmap(jax.vmap(compute_pi_U))(grid_X, grid_Y)

    logging.info('Finished cost')

    # Now simulate the robot, accumulating up things to plot.
    def loop(i, val):
        X, X_lqr, data, params = val
        t = data.t.at[i].set(i * problem.dt)

        normalized_U, _, _ = state.pi_apply(
            rng,
            params,
            observation=state.problem.unwrap_angles(X),
            R=goal,
            deterministic=True)
        U_lqr = problem.F @ (goal - X_lqr)

        cost = jax.numpy.minimum(
            state.q1_apply(params,
                           observation=state.problem.unwrap_angles(X),
                           R=goal,
                           action=normalized_U),
            state.q2_apply(params,
                           observation=state.problem.unwrap_angles(X),
                           R=goal,
                           action=normalized_U))

        U = normalized_U * problem.action_limit

        U_plot = data.U.at[i, :].set(U)
        U_lqr_plot = data.U_lqr.at[i, :].set(U_lqr)
        X_plot = data.X.at[i, :].set(X)
        X_lqr_plot = data.X_lqr.at[i, :].set(X_lqr)
        cost_plot = data.cost.at[i, :].set(cost)
        cost_lqr_plot = data.cost_lqr.at[i, :].set(lqr_cost(*X_lqr))

        X = problem.A @ X + problem.B @ U
        X_lqr = problem.A @ X_lqr + problem.B @ U_lqr

        reward = data.reward + state.problem.reward(X, normalized_U, goal)
        reward_lqr = data.reward_lqr + state.problem.reward(
            X_lqr, U_lqr / problem.action_limit, goal)

        return X, X_lqr, data._replace(
            t=t,
            U=U_plot,
            U_lqr=U_lqr_plot,
            X=X_plot,
            X_lqr=X_lqr_plot,
            cost=cost_plot,
            cost_lqr=cost_lqr_plot,
            reward=reward,
            reward_lqr=reward_lqr,
        ), params

    # Do it.
    @jax.jit
    def integrate(data, X, X_lqr, params):
        return jax.lax.fori_loop(0, FLAGS.horizon, loop,
                                 (X, X_lqr, data, params))

    X, X_lqr, data, params = integrate(
        Data(
            t=jax.numpy.zeros((FLAGS.horizon, )),
            X=jax.numpy.zeros((FLAGS.horizon, state.problem.num_states)),
            X_lqr=jax.numpy.zeros((FLAGS.horizon, state.problem.num_states)),
            U=jax.numpy.zeros((FLAGS.horizon, state.problem.num_outputs)),
            U_lqr=jax.numpy.zeros((FLAGS.horizon, state.problem.num_outputs)),
            cost=jax.numpy.zeros((FLAGS.horizon, 1)),
            cost_lqr=jax.numpy.zeros((FLAGS.horizon, 1)),
            q1_grid=cost_grid1,
            q2_grid=cost_grid2,
            q_grid=cost_grid,
            target_q_grid=target_cost_grid,
            lqr_grid=lqr_cost_grid,
            pi_grid_U=pi_cost_U,
            lqr_grid_U=lqr_cost_U,
            grid_X=grid_X,
            grid_Y=grid_Y,
            reward=0.0,
            reward_lqr=0.0,
            step=state.step,
        ), X, X_lqr, state.params)

    logging.info('Finished integrating, reward of %f, lqr reward of %f',
                 data.reward, data.reward_lqr)

    # Convert back to numpy for plotting.
    return Data(
        t=numpy.array(data.t),
        X=numpy.array(data.X),
        X_lqr=numpy.array(data.X_lqr),
        U=numpy.array(data.U),
        U_lqr=numpy.array(data.U_lqr),
        cost=numpy.array(data.cost),
        cost_lqr=numpy.array(data.cost_lqr),
        q1_grid=numpy.array(data.q1_grid),
        q2_grid=numpy.array(data.q2_grid),
        q_grid=numpy.array(data.q_grid),
        target_q_grid=numpy.array(data.target_q_grid),
        lqr_grid=numpy.array(data.lqr_grid),
        pi_grid_U=numpy.array(data.pi_grid_U),
        lqr_grid_U=numpy.array(data.lqr_grid_U),
        grid_X=numpy.array(data.grid_X),
        grid_Y=numpy.array(data.grid_Y),
        reward=float(data.reward),
        reward_lqr=float(data.reward_lqr),
        step=data.step,
    )


class Plotter(object):

    def __init__(self, data):
        # Make all the plots and axis.
        self.fig0, self.axs0 = pylab.subplots(3)
        self.fig0.supxlabel('Seconds')

        self.axs_velocity = self.axs0[0].twinx()

        self.x, = self.axs0[0].plot([], [], label="x")
        self.x_lqr, = self.axs0[0].plot([], [], label="x_lqr")

        self.axs0[0].set_ylabel('Position')
        self.axs0[0].legend()
        self.axs0[0].grid()

        self.v, = self.axs_velocity.plot([], [], label="v", color='C2')
        self.v_lqr, = self.axs_velocity.plot([], [], label="v_lqr", color='C3')
        self.axs_velocity.set_ylabel('Velocity')
        self.axs_velocity.legend()

        self.uaxis, = self.axs0[1].plot([], [], label="U")
        self.uaxis_lqr, = self.axs0[1].plot([], [], label="U_lqr")

        self.axs0[1].set_ylabel('Amps')
        self.axs0[1].legend()
        self.axs0[1].grid()

        self.costaxis, = self.axs0[2].plot([], [], label="cost")
        self.costlqraxis, = self.axs0[2].plot([], [], label="cost lqr")
        self.axs0[2].set_ylabel('Cost')
        self.axs0[2].legend()
        self.axs0[2].grid()

        self.costfig = pyplot.figure(figsize=pyplot.figaspect(0.5))
        self.cost3dax = [
            self.costfig.add_subplot(2, 3, 1, projection='3d'),
            self.costfig.add_subplot(2, 3, 2, projection='3d'),
            self.costfig.add_subplot(2, 3, 3, projection='3d'),
            self.costfig.add_subplot(2, 3, 4, projection='3d'),
            self.costfig.add_subplot(2, 3, 5, projection='3d'),
            self.costfig.add_subplot(2, 3, 6, projection='3d'),
        ]

        self.Ufig = pyplot.figure(figsize=pyplot.figaspect(0.5))
        self.Uax = [
            self.Ufig.add_subplot(1, 3, 1, projection='3d'),
            self.Ufig.add_subplot(1, 3, 2, projection='3d'),
            self.Ufig.add_subplot(1, 3, 3, projection='3d'),
        ]

        self.last_trajectory_step = 0
        self.last_cost_step = 0
        self.last_U_step = 0

    def update_trajectory_plot(self, data):
        if data.step == self.last_trajectory_step:
            return
        self.last_trajectory_step = data.step
        logging.info('Updating trajectory plots')

        # Put data in the trajectory plots.
        self.x.set_data(data.t, data.X[:, 0])
        self.v.set_data(data.t, data.X[:, 1])
        self.x_lqr.set_data(data.t, data.X_lqr[:, 0])
        self.v_lqr.set_data(data.t, data.X_lqr[:, 1])

        self.uaxis.set_data(data.t, data.U[:, 0])
        self.uaxis_lqr.set_data(data.t, data.U_lqr[:, 0])
        self.costaxis.set_data(data.t, data.cost[:, 0] - data.cost[-1, 0])
        self.costlqraxis.set_data(data.t, data.cost_lqr[:, 0])

        self.axs0[0].relim()
        self.axs0[0].autoscale_view()

        self.axs_velocity.relim()
        self.axs_velocity.autoscale_view()

        self.axs0[1].relim()
        self.axs0[1].autoscale_view()

        self.axs0[2].relim()
        self.axs0[2].autoscale_view()

        return self.x, self.v, self.uaxis, self.costaxis, self.costlqraxis

    def update_cost_plot(self, data):
        if data.step == self.last_cost_step:
            return
        logging.info('Updating cost plots')
        self.last_cost_step = data.step
        # Put data in the cost plots.
        if hasattr(self, 'costsurf'):
            for surf in self.costsurf:
                surf.remove()

        plots = [
            (data.q1_grid, 'q1'),
            (data.q2_grid, 'q2'),
            (data.q_grid, 'q'),
            (data.target_q_grid, 'target q'),
            (data.lqr_grid, 'lqr'),
            (data.q_grid - data.q_grid.max() - data.lqr_grid, 'error'),
        ]

        self.costsurf = [
            self.cost3dax[i].plot_surface(
                data.grid_X,
                data.grid_Y,
                Z,
                cmap="magma",
                label=label,
            ) for i, (Z, label) in enumerate(plots)
        ]

        for axis in self.cost3dax:
            axis.legend()
            axis.relim()
            axis.autoscale_view()

        return self.costsurf

    def update_U_plot(self, data):
        if data.step == self.last_U_step:
            return
        self.last_U_step = data.step
        logging.info('Updating U plots')
        # Put data in the controller plots.
        if hasattr(self, 'Usurf'):
            for surf in self.Usurf:
                surf.remove()

        plots = [
            (data.lqr_grid_U, 'lqr'),
            (data.pi_grid_U, 'pi'),
            ((data.lqr_grid_U - data.pi_grid_U), 'error'),
        ]

        self.Usurf = [
            self.Uax[i].plot_surface(
                data.grid_X,
                data.grid_Y,
                Z,
                cmap="magma",
                label=label,
            ) for i, (Z, label) in enumerate(plots)
        ]

        for axis in self.Uax:
            axis.legend()
            axis.relim()
            axis.autoscale_view()

        return self.Usurf


def main(argv):
    if len(argv) > 1:
        raise absl.app.UsageError('Too many command-line arguments.')

    tf.config.experimental.set_visible_devices([], 'GPU')

    lock = threading.Lock()

    # Load data.
    data = generate_data()

    plotter = Plotter(data)

    # Event for shutting down the thread.
    shutdown = threading.Event()

    # Thread to grab new data periodically.
    def do_update():
        while True:
            nonlocal data

            my_data = generate_data(data.step)

            if my_data is not None:
                with lock:
                    data = my_data

            if shutdown.wait(timeout=3):
                return

    update_thread = threading.Thread(target=do_update)
    update_thread.start()

    # Now, update each of the plots every second with the new data.
    def update0(frame):
        with lock:
            my_data = data

        return plotter.update_trajectory_plot(my_data)

    def update1(frame):
        with lock:
            my_data = data

        return plotter.update_cost_plot(my_data)

    def update2(frame):
        with lock:
            my_data = data

        return plotter.update_U_plot(my_data)

    animation0 = FuncAnimation(plotter.fig0, update0, interval=1000)
    animation1 = FuncAnimation(plotter.costfig, update1, interval=1000)
    animation2 = FuncAnimation(plotter.Ufig, update2, interval=1000)

    pyplot.show()

    shutdown.set()
    update_thread.join()


if __name__ == '__main__':
    absl.flags.mark_flags_as_required(['workdir'])
    absl.app.run(main)
