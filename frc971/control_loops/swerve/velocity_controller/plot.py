#!/usr/bin/env python3

import os

os.environ['XLA_FLAGS'] = ' '.join([
    # Teach it where to find CUDA
    '--xla_gpu_cuda_data_dir=/usr/lib/cuda',
    # Use up to 20 cores
    '--xla_force_host_platform_device_count=20',
    # Dump XLA to /tmp/foo to aid debugging
    #'--xla_dump_to=/tmp/foo',
    #'--xla_gpu_enable_command_buffer='
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
import tensorflow as tf
from jax.experimental.ode import odeint
import threading
import collections

from frc971.control_loops.swerve.velocity_controller.model import *
from frc971.control_loops.swerve.velocity_controller.physics import *

# Container for the data.
Data = collections.namedtuple('Data', [
    't', 'X', 'U', 'logp_pi', 'cost', 'q1_grid', 'q2_grid', 'q_grid',
    'target_q_grid', 'pi_grid_U', 'grid_X', 'grid_Y', 'reward', 'step',
    'rewards'
])

FLAGS = absl.flags.FLAGS

absl.flags.DEFINE_string('workdir', None, 'Directory to store model data.')

absl.flags.DEFINE_integer(
    'horizon',
    default=100,
    help='MPC horizon',
)

numpy.set_printoptions(linewidth=200, )

absl.flags.DEFINE_float(
    'alpha',
    default=0.2,
    help='Entropy.  If negative, automatically solve for it.',
)


def restore_checkpoint(state: TrainState, workdir: str):
    return checkpoints.restore_checkpoint(workdir, state)


dt = 0.005


def X0Full():
    module_theta = 0.0
    module_omega = 0.0
    theta = 0.0
    vx = 1.0
    dtheta = 0.05
    vy = 0.0
    drive_omega = jax.numpy.hypot(vx, vy) / jax_dynamics.WHEEL_RADIUS
    omega = 0.0
    return jax.numpy.array([
        module_theta + dtheta,
        0.0,
        module_omega,
        drive_omega,
        module_theta + dtheta,
        0.0,
        module_omega,
        drive_omega,
        module_theta - dtheta,
        0.0,
        module_omega,
        drive_omega,
        module_theta - dtheta,
        0.0,
        module_omega,
        drive_omega,
        0.0,
        0.0,
        theta,
        vx,
        vy,
        omega,
        0.0,
        0.0,
        0.0,
    ])


def generate_data(step=None):
    grid_X = numpy.arange(-1, 1, 0.1)
    grid_Y = numpy.arange(-10, 10, 0.1)
    grid_X, grid_Y = numpy.meshgrid(grid_X, grid_Y)
    grid_X = jax.numpy.array(grid_X)
    grid_Y = jax.numpy.array(grid_Y)
    # Load the training state.
    physics_constants = jax_dynamics.Coefficients()
    problem = physics.SwerveProblem(physics_constants)

    rng = jax.random.key(0)
    rng, init_rng = jax.random.split(rng)

    state = create_train_state(init_rng,
                               problem,
                               FLAGS.q_learning_rate,
                               FLAGS.pi_learning_rate,
                               alpha_learning_rate=0.001)

    state = restore_checkpoint(state, FLAGS.workdir)

    X = X0Full()
    X_lqr = X.copy()
    goal = jax.numpy.array([1.0, 0.0, 0.0])

    logging.info('X: %s', X)
    logging.info('goal: %s', goal)
    logging.debug('params: %s', state.params)

    # Now simulate the robot, accumulating up things to plot.
    def loop(i, val):
        X, data, params = val
        t = data.t.at[i].set(i * problem.dt)

        U, logp_pi, std = state.pi_apply(
            rng,
            params,
            observation=state.problem.unwrap_angles(
                jax_dynamics.to_velocity_state(X)),
            R=goal,
            deterministic=True)

        logp_pi = logp_pi * jax.numpy.exp(params['logalpha'])

        jax.debug.print('mu: {mu} std: {std}', mu=U, std=std)

        step_reward = state.problem.q_reward(jax_dynamics.to_velocity_state(X),
                                             U, goal)
        reward = data.reward + step_reward

        cost = jax.numpy.minimum(
            state.q1_apply(params,
                           observation=state.problem.unwrap_angles(
                               jax_dynamics.to_velocity_state(X)),
                           R=goal,
                           action=U),
            state.q2_apply(params,
                           observation=state.problem.unwrap_angles(
                               jax_dynamics.to_velocity_state(X)),
                           R=goal,
                           action=U))

        U = U * problem.action_limit
        U_plot = data.U.at[i, :].set(U)
        rewards = data.rewards.at[i, :].set(step_reward)
        X_plot = data.X.at[i, :].set(X)
        cost_plot = data.cost.at[i, :].set(cost)
        logp_pi_plot = data.logp_pi.at[i, :].set(logp_pi)

        # TODO(austin): I'd really like to visualize the slip angle per wheel.
        # Maybe also the force deviation, etc.
        # I think that would help enormously in figuring out how good a specific solution is.

        def fn(X, t):
            return jax_dynamics.full_dynamics(problem.coefficients, X,
                                              U).flatten()

        X = odeint(fn, X, jax.numpy.array([0, problem.dt]))

        return X[1, :], data._replace(
            t=t,
            U=U_plot,
            X=X_plot,
            logp_pi=logp_pi_plot,
            cost=cost_plot,
            reward=reward,
            rewards=rewards,
        ), params

    # Do it.
    @jax.jit
    def integrate(data, X, params):
        return jax.lax.fori_loop(0, FLAGS.horizon, loop, (X, data, params))

    X, data, params = integrate(
        Data(
            t=jax.numpy.zeros((FLAGS.horizon, )),
            X=jax.numpy.zeros((FLAGS.horizon, jax_dynamics.NUM_STATES)),
            U=jax.numpy.zeros((FLAGS.horizon, state.problem.num_outputs)),
            logp_pi=jax.numpy.zeros((FLAGS.horizon, 1)),
            rewards=jax.numpy.zeros((FLAGS.horizon, 1)),
            cost=jax.numpy.zeros((FLAGS.horizon, 1)),
            q1_grid=jax.numpy.zeros(grid_X.shape),
            q2_grid=jax.numpy.zeros(grid_X.shape),
            q_grid=jax.numpy.zeros(grid_X.shape),
            target_q_grid=jax.numpy.zeros(grid_X.shape),
            pi_grid_U=jax.numpy.zeros(grid_X.shape),
            grid_X=grid_X,
            grid_Y=grid_Y,
            reward=0.0,
            step=state.step,
        ), X, state.params)

    logging.info('Reward: %s', float(data.reward))

    # Convert back to numpy for plotting.
    return Data(
        t=numpy.array(data.t),
        X=numpy.array(data.X),
        U=numpy.array(data.U),
        logp_pi=numpy.array(data.logp_pi),
        cost=numpy.array(data.cost),
        q1_grid=numpy.array(data.q1_grid),
        q2_grid=numpy.array(data.q2_grid),
        q_grid=numpy.array(data.q_grid),
        target_q_grid=numpy.array(data.target_q_grid),
        pi_grid_U=numpy.array(data.pi_grid_U),
        grid_X=numpy.array(data.grid_X),
        grid_Y=numpy.array(data.grid_Y),
        rewards=numpy.array(data.rewards),
        reward=float(data.reward),
        step=data.step,
    )


class Plotter(object):

    def __init__(self, data):
        # Make all the plots and axis.
        self.fig0, self.axs0 = pylab.subplots(3)
        self.fig0.supxlabel('Seconds')

        self.vx, = self.axs0[0].plot([], [], label="vx")
        self.vy, = self.axs0[0].plot([], [], label="vy")
        self.omega, = self.axs0[0].plot([], [], label="omega")
        self.axs0[0].set_ylabel('Velocity')
        self.axs0[0].legend()
        self.axs0[0].grid()

        self.steer0, = self.axs0[1].plot([], [], label="Steer0")
        self.steer1, = self.axs0[1].plot([], [], label="Steer1")
        self.steer2, = self.axs0[1].plot([], [], label="Steer2")
        self.steer3, = self.axs0[1].plot([], [], label="Steer3")
        self.axs0[1].set_ylabel('Amps')
        self.axs0[1].legend()
        self.axs0[1].grid()

        self.drive0, = self.axs0[2].plot([], [], label="Drive0")
        self.drive1, = self.axs0[2].plot([], [], label="Drive1")
        self.drive2, = self.axs0[2].plot([], [], label="Drive2")
        self.drive3, = self.axs0[2].plot([], [], label="Drive3")
        self.axs0[2].set_ylabel('Amps')
        self.axs0[2].legend()
        self.axs0[2].grid()

        self.fig1, self.axs1 = pylab.subplots(3)
        self.fig1.supxlabel('Seconds')

        self.theta0, = self.axs1[0].plot([], [], label='steer position0')
        self.theta1, = self.axs1[0].plot([], [], label='steer position1')
        self.theta2, = self.axs1[0].plot([], [], label='steer position2')
        self.theta3, = self.axs1[0].plot([], [], label='steer position3')
        self.axs1[0].set_ylabel('Radians')
        self.axs1[0].legend()
        self.omega0, = self.axs1[1].plot([], [], label='steer velocity0')
        self.omega1, = self.axs1[1].plot([], [], label='steer velocity1')
        self.omega2, = self.axs1[1].plot([], [], label='steer velocity2')
        self.omega3, = self.axs1[1].plot([], [], label='steer velocity3')
        self.axs1[1].set_ylabel('Radians/second')
        self.axs1[1].legend()

        self.logp_axis = self.axs1[2].twinx()
        self.cost, = self.axs1[2].plot([], [], label='cost')
        self.reward, = self.axs1[2].plot([], [], label='reward')
        self.axs1[2].set_ylabel('Radians/second')
        self.axs1[2].legend()

        self.logp_pi, = self.logp_axis.plot([], [],
                                            label='logp_pi',
                                            color='C2')
        self.logp_axis.set_ylabel('log(liklihood)*alpha')
        self.logp_axis.legend()

        self.last_robot_step = 0
        self.last_steer_step = 0

    def update_robot_plot(self, data):
        if data.step is not None and data.step == self.last_robot_step:
            return
        self.last_robot_step = data.step
        logging.info('Updating robot plots')
        self.fig0.suptitle(f'Step {data.step}')

        self.vx.set_data(data.t, data.X[:, jax_dynamics.STATE_VX])
        self.vy.set_data(data.t, data.X[:, jax_dynamics.STATE_VY])
        self.omega.set_data(data.t, data.X[:, jax_dynamics.STATE_OMEGA])

        self.axs0[0].relim()
        self.axs0[0].autoscale_view()

        self.steer0.set_data(data.t, data.U[:, 0])
        self.steer1.set_data(data.t, data.U[:, 2])
        self.steer2.set_data(data.t, data.U[:, 4])
        self.steer3.set_data(data.t, data.U[:, 6])
        self.axs0[1].relim()
        self.axs0[1].autoscale_view()

        self.drive0.set_data(data.t, data.U[:, 1])
        self.drive1.set_data(data.t, data.U[:, 3])
        self.drive2.set_data(data.t, data.U[:, 5])
        self.drive3.set_data(data.t, data.U[:, 7])
        self.axs0[2].relim()
        self.axs0[2].autoscale_view()

        return (self.vx, self.vy, self.omega, self.steer0, self.steer1,
                self.steer2, self.steer3, self.drive0, self.drive1,
                self.drive2, self.drive3)

    def update_steer_plot(self, data):
        if data.step == self.last_steer_step:
            return
        self.last_steer_step = data.step
        logging.info('Updating steer plots')
        self.fig1.suptitle(f'Step {data.step}')

        self.theta0.set_data(data.t, data.X[:, jax_dynamics.STATE_THETAS0])
        self.theta1.set_data(data.t, data.X[:, jax_dynamics.STATE_THETAS1])
        self.theta2.set_data(data.t, data.X[:, jax_dynamics.STATE_THETAS2])
        self.theta3.set_data(data.t, data.X[:, jax_dynamics.STATE_THETAS3])
        self.axs1[0].relim()
        self.axs1[0].autoscale_view()

        self.omega0.set_data(data.t, data.X[:, jax_dynamics.STATE_OMEGAS0])
        self.omega1.set_data(data.t, data.X[:, jax_dynamics.STATE_OMEGAS1])
        self.omega2.set_data(data.t, data.X[:, jax_dynamics.STATE_OMEGAS2])
        self.omega3.set_data(data.t, data.X[:, jax_dynamics.STATE_OMEGAS3])
        self.axs1[1].relim()
        self.axs1[1].autoscale_view()

        self.cost.set_data(data.t, data.cost)
        self.reward.set_data(data.t, data.rewards)
        self.logp_pi.set_data(data.t, data.logp_pi)
        self.axs1[2].relim()
        self.axs1[2].autoscale_view()
        self.logp_axis.relim()
        self.logp_axis.autoscale_view()

        return (self.theta0, self.theta1, self.theta2, self.theta3,
                self.omega0, self.omega1, self.omega2, self.omega3, self.cost,
                self.logp_pi, self.reward)


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

        return plotter.update_robot_plot(my_data)

    def update1(frame):
        with lock:
            my_data = data

        return plotter.update_steer_plot(my_data)

    animation0 = FuncAnimation(plotter.fig0, update0, interval=1000)
    animation1 = FuncAnimation(plotter.fig1, update1, interval=1000)

    pyplot.show()

    shutdown.set()
    update_thread.join()


if __name__ == '__main__':
    absl.flags.mark_flags_as_required(['workdir'])
    absl.app.run(main)
