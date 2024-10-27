#!/usr/bin/env python3
import os, sys

# Setup XLA first.
os.environ['XLA_FLAGS'] = ' '.join([
    # Teach it where to find CUDA
    '--xla_gpu_cuda_data_dir=/usr/lib/cuda',
    # Use up to 20 cores
    #'--xla_force_host_platform_device_count=6',
    # Dump XLA to /tmp/foo to aid debugging
    #'--xla_dump_to=/tmp/foo',
    #'--xla_gpu_enable_command_buffer='
    # Dump sharding
    #"--xla_dump_to=/tmp/foo",
    #"--xla_dump_hlo_pass_re=spmd|propagation"
])
os.environ['JAX_PLATFORMS'] = 'cpu'
os.environ['XLA_PYTHON_CLIENT_PREALLOCATE'] = 'false'
os.environ['XLA_PYTHON_CLIENT_MEM_FRACTION'] = '.50'

from absl import flags
from absl import app
import pickle
import numpy
from frc971.control_loops.swerve import dynamics
from frc971.control_loops.swerve.casadi_velocity_mpc_lib import MPC
import jax
import tensorflow as tf
from frc971.control_loops.swerve.velocity_controller.physics import SwerveProblem
from frc971.control_loops.swerve import jax_dynamics

FLAGS = flags.FLAGS

flags.DEFINE_bool('compileonly', False,
                  'If true, load casadi, don\'t compile it')

flags.DEFINE_float('vx', 1.0, 'Goal velocity in m/s in x')
flags.DEFINE_float('vy', 0.0, 'Goal velocity in m/s in y')
flags.DEFINE_float('omega', 0.0, 'Goal velocity in m/s in omega')
flags.DEFINE_integer('seed', 0, 'Seed for random initial state.')

flags.DEFINE_bool('save_plots', True,
                  'If true, save plots for each run as well.')
flags.DEFINE_string('outputdir', None, 'Directory to write problem results to')
flags.DEFINE_bool('quiet', False, 'If true, print a lot less')

flags.DEFINE_integer('num_solutions', 100,
                     'Number of random problems to solve.')
flags.DEFINE_integer('horizon', 200, 'Horizon to solve for')

try:
    from matplotlib import pylab
except ModuleNotFoundError:
    pass


def collect_experience(problem, mpc, rng):
    X_initial = numpy.array(problem.random_states(rng,
                                                  dimensions=1)).transpose()

    R_goal = numpy.zeros((3, 1))
    R_goal[0, 0] = FLAGS.vx
    R_goal[1, 0] = FLAGS.vy
    R_goal[2, 0] = FLAGS.omega

    solution = mpc.solve(p=numpy.vstack((X_initial, R_goal)))

    # Solver doesn't solve for the last state.  So we get N-1 states back.
    experience = {
        'observations1': numpy.zeros((mpc.N - 1, problem.num_states)),
        'observations2': numpy.zeros((mpc.N - 1, problem.num_states)),
        'actions': numpy.zeros((mpc.N - 1, problem.num_outputs)),
        'rewards': numpy.zeros((mpc.N - 1, 1)),
        'goals': numpy.zeros((mpc.N - 1, problem.num_goals)),
    }

    if not FLAGS.quiet:
        print('x(0):', X_initial.transpose())

    X_prior = X_initial.squeeze()
    for j in range(mpc.N - 1):
        if not FLAGS.quiet:
            print(f'u({j}): ', mpc.unpack_u(solution, j))
            print(f'x({j+1}): ', mpc.unpack_x(solution, j + 1))
        experience['observations1'][j, :] = X_prior
        X_prior = mpc.unpack_x(solution, j + 1)
        experience['observations2'][j, :] = X_prior
        experience['actions'][j, :] = mpc.unpack_u(solution, j)
        experience['rewards'][j, :] = problem.reward(
            X=X_prior,
            U=mpc.unpack_u(solution, j),
            goal=R_goal[:, 0],
        )
        experience['goals'][j, :] = R_goal[:, 0]
        sys.stderr.flush()
        sys.stdout.flush()

    return experience


def save_experience(problem, mpc, experience, experience_number):
    with open(f'experience_{experience_number}.pkl', 'wb') as f:
        pickle.dump(experience, f)

    if not FLAGS.save_plots:
        return

    fig0, axs0 = pylab.subplots(3)
    fig1, axs1 = pylab.subplots(2)

    axs0[0].clear()
    axs0[1].clear()
    axs0[2].clear()

    t = problem.dt * numpy.array(list(range(mpc.N - 1)))

    X_plot = experience['observations1']
    U_plot = experience['actions']

    axs0[0].plot(t, X_plot[:, dynamics.VELOCITY_STATE_VX], label="vx")
    axs0[0].plot(t, X_plot[:, dynamics.VELOCITY_STATE_VY], label="vy")
    axs0[0].plot(t, X_plot[:, dynamics.VELOCITY_STATE_OMEGA], label="omega")
    axs0[0].legend()

    axs0[1].plot(t, U_plot[:, 0], label="Is0")
    axs0[1].plot(t, U_plot[:, 2], label="Is1")
    axs0[1].plot(t, U_plot[:, 4], label="Is2")
    axs0[1].plot(t, U_plot[:, 6], label="Is3")
    axs0[1].legend()

    axs0[2].plot(t, U_plot[:, 1], label="Id0")
    axs0[2].plot(t, U_plot[:, 3], label="Id1")
    axs0[2].plot(t, U_plot[:, 5], label="Id2")
    axs0[2].plot(t, U_plot[:, 7], label="Id3")
    axs0[2].legend()

    axs1[0].clear()
    axs1[1].clear()

    axs1[0].plot(t, X_plot[:, dynamics.VELOCITY_STATE_THETAS0], label='steer0')
    axs1[0].plot(t, X_plot[:, dynamics.VELOCITY_STATE_THETAS1], label='steer1')
    axs1[0].plot(t, X_plot[:, dynamics.VELOCITY_STATE_THETAS2], label='steer2')
    axs1[0].plot(t, X_plot[:, dynamics.VELOCITY_STATE_THETAS3], label='steer3')
    axs1[0].legend()
    axs1[1].plot(t,
                 X_plot[:, dynamics.VELOCITY_STATE_OMEGAS0],
                 label='steer_velocity0')
    axs1[1].plot(t,
                 X_plot[:, dynamics.VELOCITY_STATE_OMEGAS1],
                 label='steer_velocity1')
    axs1[1].plot(t,
                 X_plot[:, dynamics.VELOCITY_STATE_OMEGAS2],
                 label='steer_velocity2')
    axs1[1].plot(t,
                 X_plot[:, dynamics.VELOCITY_STATE_OMEGAS3],
                 label='steer_velocity3')
    axs1[1].legend()

    fig0.savefig(f'state_{experience_number}.svg')
    fig1.savefig(f'steer_{experience_number}.svg')


def main(argv):
    if FLAGS.outputdir:
        os.chdir(FLAGS.outputdir)

    # Hide any GPUs from TensorFlow. Otherwise it might reserve memory.
    tf.config.experimental.set_visible_devices([], 'GPU')
    rng = jax.random.key(FLAGS.seed)

    physics_constants = jax_dynamics.Coefficients()
    problem = SwerveProblem(physics_constants)
    mpc = MPC(solver='ipopt', N=(FLAGS.horizon + 1))

    if FLAGS.compileonly:
        return

    for i in range(FLAGS.num_solutions):
        rng, rng_init = jax.random.split(rng)
        experience = collect_experience(problem, mpc, rng_init)

        save_experience(problem, mpc, experience, i)
        logging.info('Solved problem %d', i)


if __name__ == '__main__':
    app.run(main)
