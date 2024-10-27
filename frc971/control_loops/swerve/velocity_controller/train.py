# Machine learning based on Soft Actor Critic(SAC) which was initially proposed in https://arxiv.org/pdf/1801.01290.
# Our implementation was heavily based on OpenAI's spinning up reference implementation https://spinningup.openai.com/en/latest/algorithms/sac.html.

import absl
import time
import collections
from absl import logging
import flax
import matplotlib
from matplotlib import pyplot
from flax import linen as nn
from flax.training import train_state
from flax.training import checkpoints
import jax
import inspect
import aim
import jax.numpy as jnp
import ml_collections
import numpy as np
import optax
import numpy
from frc971.control_loops.swerve import jax_dynamics
from functools import partial
import flashbax
from jax.experimental.ode import odeint
import orbax.checkpoint
from frc971.control_loops.swerve.velocity_controller.model import *
from frc971.control_loops.swerve.velocity_controller.physics import *

numpy.set_printoptions(linewidth=200, )

FLAGS = absl.flags.FLAGS

absl.flags.DEFINE_integer(
    'horizon',
    default=25,
    help='MPC horizon',
)

absl.flags.DEFINE_integer(
    'random_sample_steps',
    default=10000,
    help='Number of steps to randomly sample before using the policy',
)

absl.flags.DEFINE_integer(
    'steps',
    default=400000,
    help='Number of steps to run and train the agent',
)

absl.flags.DEFINE_integer(
    'warmup_steps',
    default=300000,
    help='Number of steps to warm up training',
)

absl.flags.DEFINE_float(
    'gamma',
    default=0.999,
    help='Future discount.',
)

absl.flags.DEFINE_float(
    'alpha',
    default=0.2,
    help='Entropy.  If negative, automatically solve for it.',
)

absl.flags.DEFINE_float(
    'polyak',
    default=0.995,
    help='Time constant in polyak averaging for the target network.',
)

absl.flags.DEFINE_bool(
    'debug_nan',
    default=False,
    help='If true, explode on any NaNs found, and print them.',
)

absl.flags.DEFINE_bool(
    'maximum_entropy_q',
    default=True,
    help=
    'If false, do not add the maximum entropy term to the bellman backup for Q.',
)


def save_checkpoint(state: TrainState, workdir: str):
    """Saves a checkpoint in the workdir."""
    # TODO(austin): use orbax directly.
    step = int(state.step)
    logging.info('Saving checkpoint step %d.', step)
    checkpoints.save_checkpoint_multiprocess(workdir, state, step, keep=10)


def restore_checkpoint(state: TrainState, workdir: str):
    """Restores the latest checkpoint from the workdir."""
    return checkpoints.restore_checkpoint(workdir, state)


def has_nan(x):
    if isinstance(x, jnp.ndarray):
        return jnp.any(jnp.isnan(x))
    else:
        return jnp.any(
            jax.numpy.array([has_nan(v)
                             for v in jax.tree_util.tree_leaves(x)]))


def print_nan(step, x):
    if not FLAGS.debug_nan:
        return

    caller = inspect.getframeinfo(inspect.stack()[1][0])
    # TODO(austin): It is helpful to sometimes start printing at a certain step.
    jax.lax.cond(
        has_nan(x), lambda: jax.debug.print(caller.filename +
                                            ':{l} step {s} isnan(X) -> {x}',
                                            l=caller.lineno,
                                            s=step,
                                            x=x), lambda: None)


@jax.jit
def compute_loss_q(state: TrainState, rng: PRNGKey, params, data: ArrayLike):
    """Computes the Soft Actor-Critic loss for Q1 and Q2."""
    observations1 = data['observations1']
    actions = data['actions']
    rewards = data['rewards']
    observations2 = data['observations2']
    R = data['goals']

    # Compute the ending actions from the current network.
    action2, logp_pi2, _ = state.pi_apply(rng=rng,
                                          params=params,
                                          observation=observations2,
                                          R=R)

    # Compute target network Q values
    q1_pi_target = state.q1_apply(state.target_params,
                                  observation=observations2,
                                  R=R,
                                  action=action2)
    q2_pi_target = state.q2_apply(state.target_params,
                                  observation=observations2,
                                  R=R,
                                  action=action2)
    q_pi_target = jax.numpy.minimum(q1_pi_target, q2_pi_target)

    alpha = jax.numpy.exp(params['logalpha'])

    # Now we can compute the Bellman backup
    # Max entropy SAC is based on https://arxiv.org/pdf/1812.05905.
    if FLAGS.maximum_entropy_q:
        bellman_backup = jax.lax.stop_gradient(
            rewards + FLAGS.gamma * (q_pi_target - alpha * logp_pi2))
    else:
        bellman_backup = jax.lax.stop_gradient(rewards +
                                               FLAGS.gamma * q_pi_target)

    # Compute the starting Q values from the Q network being optimized.
    q1 = state.q1_apply(params, observation=observations1, R=R, action=actions)
    q2 = state.q2_apply(params, observation=observations1, R=R, action=actions)

    # Mean squared error loss against Bellman backup
    q1_loss = ((q1 - bellman_backup)**2).mean()
    q2_loss = ((q2 - bellman_backup)**2).mean()
    return q1_loss + q2_loss


@jax.jit
def compute_loss_pi(state: TrainState, rng: PRNGKey, params, data: ArrayLike):
    """Computes the Soft Actor-Critic loss for pi."""
    observations1 = data['observations1']
    R = data['goals']
    # TODO(austin): We've got differentiable policy and differentiable physics.  Can we use those here?  Have Q learn the future, not the current step?

    # Compute the action
    pi, logp_pi, _ = state.pi_apply(rng=rng,
                                    params=params,
                                    observation=observations1,
                                    R=R)
    q1_pi = state.q1_apply(jax.lax.stop_gradient(params),
                           observation=observations1,
                           R=R,
                           action=pi)
    q2_pi = state.q2_apply(jax.lax.stop_gradient(params),
                           observation=observations1,
                           R=R,
                           action=pi)

    # And compute the Q of that action.
    q_pi = jax.numpy.minimum(q1_pi, q2_pi)

    alpha = jax.lax.stop_gradient(jax.numpy.exp(params['logalpha']))

    # Compute the entropy-regularized policy loss
    return (alpha * logp_pi - q_pi).mean()


@jax.jit
def compute_loss_alpha(state: TrainState, rng: PRNGKey, params,
                       data: ArrayLike):
    """Computes the Soft Actor-Critic loss for alpha."""
    observations1 = data['observations1']
    R = data['goals']
    pi, logp_pi, _ = jax.lax.stop_gradient(
        state.pi_apply(rng=rng, params=params, observation=observations1, R=R))

    return (-jax.numpy.exp(params['logalpha']) *
            (logp_pi + state.target_entropy)).mean(), logp_pi.mean()


@jax.jit
def compute_batched_loss_q(state: TrainState, rng: PRNGKey, params,
                           data: ArrayLike):

    def bound_compute_loss_q(rng, data):
        return compute_loss_q(state, rng, params, data)

    return jax.vmap(bound_compute_loss_q)(
        jax.random.split(rng, FLAGS.num_agents),
        data,
    ).mean()


@jax.jit
def compute_batched_loss_pi(state: TrainState, rng: PRNGKey, params,
                            data: ArrayLike):

    def bound_compute_loss_pi(rng, data):
        return compute_loss_pi(state, rng, params, data)

    return jax.vmap(bound_compute_loss_pi)(
        jax.random.split(rng, FLAGS.num_agents),
        data,
    ).mean()


@jax.jit
def compute_batched_loss_alpha(state: TrainState, rng: PRNGKey, params,
                               data: ArrayLike):

    def bound_compute_loss_alpha(rng, data):
        return compute_loss_alpha(state, rng, params, data)

    loss, entropy = jax.vmap(bound_compute_loss_alpha)(
        jax.random.split(rng, FLAGS.num_agents),
        data,
    )
    return (loss.mean(), entropy.mean())


@jax.jit
def train_step(state: TrainState, data, update_rng: PRNGKey,
               step: int) -> TrainState:
    """Updates the parameters for Q, Pi, target Q, and alpha."""
    update_rng, q_grad_rng = jax.random.split(update_rng)
    print_nan(step, data)

    # Update Q
    q_grad_fn = jax.value_and_grad(
        lambda params: compute_batched_loss_q(state, q_grad_rng, params, data))
    q_loss, q_grads = q_grad_fn(state.params)
    print_nan(step, q_loss)
    print_nan(step, q_grads)

    state = state.q_apply_gradients(step=step, grads=q_grads)

    # Update pi
    update_rng, pi_grad_rng = jax.random.split(update_rng)
    pi_grad_fn = jax.value_and_grad(lambda params: compute_batched_loss_pi(
        state, pi_grad_rng, params, data))
    pi_loss, pi_grads = pi_grad_fn(state.params)
    state = state.pi_apply_gradients(step=step, grads=pi_grads)

    update_rng, alpha_grad_rng = jax.random.split(update_rng)

    if FLAGS.alpha < 0.0:
        # Update alpha
        alpha_grad_fn = jax.value_and_grad(
            lambda params: compute_batched_loss_alpha(state, alpha_grad_rng,
                                                      params, data),
            has_aux=True,
        )
        (alpha_loss, entropy), alpha_grads = alpha_grad_fn(state.params)
        print_nan(step, alpha_loss)
        print_nan(step, alpha_grads)
        state = state.alpha_apply_gradients(step=step, grads=alpha_grads)
    else:
        entropy = 0.0
        alpha_loss = 0.0

    return state, q_loss, pi_loss, alpha_loss, entropy


@jax.jit
def collect_experience(state: TrainState, replay_buffer_state,
                       step_rng: PRNGKey, step):
    """Collects experience by simulating."""
    pi_rng = jax.random.fold_in(step_rng, step)
    pi_rng, initialization_rng = jax.random.split(pi_rng)
    pi_rng, goal_rng = jax.random.split(pi_rng)

    observation = jax.lax.with_sharding_constraint(
        state.problem.random_states(initialization_rng, FLAGS.num_agents),
        state.sharding)

    R = state.problem.random_goals(goal_rng, FLAGS.num_agents)

    def loop(i, val):
        """Runs 1 step of our simulation."""
        observation, pi_rng, replay_buffer_state = val
        pi_rng, action_rng = jax.random.split(pi_rng)
        logging.info('Observation shape: %s', observation.shape)

        def true_fn(i):
            # We are at the beginning of the process, pick a random action.
            return state.problem.random_actions(action_rng,
                                                X=observation,
                                                goal=R,
                                                dimensions=FLAGS.num_agents)

        def false_fn(i):
            # We are past the beginning of the process, use the trained network.
            pi_action, _, _ = state.pi_apply(rng=action_rng,
                                             params=state.params,
                                             observation=observation,
                                             R=R,
                                             deterministic=False)
            return pi_action

        pi_action = jax.lax.cond(
            step <= FLAGS.random_sample_steps,
            true_fn,
            false_fn,
            i,
        )

        # Compute the destination state.
        observation2 = jax.vmap(
            lambda o, pi: state.problem.integrate_dynamics(o, pi),
            in_axes=(0, 0))(observation, pi_action)

        reward = jax.vmap(state.problem.reward)(X=observation2,
                                                U=pi_action,
                                                goal=R)

        replay_buffer_state = state.replay_buffer.add(
            replay_buffer_state, {
                'observations1': observation,
                'observations2': observation2,
                'actions': pi_action,
                'rewards': reward.reshape((FLAGS.num_agents, 1)),
                'goals': R,
            })

        return observation2, pi_rng, replay_buffer_state

    # Run 1 horizon of simulation
    final_observation, final_pi_rng, final_replay_buffer_state = jax.lax.fori_loop(
        0, FLAGS.horizon + 1, loop, (observation, pi_rng, replay_buffer_state))

    return state, final_replay_buffer_state


@jax.jit
def update_gradients(rng: PRNGKey, state: TrainState, replay_buffer_state,
                     step: int):
    rng, sample_rng = jax.random.split(rng)

    def update_iteration(i, val):
        rng, state, q_loss, pi_loss, alpha_loss, replay_buffer_state, entropy = val
        rng, sample_rng, update_rng = jax.random.split(rng, 3)

        batch = state.replay_buffer.sample(replay_buffer_state, sample_rng)

        print_nan(i, replay_buffer_state)
        print_nan(i, batch)

        state, q_loss, pi_loss, alpha_loss, entropy = train_step(
            state, data=batch.experience, update_rng=update_rng, step=i)

        return rng, state, q_loss, pi_loss, alpha_loss, replay_buffer_state, entropy

    rng, state, q_loss, pi_loss, alpha_loss, replay_buffer_state, entropy = jax.lax.fori_loop(
        step, step + FLAGS.horizon + 1, update_iteration,
        (rng, state, 0.0, 0.0, 0.0, replay_buffer_state, 0))

    state = state.target_apply_gradients(step=state.step)

    return rng, state, q_loss, pi_loss, alpha_loss, entropy


def train(workdir: str, problem: Problem) -> train_state.TrainState:
    """Trains a Soft Actor-Critic controller."""
    rng = jax.random.key(0)
    rng, r_rng = jax.random.split(rng)

    run = aim.Run(repo='aim://127.0.0.1:53800')

    run['hparams'] = {
        'q_learning_rate': FLAGS.q_learning_rate,
        'pi_learning_rate': FLAGS.pi_learning_rate,
        'alpha_learning_rate': FLAGS.alpha_learning_rate,
        'random_sample_steps': FLAGS.random_sample_steps,
        'batch_size': FLAGS.batch_size,
        'horizon': FLAGS.horizon,
        'warmup_steps': FLAGS.warmup_steps,
        'steps': FLAGS.steps,
        'replay_size': FLAGS.replay_size,
        'num_agents': FLAGS.num_agents,
        'polyak': FLAGS.polyak,
        'gamma': FLAGS.gamma,
        'alpha': FLAGS.alpha,
        'final_q_learning_rate': FLAGS.final_q_learning_rate,
        'final_pi_learning_rate': FLAGS.final_pi_learning_rate,
    }

    # Setup TrainState
    rng, init_rng = jax.random.split(rng)
    q_learning_rate = create_learning_rate_fn(
        base_learning_rate=FLAGS.q_learning_rate,
        final_learning_rate=FLAGS.final_q_learning_rate)
    pi_learning_rate = create_learning_rate_fn(
        base_learning_rate=FLAGS.pi_learning_rate,
        final_learning_rate=FLAGS.final_pi_learning_rate)
    state = create_train_state(
        init_rng,
        problem,
        q_learning_rate=q_learning_rate,
        pi_learning_rate=pi_learning_rate,
        alpha_learning_rate=FLAGS.alpha_learning_rate,
    )
    state = restore_checkpoint(state, workdir)

    logging.debug(nn.get_sharding(state, state.mesh))

    replay_buffer_state = state.replay_buffer.init({
        'observations1':
        jax.numpy.zeros((problem.num_states, )),
        'observations2':
        jax.numpy.zeros((problem.num_states, )),
        'actions':
        jax.numpy.zeros((problem.num_outputs, )),
        'rewards':
        jax.numpy.zeros((1, )),
        'goals':
        jax.numpy.zeros((problem.num_goals, )),
    })

    logging.debug(nn.get_sharding(replay_buffer_state, state.mesh))

    # Number of gradients to accumulate before doing decent.
    update_after = FLAGS.batch_size // FLAGS.num_agents

    @partial(jax.jit, donate_argnums=(1, 2))
    def train_loop(state: TrainState, replay_buffer_state, rng: PRNGKey,
                   step: int):
        rng, step_rng = jax.random.split(rng)
        # Collect experience
        state, replay_buffer_state = collect_experience(
            state,
            replay_buffer_state,
            step_rng,
            step,
        )

        def nop(rng, state, replay_buffer_state, step):
            return rng, state.update_step(step=step), 0.0, 0.0, 0.0, 0.0

        # Train
        rng, state, q_loss, pi_loss, alpha_loss, entropy = jax.lax.cond(
            step >= update_after, update_gradients, nop, rng, state,
            replay_buffer_state, step)

        return state, replay_buffer_state, rng, q_loss, pi_loss, alpha_loss, entropy

    last_time = time.time()
    for step in range(0, FLAGS.steps, FLAGS.horizon):
        state, replay_buffer_state, rng, q_loss, pi_loss, alpha_loss, entropy = train_loop(
            state, replay_buffer_state, rng, step)

        if FLAGS.debug_nan and has_nan(state.params):
            logging.fatal('Nan params, aborting')

        logging.info(
            'Step %s: q_loss=%s, pi_loss=%s, alpha_loss=%s, q_learning_rate=%s, pi_learning_rate=%s, alpha=%s, entropy=%s, random=%s',
            step,
            q_loss,
            pi_loss,
            alpha_loss,
            q_learning_rate(step),
            pi_learning_rate(step),
            jax.numpy.exp(state.params['logalpha']),
            entropy,
            step <= FLAGS.random_sample_steps,
        )

        run.track(
            {
                'q_loss': float(q_loss),
                'pi_loss': float(pi_loss),
                'alpha_loss': float(alpha_loss),
                'alpha': float(jax.numpy.exp(state.params['logalpha'])),
                'entropy': entropy,
            },
            step=step)

        if time.time() > last_time + 3.0 and step > update_after:
            # TODO(austin): Simulate a rollout and accumulate the reward.  How good are we doing?
            save_checkpoint(state, workdir)
            last_time = time.time()

    return state
