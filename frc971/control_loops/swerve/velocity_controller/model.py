from __future__ import annotations
import flax
import flashbax
from typing import Any
import dataclasses
import absl
from absl import logging
import numpy
import jax
from flax import linen as nn
from jaxtyping import Array, ArrayLike
import optax
from flax.training import train_state
from jax.experimental import mesh_utils
from jax.sharding import Mesh, PartitionSpec, NamedSharding
from frc971.control_loops.swerve import jax_dynamics
from frc971.control_loops.swerve.velocity_controller import physics
from frc971.control_loops.swerve.velocity_controller import experience_buffer
from tensorflow_probability.substrates import jax as tfp

tfd = tfp.distributions
tfb = tfp.bijectors

from flax.typing import PRNGKey

FLAGS = absl.flags.FLAGS

absl.flags.DEFINE_integer(
    'num_agents',
    default=10,
    help='Training batch size.',
)

absl.flags.DEFINE_float(
    'q_learning_rate',
    default=0.002,
    help='Training learning rate.',
)

absl.flags.DEFINE_float(
    'final_q_learning_rate',
    default=0.001,
    help='Fraction of --q_learning_rate to reduce by by the end.',
)

absl.flags.DEFINE_float(
    'pi_learning_rate',
    default=0.002,
    help='Training learning rate.',
)

absl.flags.DEFINE_float(
    'final_pi_learning_rate',
    default=0.001,
    help='Fraction of --pi_learning_rate to reduce by by the end.',
)

absl.flags.DEFINE_integer(
    'replay_size',
    default=2000000,
    help='Number of steps to save in our replay buffer',
)

absl.flags.DEFINE_integer(
    'batch_size',
    default=10000,
    help='Batch size for learning Q and pi',
)

HIDDEN_WEIGHTS = 256

LOG_STD_MIN = -20
LOG_STD_MAX = 2


def gaussian_likelihood(noise: ArrayLike, log_std: ArrayLike):
    pre_sum = -0.5 * (noise**2 + 2 * log_std + jax.numpy.log(2 * jax.numpy.pi))

    if len(pre_sum.shape) > 1:
        return jax.numpy.sum(pre_sum, keepdims=True, axis=1)
    else:
        return jax.numpy.sum(pre_sum, keepdims=True)


class SquashedGaussianMLPActor(nn.Module):
    """Actor model."""

    # Number of dimensions in the action space
    action_space: int = 8

    hidden_sizes: list[int] = dataclasses.field(
        default_factory=lambda: [HIDDEN_WEIGHTS, HIDDEN_WEIGHTS])

    # Activation function
    activation: Callable = nn.activation.relu

    # Max action we can apply
    action_limit: float = 1

    @nn.compact
    def __call__(self,
                 observation: ArrayLike,
                 R: ArrayLike,
                 deterministic: bool = False,
                 rng: PRNGKey | None = None):
        x = jax.numpy.hstack((observation, R))
        # Apply the dense layers
        for i, hidden_size in enumerate(self.hidden_sizes):
            x = nn.Dense(
                name=f'denselayer{i}',
                features=hidden_size,
            )(x)
            x = self.activation(x)

        # Average policy is a dense function of the space.
        mu = nn.Dense(
            features=self.action_space,
            name='mu',
        )(x)

        log_std_layer = nn.Dense(features=self.action_space,
                                 name='log_std_layer')(x)

        # Clip the log of the standard deviation in a soft manner.
        log_std = LOG_STD_MIN + 0.5 * (LOG_STD_MAX - LOG_STD_MIN) * (
            flax.linen.activation.tanh(log_std_layer) + 1)

        std = jax.numpy.exp(log_std)

        if rng is None:
            rng = self.make_rng('pi')

        pi_distribution = tfd.TransformedDistribution(
            distribution=tfd.Normal(loc=mu, scale=std),
            bijector=tfb.Tanh(),
        )

        if deterministic:
            # We are testing the optimal policy, just use the mean.
            pi_action = flax.linen.activation.tanh(mu)
        else:
            pi_action = pi_distribution.sample(shape=(1, ), seed=rng)

        logp_pi = pi_distribution.log_prob(pi_action)

        if len(logp_pi.shape) > 1:
            logp_pi = jax.numpy.sum(logp_pi, keepdims=True, axis=1)
        else:
            logp_pi = jax.numpy.sum(logp_pi, keepdims=True)

        return pi_action, logp_pi, self.action_limit * std


class MLPQFunction(nn.Module):

    # Number and size of the hidden layers.
    hidden_sizes: list[int] = dataclasses.field(
        default_factory=lambda: [HIDDEN_WEIGHTS, HIDDEN_WEIGHTS])

    activation: Callable = nn.activation.tanh

    @nn.compact
    def __call__(self, observation: ArrayLike, R: ArrayLike,
                 action: ArrayLike):
        # Estimate Q with a simple multi layer dense network.
        x = jax.numpy.hstack((observation, R, action))
        for i, hidden_size in enumerate(self.hidden_sizes):
            x = nn.Dense(
                name=f'denselayer{i}',
                features=hidden_size,
            )(x)
            x = self.activation(x)

        x = nn.Dense(
            name=f'q',
            features=1,
        )(x)

        return x


class TrainState(flax.struct.PyTreeNode):
    problem: Problem = flax.struct.field(pytree_node=False)

    step: int | jax.Array = flax.struct.field(pytree_node=True)
    substep: int | jax.Array = flax.struct.field(pytree_node=True)

    params: flax.core.FrozenDict[str, typing.Any] = flax.struct.field(
        pytree_node=True)

    target_params: flax.core.FrozenDict[str, typing.Any] = flax.struct.field(
        pytree_node=True)

    pi_apply_fn: Callable = flax.struct.field(pytree_node=False)
    q1_apply_fn: Callable = flax.struct.field(pytree_node=False)
    q2_apply_fn: Callable = flax.struct.field(pytree_node=False)

    pi_tx: optax.GradientTransformation = flax.struct.field(pytree_node=False)
    pi_opt_state: optax.OptState = flax.struct.field(pytree_node=True)
    q_tx: optax.GradientTransformation = flax.struct.field(pytree_node=False)
    q_opt_state: optax.OptState = flax.struct.field(pytree_node=True)

    alpha_tx: optax.GradientTransformation = flax.struct.field(
        pytree_node=False)
    alpha_opt_state: optax.OptState = flax.struct.field(pytree_node=True)

    target_entropy: float = flax.struct.field(pytree_node=True)

    mesh: Mesh = flax.struct.field(pytree_node=False)
    sharding: NamedSharding = flax.struct.field(pytree_node=False)
    replicated_sharding: NamedSharding = flax.struct.field(pytree_node=False)

    replay_buffer: flashbax.buffers.trajectory_buffer.TrajectoryBuffer = flax.struct.field(
        pytree_node=False)

    def pi_apply(self,
                 rng: PRNGKey,
                 params: flax.core.FrozenDict[str, typing.Any],
                 observation: ArrayLike,
                 R: ArrayLike,
                 deterministic: bool = False):
        return self.pi_apply_fn(
            {'params': params['pi']},
            observation=self.problem.unwrap_angles(observation),
            R=R,
            deterministic=deterministic,
            rngs={'pi': rng})

    def q1_apply(self, params: flax.core.FrozenDict[str, typing.Any],
                 observation: ArrayLike, R: ArrayLike, action: ArrayLike):
        return self.q1_apply_fn(
            {'params': params['q1']},
            observation=self.problem.unwrap_angles(observation),
            R=R,
            action=action)

    def q2_apply(self, params: flax.core.FrozenDict[str, typing.Any],
                 observation: ArrayLike, R: ArrayLike, action: ArrayLike):
        return self.q2_apply_fn(
            {'params': params['q2']},
            observation=self.problem.unwrap_angles(observation),
            R=R,
            action=action)

    def pi_apply_gradients(self, step: int, grads):
        updates, new_pi_opt_state = self.pi_tx.update(grads, self.pi_opt_state,
                                                      self.params)
        new_params = optax.apply_updates(self.params, updates)

        return self.replace(
            step=step,
            substep=jax.lax.select(step != self.step, 0, self.substep + 1),
            params=new_params,
            pi_opt_state=new_pi_opt_state,
        )

    def q_apply_gradients(self, step: int, grads):
        updates, new_q_opt_state = self.q_tx.update(grads, self.q_opt_state,
                                                    self.params)
        new_params = optax.apply_updates(self.params, updates)

        return self.replace(
            step=step,
            substep=jax.lax.select(step != self.step, 0, self.substep + 1),
            params=new_params,
            q_opt_state=new_q_opt_state,
        )

    def target_apply_gradients(self, step):
        new_target_params = optax.incremental_update(self.params,
                                                     self.target_params,
                                                     1 - FLAGS.polyak)

        return self.replace(
            step=step,
            substep=jax.lax.select(step != self.step, 0, self.substep + 1),
            target_params=new_target_params,
        )

    def alpha_apply_gradients(self, step, grads):
        updates, new_alpha_opt_state = self.alpha_tx.update(
            grads, self.alpha_opt_state, self.params)
        new_params = optax.apply_updates(self.params, updates)

        return self.replace(
            step=step,
            substep=jax.lax.select(step != self.step, 0, self.substep + 1),
            params=new_params,
            alpha_opt_state=new_alpha_opt_state,
        )

    def update_step(self, step):
        return self.replace(
            step=step,
            substep=jax.lax.select(step != self.step, 0, self.substep + 1),
        )

    @classmethod
    def create(cls, *, problem: Problem, params, pi_tx, q_tx, alpha_tx,
               pi_apply_fn, q1_apply_fn, q2_apply_fn, **kwargs):
        """Creates a new instance with ``step=0`` and initialized ``opt_state``."""

        pi_tx = optax.multi_transform(
            {
                'train': pi_tx,
                'freeze': optax.set_to_zero()
            },
            param_labels=flax.traverse_util.path_aware_map(
                lambda path, x: 'train'
                if path[0] == 'pi' else 'freeze', params),
        )
        pi_opt_state = pi_tx.init(params)

        q_tx = optax.multi_transform(
            {
                'train': q_tx,
                'freeze': optax.set_to_zero()
            },
            param_labels=flax.traverse_util.path_aware_map(
                lambda path, x: 'train'
                if path[0] == 'q1' or path[0] == 'q2' else 'freeze', params),
        )
        q_opt_state = q_tx.init(params)

        alpha_tx = optax.multi_transform(
            {
                'train': alpha_tx,
                'freeze': optax.set_to_zero()
            },
            param_labels=flax.traverse_util.path_aware_map(
                lambda path, x: 'train'
                if path[0] == 'logalpha' else 'freeze', params),
        )
        alpha_opt_state = alpha_tx.init(params)

        mesh = Mesh(
            devices=mesh_utils.create_device_mesh(len(jax.devices())),
            axis_names=('batch', ),
        )
        print('Devices:', jax.devices())
        sharding = NamedSharding(mesh, PartitionSpec('batch'))
        replicated_sharding = NamedSharding(mesh, PartitionSpec())

        replay_buffer = experience_buffer.make_experience_buffer(
            num_agents=FLAGS.num_agents,
            sample_batch_size=FLAGS.batch_size,
            length=FLAGS.replay_size)

        result = cls(
            problem=problem,
            step=0,
            substep=0,
            params=params,
            target_params=params,
            q1_apply_fn=q1_apply_fn,
            q2_apply_fn=q2_apply_fn,
            pi_apply_fn=pi_apply_fn,
            pi_tx=pi_tx,
            pi_opt_state=pi_opt_state,
            q_tx=q_tx,
            q_opt_state=q_opt_state,
            alpha_tx=alpha_tx,
            alpha_opt_state=alpha_opt_state,
            target_entropy=-problem.num_states,
            mesh=mesh,
            sharding=sharding,
            replicated_sharding=replicated_sharding,
            replay_buffer=replay_buffer,
        )

        return jax.device_put(result, replicated_sharding)


def create_train_state(rng: PRNGKey, problem: Problem, q_learning_rate,
                       pi_learning_rate):
    """Creates initial `TrainState`."""
    pi = SquashedGaussianMLPActor(activation=nn.activation.gelu,
                                  action_space=problem.num_outputs,
                                  action_limit=problem.action_limit)
    # We want q1 and q2 to have different network architectures so they pick up differnet things.
    q1 = MLPQFunction(activation=nn.activation.gelu, hidden_sizes=[128, 256])
    q2 = MLPQFunction(activation=nn.activation.gelu, hidden_sizes=[256, 128])

    @jax.jit
    def init_params(rng):
        pi_rng, q1_rng, q2_rng = jax.random.split(rng, num=3)

        pi_params = pi.init(
            pi_rng,
            observation=jax.numpy.ones([problem.num_unwrapped_states]),
            R=jax.numpy.ones([problem.num_goals]),
        )['params']
        q1_params = q1.init(
            q1_rng,
            observation=jax.numpy.ones([problem.num_unwrapped_states]),
            R=jax.numpy.ones([problem.num_goals]),
            action=jax.numpy.ones([problem.num_outputs]),
        )['params']
        q2_params = q2.init(
            q2_rng,
            observation=jax.numpy.ones([problem.num_unwrapped_states]),
            R=jax.numpy.ones([problem.num_goals]),
            action=jax.numpy.ones([problem.num_outputs]),
        )['params']

        if FLAGS.alpha < 0.0:
            logalpha = 0.0
        else:
            logalpha = jax.numpy.log(FLAGS.alpha)

        return {
            'q1': q1_params,
            'q2': q2_params,
            'pi': pi_params,
            'logalpha': logalpha,
        }

    pi_tx = optax.sgd(learning_rate=pi_learning_rate)
    q_tx = optax.sgd(learning_rate=q_learning_rate)
    alpha_tx = optax.sgd(learning_rate=q_learning_rate)

    result = TrainState.create(
        problem=problem,
        params=init_params(rng),
        pi_tx=pi_tx,
        q_tx=q_tx,
        alpha_tx=alpha_tx,
        pi_apply_fn=pi.apply,
        q1_apply_fn=q1.apply,
        q2_apply_fn=q2.apply,
    )

    return result


def create_learning_rate_fn(
    base_learning_rate: float,
    final_learning_rate: float,
):
    """Create learning rate schedule."""
    warmup_fn = optax.linear_schedule(
        init_value=base_learning_rate,
        end_value=base_learning_rate,
        transition_steps=FLAGS.warmup_steps,
    )

    cosine_epochs = max(FLAGS.steps - FLAGS.warmup_steps, 1)
    cosine_fn = optax.cosine_decay_schedule(init_value=base_learning_rate,
                                            decay_steps=cosine_epochs,
                                            alpha=final_learning_rate)

    schedule_fn = optax.join_schedules(
        schedules=[warmup_fn, cosine_fn],
        boundaries=[FLAGS.warmup_steps],
    )
    return schedule_fn
