import jax
from functools import partial
from frc971.control_loops.swerve import dynamics
from absl import logging
from frc971.control_loops.swerve import jax_dynamics
from flax.typing import PRNGKey


class Problem(object):

    def __init__(self, num_states: int, num_unwrapped_states: int,
                 num_outputs: int, num_goals: int, action_limit: float):
        self.num_states = num_states
        self.num_unwrapped_states = num_unwrapped_states
        self.num_outputs = num_outputs
        self.num_goals = num_goals
        self.action_limit = action_limit

    def integrate_dynamics(self, X: jax.typing.ArrayLike,
                           U: jax.typing.ArrayLike):
        m = 2  # RK4 steps per interval
        dt = 0.005 / m

        def iteration(i, X):
            weights = jax.numpy.array([[0.0, 0.5, 0.5, 1.0],
                                       [1.0, 2.0, 2.0, 1.0]])

            def rk_iteration(i, val):
                kx_previous, weighted_sum = val
                kx = self.xdot(X + dt * weights[0, i] * kx_previous, U)
                weighted_sum += dt * weights[1, i] * kx / 6.0
                return (kx, weighted_sum)

            return jax.lax.fori_loop(lower=0,
                                     upper=4,
                                     body_fun=rk_iteration,
                                     init_val=(X, X))[1]

        return jax.lax.fori_loop(lower=0,
                                 upper=m,
                                 body_fun=iteration,
                                 init_val=X)

    def unwrap_angles(self, X: jax.typing.ArrayLike):
        return X

    @partial(jax.jit, static_argnums=[0])
    def xdot(self, X: jax.typing.ArrayLike, U: jax.typing.ArrayLike):
        raise NotImplemented("xdot not implemented")

    @partial(jax.jit, static_argnums=[0])
    def cost(self, X: jax.typing.ArrayLike, U: jax.typing.ArrayLike,
             goal: jax.typing.ArrayLike):
        raise NotImplemented("cost not implemented")

    @partial(jax.jit, static_argnums=[0])
    def random_states(self, rng: PRNGKey, dimensions=None):
        raise NotImplemented("random_states not implemented")

    #@partial(jax.jit, static_argnums=[0])
    def random_actions(self, rng: PRNGKey, dimensions=None):
        """Produces a uniformly random action in the action space."""
        return jax.random.uniform(
            rng,
            (dimensions or FLAGS.num_agents, self.num_outputs),
            minval=-self.action_limit,
            maxval=self.action_limit,
        )


class TurretProblem(Problem):

    def __init__(self):
        super().__init__(num_states=2,
                         num_unwrapped_states=2,
                         num_outputs=1,
                         num_goals=2,
                         action_limit=30.0)

    @partial(jax.jit, static_argnums=[0])
    def xdot(self, X: jax.typing.ArrayLike, U: jax.typing.ArrayLike):
        A = jax.numpy.array([[0., 1.], [0., -36.85154548]])
        B = jax.numpy.array([[0.], [56.08534375]])

        return A @ X + B @ U

    @partial(jax.jit, static_argnums=[0])
    def cost(self, X: jax.typing.ArrayLike, U: jax.typing.ArrayLike,
             goal: jax.typing.ArrayLike):
        Q = jax.numpy.array([[2.77777778, 0.], [0., 0.01]])
        R = jax.numpy.array([[0.00694444]])

        return X.T @ Q @ X + U.T @ R @ U

    #@partial(jax.jit, static_argnums=[0])
    def random_states(self, rng: PRNGKey, dimensions=None):
        rng1, rng2 = jax.random.split(rng)

        return jax.numpy.hstack(
            (jax.random.uniform(rng1, (dimensions or FLAGS.num_agents, 1),
                                minval=-1.0,
                                maxval=1.0),
             jax.random.uniform(rng2, (dimensions or FLAGS.num_agents, 1),
                                minval=-10.0,
                                maxval=10.0)))
