import jax, numpy
from functools import partial
from absl import logging
from frc971.control_loops.swerve import jax_dynamics
from frc971.control_loops.python import controls
from flax.typing import PRNGKey


class Problem(object):

    def __init__(self, num_states: int, num_unwrapped_states: int,
                 num_outputs: int, num_goals: int, action_limit: float):
        self.num_states = num_states
        self.num_unwrapped_states = num_unwrapped_states
        self.num_outputs = num_outputs
        self.num_goals = num_goals
        self.action_limit = action_limit
        self.dt = 0.005

    def integrate_dynamics(self, X: jax.typing.ArrayLike,
                           U: jax.typing.ArrayLike):
        m = 2  # RK4 steps per interval
        dt = self.dt / m

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

    def xdot(self, X: jax.typing.ArrayLike, U: jax.typing.ArrayLike):
        raise NotImplemented("xdot not implemented")

    def cost(self, X: jax.typing.ArrayLike, U: jax.typing.ArrayLike,
             goal: jax.typing.ArrayLike):
        raise NotImplemented("cost not implemented")

    def random_states(self, rng: PRNGKey, dimensions=None):
        raise NotImplemented("random_states not implemented")

    def random_actions(self,
                       rng: PRNGKey,
                       X: jax.typing.ArrayLike,
                       goal: jax.typing.ArrayLike,
                       dimensions=None):
        """Produces a uniformly random action in the action space."""
        return jax.random.uniform(
            rng,
            (dimensions or FLAGS.num_agents, self.num_outputs),
            minval=-1.0,
            maxval=1.0,
        )

    def random_goals(self, rng: PRNGKey, dimensions=None):
        """Produces a random goal in the goal space."""
        raise NotImplemented("random_states not implemented")


class TurretProblem(Problem):

    def __init__(self):
        super().__init__(num_states=2,
                         num_unwrapped_states=2,
                         num_outputs=1,
                         num_goals=2,
                         action_limit=30.0)
        self.A = numpy.matrix([[1., 0.00456639], [0., 0.83172142]])
        self.B = numpy.matrix([[0.00065992], [0.25610763]])

        self.Q = numpy.matrix([[2.77777778, 0.], [0., 0.01]])
        self.R = numpy.matrix([[0.00694444]])

        # Compute the optimal LQR cost + controller.
        self.F, self.P = controls.dlqr(self.A,
                                       self.B,
                                       self.Q,
                                       self.R,
                                       optimal_cost_function=True)

    def xdot(self, X: jax.typing.ArrayLike, U: jax.typing.ArrayLike):
        A_continuous = jax.numpy.array([[0., 1.], [0., -36.85154548]])
        B_continuous = jax.numpy.array([[0.], [56.08534375]])

        U = U * self.action_limit
        return A_continuous @ X + B_continuous @ U

    def reward(self, X: jax.typing.ArrayLike, U: jax.typing.ArrayLike,
               goal: jax.typing.ArrayLike):
        return -(X - goal).T @ jax.numpy.array(
            self.Q) @ (X - goal) - U.T @ jax.numpy.array(self.R) @ U

    def random_states(self, rng: PRNGKey, dimensions=None):
        rng1, rng2 = jax.random.split(rng)

        return jax.numpy.hstack(
            (jax.random.uniform(rng1, (dimensions or FLAGS.num_agents, 1),
                                minval=-1.0,
                                maxval=1.0),
             jax.random.uniform(rng2, (dimensions or FLAGS.num_agents, 1),
                                minval=-10.0,
                                maxval=10.0)))

    def random_goals(self, rng: PRNGKey, dimensions=None):
        """Produces a random goal in the goal space."""
        return jax.numpy.hstack((
            jax.random.uniform(rng, (dimensions or FLAGS.num_agents, 1),
                               minval=-0.1,
                               maxval=0.1),
            jax.numpy.zeros((dimensions or FLAGS.num_agents, 1)),
        ))
