import jax
from functools import partial
from frc971.control_loops.swerve import dynamics
from absl import logging
from frc971.control_loops.swerve import jax_dynamics


@partial(jax.jit, static_argnums=[0])
def xdot_physics(physics_constants: jax_dynamics.CoefficientsType, X, U):
    A = jax.numpy.array([[0., 1.], [0., -36.85154548]])
    B = jax.numpy.array([[0.], [56.08534375]])

    return A @ X + B @ U


def unwrap_angles(X):
    return X


@jax.jit
def swerve_cost(X: jax.typing.ArrayLike, U: jax.typing.ArrayLike,
                goal: jax.typing.ArrayLike):

    Q = jax.numpy.array([[2.77777778, 0.], [0., 0.01]])
    R = jax.numpy.array([[0.00694444]])

    return (X).T @ Q @ (X) + U.T @ R @ U


@partial(jax.jit, static_argnums=[0])
def integrate_dynamics(physics_constants: jax_dynamics.CoefficientsType, X, U):
    m = 2  # RK4 steps per interval
    dt = 0.005 / m

    def iteration(i, X):
        weights = jax.numpy.array([[0.0, 0.5, 0.5, 1.0], [1.0, 2.0, 2.0, 1.0]])

        def rk_iteration(i, val):
            kx_previous, weighted_sum = val
            kx = xdot_physics(physics_constants,
                              X + dt * weights[0, i] * kx_previous, U)
            weighted_sum += dt * weights[1, i] * kx / 6.0
            return (kx, weighted_sum)

        return jax.lax.fori_loop(lower=0,
                                 upper=4,
                                 body_fun=rk_iteration,
                                 init_val=(X, X))[1]

    return jax.lax.fori_loop(lower=0, upper=m, body_fun=iteration, init_val=X)


state_cost = swerve_cost
NUM_STATES = 2
NUM_UNWRAPPED_STATES = 2
NUM_OUTPUTS = 1
ACTION_LIMIT = 10.0


def random_states(rng, dimensions=None):
    rng1, rng2 = jax.random.split(rng)

    return jax.numpy.hstack(
        (jax.random.uniform(rng1, (dimensions or FLAGS.num_agents, 1),
                            minval=-1.0,
                            maxval=1.0),
         jax.random.uniform(rng2, (dimensions or FLAGS.num_agents, 1),
                            minval=-10.0,
                            maxval=10.0)))
