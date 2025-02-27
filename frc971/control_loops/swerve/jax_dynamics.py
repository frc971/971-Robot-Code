#!/usr/bin/env python3

from functools import partial
from collections import namedtuple
import jax

from frc971.control_loops.swerve.dynamics_constants import *

# Note: this physics needs to match the symengine code.  We have tests that
# confirm they match in all the cases we care about.

CoefficientsType = namedtuple('CoefficientsType', JAX_CONSTANTS.keys())


def Coefficients() -> CoefficientsType:
    return CoefficientsType(**JAX_CONSTANTS)


def R(theta):
    stheta = jax.numpy.sin(theta)
    ctheta = jax.numpy.cos(theta)
    return jax.numpy.array([[ctheta, -stheta], [stheta, ctheta]])


def angle_cross(vector, omega):
    return jax.numpy.array([-vector[1] * omega, vector[0] * omega])


def force_cross(r, f):
    return r[0] * f[1] - r[1] * f[0]


def softsign(x, gain):
    return 1 - 2.0 * jax.nn.sigmoid(-gain * x)


def soft_atan2(y, x):
    kMaxLogGain = 1.0 / 0.05
    kAbsLogGain = 1.0 / 0.01

    softabs_x = x * softsign(x, kAbsLogGain)

    return jax.numpy.arctan2(
        y,
        jax.scipy.special.logsumexp(
            jax.numpy.array([1.0, softabs_x * kMaxLogGain])) / kMaxLogGain)


def full_module_physics(coefficients: CoefficientsType, Rtheta,
                        module_index: int, mounting_location, X, U):
    X_module = X[module_index * 4:(module_index + 1) * 4]
    Is = U[2 * module_index + 0]
    Id = U[2 * module_index + 1]

    Rthetaplusthetas = R(X[STATE_THETA] + X_module[STATE_THETAS0])

    caster_vector = jax.numpy.array([-coefficients.caster, 0.0])

    robot_velocity = X[STATE_VX:STATE_VY + 1]

    contact_patch_velocity = (
        angle_cross(Rtheta @ mounting_location, X[STATE_OMEGA]) +
        robot_velocity +
        angle_cross(Rthetaplusthetas @ caster_vector,
                    (X[STATE_OMEGA] + X_module[STATE_OMEGAS0])))

    wheel_ground_velocity = Rthetaplusthetas.T @ contact_patch_velocity

    wheel_velocity = jax.numpy.array(
        [coefficients.rw * X_module[STATE_OMEGAD0], 0.0])

    wheel_slip_velocity = wheel_velocity - wheel_ground_velocity

    slip_angle = jax.numpy.sin(
        -soft_atan2(wheel_ground_velocity[1], wheel_ground_velocity[0]))

    slip_ratio = (coefficients.rw * X_module[STATE_OMEGAD0] -
                  wheel_ground_velocity[0]) / jax.numpy.max(
                      jax.numpy.array(
                          [0.02, jax.numpy.abs(wheel_ground_velocity[0])]))

    Fwx = coefficients.Cx * slip_ratio
    Fwy = coefficients.Cy * slip_angle

    softsign_velocity = softsign(wheel_ground_velocity[0], 100)

    Ms = -Fwy * (
        (softsign_velocity * coefficients.contact_patch_length / 3.0) +
        coefficients.caster)

    alphas = (Ms + coefficients.Kts * Is / coefficients.Gs +
              (-coefficients.wb + (coefficients.rs + coefficients.rp) *
               (1 - coefficients.rb1 / coefficients.rp)) *
              (coefficients.rw / coefficients.rb2 *
               (-Fwx))) / (coefficients.Jsm +
                           (coefficients.Js /
                            (coefficients.Gs * coefficients.Gs)))

    # Then solve for alphad
    alphad = (coefficients.rs * coefficients.Jdm * coefficients.Gd3 * alphas +
              coefficients.rp * coefficients.Ktd * Id * coefficients.Gd -
              coefficients.rw * coefficients.rp * coefficients.Gd * Fwx *
              coefficients.Gd) / (coefficients.rp * coefficients.Jdm)

    F = Rthetaplusthetas @ jax.numpy.array([Fwx, Fwy])

    torque = force_cross(Rtheta @ mounting_location, F)

    X_dot_contribution = jax.numpy.hstack((jax.numpy.zeros(
        (4, )), ) * (module_index) + (jax.numpy.array([
            X_module[STATE_OMEGAS0],
            X_module[STATE_OMEGAD0],
            alphas,
            alphad,
        ]), ) + (jax.numpy.zeros((4, )), ) * (3 - module_index) + (
            jax.numpy.zeros((3, )),
            F / coefficients.m,
            jax.numpy.array([torque / coefficients.J, 0, 0, 0]),
        ))

    return X_dot_contribution


@partial(jax.jit, static_argnames=['coefficients'])
def full_dynamics(coefficients: CoefficientsType, X, U):
    Rtheta = R(X[STATE_THETA])

    module0 = full_module_physics(
        coefficients, Rtheta, 0,
        jax.numpy.array(
            [coefficients.robot_width / 2.0, coefficients.robot_width / 2.0]),
        X, U)
    module1 = full_module_physics(
        coefficients, Rtheta, 1,
        jax.numpy.array(
            [-coefficients.robot_width / 2.0, coefficients.robot_width / 2.0]),
        X, U)
    module2 = full_module_physics(
        coefficients, Rtheta, 2,
        jax.numpy.array(
            [-coefficients.robot_width / 2.0,
             -coefficients.robot_width / 2.0]), X, U)
    module3 = full_module_physics(
        coefficients, Rtheta, 3,
        jax.numpy.array(
            [coefficients.robot_width / 2.0, -coefficients.robot_width / 2.0]),
        X, U)

    X_dot = module0 + module1 + module2 + module3

    X_dot = X_dot.at[STATE_VX].add(X[STATE_FX] / coefficients.m)
    X_dot = X_dot.at[STATE_VY].add(X[STATE_FY] / coefficients.m)
    X_dot = X_dot.at[STATE_OMEGA].add(X[STATE_MOMENT] / coefficients.J)

    X_dot = X_dot.at[STATE_X:STATE_THETA + 1].set(
        jax.numpy.array([
            X[STATE_VX],
            X[STATE_VY],
            X[STATE_OMEGA],
        ]))

    return X_dot


def velocity_module_physics(coefficients: CoefficientsType,
                            Rtheta: jax.typing.ArrayLike, module_index: int,
                            mounting_location: jax.typing.ArrayLike,
                            X: jax.typing.ArrayLike, U: jax.typing.ArrayLike):
    X_module = X[module_index * 2:(module_index + 1) * 2]
    Is = U[2 * module_index + 0]
    Id = U[2 * module_index + 1]

    rotated_mounting_location = Rtheta @ mounting_location

    Rthetaplusthetas = R(X[VELOCITY_STATE_THETA] +
                         X_module[VELOCITY_STATE_THETAS0])

    caster_vector = jax.numpy.array([-coefficients.caster, 0.0])

    robot_velocity = X[VELOCITY_STATE_VX:VELOCITY_STATE_VY + 1]

    contact_patch_velocity = (
        angle_cross(rotated_mounting_location, X[VELOCITY_STATE_OMEGA]) +
        robot_velocity + angle_cross(
            Rthetaplusthetas @ caster_vector,
            (X[VELOCITY_STATE_OMEGA] + X_module[VELOCITY_STATE_OMEGAS0])))

    # Velocity of the contact patch over the field projected into the direction
    # of the wheel.
    wheel_ground_velocity = Rthetaplusthetas.T @ contact_patch_velocity

    slip_angle = jax.numpy.sin(
        -soft_atan2(wheel_ground_velocity[1], wheel_ground_velocity[0]))

    Fwx = (coefficients.Ktd / (coefficients.Gd * coefficients.rw)) * Id
    Fwy = coefficients.Cy * slip_angle

    softsign_velocity = softsign(wheel_ground_velocity[0], 100.0)

    Ms = -Fwy * (
        (softsign_velocity * coefficients.contact_patch_length / 3.0) +
        coefficients.caster)

    alphas = (Ms + coefficients.Kts * Is / coefficients.Gs +
              (-coefficients.wb + (coefficients.rs + coefficients.rp) *
               (1 - coefficients.rb1 / coefficients.rp)) *
              (coefficients.rw / coefficients.rb2 *
               (-Fwx))) / (coefficients.Jsm +
                           (coefficients.Js /
                            (coefficients.Gs * coefficients.Gs)))

    F = Rthetaplusthetas @ jax.numpy.array([Fwx, Fwy])

    torque = force_cross(rotated_mounting_location, F)

    X_dot_contribution = jax.numpy.hstack((jax.numpy.zeros(
        (2, )), ) * (module_index) + (jax.numpy.array([
            X_module[VELOCITY_STATE_OMEGAS0],
            alphas,
        ]), ) + (jax.numpy.zeros((2, )), ) * (3 - module_index) + (
            jax.numpy.zeros((1, )),
            F / coefficients.m,
            jax.numpy.array([torque / coefficients.J]),
        ))

    return X_dot_contribution, F, torque


@partial(jax.jit, static_argnames=['coefficients'])
def velocity_dynamics(coefficients: CoefficientsType, X: jax.typing.ArrayLike,
                      U: jax.typing.ArrayLike):
    Rtheta = R(X[VELOCITY_STATE_THETA])

    module0, _, _ = velocity_module_physics(
        coefficients, Rtheta, 0,
        jax.numpy.array(
            [coefficients.robot_width / 2.0, coefficients.robot_width / 2.0]),
        X, U)
    module1, _, _ = velocity_module_physics(
        coefficients, Rtheta, 1,
        jax.numpy.array(
            [-coefficients.robot_width / 2.0, coefficients.robot_width / 2.0]),
        X, U)
    module2, _, _ = velocity_module_physics(
        coefficients, Rtheta, 2,
        jax.numpy.array(
            [-coefficients.robot_width / 2.0,
             -coefficients.robot_width / 2.0]), X, U)
    module3, _, _ = velocity_module_physics(
        coefficients, Rtheta, 3,
        jax.numpy.array(
            [coefficients.robot_width / 2.0, -coefficients.robot_width / 2.0]),
        X, U)

    X_dot = module0 + module1 + module2 + module3

    return X_dot.at[VELOCITY_STATE_THETA].set(X[VELOCITY_STATE_OMEGA])


def to_velocity_state(X):
    return jax.numpy.array([
        X[STATE_THETAS0],
        X[STATE_OMEGAS0],
        X[STATE_THETAS1],
        X[STATE_OMEGAS1],
        X[STATE_THETAS2],
        X[STATE_OMEGAS2],
        X[STATE_THETAS3],
        X[STATE_OMEGAS3],
        X[STATE_THETA],
        X[STATE_VX],
        X[STATE_VY],
        X[STATE_OMEGA],
    ])


@jax.jit
def mpc_cost(coefficients: CoefficientsType, X, U, goal):
    J = 0

    rnorm = jax.numpy.linalg.norm(goal[0:2])

    vnorm = jax.lax.select(rnorm > 0.0001, goal[0:2] / rnorm,
                           jax.numpy.array([1.0, 0.0]))
    vperp = jax.lax.select(rnorm > 0.0001,
                           jax.numpy.array([-vnorm[1], vnorm[0]]),
                           jax.numpy.array([0.0, 1.0]))

    velocity_error = goal[0:2] - X[VELOCITY_STATE_VX:VELOCITY_STATE_VY + 1]

    # TODO(austin): Do we want to do something more special for 0?

    J += 75 * (jax.numpy.dot(velocity_error, vnorm)**2.0)
    J += 1500 * (jax.numpy.dot(velocity_error, vperp)**2.0)
    J += 1000 * (goal[2] - X[VELOCITY_STATE_OMEGA])**2.0

    kSteerVelocityGain = 0.10
    J += kSteerVelocityGain * (X[VELOCITY_STATE_OMEGAS0])**2.0
    J += kSteerVelocityGain * (X[VELOCITY_STATE_OMEGAS1])**2.0
    J += kSteerVelocityGain * (X[VELOCITY_STATE_OMEGAS2])**2.0
    J += kSteerVelocityGain * (X[VELOCITY_STATE_OMEGAS3])**2.0

    mounting_locations = jax.numpy.array(
        [[coefficients.robot_width / 2.0, coefficients.robot_width / 2.0],
         [-coefficients.robot_width / 2.0, coefficients.robot_width / 2.0],
         [-coefficients.robot_width / 2.0, -coefficients.robot_width / 2.0],
         [coefficients.robot_width / 2.0, -coefficients.robot_width / 2.0]])

    Rtheta = R(X[VELOCITY_STATE_THETA])
    _, F0, torque0 = velocity_module_physics(coefficients, Rtheta, 0,
                                             mounting_locations[0], X, U)
    _, F1, torque1 = velocity_module_physics(coefficients, Rtheta, 1,
                                             mounting_locations[1], X, U)
    _, F2, torque2 = velocity_module_physics(coefficients, Rtheta, 2,
                                             mounting_locations[2], X, U)
    _, F3, torque3 = velocity_module_physics(coefficients, Rtheta, 3,
                                             mounting_locations[3], X, U)

    forces = [F0, F1, F2, F3]

    F = (F0 + F1 + F2 + F3)
    torque = (torque0 + torque1 + torque2 + torque3)

    def force_cross(torque, r):
        r_squared_norm = jax.numpy.inner(r, r)

        return jax.numpy.array(
            [-r[1] * torque / r_squared_norm, r[0] * torque / r_squared_norm])

    # TODO(austin): Are these penalties reasonable?  Do they give us a decent time constant?
    for i in range(4):
        desired_force = F / 4.0 + force_cross(
            torque / 4.0, Rtheta @ mounting_locations[i, :])
        force_error = desired_force - forces[i]
        J += 0.01 * jax.numpy.inner(force_error, force_error)

    for i in range(4):
        Is = U[2 * i + 0]
        Id = U[2 * i + 1]
        # Steer
        J += ((Is + STEER_CURRENT_COUPLING_FACTOR * Id)**2.0) / 100000.0
        # Drive
        J += (Id**2.0) / 1000.0

    return J
