#!/usr/bin/env python3

from functools import partial
from collections import namedtuple
import jax

from frc971.control_loops.swerve import dynamics
from frc971.control_loops.python.control_loop import KrakenFOC

# Note: this physics needs to match the symengine code.  We have tests that
# confirm they match in all the cases we care about.

CoefficientsType = namedtuple('CoefficientsType', [
    'Cx',
    'Cy',
    'rw',
    'm',
    'J',
    'Gd1',
    'rs',
    'rp',
    'Gd2',
    'rb1',
    'rb2',
    'Gd3',
    'Gd',
    'Js',
    'Gs',
    'wb',
    'Jdm',
    'Jsm',
    'Kts',
    'Ktd',
    'robot_width',
    'caster',
    'contact_patch_length',
])


def Coefficients(
    Cx: float = 25.0 * 9.8 / 4.0 / 0.05,
    Cy: float = 5 * 9.8 / 0.05 / 4.0,
    rw: float = 2 * 0.0254,

    # base is 20 kg without battery
    m: float = 25.0,
    J: float = 6.0,
    Gd1: float = 12.0 / 42.0,
    rs: float = 28.0 / 20.0 / 2.0,
    rp: float = 18.0 / 20.0 / 2.0,

    # 15 / 45 bevel ratio, calculated using python script ported over to
    # GetBevelPitchRadius(double)
    # TODO(Justin): Use the function instead of computed constantss
    rb1: float = 0.3805473,
    rb2: float = 1.14164,
    Js: float = 0.001,
    Gs: float = 35.0 / 468.0,
    wb: float = 0.725,
    drive_motor=KrakenFOC(),
    steer_motor=KrakenFOC(),
    robot_width: float = 24.75 * 0.0254,
    caster: float = 0.01,
    contact_patch_length: float = 0.02,
) -> CoefficientsType:

    Gd2 = rs / rp
    Gd3 = rb1 / rb2
    Gd = Gd1 * Gd2 * Gd3

    Jdm = drive_motor.motor_inertia
    Jsm = steer_motor.motor_inertia
    Kts = steer_motor.Kt
    Ktd = drive_motor.Kt

    return CoefficientsType(
        Cx=Cx,
        Cy=Cy,
        rw=rw,
        m=m,
        J=J,
        Gd1=Gd1,
        rs=rs,
        rp=rp,
        Gd2=Gd2,
        rb1=rb1,
        rb2=rb2,
        Gd3=Gd3,
        Gd=Gd,
        Js=Js,
        Gs=Gs,
        wb=wb,
        Jdm=Jdm,
        Jsm=Jsm,
        Kts=Kts,
        Ktd=Ktd,
        robot_width=robot_width,
        caster=caster,
        contact_patch_length=contact_patch_length,
    )


def R(theta):
    stheta = jax.numpy.sin(theta)
    ctheta = jax.numpy.cos(theta)
    return jax.numpy.array([[ctheta, -stheta], [stheta, ctheta]])


def angle_cross(vector, omega):
    return jax.numpy.array([-vector[1] * omega, vector[0] * omega])


def force_cross(r, f):
    return r[0] * f[1] - r[1] * f[0]


def softsign(x, gain):
    return -2 / (1 + jax.numpy.exp(gain * x)) + 1


def soft_atan2(y, x):
    kMaxLogGain = 1.0 / 0.05
    kAbsLogGain = 1.0 / 0.01

    softabs_x = x * (1.0 - 2.0 / (1 + jax.numpy.exp(kAbsLogGain * x)))

    return jax.numpy.arctan2(
        y,
        jax.scipy.special.logsumexp(
            jax.numpy.array([1.0, softabs_x * kMaxLogGain])) / kMaxLogGain)


def full_module_physics(coefficients: dict, Rtheta, module_index: int,
                        mounting_location, X, U):
    X_module = X[module_index * 4:(module_index + 1) * 4]
    Is = U[2 * module_index + 0]
    Id = U[2 * module_index + 1]

    Rthetaplusthetas = R(X[dynamics.STATE_THETA] +
                         X_module[dynamics.STATE_THETAS0])

    caster_vector = jax.numpy.array([-coefficients.caster, 0.0])

    robot_velocity = X[dynamics.STATE_VX:dynamics.STATE_VY + 1]

    contact_patch_velocity = (
        angle_cross(Rtheta @ mounting_location, X[dynamics.STATE_OMEGA]) +
        robot_velocity + angle_cross(
            Rthetaplusthetas @ caster_vector,
            (X[dynamics.STATE_OMEGA] + X_module[dynamics.STATE_OMEGAS0])))

    wheel_ground_velocity = Rthetaplusthetas.T @ contact_patch_velocity

    wheel_velocity = jax.numpy.array(
        [coefficients.rw * X_module[dynamics.STATE_OMEGAD0], 0.0])

    wheel_slip_velocity = wheel_velocity - wheel_ground_velocity

    slip_angle = jax.numpy.sin(
        -soft_atan2(wheel_ground_velocity[1], wheel_ground_velocity[0]))

    slip_ratio = (coefficients.rw * X_module[dynamics.STATE_OMEGAD0] -
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

    torque = force_cross(mounting_location, F)

    X_dot_contribution = jax.numpy.hstack((jax.numpy.zeros(
        (4, )), ) * (module_index) + (jax.numpy.array([
            X_module[dynamics.STATE_OMEGAS0],
            X_module[dynamics.STATE_OMEGAD0],
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
    Rtheta = R(X[dynamics.STATE_THETA])

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

    X_dot = X_dot.at[dynamics.STATE_X:dynamics.STATE_THETA + 1].set(
        jax.numpy.array([
            X[dynamics.STATE_VX],
            X[dynamics.STATE_VY],
            X[dynamics.STATE_OMEGA],
        ]))

    return X_dot


def velocity_module_physics(coefficients: dict, Rtheta, module_index: int,
                            mounting_location, X, U):
    X_module = X[module_index * 2:(module_index + 1) * 2]
    Is = U[2 * module_index + 0]
    Id = U[2 * module_index + 1]

    Rthetaplusthetas = R(X[dynamics.VELOCITY_STATE_THETA] +
                         X_module[dynamics.VELOCITY_STATE_THETAS0])

    caster_vector = jax.numpy.array([-coefficients.caster, 0.0])

    robot_velocity = X[dynamics.VELOCITY_STATE_VX:dynamics.VELOCITY_STATE_VY +
                       1]

    contact_patch_velocity = (
        angle_cross(Rtheta @ mounting_location,
                    X[dynamics.VELOCITY_STATE_OMEGA]) + robot_velocity +
        angle_cross(Rthetaplusthetas @ caster_vector,
                    (X[dynamics.VELOCITY_STATE_OMEGA] +
                     X_module[dynamics.VELOCITY_STATE_OMEGAS0])))

    wheel_ground_velocity = Rthetaplusthetas.T @ contact_patch_velocity

    slip_angle = jax.numpy.sin(
        -soft_atan2(wheel_ground_velocity[1], wheel_ground_velocity[0]))

    Fwx = (coefficients.Ktd / (coefficients.Gd * coefficients.rw)) * Id
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

    F = Rthetaplusthetas @ jax.numpy.array([Fwx, Fwy])

    torque = force_cross(mounting_location, F)

    X_dot_contribution = jax.numpy.hstack((jax.numpy.zeros(
        (2, )), ) * (module_index) + (jax.numpy.array([
            X_module[dynamics.VELOCITY_STATE_OMEGAS0],
            alphas,
        ]), ) + (jax.numpy.zeros((2, )), ) * (3 - module_index) + (
            jax.numpy.zeros((1, )),
            F / coefficients.m,
            jax.numpy.array([torque / coefficients.J]),
        ))

    return X_dot_contribution


@partial(jax.jit, static_argnames=['coefficients'])
def velocity_dynamics(coefficients: CoefficientsType, X, U):
    Rtheta = R(X[dynamics.VELOCITY_STATE_THETA])

    module0 = velocity_module_physics(
        coefficients, Rtheta, 0,
        jax.numpy.array(
            [coefficients.robot_width / 2.0, coefficients.robot_width / 2.0]),
        X, U)
    module1 = velocity_module_physics(
        coefficients, Rtheta, 1,
        jax.numpy.array(
            [-coefficients.robot_width / 2.0, coefficients.robot_width / 2.0]),
        X, U)
    module2 = velocity_module_physics(
        coefficients, Rtheta, 2,
        jax.numpy.array(
            [-coefficients.robot_width / 2.0,
             -coefficients.robot_width / 2.0]), X, U)
    module3 = velocity_module_physics(
        coefficients, Rtheta, 3,
        jax.numpy.array(
            [coefficients.robot_width / 2.0, -coefficients.robot_width / 2.0]),
        X, U)

    X_dot = module0 + module1 + module2 + module3

    return X_dot.at[dynamics.VELOCITY_STATE_THETA].set(
        X[dynamics.VELOCITY_STATE_OMEGA])
