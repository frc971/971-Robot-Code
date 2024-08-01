#!/usr/bin/python3

import casadi, numpy

from frc971.control_loops.swerve import dynamics


def state_vector(
    velocity=numpy.array([[1.0], [0.0]]),
    dx=0.0,
    dy=0.0,
    theta=0.0,
    omega=0.0,
    module_omega=0.0,
    module_angle=0.0,
    drive_wheel_velocity=None,
    module_angles=None,
):
    """Returns the state vector with the requested state."""
    X_initial = numpy.zeros((25, 1))
    # All the wheels are spinning at the speed needed to hit the velocity in m/s
    drive_wheel_velocity = drive_wheel_velocity or numpy.linalg.norm(velocity)

    X_initial[2, 0] = module_omega
    X_initial[3, 0] = drive_wheel_velocity / (dynamics.WHEEL_RADIUS)

    X_initial[6, 0] = module_omega
    X_initial[7, 0] = drive_wheel_velocity / (dynamics.WHEEL_RADIUS)

    X_initial[10, 0] = module_omega
    X_initial[11, 0] = drive_wheel_velocity / (dynamics.WHEEL_RADIUS)

    X_initial[14, 0] = module_omega
    X_initial[15, 0] = drive_wheel_velocity / (dynamics.WHEEL_RADIUS)

    X_initial[0, 0] = module_angle
    X_initial[4, 0] = module_angle
    X_initial[8, 0] = module_angle
    X_initial[12, 0] = module_angle

    if module_angles is not None:
        assert len(module_angles) == 4
        X_initial[0, 0] = module_angles[0]
        X_initial[4, 0] = module_angles[1]
        X_initial[8, 0] = module_angles[2]
        X_initial[12, 0] = module_angles[3]

    X_initial[18, 0] = theta

    X_initial[19, 0] = velocity[0, 0] + dx
    X_initial[20, 0] = velocity[1, 0] + dy
    X_initial[21, 0] = omega

    return X_initial


def wrap(fn):
    evaluated_fn = fn(casadi.SX.sym("X", 25, 1), casadi.SX.sym("U", 8, 1))
    return lambda X, U: numpy.array(evaluated_fn(X, U))


def wrap_module(fn, i):
    evaluated_fn = fn(i, casadi.SX.sym("X", 25, 1), casadi.SX.sym("U", 8, 1))
    return lambda X, U: numpy.array(evaluated_fn(X, U))
