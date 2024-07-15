#!/usr/bin/python3

import numpy
import sys, os
import casadi
from numpy.testing import assert_array_equal, assert_array_almost_equal
import unittest

from frc971.control_loops.swerve import dynamics


def state_vector(velocity=numpy.array([[1.0], [0.0]]),
                 dx=0.0,
                 dy=0.0,
                 theta=0.0,
                 omega=0.0,
                 module_omega=0.0,
                 module_angle=0.0,
                 drive_wheel_velocity=None,
                 module_angles=None):
    """Returns the state vector with the requested state."""
    X_initial = numpy.zeros((25, 1))
    # All the wheels are spinning at the speed needed to hit the velocity in m/s
    drive_wheel_velocity = (drive_wheel_velocity
                            or numpy.linalg.norm(velocity))

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
        assert (len(module_angles) == 4)
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


class TestSwervePhysics(unittest.TestCase):
    I = numpy.zeros((8, 1))

    def setUp(self):
        self.swerve_physics = wrap(dynamics.swerve_physics)
        self.contact_patch_velocity = [
            wrap_module(dynamics.contact_patch_velocity, i) for i in range(4)
        ]
        self.wheel_ground_velocity = [
            wrap_module(dynamics.wheel_ground_velocity, i) for i in range(4)
        ]
        self.wheel_slip_velocity = [
            wrap_module(dynamics.wheel_slip_velocity, i) for i in range(4)
        ]
        self.wheel_force = [
            wrap_module(dynamics.wheel_force, i) for i in range(4)
        ]
        self.module_angular_accel = [
            wrap_module(dynamics.module_angular_accel, i) for i in range(4)
        ]
        self.F = [wrap_module(dynamics.F, i) for i in range(4)]
        self.mounting_location = [
            wrap_module(dynamics.mounting_location, i) for i in range(4)
        ]

        self.slip_angle = [
            wrap_module(dynamics.slip_angle, i) for i in range(4)
        ]
        self.slip_ratio = [
            wrap_module(dynamics.slip_ratio, i) for i in range(4)
        ]
        self.Ms = [wrap_module(dynamics.Ms, i) for i in range(4)]

    def test_contact_patch_velocity(self):
        """Tests that the contact patch velocity makes sense."""
        for i in range(4):
            contact_patch_velocity = wrap_module(
                dynamics.contact_patch_velocity, i)
            wheel_ground_velocity = wrap_module(dynamics.wheel_ground_velocity,
                                                i)

            # No angular velocity should result in just linear motion.
            for velocity in [
                    numpy.array([[1.5], [0.0]]),
                    numpy.array([[0.0], [1.0]]),
                    numpy.array([[-1.5], [0.0]]),
                    numpy.array([[0.0], [-1.0]]),
                    numpy.array([[2.0], [-1.7]]),
            ]:
                for theta in [0.0, 1.0, -1.0, 100.0]:
                    patch_velocity = contact_patch_velocity(
                        state_vector(velocity=velocity, theta=theta), self.I)

                    assert_array_equal(patch_velocity, velocity)

            # Now, test that spinning the robot results in module velocities.
            # We are assuming that each module is on a square robot.
            module_center_of_mass_angle = i * numpy.pi / 2.0 + numpy.pi / 4.0
            for theta in [-module_center_of_mass_angle, 0.0, 1.0, -1.0, 100.0]:
                for omega in [0.65, -0.1]:
                    # Point each module to the center to make the math easier.
                    patch_velocity = contact_patch_velocity(
                        state_vector(velocity=numpy.array([[0.0], [0.0]]),
                                     theta=theta,
                                     omega=omega,
                                     module_angle=module_center_of_mass_angle),
                        self.I)

                    assert_array_almost_equal(
                        patch_velocity,
                        (dynamics.ROBOT_WIDTH / numpy.sqrt(2.0) -
                         dynamics.CASTER) * omega * numpy.array([[
                             -numpy.sin(theta + module_center_of_mass_angle)
                         ], [numpy.cos(theta + module_center_of_mass_angle)]]))

                    # Point the wheel along +x, rotate it by theta, then spin it.
                    # Confirm the velocities come out right.
                    patch_velocity = contact_patch_velocity(
                        state_vector(
                            velocity=numpy.array([[0.0], [0.0]]),
                            theta=-module_center_of_mass_angle,
                            module_omega=omega,
                            module_angle=(theta +
                                          module_center_of_mass_angle)),
                        self.I)

                    assert_array_almost_equal(
                        patch_velocity, -dynamics.CASTER * omega *
                        numpy.array([[-numpy.sin(theta)], [numpy.cos(theta)]]))

            # Now, test that the rotation back to wheel coordinates works.
            # The easiest way to do this is to point the wheel in a direction,
            # move in that direction, and confirm that there is no lateral velocity.
            for robot_angle in [0.0, 1.0, -5.0]:
                for module_angle in [0.0, 1.0, -5.0]:
                    wheel_patch_velocity = numpy.array(
                        wheel_ground_velocity(
                            state_vector(velocity=numpy.array(
                                [[numpy.cos(robot_angle + module_angle)],
                                 [numpy.sin(robot_angle + module_angle)]]),
                                         theta=robot_angle,
                                         module_angle=module_angle), self.I))

                    assert_array_almost_equal(wheel_patch_velocity,
                                              numpy.array([[1], [0]]))

    def test_slip_angle(self):
        """Tests that the slip_angle calculation works."""
        velocity = numpy.array([[1.5], [0.0]])

        for i in range(4):
            x = casadi.SX.sym("x")
            y = casadi.SX.sym("y")
            half_atan2 = casadi.Function('half_atan2', [y, x],
                                         [dynamics.half_atan2(y, x)])

            for wrap in range(-3, 3):
                for theta in [0.0, 0.6, -0.4]:
                    module_angle = numpy.pi * wrap + theta

                    self.assertAlmostEqual(
                        theta,
                        half_atan2(numpy.sin(module_angle),
                                   numpy.cos(module_angle)))

                    computed_angle = self.slip_angle[i](state_vector(
                        velocity=velocity,
                        module_angle=numpy.pi * wrap + theta), self.I)[0, 0]

                    self.assertAlmostEqual(theta, computed_angle)

    def test_wheel_torque(self):
        """Tests that the per module self aligning forces have the right signs."""
        X = state_vector(module_angles=[-0.001, -0.001, 0.001, 0.001])
        xdot_equal = self.swerve_physics(X, self.I)

        self.assertGreater(xdot_equal[2, 0], 0.0)
        self.assertAlmostEqual(xdot_equal[3, 0], 0.0, places=1)
        self.assertGreater(self.Ms[0](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_equal[6, 0], 0.0)
        self.assertAlmostEqual(xdot_equal[7, 0], 0.0, places=1)
        self.assertGreater(self.Ms[1](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_equal[10, 0], 0.0)
        self.assertAlmostEqual(xdot_equal[11, 0], 0.0, places=1)
        self.assertLess(self.Ms[2](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_equal[14, 0], 0.0)
        self.assertAlmostEqual(xdot_equal[15, 0], 0.0, places=1)
        self.assertLess(self.Ms[3](X, self.I)[0, 0], 0.0)

        # Shouldn't be spinning.
        self.assertAlmostEqual(xdot_equal[21, 0], 0.0, places=2)

        # Now, make the bot want to go left.
        # The wheels will be going too fast based on our calcs, so they should be decelerating.
        X = state_vector(module_angles=[0.01, 0.01, 0.01, 0.01])
        xdot_left = self.swerve_physics(X, self.I)

        self.assertLess(xdot_left[2, 0], -0.1)
        self.assertLess(xdot_left[3, 0], 0.0)
        self.assertLess(self.Ms[0](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_left[6, 0], -0.1)
        self.assertLess(xdot_left[7, 0], 0.0)
        self.assertLess(self.Ms[1](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_left[10, 0], -0.1)
        self.assertLess(xdot_left[11, 0], 0.0)
        self.assertLess(self.Ms[2](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_left[14, 0], -0.1)
        self.assertLess(xdot_left[15, 0], 0.0)
        self.assertLess(self.Ms[3](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_left[19, 0], 0.0001)
        self.assertGreater(xdot_left[20, 0], 0.1)
        # Shouldn't be spinning.
        self.assertAlmostEqual(xdot_left[21, 0], 0.0)

        # And now do it to the right too.
        X = state_vector(module_angles=[-0.01, -0.01, -0.01, -0.01])
        xdot_right = self.swerve_physics(X, self.I)

        self.assertGreater(xdot_right[2, 0], 0.1)
        self.assertLess(xdot_right[3, 0], 0.0)
        self.assertGreater(self.Ms[0](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_right[6, 0], 0.1)
        self.assertLess(xdot_right[7, 0], 0.0)
        self.assertGreater(self.Ms[1](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_right[10, 0], 0.1)
        self.assertLess(xdot_right[11, 0], 0.0)
        self.assertGreater(self.Ms[2](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_right[14, 0], 0.1)
        self.assertLess(xdot_right[15, 0], 0.0)
        self.assertGreater(self.Ms[3](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_right[19, 0], 0.0001)
        self.assertLess(xdot_right[20, 0], -0.1)
        # Shouldn't be spinning.
        self.assertAlmostEqual(xdot_right[21, 0], 0.0)

    def test_wheel_forces(self):
        """Tests that the per module forces have the right signs."""
        for i in range(4):
            wheel_force = wrap_module(dynamics.wheel_force, i)

            X = state_vector()
            robot_equal = wheel_force(X, self.I)
            xdot_equal = self.swerve_physics(X, self.I)
            self.assertEqual(robot_equal[0, 0], 0.0)
            self.assertEqual(robot_equal[1, 0], 0.0)
            self.assertEqual(xdot_equal[2 + 4 * i], 0.0)
            self.assertEqual(xdot_equal[3 + 4 * i], 0.0)

            # Robot is moving faster than the wheels, it should decelerate.
            X = state_vector(dx=0.01)
            robot_faster = wheel_force(X, self.I)
            xdot_faster = self.swerve_physics(X, self.I)
            self.assertLess(robot_faster[0, 0], -0.1)
            self.assertEqual(robot_faster[1, 0], 0.0)
            self.assertGreater(xdot_faster[3 + 4 * i], 0.0)

            # Robot is now going slower than the wheels.  It should accelerate.
            X = state_vector(dx=-0.01)
            robot_slower = wheel_force(X, self.I)
            xdot_slower = self.swerve_physics(X, self.I)
            self.assertGreater(robot_slower[0, 0], 0.1)
            self.assertEqual(robot_slower[1, 0], 0.0)
            self.assertLess(xdot_slower[3 + 4 * i], 0.0)

            # Positive lateral velocity -> negative force.
            robot_left = wheel_force(state_vector(dy=0.01), self.I)
            self.assertEqual(robot_left[0, 0], 0.0)
            self.assertLess(robot_left[1, 0], -0.1)

            # Negative lateral velocity -> positive force.
            robot_right = wheel_force(state_vector(dy=-0.01), self.I)
            self.assertEqual(robot_right[0, 0], 0.0)
            self.assertGreater(robot_right[1, 0], 0.1)


if __name__ == '__main__':
    unittest.main()
