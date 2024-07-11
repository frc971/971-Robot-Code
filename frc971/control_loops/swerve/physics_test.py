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
                 module_angle=0.0):
    """Returns the state vector with the requested state."""
    X_initial = numpy.zeros((25, 1))
    # All the wheels are spinning at the speed needed to hit the velocity in m/s
    X_initial[2, 0] = module_omega
    X_initial[3, 0] = numpy.linalg.norm(velocity) / (dynamics.WHEEL_RADIUS)

    X_initial[6, 0] = module_omega
    X_initial[7, 0] = numpy.linalg.norm(velocity) / (dynamics.WHEEL_RADIUS)

    X_initial[10, 0] = module_omega
    X_initial[11, 0] = numpy.linalg.norm(velocity) / (dynamics.WHEEL_RADIUS)

    X_initial[14, 0] = module_omega
    X_initial[15, 0] = numpy.linalg.norm(velocity) / (dynamics.WHEEL_RADIUS)

    X_initial[0, 0] = module_angle
    X_initial[4, 0] = module_angle
    X_initial[8, 0] = module_angle
    X_initial[12, 0] = module_angle

    X_initial[18, 0] = theta

    X_initial[19, 0] = velocity[0, 0] + dx
    X_initial[20, 0] = velocity[1, 0] + dy
    X_initial[21, 0] = omega

    return X_initial


class TestHPolytope(unittest.TestCase):
    I = numpy.zeros((8, 1))

    def setUp(self):
        pass

    def test_contact_patch_velocity(self):
        """Tests that the contact patch velocity makes sense."""
        for i in range(4):
            contact_patch_velocity = dynamics.contact_patch_velocity(
                i, casadi.SX.sym("X", 25, 1), casadi.SX.sym("U", 8, 1))
            wheel_ground_velocity = dynamics.wheel_ground_velocity(
                i, casadi.SX.sym("X", 25, 1), casadi.SX.sym("U", 8, 1))

            # No angular velocity should result in just linear motion.
            for velocity in [
                    numpy.array([[1.5], [0.0]]),
                    numpy.array([[0.0], [1.0]]),
                    numpy.array([[-1.5], [0.0]]),
                    numpy.array([[0.0], [-1.0]]),
                    numpy.array([[2.0], [-1.7]]),
            ]:
                for theta in [0.0, 1.0, -1.0, 100.0]:
                    patch_velocity = numpy.array(
                        contact_patch_velocity(
                            state_vector(velocity=velocity, theta=theta),
                            self.I))

                    assert_array_equal(patch_velocity, velocity)

            # Now, test that spinning the robot results in module velocities.
            # We are assuming that each module is on a square robot.
            module_center_of_mass_angle = i * numpy.pi / 2.0 + numpy.pi / 4.0
            for theta in [-module_center_of_mass_angle, 0.0, 1.0, -1.0, 100.0]:
                for omega in [0.65, -0.1]:
                    # Point each module to the center to make the math easier.
                    patch_velocity = numpy.array(
                        contact_patch_velocity(
                            state_vector(
                                velocity=numpy.array([[0.0], [0.0]]),
                                theta=theta,
                                omega=omega,
                                module_angle=module_center_of_mass_angle),
                            self.I))

                    assert_array_almost_equal(
                        patch_velocity,
                        (dynamics.ROBOT_WIDTH / numpy.sqrt(2.0) +
                         dynamics.CASTER) * omega * numpy.array([[
                             -numpy.sin(theta + module_center_of_mass_angle)
                         ], [numpy.cos(theta + module_center_of_mass_angle)]]))

                    # Point the wheel along +x, rotate it by theta, then spin it.
                    # Confirm the velocities come out right.
                    patch_velocity = numpy.array(
                        contact_patch_velocity(
                            state_vector(
                                velocity=numpy.array([[0.0], [0.0]]),
                                theta=-module_center_of_mass_angle,
                                module_omega=omega,
                                module_angle=(theta +
                                              module_center_of_mass_angle)),
                            self.I))

                    assert_array_almost_equal(
                        patch_velocity,
                        dynamics.CASTER * omega *
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

            slip_angle = dynamics.slip_angle(i, casadi.SX.sym("X", 25, 1),
                                             casadi.SX.sym("U", 8, 1))

            for wrap in range(-3, 3):
                for theta in [0.0, 0.6, -0.4]:
                    module_angle = numpy.pi * wrap + theta

                    self.assertAlmostEqual(
                        theta,
                        half_atan2(numpy.sin(module_angle),
                                   numpy.cos(module_angle)))

                    computed_angle = slip_angle(
                        state_vector(velocity=velocity,
                                     module_angle=numpy.pi * wrap + theta),
                        self.I)

                    self.assertAlmostEqual(theta, computed_angle)

    def test_wheel_forces(self):
        """Tests that the per module forces have the right signs."""
        for i in range(4):
            wheel_force = dynamics.wheel_force(i, casadi.SX.sym("X", 25, 1),
                                               casadi.SX.sym("U", 8, 1))

            # Robot is moving faster than the wheels, it should decelerate.
            robot_faster = numpy.array(
                wheel_force(state_vector(dx=0.01), self.I))
            self.assertLess(robot_faster[0, 0], -0.1)
            self.assertEqual(robot_faster[1, 0], 0.0)

            # Robot is now going slower than the wheels.  It should accelerate.
            robot_slower = numpy.array(
                wheel_force(state_vector(dx=-0.01), self.I))
            self.assertGreater(robot_slower[0, 0], 0.1)
            self.assertEqual(robot_slower[1, 0], 0.0)

            # Positive lateral velocity -> negative force.
            robot_left = numpy.array(wheel_force(state_vector(dy=0.01),
                                                 self.I))
            self.assertEqual(robot_left[0, 0], 0.0)
            self.assertLess(robot_left[1, 0], -0.1)

            # Negative lateral velocity -> positive force.
            robot_right = numpy.array(
                wheel_force(state_vector(dy=-0.01), self.I))
            self.assertEqual(robot_right[0, 0], 0.0)
            self.assertGreater(robot_right[1, 0], 0.1)


if __name__ == '__main__':
    unittest.main()
