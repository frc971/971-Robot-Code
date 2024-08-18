#!/usr/bin/python3

import numpy
import sys, os
import casadi
import scipy
from numpy.testing import assert_array_equal, assert_array_almost_equal
import unittest

from frc971.control_loops.swerve import bigcaster_dynamics
from frc971.control_loops.swerve import dynamics
from frc971.control_loops.swerve import nocaster_dynamics
from frc971.control_loops.swerve import physics_test_utils as utils


class TestSwervePhysics(unittest.TestCase):
    I = numpy.zeros((8, 1))

    def to_velocity_state(self, X):
        return dynamics.to_velocity_state(X)

    def swerve_full_dynamics(self, X, U, skip_compare=False):
        X_velocity = self.to_velocity_state(X)
        Xdot = self.position_swerve_full_dynamics(X, U)
        if not skip_compare:
            velocity_states = self.to_velocity_state(Xdot)
            velocity_physics = self.velocity_swerve_physics(X_velocity, U)
            self.assertLess(
                numpy.linalg.norm(velocity_states - velocity_physics),
                2e-2,
                msg=
                f'Norm failed, full physics -> {velocity_states.T}, velocity physics -> {velocity_physics}, difference -> {velocity_physics - velocity_states}',
            )

        return Xdot

    def wrap(self, python_module):
        self.position_swerve_full_dynamics = utils.wrap(
            python_module.swerve_full_dynamics)

        evaluated_fn = python_module.velocity_swerve_physics(
            casadi.SX.sym("X", dynamics.NUM_VELOCITY_STATES, 1),
            casadi.SX.sym("U", 8, 1))
        self.velocity_swerve_physics = lambda X, U: numpy.array(
            evaluated_fn(X, U))

        self.contact_patch_velocity = [
            utils.wrap_module(python_module.contact_patch_velocity, i)
            for i in range(4)
        ]
        self.wheel_ground_velocity = [
            utils.wrap_module(python_module.wheel_ground_velocity, i)
            for i in range(4)
        ]
        self.wheel_slip_velocity = [
            utils.wrap_module(python_module.wheel_slip_velocity, i)
            for i in range(4)
        ]
        self.wheel_force = [
            utils.wrap_module(python_module.wheel_force, i) for i in range(4)
        ]
        self.module_angular_accel = [
            utils.wrap_module(python_module.module_angular_accel, i)
            for i in range(4)
        ]
        self.F = [utils.wrap_module(python_module.F, i) for i in range(4)]
        self.mounting_location = [
            utils.wrap_module(python_module.mounting_location, i)
            for i in range(4)
        ]

        self.slip_angle = [
            utils.wrap_module(python_module.slip_angle, i) for i in range(4)
        ]
        self.slip_ratio = [
            utils.wrap_module(python_module.slip_ratio, i) for i in range(4)
        ]
        self.Ms = [utils.wrap_module(python_module.Ms, i) for i in range(4)]

    def setUp(self):
        self.wrap(dynamics)

    def test_contact_patch_velocity(self):
        """Tests that the contact patch velocity makes sense."""
        for i in range(4):
            contact_patch_velocity = self.contact_patch_velocity[i]
            wheel_ground_velocity = self.wheel_ground_velocity[i]

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
                        utils.state_vector(velocity=velocity, theta=theta),
                        self.I)

                    assert_array_equal(patch_velocity, velocity)

            # Now, test that spinning the robot results in module velocities.
            # We are assuming that each module is on a square robot.
            module_center_of_mass_angle = i * numpy.pi / 2.0 + numpy.pi / 4.0
            for theta in [-module_center_of_mass_angle, 0.0, 1.0, -1.0, 100.0]:
                for omega in [0.65, -0.1]:
                    # Point each module to the center to make the math easier.
                    patch_velocity = contact_patch_velocity(
                        utils.state_vector(
                            velocity=numpy.array([[0.0], [0.0]]),
                            theta=theta,
                            omega=omega,
                            module_angle=module_center_of_mass_angle,
                        ),
                        self.I,
                    )

                    assert_array_almost_equal(
                        patch_velocity,
                        (dynamics.ROBOT_WIDTH / numpy.sqrt(2.0) -
                         dynamics.CASTER) * omega * numpy.array([
                             [-numpy.sin(theta + module_center_of_mass_angle)],
                             [numpy.cos(theta + module_center_of_mass_angle)],
                         ]),
                    )

                    # Point the wheel along +x, rotate it by theta, then spin it.
                    # Confirm the velocities come out right.
                    patch_velocity = contact_patch_velocity(
                        utils.state_vector(
                            velocity=numpy.array([[0.0], [0.0]]),
                            theta=-module_center_of_mass_angle,
                            module_omega=omega,
                            module_angle=(theta + module_center_of_mass_angle),
                        ),
                        self.I,
                    )

                    assert_array_almost_equal(
                        patch_velocity,
                        -dynamics.CASTER * omega *
                        numpy.array([[-numpy.sin(theta)], [numpy.cos(theta)]]),
                    )

            # Now, test that the rotation back to wheel coordinates works.
            # The easiest way to do this is to point the wheel in a direction,
            # move in that direction, and confirm that there is no lateral velocity.
            for robot_angle in [0.0, 1.0, -5.0]:
                for module_angle in [0.0, 1.0, -5.0]:
                    wheel_patch_velocity = numpy.array(
                        wheel_ground_velocity(
                            utils.state_vector(
                                velocity=numpy.array([
                                    [numpy.cos(robot_angle + module_angle)],
                                    [numpy.sin(robot_angle + module_angle)],
                                ]),
                                theta=robot_angle,
                                module_angle=module_angle,
                            ),
                            self.I,
                        ))

                    assert_array_almost_equal(wheel_patch_velocity,
                                              numpy.array([[1], [0]]))

    def test_slip_angle(self):
        """Tests that the slip_angle calculation works."""
        velocity = numpy.array([[1.5], [0.0]])

        for i in range(4):
            for wrap in range(-1, 2):
                for theta in [0.0, 0.6, -0.4]:
                    module_angle = numpy.pi * wrap + theta

                    # We have redefined the angle to be the softened sin of the angle.
                    # That way, when the module flips directions, the slip angle also flips
                    # directions to keep it stable.
                    computed_angle = self.slip_angle[i](
                        utils.state_vector(velocity=velocity,
                                           module_angle=module_angle),
                        self.I,
                    )[0, 0]

                    # Compute out the expected value directly to confirm we got it right.
                    loggain = 20.0
                    vy = 1.5 * numpy.sin(-module_angle)
                    vx = 1.5 * numpy.cos(-module_angle)
                    expected = numpy.sin(-numpy.arctan2(
                        vy,
                        scipy.special.logsumexp([1.0, abs(vx) * loggain]) /
                        loggain))

                    self.assertAlmostEqual(
                        expected,
                        computed_angle,
                        msg=f"Trying wrap {wrap} theta {theta}",
                    )

    def test_wheel_torque(self):
        """Tests that the per module self aligning forces have the right signs."""
        # Point all the modules in a little bit.
        X = utils.state_vector(
            velocity=numpy.array([[1.0], [0.0]]),
            module_angles=[-0.001, -0.001, 0.001, 0.001],
        )
        xdot_equal = self.swerve_full_dynamics(X, self.I)

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

        # Now, make the bot want to go left by going to the other side.
        # The wheels will be going too fast based on our calcs, so they should be decelerating.
        X = utils.state_vector(
            velocity=numpy.array([[1.0], [0.0]]),
            module_angles=[0.01, 0.01, 0.01, 0.01],
        )
        xdot_left = self.swerve_full_dynamics(X, self.I)

        self.assertLess(xdot_left[2, 0], -0.05)
        self.assertLess(xdot_left[3, 0], 0.0)
        self.assertLess(self.Ms[0](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_left[6, 0], -0.05)
        self.assertLess(xdot_left[7, 0], 0.0)
        self.assertLess(self.Ms[1](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_left[10, 0], -0.05)
        self.assertLess(xdot_left[11, 0], 0.0)
        self.assertLess(self.Ms[2](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_left[14, 0], -0.05)
        self.assertLess(xdot_left[15, 0], 0.0)
        self.assertLess(self.Ms[3](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_left[19, 0], 0.0001)
        self.assertGreater(xdot_left[20, 0], 0.05)
        # Shouldn't be spinning.
        self.assertAlmostEqual(xdot_left[21, 0], 0.0)

        # And now do it to the right too.
        X = utils.state_vector(
            velocity=numpy.array([[1.0], [0.0]]),
            module_angles=[-0.01, -0.01, -0.01, -0.01],
        )
        xdot_right = self.swerve_full_dynamics(X, self.I)

        self.assertGreater(xdot_right[2, 0], 0.05)
        self.assertLess(xdot_right[3, 0], 0.0)
        self.assertGreater(self.Ms[0](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_right[6, 0], 0.05)
        self.assertLess(xdot_right[7, 0], 0.0)
        self.assertGreater(self.Ms[1](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_right[10, 0], 0.05)
        self.assertLess(xdot_right[11, 0], 0.0)
        self.assertGreater(self.Ms[2](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_right[14, 0], 0.05)
        self.assertLess(xdot_right[15, 0], 0.0)
        self.assertGreater(self.Ms[3](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_right[19, 0], 0.0001)
        self.assertLess(xdot_right[20, 0], -0.05)
        # Shouldn't be spinning.
        self.assertAlmostEqual(xdot_right[21, 0], 0.0)

    def test_wheel_torque_backwards_nocaster(self):
        """Tests that the per module self aligning forces have the right signs when going backwards."""
        self.wrap(nocaster_dynamics)
        # Point all the modules in a little bit, going backwards.
        X = utils.state_vector(
            velocity=numpy.array([[1.0], [0.0]]),
            module_angles=[
                numpy.pi - 0.001,
                numpy.pi - 0.001,
                numpy.pi + 0.001,
                numpy.pi + 0.001,
            ],
            drive_wheel_velocity=-1,
        )
        xdot_equal = self.swerve_full_dynamics(X, self.I)

        self.assertGreater(xdot_equal[2, 0], 0.0, msg="Steering backwards")
        self.assertAlmostEqual(xdot_equal[3, 0], 0.0, places=1)
        self.assertGreater(self.Ms[0](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_equal[6, 0], 0.0, msg="Steering backwards")
        self.assertAlmostEqual(xdot_equal[7, 0], 0.0, places=1)
        self.assertGreater(self.Ms[1](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_equal[10, 0], 0.0, msg="Steering backwards")
        self.assertAlmostEqual(xdot_equal[11, 0], 0.0, places=1)
        self.assertLess(self.Ms[2](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_equal[14, 0], 0.0, msg="Steering backwards")
        self.assertAlmostEqual(xdot_equal[15, 0], 0.0, places=1)
        self.assertLess(self.Ms[3](X, self.I)[0, 0], 0.0)

        # Shouldn't be spinning.
        self.assertAlmostEqual(xdot_equal[21, 0], 0.0, places=2)

        # Now, make the bot want to go left by going to the other side.
        # The wheels will be going too fast based on our calcs, so they should be decelerating.
        X = utils.state_vector(
            velocity=numpy.array([[1.0], [0.0]]),
            module_angles=[numpy.pi + 0.01] * 4,
            drive_wheel_velocity=-1,
        )
        xdot_left = self.swerve_full_dynamics(X, self.I)

        self.assertLess(xdot_left[2, 0], -0.05)
        self.assertGreater(xdot_left[3, 0], 0.0)
        self.assertLess(self.Ms[0](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_left[6, 0], -0.05)
        self.assertGreater(xdot_left[7, 0], 0.0)
        self.assertLess(self.Ms[1](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_left[10, 0], -0.05)
        self.assertGreater(xdot_left[11, 0], 0.0)
        self.assertLess(self.Ms[2](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_left[14, 0], -0.05)
        self.assertGreater(xdot_left[15, 0], 0.0)
        self.assertLess(self.Ms[3](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_left[19, 0], 0.0001)
        self.assertGreater(xdot_left[20, 0], 0.05)
        # Shouldn't be spinning.
        self.assertAlmostEqual(xdot_left[21, 0], 0.0)

        # And now do it to the right too.
        X = utils.state_vector(
            velocity=numpy.array([[1.0], [0.0]]),
            drive_wheel_velocity=-1,
            module_angles=[-0.01 + numpy.pi] * 4,
        )
        xdot_right = self.swerve_full_dynamics(X, self.I)

        self.assertGreater(xdot_right[2, 0], 0.05)
        self.assertGreater(xdot_right[3, 0], 0.0)
        self.assertGreater(self.Ms[0](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_right[6, 0], 0.05)
        self.assertGreater(xdot_right[7, 0], 0.0)
        self.assertGreater(self.Ms[1](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_right[10, 0], 0.05)
        self.assertGreater(xdot_right[11, 0], 0.0)
        self.assertGreater(self.Ms[2](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_right[14, 0], 0.05)
        self.assertGreater(xdot_right[15, 0], 0.0)
        self.assertGreater(self.Ms[3](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_right[19, 0], 0.0001)
        self.assertLess(xdot_right[20, 0], -0.05)
        # Shouldn't be spinning.
        self.assertAlmostEqual(xdot_right[21, 0], 0.0)

    def test_wheel_torque_backwards_caster(self):
        """Tests that the per module self aligning forces have the right signs when going backwards with a lot of caster."""
        self.wrap(bigcaster_dynamics)
        # Point all the modules in a little bit, going backwards.
        X = utils.state_vector(
            velocity=numpy.array([[1.0], [0.0]]),
            module_angles=[
                numpy.pi - 0.001,
                numpy.pi - 0.001,
                numpy.pi + 0.001,
                numpy.pi + 0.001,
            ],
            drive_wheel_velocity=-1,
        )
        xdot_equal = self.swerve_full_dynamics(X, self.I)

        self.assertLess(xdot_equal[2, 0], 0.0, msg="Steering backwards")
        self.assertAlmostEqual(xdot_equal[3, 0], 0.0, places=1)
        self.assertLess(self.Ms[0](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_equal[6, 0], 0.0, msg="Steering backwards")
        self.assertAlmostEqual(xdot_equal[7, 0], 0.0, places=1)
        self.assertLess(self.Ms[1](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_equal[10, 0], 0.0, msg="Steering backwards")
        self.assertAlmostEqual(xdot_equal[11, 0], 0.0, places=1)
        self.assertGreater(self.Ms[2](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_equal[14, 0], 0.0, msg="Steering backwards")
        self.assertAlmostEqual(xdot_equal[15, 0], 0.0, places=1)
        self.assertGreater(self.Ms[3](X, self.I)[0, 0], 0.0)

        # Shouldn't be spinning.
        self.assertAlmostEqual(xdot_equal[21, 0], 0.0, places=2)

        # Now, make the bot want to go left by going to the other side.
        # The wheels will be going too fast based on our calcs, so they should be decelerating.
        X = utils.state_vector(
            velocity=numpy.array([[1.0], [0.0]]),
            module_angles=[numpy.pi + 0.01] * 4,
            drive_wheel_velocity=-1,
        )
        xdot_left = self.swerve_full_dynamics(X, self.I)

        self.assertGreater(xdot_left[2, 0], -0.05)
        self.assertGreater(xdot_left[3, 0], 0.0)
        self.assertGreater(self.Ms[0](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_left[6, 0], -0.05)
        self.assertGreater(xdot_left[7, 0], 0.0)
        self.assertGreater(self.Ms[1](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_left[10, 0], -0.05)
        self.assertGreater(xdot_left[11, 0], 0.0)
        self.assertGreater(self.Ms[2](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_left[14, 0], -0.05)
        self.assertGreater(xdot_left[15, 0], 0.0)
        self.assertGreater(self.Ms[3](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_left[19, 0], 0.0001)
        self.assertGreater(xdot_left[20, 0], 0.05)
        # Shouldn't be spinning.
        self.assertAlmostEqual(xdot_left[21, 0], 0.0)

        # And now do it to the right too.
        X = utils.state_vector(
            velocity=numpy.array([[1.0], [0.0]]),
            drive_wheel_velocity=-1,
            module_angles=[-0.01 + numpy.pi] * 4,
        )
        xdot_right = self.swerve_full_dynamics(X, self.I)

        self.assertLess(xdot_right[2, 0], 0.05)
        self.assertGreater(xdot_right[3, 0], 0.0)
        self.assertLess(self.Ms[0](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_right[6, 0], 0.05)
        self.assertGreater(xdot_right[7, 0], 0.0)
        self.assertLess(self.Ms[1](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_right[10, 0], 0.05)
        self.assertGreater(xdot_right[11, 0], 0.0)
        self.assertLess(self.Ms[2](X, self.I)[0, 0], 0.0)

        self.assertLess(xdot_right[14, 0], 0.05)
        self.assertGreater(xdot_right[15, 0], 0.0)
        self.assertLess(self.Ms[3](X, self.I)[0, 0], 0.0)

        self.assertGreater(xdot_right[19, 0], 0.0001)
        self.assertLess(xdot_right[20, 0], -0.05)
        # Shouldn't be spinning.
        self.assertAlmostEqual(xdot_right[21, 0], 0.0)

    def test_wheel_forces(self):
        """Tests that the per module forces have the right signs."""
        for i in range(4):
            wheel_force = self.wheel_force[i]

            X = utils.state_vector()
            robot_equal = wheel_force(X, self.I)
            xdot_equal = self.swerve_full_dynamics(X, self.I)
            self.assertEqual(robot_equal[0, 0], 0.0)
            self.assertEqual(robot_equal[1, 0], 0.0)
            self.assertEqual(xdot_equal[2 + 4 * i], 0.0)
            self.assertEqual(xdot_equal[3 + 4 * i], 0.0)

            # Robot is moving faster than the wheels, it should decelerate.
            X = utils.state_vector(dx=0.01)
            robot_faster = wheel_force(X, self.I)
            xdot_faster = self.swerve_full_dynamics(X,
                                                    self.I,
                                                    skip_compare=True)
            self.assertLess(robot_faster[0, 0], -0.1)
            self.assertEqual(robot_faster[1, 0], 0.0)
            self.assertGreater(xdot_faster[3 + 4 * i], 0.0)

            # Robot is now going slower than the wheels.  It should accelerate.
            X = utils.state_vector(dx=-0.01)
            robot_slower = wheel_force(X, self.I)
            xdot_slower = self.swerve_full_dynamics(X,
                                                    self.I,
                                                    skip_compare=True)
            self.assertGreater(robot_slower[0, 0], 0.1)
            self.assertEqual(robot_slower[1, 0], 0.0)
            self.assertLess(xdot_slower[3 + 4 * i], 0.0)

            # Positive lateral velocity -> negative force.
            robot_left = wheel_force(utils.state_vector(dy=0.01), self.I)
            self.assertEqual(robot_left[0, 0], 0.0)
            self.assertLess(robot_left[1, 0], -0.1)

            # Negative lateral velocity -> positive force.
            robot_right = wheel_force(utils.state_vector(dy=-0.01), self.I)
            self.assertEqual(robot_right[0, 0], 0.0)
            self.assertGreater(robot_right[1, 0], 0.1)

    def test_steer_accel(self):
        """Tests that applying a steer torque accelerates the steer reasonably."""

        for velocity in [
                numpy.array([[0.0], [0.0]]),
                numpy.array([[1.0], [0.0]]),
                numpy.array([[2.0], [0.0]])
        ]:
            module_angles = [0.0] * 4

            X = utils.state_vector(
                velocity=velocity,
                omega=0.0,
                module_angles=module_angles,
            )
            steer_I = numpy.array([[1.0], [0.0]] * 4)
            xdot = self.swerve_full_dynamics(X, steer_I)

            self.assertAlmostEqual(xdot[dynamics.STATE_OMEGAS0, 0],
                                   1.4,
                                   places=0)
            self.assertAlmostEqual(xdot[dynamics.STATE_OMEGAS1, 0],
                                   1.4,
                                   places=0)
            self.assertAlmostEqual(xdot[dynamics.STATE_OMEGAS2, 0],
                                   1.4,
                                   places=0)
            self.assertAlmostEqual(xdot[dynamics.STATE_OMEGAS3, 0],
                                   1.4,
                                   places=0)

    def test_drive_accel(self):
        """Tests that applying a drive torque accelerates the drive reasonably."""

        for velocity in [
                numpy.array([[0.01], [0.0]]),
                numpy.array([[1.0], [0.0]]),
                numpy.array([[2.0], [0.0]])
        ]:
            module_angles = [0.0] * 4

            X = utils.state_vector(
                velocity=velocity,
                omega=0.0,
                module_angles=module_angles,
            )
            steer_I = numpy.array([[0.0], [100.0]] * 4)
            # Since the wheels are spinning at the same speed as the ground, there is no force accelerating the robot.  Which means there is not steering moment.  The two physics diverge pretty heavily.
            xdot = self.swerve_full_dynamics(X, steer_I, skip_compare=True)

            self.assertGreater(xdot[dynamics.STATE_OMEGAD0, 0], 100.0)
            self.assertGreater(xdot[dynamics.STATE_OMEGAD1, 0], 100.0)
            self.assertGreater(xdot[dynamics.STATE_OMEGAD2, 0], 100.0)
            self.assertGreater(xdot[dynamics.STATE_OMEGAD3, 0], 100.0)

            X = utils.state_vector(
                velocity=velocity,
                omega=0.0,
                module_angles=module_angles,
            )
            steer_I = numpy.array([[0.0], [-100.0]] * 4)
            xdot = self.swerve_full_dynamics(X, steer_I, skip_compare=True)

            self.assertLess(xdot[dynamics.STATE_OMEGAD0, 0], -100.0)
            self.assertLess(xdot[dynamics.STATE_OMEGAD1, 0], -100.0)
            self.assertLess(xdot[dynamics.STATE_OMEGAD2, 0], -100.0)
            self.assertLess(xdot[dynamics.STATE_OMEGAD3, 0], -100.0)


if __name__ == "__main__":
    unittest.main()
