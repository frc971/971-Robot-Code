#!/usr/bin/python3

import numpy, scipy

from matplotlib import pylab
from frc971.control_loops.swerve import physics_test_utils as utils
from frc971.control_loops.swerve import dynamics


class PhysicsDebug(object):

    def wrap(self, python_module):
        self.swerve_physics = utils.wrap(python_module.swerve_full_dynamics)
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

    def print_state(self, swerve_physics, I, x):
        xdot = swerve_physics(x, I)

        for i in range(4):
            print(f"  Slip Angle {i} {self.slip_angle[0](x, I)}")
            print(f"  Slip Ratio {i} {self.slip_ratio[0](x, I)}")

        print("  Steering angle0", x[0])
        print("  Steering angle1", x[4])
        print("  Steering angle2", x[8])
        print("  Steering angle3", x[12])
        print("  Steering velocity0", xdot[0])
        print("  Steering velocity1", xdot[4])
        print("  Steering velocity2", xdot[8])
        print("  Steering velocity3", xdot[12])
        print("  Steering accel0", xdot[2])
        print("  Steering accel1", xdot[6])
        print("  Steering accel2", xdot[10])
        print("  Steering accel3", xdot[14])
        print("  Drive accel0", xdot[3])
        print("  Drive accel1", xdot[7])
        print("  Drive accel2", xdot[11])
        print("  Drive accel3", xdot[15])
        print("  Drive velocity0", x[3] * 2 * 0.0254)
        print("  Drive velocity1", x[7] * 2 * 0.0254)
        print("  Drive velocity2", x[11] * 2 * 0.0254)
        print("  Drive velocity3", x[15] * 2 * 0.0254)
        print("  Theta ", x[18])
        print("  Omega ", x[21])
        print("  Alpha", xdot[21])
        print("  vx", xdot[16])
        print("  vy", xdot[17])
        print("  ax", xdot[19])
        print("  ay", xdot[20])
        print("  Fdx ", x[22])
        print("  Fdy ", x[23])
        print("  Moment_d", xdot[24])

    def plot(self):
        velocity = numpy.array([[1.0], [0.0]])

        flip = False
        if flip:
            module_angles = [0.01 + numpy.pi] * 4
        else:
            module_angles = [-0.1, -0.1, 0.1, 0.1]

        X = utils.state_vector(
            velocity=velocity,
            drive_wheel_velocity=-1.0 if flip else 1.0,
            omega=0.0,
            module_angles=module_angles,
        )

        self.I = numpy.array([[40.0], [0.0], [40.0], [0.0], [40.0], [0.0],
                              [40.0], [0.0]])

        def calc_I(t, x):
            x_goal = numpy.zeros(16)

            Kp_steer = 15.0
            Kp_drive = 0.0
            Kd_steer = 7.0
            Kd_drive = 0.0
            Idrive = 5.0 if t < 5.0 else 15.0

            return numpy.array([
                [
                    Kd_steer * (x_goal[2] - x[2]) + Kp_steer *
                    (x_goal[0] - x[0])
                ],
                [Idrive],
                [
                    Kd_steer * (x_goal[6] - x[6]) + Kp_steer *
                    (x_goal[4] - x[4])
                ],
                [Idrive],
                [
                    Kd_steer * (x_goal[10] - x[10]) + Kp_steer *
                    (x_goal[8] - x[8])
                ],
                [Idrive],
                [
                    Kd_steer * (x_goal[14] - x[14]) + Kp_steer *
                    (x_goal[12] - x[12])
                ],
                [Idrive],
            ]).flatten()
            return numpy.array([
                [
                    Kd_steer * (x_goal[2] - x[2]) + Kp_steer *
                    (x_goal[0] - x[0])
                ],
                [
                    Kd_drive * (x_goal[3] - x[3]) + Kp_drive *
                    (x_goal[1] - x[1])
                ],
                [
                    Kd_steer * (x_goal[6] - x[6]) + Kp_steer *
                    (x_goal[4] - x[4])
                ],
                [
                    Kd_drive * (x_goal[7] - x[7]) + Kp_drive *
                    (x_goal[5] - x[5])
                ],
                [
                    Kd_steer * (x_goal[10] - x[10]) + Kp_steer *
                    (x_goal[8] - x[8])
                ],
                [
                    Kd_drive * (x_goal[11] - x[11]) + Kp_drive *
                    (x_goal[9] - x[9])
                ],
                [
                    Kd_steer * (x_goal[14] - x[14]) + Kp_steer *
                    (x_goal[12] - x[12])
                ],
                [
                    Kd_drive * (x_goal[15] - x[15]) + Kp_drive *
                    (x_goal[13] - x[13])
                ],
            ]).flatten()

        t_eval = numpy.arange(0, 10.0, 0.005)
        self.wrap(dynamics)
        result = scipy.integrate.solve_ivp(
            lambda t, x: self.swerve_physics(x, calc_I(t, x)).flatten(),
            [0, t_eval[-1]],
            X.flatten(),
            t_eval=t_eval,
        )
        print("Function evaluations", result.nfev)

        # continue
        print(result.y.shape)
        print(result.t.shape)
        xdot = numpy.zeros(result.y.shape)
        print("shape", xdot.shape)
        for i in range(xdot.shape[1]):
            xdot[:, i] = self.swerve_physics(result.y[:, i], self.I)[:, 0]

        for i in range(2):
            print(f"For t {i * 0.005}")
            self.print_state(self.swerve_physics, self.I, result.y[:, i])

        def ev(fn, Y):
            return [fn(Y[:, i], self.I)[0, 0] for i in range(Y.shape[1])]

        fig, axs = pylab.subplots(2)
        axs[0].plot(result.t, result.y[0, :], label="steer0")
        axs[0].plot(result.t, result.y[4, :], label="steer1")
        axs[0].plot(result.t, result.y[8, :], label="steer2")
        axs[0].plot(result.t, result.y[12, :], label="steer3")
        axs[0].legend()
        axs[1].plot(result.t, result.y[2, :], label="steer_velocity0")
        axs[1].plot(result.t, result.y[6, :], label="steer_velocity1")
        axs[1].plot(result.t, result.y[10, :], label="steer_velocity2")
        axs[1].plot(result.t, result.y[14, :], label="steer_velocity3")
        axs[1].legend()

        fig, axs = pylab.subplots(2)
        for i in range(len(axs)):
            axs[i].plot(result.t,
                        ev(self.slip_angle[i], result.y),
                        label=f"slip_angle{i}")
            axs[i].plot(result.t,
                        ev(self.slip_ratio[i], result.y),
                        label=f"slip_ratio{i}")
            axs[i].plot(result.t,
                        xdot[2 + i * 4, :],
                        label=f'steering_accel{i}')
            axs[i].legend()

        fig, axs = pylab.subplots(3)
        axs[0].plot(result.t, result.y[3, :], label="drive_velocity0")
        axs[0].plot(result.t, result.y[7, :], label="drive_velocity1")
        axs[0].plot(result.t, result.y[11, :], label="drive_velocity2")
        axs[0].plot(result.t, result.y[15, :], label="drive_velocity3")
        axs[0].legend()

        axs[1].plot(result.t, result.y[20, :], label="vy")
        axs[1].plot(result.t, result.y[21, :], label="omega")
        axs[1].plot(result.t, xdot[20, :], label="ay")
        axs[1].plot(result.t, xdot[21, :], label="alpha")
        axs[1].legend()

        axs[2].plot(result.t, result.y[19, :], label="vx")
        axs[2].plot(result.t, xdot[19, :], label="ax")

        axs[2].plot(result.t,
                    numpy.hypot(result.y[19, :], result.y[20, :]),
                    label="speed")
        axs[2].legend()

        pylab.figure()
        U_control = numpy.zeros((8, 1))

        for i in range(numpy.shape(result.t)[0]):
            U_control = numpy.hstack((
                U_control,
                numpy.reshape(
                    calc_I(result.t[i], result.y[:, i]),
                    (8, 1),
                ),
            ))

        U_control = numpy.delete(U_control, 0, 1)

        pylab.plot(result.t, U_control[0, :], label="Is0")
        pylab.plot(result.t, U_control[1, :], label="Id0")
        pylab.legend()

        pylab.show()


if __name__ == "__main__":
    debug = PhysicsDebug()
    debug.plot()
