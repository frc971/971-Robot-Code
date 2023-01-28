#!/usr/bin/python3

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
import math
import sys
from matplotlib import pylab
import glog


class DrivetrainParams(object):

    def __init__(self,
                 J,
                 mass,
                 robot_radius,
                 wheel_radius,
                 G=None,
                 G_high=None,
                 G_low=None,
                 q_pos=None,
                 q_pos_low=None,
                 q_pos_high=None,
                 q_vel=None,
                 q_vel_low=None,
                 q_vel_high=None,
                 efficiency=0.60,
                 has_imu=False,
                 force=False,
                 kf_q_voltage=10.0,
                 motor_type=control_loop.CIM(),
                 num_motors=2,
                 dt=0.00505,
                 controller_poles=[0.90, 0.90],
                 observer_poles=[0.02, 0.02],
                 robot_cg_offset=0.0,
                 coefficient_of_friction=1.0):
        """Defines all constants of a drivetrain.

        Args:
          J: float, Moment of inertia of drivetrain in kg m^2
          mass: float, Mass of the robot in kg.
          robot_radius: float, Radius of the robot, in meters (requires tuning by
            hand).
          wheel_radius: float, Radius of the wheels, in meters.
          G: float, Gear ratio for a single speed.
          G_high: float, Gear ratio for high gear.
          G_low: float, Gear ratio for low gear.
          dt: float, Control loop time step.
          q_pos: float, q position for a single speed LQR controller.
          q_pos_low: float, q position low gear LQR controller.
          q_pos_high: float, q position high gear LQR controller.
          q_vel: float, q velocity for a single speed LQR controller.
          q_vel_low: float, q velocity low gear LQR controller.
          q_vel_high: float, q velocity high gear LQR controller.
          efficiency: float, gear box effiency.
          has_imu: bool, true if imu is present.
          force: bool, true if force.
          kf_q_voltage: float
          motor_type: object, class of values defining the motor in drivetrain.
          num_motors: int, number of motors on one side of drivetrain.
          controller_poles: array, An array of poles for the polydrivetrain
            controller. (See control_loop.py)
          observer_poles: array, An array of poles. (See control_loop.py)
          robot_cg_offset: offset in meters of CG from robot center to left side
        """
        if G is not None:
            assert (G_high is None)
            assert (G_low is None)
            G_high = G
            G_low = G
        assert (G_high is not None)
        assert (G_low is not None)

        if q_pos is not None:
            assert (q_pos_low is None)
            assert (q_pos_high is None)
            q_pos_low = q_pos
            q_pos_high = q_pos
        assert (q_pos_low is not None)
        assert (q_pos_high is not None)

        if q_vel is not None:
            assert (q_vel_low is None)
            assert (q_vel_high is None)
            q_vel_low = q_vel
            q_vel_high = q_vel
        assert (q_vel_low is not None)
        assert (q_vel_high is not None)

        self.J = J
        self.mass = mass
        self.robot_radius = robot_radius
        self.robot_cg_offset = robot_cg_offset
        self.wheel_radius = wheel_radius
        self.G_high = G_high
        self.G_low = G_low
        self.dt = dt
        self.q_pos_low = q_pos_low
        self.q_pos_high = q_pos_high
        self.q_vel_low = q_vel_low
        self.q_vel_high = q_vel_high
        self.efficiency = efficiency
        self.has_imu = has_imu
        self.kf_q_voltage = kf_q_voltage
        self.motor_type = motor_type
        self.force = force
        self.num_motors = num_motors
        self.controller_poles = controller_poles
        self.observer_poles = observer_poles
        self.coefficient_of_friction = coefficient_of_friction


class Drivetrain(control_loop.ControlLoop):

    def __init__(self,
                 drivetrain_params,
                 name="Drivetrain",
                 left_low=True,
                 right_low=True):
        """Defines a base drivetrain for a robot.

        Args:
          drivetrain_params: DrivetrainParams, class of values defining the drivetrain.
          name: string, Name of this drivetrain.
          left_low: bool, Whether the left is in high gear.
          right_low: bool, Whether the right is in high gear.
        """
        super(Drivetrain, self).__init__(name)

        # Moment of inertia of the drivetrain in kg m^2
        self.J = drivetrain_params.J
        # Mass of the robot, in kg.
        self.mass = drivetrain_params.mass
        # Radius of the robot, in meters (requires tuning by hand)
        self.robot_radius = drivetrain_params.robot_radius
        # Radius of the wheels, in meters.
        self.r = drivetrain_params.wheel_radius
        self.has_imu = drivetrain_params.has_imu
        # Offset in meters of the CG from the center of the robot to the left side
        # of the robot.  Since the arm is on the right side, the offset will
        # likely be a negative number.
        self.robot_cg_offset = drivetrain_params.robot_cg_offset
        # Distance from the left side of the robot to the Center of Gravity
        self.robot_radius_l = drivetrain_params.robot_radius - self.robot_cg_offset
        # Distance from the right side of the robot to the Center of Gravity
        self.robot_radius_r = drivetrain_params.robot_radius + self.robot_cg_offset

        # Gear ratios
        self.G_low = drivetrain_params.G_low
        self.G_high = drivetrain_params.G_high
        if left_low:
            self.Gl = self.G_low
        else:
            self.Gl = self.G_high
        if right_low:
            self.Gr = self.G_low
        else:
            self.Gr = self.G_high

        # Control loop time step
        self.dt = drivetrain_params.dt

        self.efficiency = drivetrain_params.efficiency
        self.force = drivetrain_params.force

        self.BuildDrivetrain(drivetrain_params.motor_type,
                             drivetrain_params.num_motors)

        if left_low or right_low:
            q_pos = drivetrain_params.q_pos_low
            q_vel = drivetrain_params.q_vel_low
        else:
            q_pos = drivetrain_params.q_pos_high
            q_vel = drivetrain_params.q_vel_high

        self.BuildDrivetrainController(q_pos, q_vel)

        self.InitializeState()

    def BuildDrivetrain(self, motor, num_motors_per_side):
        self.motor = motor
        # Number of motors per side
        self.num_motors = num_motors_per_side
        # Stall Torque in N m
        self.stall_torque = motor.stall_torque * self.num_motors * self.efficiency
        # Stall Current in Amps
        self.stall_current = motor.stall_current * self.num_motors
        # Free Speed in rad/s
        self.free_speed = motor.free_speed
        # Free Current in Amps
        self.free_current = motor.free_current * self.num_motors

        # Effective motor resistance in ohms.
        self.resistance = 12.0 / self.stall_current

        # Resistance of the motor, divided by the number of motors.
        # Motor velocity constant
        self.Kv = (self.free_speed /
                   (12.0 - self.resistance * self.free_current))
        # Torque constant
        self.Kt = self.stall_torque / self.stall_current

        # These describe the way that a given side of a robot will be influenced
        # by the other side. Units of 1 / kg.
        self.mspl = 1.0 / self.mass + self.robot_radius_l * self.robot_radius_l / self.J
        self.mspr = 1.0 / self.mass + self.robot_radius_r * self.robot_radius_r / self.J
        self.msnl = self.robot_radius_r / ( self.robot_radius_l * self.mass ) - \
            self.robot_radius_l * self.robot_radius_r / self.J
        self.msnr = self.robot_radius_l / ( self.robot_radius_r * self.mass ) - \
            self.robot_radius_l * self.robot_radius_r / self.J
        # The calculations which we will need for A and B.
        self.tcl = self.Kt / self.Kv / (self.Gl * self.Gl * self.resistance *
                                        self.r * self.r)
        self.tcr = self.Kt / self.Kv / (self.Gr * self.Gr * self.resistance *
                                        self.r * self.r)
        self.mpl = self.Kt / (self.Gl * self.resistance * self.r)
        self.mpr = self.Kt / (self.Gr * self.resistance * self.r)

        # State feedback matrices
        # X will be of the format
        # [[positionl], [velocityl], [positionr], [velocityr]]
        self.A_continuous = numpy.matrix(
            [[0, 1, 0,
              0], [0, -self.mspl * self.tcl, 0, -self.msnr * self.tcr],
             [0, 0, 0, 1],
             [0, -self.msnl * self.tcl, 0, -self.mspr * self.tcr]])
        self.B_continuous = numpy.matrix(
            [[0, 0], [self.mspl * self.mpl, self.msnr * self.mpr], [0, 0],
             [self.msnl * self.mpl, self.mspr * self.mpr]])
        self.C = numpy.matrix([[1, 0, 0, 0], [0, 0, 1, 0]])
        self.D = numpy.matrix([[0, 0], [0, 0]])

        self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                                   self.B_continuous, self.dt)

    def BuildDrivetrainController(self, q_pos, q_vel):
        # We can solve for the max velocity by setting \dot(x) = Ax + Bu to 0
        max_voltage = 12
        glog.debug(
            "Max speed %f m/s",
            -(self.B_continuous[1, 1] + self.B_continuous[1, 0]) /
            (self.A_continuous[1, 1] + self.A_continuous[1, 3]) * max_voltage)

        # Tune the LQR controller
        self.Q = numpy.matrix([[(1.0 / (q_pos**2.0)), 0.0, 0.0, 0.0],
                               [0.0, (1.0 / (q_vel**2.0)), 0.0, 0.0],
                               [0.0, 0.0, (1.0 / (q_pos**2.0)), 0.0],
                               [0.0, 0.0, 0.0, (1.0 / (q_vel**2.0))]])

        self.R = numpy.matrix([[(1.0 / (12.0**2.0)), 0.0],
                               [0.0, (1.0 / (12.0**2.0))]])
        self.K = controls.dlqr(self.A, self.B, self.Q, self.R)

        glog.debug('DT q_pos %f q_vel %s %s', q_pos, q_vel, self._name)
        glog.debug('Poles: %s',
                   str(numpy.linalg.eig(self.A - self.B * self.K)[0]))
        glog.debug(
            'Time constants: %s hz',
            str([
                numpy.log(x) / -self.dt
                for x in numpy.linalg.eig(self.A - self.B * self.K)[0]
            ]))
        glog.debug('K %s', repr(self.K))

        self.hlp = 0.3
        self.llp = 0.4
        self.PlaceObserverPoles([self.hlp, self.hlp, self.llp, self.llp])

        self.U_max = numpy.matrix([[12.0], [12.0]])
        self.U_min = numpy.matrix([[-12.0], [-12.0]])


class KFDrivetrain(Drivetrain):

    def __init__(self,
                 drivetrain_params,
                 name="KFDrivetrain",
                 left_low=True,
                 right_low=True):
        """Kalman filter values of a drivetrain.

        Args:
          drivetrain_params: DrivetrainParams, class of values defining the drivetrain.
          name: string, Name of this drivetrain.
          left_low: bool, Whether the left is in high gear.
          right_low: bool, Whether the right is in high gear.
        """
        super(KFDrivetrain, self).__init__(drivetrain_params, name, left_low,
                                           right_low)

        self.unaugmented_A_continuous = self.A_continuous
        self.unaugmented_B_continuous = self.B_continuous

        # The practical voltage applied to the wheels is
        #   V_left = U_left + left_voltage_error
        #
        # The states are
        # [left position, left velocity, right position, right velocity,
        #  left voltage error, right voltage error, angular_error]
        #
        # The left and right positions are filtered encoder positions and are not
        # adjusted for heading error.
        # The turn velocity as computed by the left and right velocities is
        # adjusted by the gyro velocity.
        # The angular_error is the angular velocity error between the wheel speed
        # and the gyro speed.
        self.A_continuous = numpy.matrix(numpy.zeros((7, 7)))
        self.B_continuous = numpy.matrix(numpy.zeros((7, 2)))
        self.A_continuous[0:4, 0:4] = self.unaugmented_A_continuous

        if self.force:
            self.A_continuous[0:4, 4:6] = numpy.matrix([[0.0, 0.0],
                                                        [self.mspl, self.msnl],
                                                        [0.0, 0.0],
                                                        [self.msnr,
                                                         self.mspr]])
            q_voltage = drivetrain_params.kf_q_voltage * self.mpl
        else:
            self.A_continuous[0:4, 4:6] = self.unaugmented_B_continuous
            q_voltage = drivetrain_params.kf_q_voltage

        self.B_continuous[0:4, 0:2] = self.unaugmented_B_continuous
        self.A_continuous[0, 6] = 1
        self.A_continuous[2, 6] = -1

        self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                                   self.B_continuous, self.dt)

        if self.has_imu:
            self.C = numpy.matrix([[1, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 1, 0, 0, 0, 0],
                                   [
                                       0,
                                       -0.5 / drivetrain_params.robot_radius,
                                       0, 0.5 / drivetrain_params.robot_radius,
                                       0, 0, 0
                                   ], [0, 0, 0, 0, 0, 0, 0]])
            gravity = 9.8
            self.C[3, 0:6] = 0.5 * (self.A_continuous[1, 0:6] +
                                    self.A_continuous[3, 0:6]) / gravity

            self.D = numpy.matrix([
                [0, 0], [0, 0], [0, 0],
                [
                    0.5 * (self.B_continuous[1, 0] + self.B_continuous[3, 0]) /
                    gravity,
                    0.5 * (self.B_continuous[1, 1] + self.B_continuous[3, 1]) /
                    gravity
                ]
            ])
        else:
            self.C = numpy.matrix([[1, 0, 0, 0, 0, 0, 0],
                                   [0, 0, 1, 0, 0, 0, 0],
                                   [
                                       0,
                                       -0.5 / drivetrain_params.robot_radius,
                                       0, 0.5 / drivetrain_params.robot_radius,
                                       0, 0, 0
                                   ]])

            self.D = numpy.matrix([[0, 0], [0, 0], [0, 0]])

        q_pos = 0.05
        q_vel = 1.00
        q_encoder_uncertainty = 2.00

        self.Q = numpy.matrix(
            [[(q_pos**2.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
             [0.0, (q_vel**2.0), 0.0, 0.0, 0.0, 0.0, 0.0],
             [0.0, 0.0, (q_pos**2.0), 0.0, 0.0, 0.0, 0.0],
             [0.0, 0.0, 0.0, (q_vel**2.0), 0.0, 0.0, 0.0],
             [0.0, 0.0, 0.0, 0.0, (q_voltage**2.0), 0.0, 0.0],
             [0.0, 0.0, 0.0, 0.0, 0.0, (q_voltage**2.0), 0.0],
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (q_encoder_uncertainty**2.0)]])

        r_pos = 0.0001
        r_gyro = 0.000001
        if self.has_imu:
            r_accelerometer = 7.0
            self.R = numpy.matrix([[(r_pos**2.0), 0.0, 0.0, 0.0],
                                   [0.0, (r_pos**2.0), 0.0, 0.0],
                                   [0.0, 0.0, (r_gyro**2.0), 0.0],
                                   [0.0, 0.0, 0.0, (r_accelerometer**2.0)]])
        else:
            self.R = numpy.matrix([[(r_pos**2.0), 0.0, 0.0],
                                   [0.0, (r_pos**2.0), 0.0],
                                   [0.0, 0.0, (r_gyro**2.0)]])

        # Solving for kf gains.
        self.KalmanGain, self.Q_steady = controls.kalman(A=self.A,
                                                         B=self.B,
                                                         C=self.C,
                                                         Q=self.Q,
                                                         R=self.R)

        # If we don't have an IMU, pad various matrices with zeros so that
        # we can still have 4 measurement outputs.
        if not self.has_imu:
            self.KalmanGain = numpy.hstack(
                (self.KalmanGain, numpy.matrix(numpy.zeros((7, 1)))))
            self.C = numpy.vstack((self.C, numpy.matrix(numpy.zeros((1, 7)))))
            self.D = numpy.vstack((self.D, numpy.matrix(numpy.zeros((1, 2)))))
            Rtmp = numpy.zeros((4, 4))
            Rtmp[0:3, 0:3] = self.R
            self.R = Rtmp

        self.L = self.A * self.KalmanGain

        unaug_K = self.K

        # Implement a nice closed loop controller for use by the closed loop
        # controller.
        self.K = numpy.matrix(numpy.zeros((self.B.shape[1], self.A.shape[0])))
        self.K[0:2, 0:4] = unaug_K
        if self.force:
            self.K[0, 4] = 1.0 / self.mpl
            self.K[1, 5] = 1.0 / self.mpr
        else:
            self.K[0, 4] = 1.0
            self.K[1, 5] = 1.0

        self.Qff = numpy.matrix(numpy.zeros((4, 4)))
        qff_pos = 0.005
        qff_vel = 1.00
        self.Qff[0, 0] = 1.0 / qff_pos**2.0
        self.Qff[1, 1] = 1.0 / qff_vel**2.0
        self.Qff[2, 2] = 1.0 / qff_pos**2.0
        self.Qff[3, 3] = 1.0 / qff_vel**2.0
        self.Kff = numpy.matrix(numpy.zeros((2, 7)))
        self.Kff[0:2,
                 0:4] = controls.TwoStateFeedForwards(self.B[0:4, :], self.Qff)

        self.InitializeState()


def WriteDrivetrain(drivetrain_files,
                    kf_drivetrain_files,
                    year_namespace,
                    drivetrain_params,
                    scalar_type='double'):

    # Write the generated constants out to a file.
    drivetrain_low_low = Drivetrain(name="DrivetrainLowLow",
                                    left_low=True,
                                    right_low=True,
                                    drivetrain_params=drivetrain_params)
    drivetrain_low_high = Drivetrain(name="DrivetrainLowHigh",
                                     left_low=True,
                                     right_low=False,
                                     drivetrain_params=drivetrain_params)
    drivetrain_high_low = Drivetrain(name="DrivetrainHighLow",
                                     left_low=False,
                                     right_low=True,
                                     drivetrain_params=drivetrain_params)
    drivetrain_high_high = Drivetrain(name="DrivetrainHighHigh",
                                      left_low=False,
                                      right_low=False,
                                      drivetrain_params=drivetrain_params)

    kf_drivetrain_low_low = KFDrivetrain(name="KFDrivetrainLowLow",
                                         left_low=True,
                                         right_low=True,
                                         drivetrain_params=drivetrain_params)
    kf_drivetrain_low_high = KFDrivetrain(name="KFDrivetrainLowHigh",
                                          left_low=True,
                                          right_low=False,
                                          drivetrain_params=drivetrain_params)
    kf_drivetrain_high_low = KFDrivetrain(name="KFDrivetrainHighLow",
                                          left_low=False,
                                          right_low=True,
                                          drivetrain_params=drivetrain_params)
    kf_drivetrain_high_high = KFDrivetrain(name="KFDrivetrainHighHigh",
                                           left_low=False,
                                           right_low=False,
                                           drivetrain_params=drivetrain_params)

    if isinstance(year_namespace, list):
        namespaces = year_namespace
    else:
        namespaces = [year_namespace, 'control_loops', 'drivetrain']
    dog_loop_writer = control_loop.ControlLoopWriter("Drivetrain", [
        drivetrain_low_low, drivetrain_low_high, drivetrain_high_low,
        drivetrain_high_high
    ],
                                                     namespaces=namespaces,
                                                     scalar_type=scalar_type)
    dog_loop_writer.AddConstant(
        control_loop.Constant("kDt", "%f", drivetrain_low_low.dt))
    dog_loop_writer.AddConstant(
        control_loop.Constant("kStallTorque", "%f",
                              drivetrain_low_low.stall_torque))
    dog_loop_writer.AddConstant(
        control_loop.Constant("kStallCurrent", "%f",
                              drivetrain_low_low.stall_current))
    dog_loop_writer.AddConstant(
        control_loop.Constant("kFreeSpeed", "%f",
                              drivetrain_low_low.free_speed))
    dog_loop_writer.AddConstant(
        control_loop.Constant("kFreeCurrent", "%f",
                              drivetrain_low_low.free_current))
    dog_loop_writer.AddConstant(
        control_loop.Constant("kJ", "%f", drivetrain_low_low.J))
    dog_loop_writer.AddConstant(
        control_loop.Constant("kMass", "%f", drivetrain_low_low.mass))
    dog_loop_writer.AddConstant(
        control_loop.Constant("kRobotRadius", "%f",
                              drivetrain_low_low.robot_radius))
    dog_loop_writer.AddConstant(
        control_loop.Constant("kWheelRadius", "%f", drivetrain_low_low.r))
    dog_loop_writer.AddConstant(
        control_loop.Constant("kR", "%f", drivetrain_low_low.resistance))
    dog_loop_writer.AddConstant(
        control_loop.Constant("kV", "%f", drivetrain_low_low.Kv))
    dog_loop_writer.AddConstant(
        control_loop.Constant("kT", "%f", drivetrain_low_low.Kt))
    dog_loop_writer.AddConstant(
        control_loop.Constant("kLowGearRatio", "%f", drivetrain_low_low.G_low))
    dog_loop_writer.AddConstant(
        control_loop.Constant("kHighGearRatio", "%f",
                              drivetrain_high_high.G_high))
    dog_loop_writer.AddConstant(
        control_loop.Constant(
            "kHighOutputRatio", "%f",
            drivetrain_high_high.G_high * drivetrain_high_high.r))

    dog_loop_writer.Write(drivetrain_files[0], drivetrain_files[1])

    kf_loop_writer = control_loop.ControlLoopWriter("KFDrivetrain", [
        kf_drivetrain_low_low, kf_drivetrain_low_high, kf_drivetrain_high_low,
        kf_drivetrain_high_high
    ],
                                                    namespaces=namespaces,
                                                    scalar_type=scalar_type)
    kf_loop_writer.Write(kf_drivetrain_files[0], kf_drivetrain_files[1])


def PlotDrivetrainSprint(drivetrain_params):
    # Simulate the response of the system to a step input.
    drivetrain = KFDrivetrain(left_low=False,
                              right_low=False,
                              drivetrain_params=drivetrain_params)
    simulated_left_position = []
    simulated_right_position = []
    simulated_left_velocity = []
    simulated_right_velocity = []

    simulated_left_motor_currents = []
    simulated_left_breaker_currents = []
    simulated_right_motor_currents = []
    simulated_right_breaker_currents = []

    simulated_battery_heat_wattages = []
    simulated_wattage = []
    motor_inverter_voltages = []
    voltage_left = []
    voltage_right = []
    simulated_motor_heat_wattages = []
    simulated_motor_wattage = []

    max_motor_currents = []
    overall_currents = []
    simulated_battery_wattage = []

    # Distance in meters to call 1/2 field.
    kSprintDistance = 8.0

    vbat = 12.6
    # Measured resistance of the battery, pd board, and breakers.
    Rw = 0.023
    top_speed = drivetrain.free_speed * (drivetrain.Gr +
                                         drivetrain.Gl) / 2.0 * drivetrain.r

    passed_distance = False
    max_breaker_current = 0
    heat_energy_usage = 0.0
    for index in range(800):
        # Current per side
        left_traction_current = (drivetrain.mass / 2.0 *
                                 drivetrain_params.coefficient_of_friction *
                                 9.81 * drivetrain.r * drivetrain.Gl /
                                 drivetrain.Kt)
        right_traction_current = (drivetrain.mass / 2.0 *
                                  drivetrain_params.coefficient_of_friction *
                                  9.81 * drivetrain.r * drivetrain.Gr /
                                  drivetrain.Kt)

        # Detect if we've traveled over the sprint distance and report stats.
        if (drivetrain.X[0, 0] + drivetrain.X[2, 0]) / 2.0 > kSprintDistance:
            if not passed_distance:
                velocity = (drivetrain.X[1, 0] + drivetrain.X[3, 0]) / 2.0
                print("Took", index * drivetrain.dt,
                      "to pass 1/2 field, going", velocity, "m/s,",
                      velocity / 0.0254 / 12.0, "Traction limit current",
                      left_traction_current / drivetrain_params.num_motors,
                      "max breaker current", max_breaker_current, "top speed",
                      top_speed, "m/s", top_speed / 0.0254 / 12.0,
                      "fps, gear ratio", drivetrain.Gl, "heat energy",
                      heat_energy_usage)
            passed_distance = True

        bemf_left = drivetrain.X[
            1, 0] / drivetrain.r / drivetrain.Gl / drivetrain.Kv
        bemf_right = drivetrain.X[
            3, 0] / drivetrain.r / drivetrain.Gr / drivetrain.Kv

        # Max current we could push through the motors is what we would get if
        # we short the battery through the battery resistance into the motor.
        max_motor_current = (vbat - (bemf_left + bemf_right) / 2.0) / (
            Rw + drivetrain.resistance / 2.0)

        max_motor_currents.append(max_motor_current /
                                  (drivetrain_params.num_motors * 2))

        # From this current, we can compute the voltage we can apply.
        # This is either the traction limit or the current limit.
        max_voltage_left = bemf_left + min(
            max_motor_current / 2,
            left_traction_current) * drivetrain.resistance
        max_voltage_right = bemf_right + min(
            max_motor_current / 2,
            right_traction_current) * drivetrain.resistance

        simulated_left_position.append(drivetrain.X[0, 0])
        simulated_left_velocity.append(drivetrain.X[1, 0])
        simulated_right_position.append(drivetrain.X[2, 0])
        simulated_right_velocity.append(drivetrain.X[3, 0])

        U = numpy.matrix([[min(max_voltage_left, vbat)],
                          [min(max_voltage_right, vbat)]])

        # Stator current
        simulated_left_motor_current = (U[0, 0] -
                                        bemf_left) / drivetrain.resistance
        simulated_right_motor_current = (U[1, 0] -
                                         bemf_right) / drivetrain.resistance

        # And this gives us the power pushed into the motors.
        power = (U[0, 0] * simulated_left_motor_current +
                 U[1, 0] * simulated_right_motor_current)

        simulated_wattage.append(power)

        # Solve for the voltage we'd have to supply to the input of the motor
        # controller to generate the power required.
        motor_inverter_voltage = (
            vbat + numpy.sqrt(vbat * vbat - 4.0 * power * Rw)) / 2.0

        overall_current = (vbat - motor_inverter_voltage) / Rw
        overall_currents.append(overall_current)

        motor_inverter_voltages.append(motor_inverter_voltage)

        # Overall left and right currents at the breaker
        simulated_left_breaker_current = (
            simulated_left_motor_current /
            drivetrain_params.num_motors) * U[0, 0] / motor_inverter_voltage
        simulated_right_breaker_current = (
            simulated_right_motor_current /
            drivetrain_params.num_motors) * U[1, 0] / motor_inverter_voltage

        simulated_left_motor_currents.append(simulated_left_motor_current /
                                             drivetrain_params.num_motors)
        simulated_left_breaker_currents.append(simulated_left_breaker_current)
        simulated_right_motor_currents.append(simulated_right_motor_current /
                                              drivetrain_params.num_motors)
        simulated_right_breaker_currents.append(
            simulated_right_breaker_current)

        # Save out the peak battery current observed.
        max_breaker_current = max(
            max_breaker_current,
            max(simulated_left_breaker_current,
                simulated_right_breaker_current))

        # Compute the heat burned in the battery
        simulated_battery_heat_wattage = math.pow(
            vbat - motor_inverter_voltage, 2.0) / Rw
        simulated_battery_heat_wattages.append(simulated_battery_heat_wattage)

        motor_heat_wattage = (math.pow(simulated_left_motor_current, 2.0) *
                              drivetrain.resistance +
                              math.pow(simulated_right_motor_current, 2.0) *
                              drivetrain.resistance)
        simulated_motor_heat_wattages.append(motor_heat_wattage)

        simulated_motor_wattage.append(simulated_left_motor_current * U[0, 0] +
                                       simulated_right_motor_current * U[1, 0])

        simulated_battery_wattage.append(vbat * overall_current)

        # And then the overall energy outputted by the battery.
        heat_energy_usage += (motor_heat_wattage +
                              simulated_battery_heat_wattage) * drivetrain.dt

        voltage_left.append(U[0, 0])
        voltage_right.append(U[1, 0])

        drivetrain.Update(U)

    t = [drivetrain.dt * x for x in range(len(simulated_left_position))]
    pylab.rc('lines', linewidth=4)
    pylab.subplot(3, 1, 1)
    pylab.plot(t, simulated_left_position, label='left position')
    pylab.plot(t, simulated_right_position, 'r--', label='right position')
    pylab.plot(t, simulated_left_velocity, label='left velocity')
    pylab.plot(t, simulated_right_velocity, label='right velocity')

    pylab.suptitle('Acceleration Test\n12 Volt Step Input')
    pylab.legend(loc='lower right')

    pylab.subplot(3, 1, 2)

    pylab.plot(t, simulated_left_motor_currents, label='left rotor current')
    pylab.plot(t,
               simulated_right_motor_currents,
               'r--',
               label='right rotor current')
    pylab.plot(t,
               simulated_left_breaker_currents,
               label='left breaker current')
    pylab.plot(t,
               simulated_right_breaker_currents,
               'r--',
               label='right breaker current')
    pylab.plot(t, motor_inverter_voltages, label='motor inverter voltage')
    pylab.plot(t, voltage_left, label='left voltage')
    pylab.plot(t, voltage_right, label='right voltage')
    pylab.plot(t, max_motor_currents, label='max_currents')
    pylab.legend(loc='lower right')

    wattage_axis = pylab.subplot(3, 1, 3)
    wattage_axis.plot(t, simulated_wattage, label='wattage')
    wattage_axis.plot(t,
                      simulated_battery_heat_wattages,
                      label='battery wattage')
    wattage_axis.plot(t,
                      simulated_motor_heat_wattages,
                      label='motor heat wattage')
    wattage_axis.plot(t, simulated_motor_wattage, label='motor wattage')
    wattage_axis.plot(t, simulated_battery_wattage, label='overall wattage')
    pylab.legend(loc='upper left')
    overall_current_axis = wattage_axis.twinx()
    overall_current_axis.plot(t, overall_currents, 'c--', label='current')

    pylab.legend(loc='lower right')

    pylab.suptitle('Acceleration Test\n12 Volt Step Input\n%f fps' %
                   (top_speed / 0.0254 / 12.0, ))
    pylab.show()


def PlotDrivetrainMotions(drivetrain_params):
    # Test out the voltage error.
    drivetrain = KFDrivetrain(left_low=False,
                              right_low=False,
                              drivetrain_params=drivetrain_params)
    close_loop_left = []
    close_loop_right = []
    left_power = []
    right_power = []
    R = numpy.matrix([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
    for _ in range(300):
        U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat), drivetrain.U_min,
                       drivetrain.U_max)
        drivetrain.CorrectObserver(U)
        drivetrain.PredictObserver(U)
        drivetrain.Update(U + numpy.matrix([[1.0], [1.0]]))
        close_loop_left.append(drivetrain.X[0, 0])
        close_loop_right.append(drivetrain.X[2, 0])
        left_power.append(U[0, 0])
        right_power.append(U[1, 0])

    t = [drivetrain.dt * x for x in range(300)]
    pylab.plot(t, close_loop_left, label='left position')
    pylab.plot(t, close_loop_right, 'm--', label='right position')
    pylab.plot(t, left_power, label='left power')
    pylab.plot(t, right_power, '--', label='right power')
    pylab.suptitle('Voltage error')
    pylab.legend()
    pylab.show()

    # Simulate the response of the system to a step input.
    drivetrain = KFDrivetrain(left_low=False,
                              right_low=False,
                              drivetrain_params=drivetrain_params)
    simulated_left = []
    simulated_right = []
    for _ in range(100):
        drivetrain.Update(numpy.matrix([[12.0], [12.0]]))
        simulated_left.append(drivetrain.X[0, 0])
        simulated_right.append(drivetrain.X[2, 0])

    pylab.rc('lines', linewidth=4)
    pylab.plot(range(100), simulated_left, label='left position')
    pylab.plot(range(100), simulated_right, 'r--', label='right position')
    pylab.suptitle('Acceleration Test\n12 Volt Step Input')
    pylab.legend(loc='lower right')
    pylab.show()

    # Simulate forwards motion.
    drivetrain = KFDrivetrain(left_low=False,
                              right_low=False,
                              drivetrain_params=drivetrain_params)
    close_loop_left = []
    close_loop_right = []
    left_power = []
    right_power = []
    R = numpy.matrix([[1.0], [0.0], [1.0], [0.0], [0.0], [0.0], [0.0]])
    for _ in range(300):
        U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat), drivetrain.U_min,
                       drivetrain.U_max)
        drivetrain.CorrectObserver(U)
        drivetrain.PredictObserver(U)
        drivetrain.Update(U)
        close_loop_left.append(drivetrain.X[0, 0])
        close_loop_right.append(drivetrain.X[2, 0])
        left_power.append(U[0, 0])
        right_power.append(U[1, 0])

    t = [drivetrain.dt * x for x in range(300)]
    pylab.plot(t, close_loop_left, label='left position')
    pylab.plot(t, close_loop_right, 'm--', label='right position')
    pylab.plot(t, left_power, label='left power')
    pylab.plot(t, right_power, '--', label='right power')
    pylab.suptitle('Linear Move\nLeft and Right Position going to 1')
    pylab.legend()
    pylab.show()

    # Try turning in place
    drivetrain = KFDrivetrain(drivetrain_params=drivetrain_params)
    close_loop_left = []
    close_loop_right = []
    R = numpy.matrix([[-1.0], [0.0], [1.0], [0.0], [0.0], [0.0], [0.0]])
    for _ in range(200):
        U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat), drivetrain.U_min,
                       drivetrain.U_max)
        drivetrain.CorrectObserver(U)
        drivetrain.PredictObserver(U)
        drivetrain.Update(U)
        close_loop_left.append(drivetrain.X[0, 0])
        close_loop_right.append(drivetrain.X[2, 0])

    pylab.plot(range(200), close_loop_left, label='left position')
    pylab.plot(range(200), close_loop_right, label='right position')
    pylab.suptitle(
        'Angular Move\nLeft position going to -1 and right position going to 1'
    )
    pylab.legend(loc='center right')
    pylab.show()

    # Try turning just one side.
    drivetrain = KFDrivetrain(drivetrain_params=drivetrain_params)
    close_loop_left = []
    close_loop_right = []
    R = numpy.matrix([[0.0], [0.0], [1.0], [0.0], [0.0], [0.0], [0.0]])
    for _ in range(300):
        U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat), drivetrain.U_min,
                       drivetrain.U_max)
        drivetrain.CorrectObserver(U)
        drivetrain.PredictObserver(U)
        drivetrain.Update(U)
        close_loop_left.append(drivetrain.X[0, 0])
        close_loop_right.append(drivetrain.X[2, 0])

    pylab.plot(range(300), close_loop_left, label='left position')
    pylab.plot(range(300), close_loop_right, label='right position')
    pylab.suptitle(
        'Pivot\nLeft position not changing and right position going to 1')
    pylab.legend(loc='center right')
    pylab.show()
