#!/usr/bin/python

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
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
                 robot_cg_offset=0.0):
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
          q_pos: float, q position for a single speed.
          q_pos_low: float, q position low gear.
          q_pos_high: float, q position high gear.
          q_vel: float, q velocity for a single speed
          q_vel_low: float, q velocity low gear.
          q_vel_high: float, q velocity high gear.
          efficiency: float, gear box effiency.
          has_imu: bool, true if imu is present.
          force: bool, true if force.
          kf_q_voltage: float
          motor_type: object, class of values defining the motor in drivetrain.
          num_motors: int, number of motors on one side of drivetrain.
          controller_poles: array, An array of poles. (See control_loop.py)
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
        self.Kv = (
            self.free_speed / (12.0 - self.resistance * self.free_current))
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
        self.tcl = self.Kt / self.Kv / (
            self.Gl * self.Gl * self.resistance * self.r * self.r)
        self.tcr = self.Kt / self.Kv / (
            self.Gr * self.Gr * self.resistance * self.r * self.r)
        self.mpl = self.Kt / (self.Gl * self.resistance * self.r)
        self.mpr = self.Kt / (self.Gr * self.resistance * self.r)

        # State feedback matrices
        # X will be of the format
        # [[positionl], [velocityl], [positionr], velocityr]]
        self.A_continuous = numpy.matrix(
            [[0, 1, 0, 0], [0, -self.mspl * self.tcl, 0, -self.msnr * self.tcr],
             [0, 0, 0, 1], [0, -self.msnl * self.tcl, 0,
                            -self.mspr * self.tcr]])
        self.B_continuous = numpy.matrix(
            [[0, 0], [self.mspl * self.mpl, self.msnr * self.mpr], [0, 0],
             [self.msnl * self.mpl, self.mspr * self.mpr]])
        self.C = numpy.matrix([[1, 0, 0, 0], [0, 0, 1, 0]])
        self.D = numpy.matrix([[0, 0], [0, 0]])

        self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                                   self.B_continuous, self.dt)

    def BuildDrivetrainController(self, q_pos, q_vel):
        # Tune the LQR controller
        self.Q = numpy.matrix([[(1.0 / (q_pos**2.0)), 0.0, 0.0, 0.0],
                               [0.0, (1.0 / (q_vel**2.0)), 0.0, 0.0],
                               [0.0, 0.0, (1.0 / (q_pos**2.0)), 0.0],
                               [0.0, 0.0, 0.0, (1.0 / (q_vel**2.0))]])

        self.R = numpy.matrix([[(1.0 / (12.0**2.0)), 0.0],
                               [0.0, (1.0 / (12.0**2.0))]])
        self.K = controls.dlqr(self.A, self.B, self.Q, self.R)

        glog.debug('DT q_pos %f q_vel %s %s', q_pos, q_vel, self._name)
        glog.debug(str(numpy.linalg.eig(self.A - self.B * self.K)[0]))
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
                                                        [self.msnr, self.mspr]])
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
            self.C = numpy.matrix([[1, 0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0, 0],
                                   [
                                       0, -0.5 / drivetrain_params.robot_radius,
                                       0, 0.5 / drivetrain_params.robot_radius,
                                       0, 0, 0
                                   ], [0, 0, 0, 0, 0, 0, 0]])
            gravity = 9.8
            self.C[3, 0:6] = 0.5 * (
                self.A_continuous[1, 0:6] + self.A_continuous[3, 0:6]) / gravity

            self.D = numpy.matrix(
                [[0, 0], [0, 0], [0, 0],
                 [
                     0.5 * (self.B_continuous[1, 0] + self.B_continuous[3, 0]) /
                     gravity,
                     0.5 * (self.B_continuous[1, 1] + self.B_continuous[3, 1]) /
                     gravity
                 ]])
        else:
            self.C = numpy.matrix([[1, 0, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0, 0],
                                   [
                                       0, -0.5 / drivetrain_params.robot_radius,
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
        self.KalmanGain, self.Q_steady = controls.kalman(
            A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)

        # If we don't have an IMU, pad various matrices with zeros so that
        # we can still have 4 measurement outputs.
        if not self.has_imu:
            self.KalmanGain = numpy.hstack((self.KalmanGain,
                                            numpy.matrix(numpy.zeros((7, 1)))))
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
        self.Kff[0:2, 0:4] = controls.TwoStateFeedForwards(
            self.B[0:4, :], self.Qff)

        self.InitializeState()


def WriteDrivetrain(drivetrain_files,
                    kf_drivetrain_files,
                    year_namespace,
                    drivetrain_params,
                    scalar_type='double'):

    # Write the generated constants out to a file.
    drivetrain_low_low = Drivetrain(
        name="DrivetrainLowLow",
        left_low=True,
        right_low=True,
        drivetrain_params=drivetrain_params)
    drivetrain_low_high = Drivetrain(
        name="DrivetrainLowHigh",
        left_low=True,
        right_low=False,
        drivetrain_params=drivetrain_params)
    drivetrain_high_low = Drivetrain(
        name="DrivetrainHighLow",
        left_low=False,
        right_low=True,
        drivetrain_params=drivetrain_params)
    drivetrain_high_high = Drivetrain(
        name="DrivetrainHighHigh",
        left_low=False,
        right_low=False,
        drivetrain_params=drivetrain_params)

    kf_drivetrain_low_low = KFDrivetrain(
        name="KFDrivetrainLowLow",
        left_low=True,
        right_low=True,
        drivetrain_params=drivetrain_params)
    kf_drivetrain_low_high = KFDrivetrain(
        name="KFDrivetrainLowHigh",
        left_low=True,
        right_low=False,
        drivetrain_params=drivetrain_params)
    kf_drivetrain_high_low = KFDrivetrain(
        name="KFDrivetrainHighLow",
        left_low=False,
        right_low=True,
        drivetrain_params=drivetrain_params)
    kf_drivetrain_high_high = KFDrivetrain(
        name="KFDrivetrainHighHigh",
        left_low=False,
        right_low=False,
        drivetrain_params=drivetrain_params)

    if isinstance(year_namespace, list):
        namespaces = year_namespace
    else:
        namespaces = [year_namespace, 'control_loops', 'drivetrain']
    dog_loop_writer = control_loop.ControlLoopWriter(
        "Drivetrain", [
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

    kf_loop_writer = control_loop.ControlLoopWriter(
        "KFDrivetrain", [
            kf_drivetrain_low_low, kf_drivetrain_low_high,
            kf_drivetrain_high_low, kf_drivetrain_high_high
        ],
        namespaces=namespaces,
        scalar_type=scalar_type)
    kf_loop_writer.Write(kf_drivetrain_files[0], kf_drivetrain_files[1])


def PlotDrivetrainMotions(drivetrain_params):
    # Simulate the response of the system to a step input.
    drivetrain = Drivetrain(
        left_low=False, right_low=False, drivetrain_params=drivetrain_params)
    simulated_left = []
    simulated_right = []
    for _ in xrange(100):
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
    drivetrain = Drivetrain(
        left_low=False, right_low=False, drivetrain_params=drivetrain_params)
    close_loop_left = []
    close_loop_right = []
    left_power = []
    right_power = []
    R = numpy.matrix([[1.0], [0.0], [1.0], [0.0]])
    for _ in xrange(300):
        U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat), drivetrain.U_min,
                       drivetrain.U_max)
        drivetrain.UpdateObserver(U)
        drivetrain.Update(U)
        close_loop_left.append(drivetrain.X[0, 0])
        close_loop_right.append(drivetrain.X[2, 0])
        left_power.append(U[0, 0])
        right_power.append(U[1, 0])

    pylab.plot(range(300), close_loop_left, label='left position')
    pylab.plot(range(300), close_loop_right, 'm--', label='right position')
    pylab.plot(range(300), left_power, label='left power')
    pylab.plot(range(300), right_power, '--', label='right power')
    pylab.suptitle('Linear Move\nLeft and Right Position going to 1')
    pylab.legend()
    pylab.show()

    # Try turning in place
    drivetrain = Drivetrain(drivetrain_params=drivetrain_params)
    close_loop_left = []
    close_loop_right = []
    R = numpy.matrix([[-1.0], [0.0], [1.0], [0.0]])
    for _ in xrange(200):
        U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat), drivetrain.U_min,
                       drivetrain.U_max)
        drivetrain.UpdateObserver(U)
        drivetrain.Update(U)
        close_loop_left.append(drivetrain.X[0, 0])
        close_loop_right.append(drivetrain.X[2, 0])

    pylab.plot(range(200), close_loop_left, label='left position')
    pylab.plot(range(200), close_loop_right, label='right position')
    pylab.suptitle(
        'Angular Move\nLeft position going to -1 and right position going to 1')
    pylab.legend(loc='center right')
    pylab.show()

    # Try turning just one side.
    drivetrain = Drivetrain(drivetrain_params=drivetrain_params)
    close_loop_left = []
    close_loop_right = []
    R = numpy.matrix([[0.0], [0.0], [1.0], [0.0]])
    for _ in xrange(300):
        U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat), drivetrain.U_min,
                       drivetrain.U_max)
        drivetrain.UpdateObserver(U)
        drivetrain.Update(U)
        close_loop_left.append(drivetrain.X[0, 0])
        close_loop_right.append(drivetrain.X[2, 0])

    pylab.plot(range(300), close_loop_left, label='left position')
    pylab.plot(range(300), close_loop_right, label='right position')
    pylab.suptitle(
        'Pivot\nLeft position not changing and right position going to 1')
    pylab.legend(loc='center right')
    pylab.show()
