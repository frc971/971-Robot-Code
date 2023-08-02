#!/usr/bin/python3

from aos.util.trapezoid_profile import TrapezoidProfile
from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
from matplotlib import pylab
import glog


class AngularSystemCurrentParams(object):

    def __init__(self,
                 name,
                 motor,
                 G,
                 J,
                 q_pos,
                 q_vel,
                 kalman_q_pos,
                 kalman_q_vel,
                 kalman_q_voltage,
                 kalman_r_position,
                 radius=None,
                 dt=0.00505):
        """Constructs an AngularSystemCurrentParams object.

        Args:
          motor: Motor object with the motor constants.
          G: float, Gear ratio.  Less than 1 means output moves slower than the
              input.
          J: float, Moment of inertia in kg m^2
          q_pos: float, controller optimizer weight on position error.
          q_vel: float, controller optimizer weight on velocity error
          kalman_q_pos: float, std deviation of the model's position prediction
          kalman_q_vel: float, std deviation of the model's velocity prediction
          kalman_q_voltage: float, std deviation of the model's voltage error prediction
          kalman_r_position: float, std deviation of the position measurement
          radius: float, radius of the mechanism in meters
          dt: float, length of the control loop period in seconds
        """
        self.name = name
        self.motor = motor
        self.G = G
        self.J = J
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.kalman_q_pos = kalman_q_pos
        self.kalman_q_vel = kalman_q_vel
        self.kalman_q_voltage = kalman_q_voltage
        self.kalman_r_position = kalman_r_position
        self.radius = radius
        self.dt = dt


# An angular system that uses current control instead of voltage
# The current through the motor directly impacts the torque
# whereas the effect of an applied voltage depends on the current velocity
class AngularSystemCurrent(control_loop.ControlLoop):

    def __init__(self, params, name="AngularSystemCurrent"):
        super(AngularSystemCurrent, self).__init__(name)
        self.params = params

        self.motor = params.motor

        # Gear ratio
        self.G = params.G

        # Moment of inertia in kg m^2
        self.J = params.J + self.motor.motor_inertia / (self.G**2.0)

        # Control loop time step
        self.dt = params.dt

        # State is [position, velocity]
        # Input is [Current]

        self.A_continuous = numpy.matrix([[0, 1], [0, 0]])

        # Start with the unmodified input
        self.B_continuous = numpy.matrix([[0],
                                          [self.motor.Kt / (self.G * self.J)]])
        glog.debug(repr(self.A_continuous))
        glog.debug(repr(self.B_continuous))

        self.C = numpy.matrix([[1, 0]])
        self.D = numpy.matrix([[0]])

        self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                                   self.B_continuous, self.dt)

        controllability = controls.ctrb(self.A, self.B)
        glog.debug('Controllability of %d',
                   numpy.linalg.matrix_rank(controllability))
        glog.debug('J: %f', self.J)
        glog.debug('Stall torque: %f (N m)', self.motor.stall_torque / self.G)
        if self.params.radius is not None:
            glog.debug('Stall force: %f (N)',
                       self.motor.stall_torque / self.G / self.params.radius)
            glog.debug(
                'Stall force: %f (lbf)', self.motor.stall_torque / self.G /
                self.params.radius * 0.224809)

        glog.debug('Stall acceleration: %f (rad/s^2)',
                   self.motor.stall_torque / self.G / self.J)

        glog.debug('Free speed is %f (rad/s)', self.motor.free_speed)

        Q_state_cost = numpy.matrix([[(1.0 / (self.params.q_pos**2.0)), 0.0],
                                     [0.0, (1.0 / (self.params.q_vel**2.0))]])

        R_control_cost = numpy.matrix([[(1.0 / (12**2.0))]])
        self.K = controls.dlqr(self.A, self.B, Q_state_cost, R_control_cost)

        q_pos_ff = 0.005
        q_vel_ff = 1.0
        self.Qff = numpy.matrix([[(1.0 / (q_pos_ff**2.0)), 0.0],
                                 [0.0, (1.0 / (q_vel_ff**2.0))]])

        self.Kff = controls.TwoStateFeedForwards(self.B, self.Qff)

        glog.debug('K %s', repr(self.K))
        glog.debug('Poles are %s',
                   repr(numpy.linalg.eig(self.A - self.B * self.K)[0]))

        # Process covariance matrix
        self.Q = numpy.matrix([[(self.params.kalman_q_pos**2.0), 0.0],
                               [0.0, (self.params.kalman_q_vel**2.0)]])

        # Measurement covariance matrix
        self.R = numpy.matrix([[(self.params.kalman_r_position**2.0)]])

        # From testing, these continuous Q and R's appear to be good approximations of Q and R.
        self.Q_continuous = self.Q / self.dt
        self.R_continuous = self.R * self.dt

        self.KalmanGain, self.P_steady_state = controls.kalman(A=self.A,
                                                               B=self.B,
                                                               C=self.C,
                                                               Q=self.Q,
                                                               R=self.R)

        glog.debug('Kal %s', repr(self.KalmanGain))

        # The box formed by U_min and U_max must encompass all possible values
        self.U_max = numpy.matrix([[800.0]])
        self.U_min = numpy.matrix([[-800.0]])

        # Max current = (12 - w G / Kv) / R

        # These are used to limit the current command based on the available voltage
        # Used in the form:
        # U_max = U_limit_coefficient * X + U_limit_constant
        # U_min = U_limit_coefficient * X - U_limit_constant

        self.U_limit_coefficient = numpy.matrix(
            [[0, -self.G / self.motor.resistance / self.motor.Kv]])
        self.U_limit_constant = numpy.matrix([[self.motor.stall_current]])

        self.delayed_u = 1

        self.InitializeState()


class IntegralAngularSystemCurrent(AngularSystemCurrent):

    def __init__(self, params, name="IntegralAngularSystemCurrent"):
        super(IntegralAngularSystemCurrent, self).__init__(params, name=name)

        self.A_continuous_unaugmented = self.A_continuous
        self.B_continuous_unaugmented = self.B_continuous

        self.A_continuous = numpy.matrix(numpy.zeros((3, 3)))
        self.A_continuous[0:2, 0:2] = self.A_continuous_unaugmented
        self.A_continuous[0:2, 2] = self.B_continuous_unaugmented

        self.B_continuous = numpy.matrix(numpy.zeros((3, 1)))
        self.B_continuous[0:2, 0] = self.B_continuous_unaugmented

        self.C_unaugmented = self.C
        self.C = numpy.matrix(numpy.zeros((1, 3)))
        self.C[0:1, 0:2] = self.C_unaugmented

        self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                                   self.B_continuous, self.dt)

        self.Q = numpy.matrix([[(self.params.kalman_q_pos**2.0), 0.0, 0.0],
                               [0.0, (self.params.kalman_q_vel**2.0), 0.0],
                               [0.0, 0.0,
                                (self.params.kalman_q_voltage**2.0)]])

        self.R = numpy.matrix([[(self.params.kalman_r_position**2.0)]])

        # From testing, these continuous Q and R's appear to be good approximations of Q and R.
        self.Q_continuous = self.Q / self.dt
        self.R_continuous = self.R * self.dt

        self.KalmanGain, self.P_steady_state = controls.kalman(A=self.A,
                                                               B=self.B,
                                                               C=self.C,
                                                               Q=self.Q,
                                                               R=self.R)

        self.K_unaugmented = self.K
        self.K = numpy.matrix(numpy.zeros((1, 3)))
        self.K[0, 0:2] = self.K_unaugmented
        self.K[0, 2] = 1

        self.Kff = numpy.concatenate(
            (self.Kff, numpy.matrix(numpy.zeros((1, 1)))), axis=1)

        self.U_limit_coefficient_unaugmented = self.U_limit_coefficient
        self.U_limit_coefficient = numpy.matrix(numpy.zeros((1, 3)))
        self.U_limit_coefficient[0, 0:2] = self.U_limit_coefficient_unaugmented

        self.InitializeState()


def RunTest(plant,
            end_goal,
            controller,
            observer=None,
            duration=1.0,
            use_profile=True,
            kick_time=0.5,
            kick_magnitude=0.0,
            max_velocity=10.0,
            max_acceleration=70.0):
    """Runs the plant with an initial condition and goal.

    Args:
      plant: plant object to use.
      end_goal: end_goal state.
      controller: AngularSystemCurrent object to get K from, or None if we should
          use plant.
      observer: AngularSystemCurrent object to use for the observer, or None if we
          should use the actual state.
      duration: float, time in seconds to run the simulation for.
      kick_time: float, time in seconds to kick the robot.
      kick_magnitude: float, disturbance in volts to apply.
      max_velocity: float, The maximum velocity for the profile.
      max_acceleration: float, The maximum acceleration for the profile.
    """
    t_plot = []
    x_plot = []
    v_plot = []
    a_plot = []
    motor_voltage_plot = []
    motor_current_plot = []
    battery_current_plot = []
    x_goal_plot = []
    v_goal_plot = []
    x_hat_plot = []
    u_plot = []
    u_max_plot = []
    offset_plot = []

    if controller is None:
        controller = plant

    vbat = 12.0

    goal = numpy.concatenate((plant.X, numpy.matrix(numpy.zeros((1, 1)))),
                             axis=0)

    profile = TrapezoidProfile(plant.dt)
    profile.set_maximum_acceleration(max_acceleration)
    profile.set_maximum_velocity(max_velocity)
    profile.SetGoal(goal[0, 0])

    U_last = numpy.matrix(numpy.zeros((1, 1)))
    iterations = int(duration / plant.dt)
    for i in range(iterations):
        t = i * plant.dt
        observer.Y = plant.Y
        observer.CorrectObserver(U_last)

        offset_plot.append(observer.X_hat[2, 0])
        x_hat_plot.append(observer.X_hat[0, 0])

        next_goal = numpy.concatenate((profile.Update(
            end_goal[0, 0], end_goal[1, 0]), numpy.matrix(numpy.zeros(
                (1, 1)))),
                                      axis=0)

        ff_U = controller.Kff * (next_goal - observer.A * goal)

        if use_profile:
            U_uncapped = controller.K * (goal - observer.X_hat) + ff_U
            x_goal_plot.append(goal[0, 0])
            v_goal_plot.append(goal[1, 0])
        else:
            U_uncapped = controller.K * (end_goal - observer.X_hat)
            x_goal_plot.append(end_goal[0, 0])
            v_goal_plot.append(end_goal[1, 0])

        U = U_uncapped.copy()

        # clip U to max current possible given battery voltage constraints
        max_current = plant.U_limit_coefficient * plant.X + plant.U_limit_constant
        min_current = plant.U_limit_coefficient * plant.X - plant.U_limit_constant

        u_max_plot.append(max_current[0, 0])
        U[0, 0] = numpy.clip(U[0, 0], min_current, max_current)

        motor_voltage = U[0, 0] * plant.motor.resistance + plant.X[
            1, 0] * plant.G / plant.motor.Kv
        motor_voltage_plot.append(motor_voltage)

        motor_current = U[0, 0]
        motor_current_plot.append(motor_current)

        battery_current = motor_current * motor_voltage / 12.0
        battery_current_plot.append(battery_current)
        x_plot.append(plant.X[0, 0])

        if v_plot:
            last_v = v_plot[-1]
        else:
            last_v = 0

        v_plot.append(plant.X[1, 0])
        a_plot.append((v_plot[-1] - last_v) / plant.dt)

        u_offset = 0.0
        if t >= kick_time:
            u_offset = kick_magnitude
        plant.Update(U + u_offset)

        observer.PredictObserver(U)

        t_plot.append(t)
        u_plot.append(U[0, 0])

        ff_U -= U_uncapped - U
        goal = controller.A * goal + controller.B * ff_U

        if U[0, 0] != U_uncapped[0, 0]:
            profile.MoveCurrentState(numpy.matrix([[goal[0, 0]], [goal[1,
                                                                       0]]]))

    glog.debug('Time: %f', t_plot[-1])
    glog.debug('goal_error %s', repr(end_goal - goal))
    glog.debug('error %s', repr(observer.X_hat - end_goal))

    pylab.subplot(3, 1, 1)
    pylab.plot(t_plot, x_plot, label='x')
    pylab.plot(t_plot, x_hat_plot, label='x_hat')
    pylab.plot(t_plot, x_goal_plot, label='x_goal')
    pylab.legend()

    pylab.subplot(3, 1, 2)
    #pylab.plot(t_plot, u_plot, label='u')
    pylab.plot(t_plot, offset_plot, label='voltage_offset')
    pylab.plot(t_plot, motor_voltage_plot, label='motor_voltage')
    pylab.legend()

    ax1 = pylab.subplot(3, 1, 3)
    ax1.set_xlabel("time(s)")
    ax1.set_ylabel("rad/s^2")
    ax1.plot(t_plot, a_plot, label='a')

    ax2 = ax1.twinx()
    ax2.set_xlabel("time(s)")
    ax2.set_ylabel("Amps")
    ax2.plot(t_plot, battery_current_plot, 'g', label='battery current')
    ax2.plot(t_plot, motor_current_plot, 'r', label='motor')
    ax2.plot(t_plot, u_max_plot, 'b', label='u_max')
    pylab.legend()

    pylab.show()


def PlotStep(params, R, plant_params=None):
    """Plots a step move to the goal.

    Args:
      params: AngularSystemParams for the controller and observer
      plant_params: AngularSystemParams for the plant.  Defaults to params if
        plant_params is None.
      R: numpy.matrix(2, 1), the goal"""
    plant = AngularSystemCurrent(plant_params or params, params.name)
    controller = IntegralAngularSystemCurrent(params, params.name)
    observer = IntegralAngularSystemCurrent(params, params.name)

    # Test moving the system.
    initial_X = numpy.matrix([[0.0], [0.0]])
    augmented_R = numpy.matrix(numpy.zeros((3, 1)))
    augmented_R[0:2, :] = R
    RunTest(plant,
            end_goal=augmented_R,
            controller=controller,
            observer=observer,
            duration=5.0,
            use_profile=False,
            kick_time=1.0,
            kick_magnitude=0.0)


def PlotKick(params, R, plant_params=None):
    """Plots a step motion with a kick at 1.0 seconds.

    Args:
      params: AngularSystemParams for the controller and observer
      plant_params: AngularSystemParams for the plant.  Defaults to params if
        plant_params is None.
      R: numpy.matrix(2, 1), the goal"""
    plant = AngularSystemCurrent(plant_params or params, params.name)
    controller = IntegralAngularSystemCurrent(params, params.name)
    observer = IntegralAngularSystemCurrent(params, params.name)

    # Test moving the system.
    initial_X = numpy.matrix([[0.0], [0.0]])
    augmented_R = numpy.matrix(numpy.zeros((3, 1)))
    augmented_R[0:2, :] = R
    RunTest(plant,
            end_goal=augmented_R,
            controller=controller,
            observer=observer,
            duration=2.0,
            use_profile=False,
            kick_time=1.0,
            kick_magnitude=2.0)


def PlotMotion(params,
               R,
               max_velocity=10.0,
               max_acceleration=70.0,
               plant_params=None):
    """Plots a trapezoidal motion.

    Args:
      params: AngularSystemParams for the controller and observer
      plant_params: AngularSystemParams for the plant.  Defaults to params if
        plant_params is None.
      R: numpy.matrix(2, 1), the goal,
      max_velocity: float, The max velocity of the profile.
      max_acceleration: float, The max acceleration of the profile.
    """
    plant = AngularSystemCurrent(plant_params or params, params.name)
    controller = IntegralAngularSystemCurrent(params, params.name)
    observer = IntegralAngularSystemCurrent(params, params.name)

    # Test moving the system.
    initial_X = numpy.matrix([[0.0], [0.0]])
    augmented_R = numpy.matrix(numpy.zeros((3, 1)))
    augmented_R[0:2, :] = R
    RunTest(plant,
            end_goal=augmented_R,
            controller=controller,
            observer=observer,
            duration=2.0,
            use_profile=True,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration)


def WriteAngularSystemCurrent(params,
                              plant_files,
                              controller_files,
                              year_namespaces,
                              plant_type='StateFeedbackPlant',
                              observer_type='StateFeedbackObserver'):
    """Writes out the constants for a angular system to a file.

    Args:
      params: list of AngularSystemCurrentParams or AngularSystemCurrentParams, the
        parameters defining the system.
      plant_files: list of strings, the cc and h files for the plant.
      controller_files: list of strings, the cc and h files for the integral
        controller.
      year_namespaces: list of strings, the namespace list to use.
    """
    # Write the generated constants out to a file.
    angular_systems = []
    integral_angular_systems = []

    if type(params) is list:
        name = params[0].name
        for index, param in enumerate(params):
            angular_systems.append(
                AngularSystemCurrent(param, param.name + str(index)))
            integral_angular_systems.append(
                IntegralAngularSystemCurrent(
                    param, 'Integral' + param.name + str(index)))
    else:
        name = params.name
        angular_systems.append(AngularSystemCurrent(params, params.name))
        integral_angular_systems.append(
            IntegralAngularSystemCurrent(params, 'Integral' + params.name))

    loop_writer = control_loop.ControlLoopWriter(name,
                                                 angular_systems,
                                                 namespaces=year_namespaces,
                                                 plant_type=plant_type,
                                                 observer_type=observer_type)
    loop_writer.AddConstant(
        control_loop.Constant('kOutputRatio', '%f', angular_systems[0].G))
    loop_writer.AddConstant(
        control_loop.Constant('kFreeSpeed', '%f',
                              angular_systems[0].motor.free_speed))
    loop_writer.Write(plant_files[0], plant_files[1])

    integral_loop_writer = control_loop.ControlLoopWriter(
        'Integral' + name,
        integral_angular_systems,
        namespaces=year_namespaces,
        plant_type=plant_type,
        observer_type=observer_type)
    integral_loop_writer.Write(controller_files[0], controller_files[1])
