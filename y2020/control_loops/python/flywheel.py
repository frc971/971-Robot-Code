from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
from matplotlib import pylab

import glog


class FlywheelParams(object):
    def __init__(self,
                 name,
                 motor,
                 G,
                 J,
                 q_pos,
                 q_vel,
                 q_voltage,
                 r_pos,
                 controller_poles,
                 dt=0.00505):
        self.name = name
        self.motor = motor
        self.G = G
        self.J = J
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.q_voltage = q_voltage
        self.r_pos = r_pos
        self.dt = dt
        self.controller_poles = controller_poles


class VelocityFlywheel(control_loop.HybridControlLoop):
    def __init__(self, params, name="Flywheel"):
        super(VelocityFlywheel, self).__init__(name=name)
        self.params = params
        # Set Motor
        self.motor = self.params.motor
        # Moment of inertia of the flywheel wheel in kg m^2
        self.J = self.params.J
        # Gear ratio
        self.G = self.params.G
        # Control loop time step
        self.dt = self.params.dt

        # State feedback matrices
        # [angular velocity]
        self.A_continuous = numpy.matrix([[
            -self.motor.Kt / self.motor.Kv /
            (self.J * self.G * self.G * self.motor.resistance)
        ]])
        self.B_continuous = numpy.matrix(
            [[self.motor.Kt / (self.J * self.G * self.motor.resistance)]])
        self.C = numpy.matrix([[1]])
        self.D = numpy.matrix([[0]])

        self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                                   self.B_continuous, self.dt)

        self.PlaceControllerPoles(self.params.controller_poles)

        # Generated controller not used.
        self.PlaceObserverPoles([0.3])

        self.U_max = numpy.matrix([[12.0]])
        self.U_min = numpy.matrix([[-12.0]])

        qff_vel = 8.0
        self.Qff = numpy.matrix([[1.0 / (qff_vel**2.0)]])

        self.Kff = controls.TwoStateFeedForwards(self.B, self.Qff)


class Flywheel(VelocityFlywheel):
    def __init__(self, params, name="Flywheel"):
        super(Flywheel, self).__init__(params, name=name)

        self.A_continuous_unaugmented = self.A_continuous
        self.B_continuous_unaugmented = self.B_continuous

        self.A_continuous = numpy.matrix(numpy.zeros((2, 2)))
        self.A_continuous[1:2, 1:2] = self.A_continuous_unaugmented
        self.A_continuous[0, 1] = 1

        self.B_continuous = numpy.matrix(numpy.zeros((2, 1)))
        self.B_continuous[1:2, 0] = self.B_continuous_unaugmented

        # State feedback matrices
        # [position, angular velocity]
        self.C = numpy.matrix([[1, 0]])
        self.D = numpy.matrix([[0]])

        self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                                   self.B_continuous, self.dt)

        rpl = 0.45
        ipl = 0.07
        self.PlaceObserverPoles([rpl + 1j * ipl, rpl - 1j * ipl])

        self.K_unaugmented = self.K
        self.K = numpy.matrix(numpy.zeros((1, 2)))
        self.K[0, 1:2] = self.K_unaugmented
        self.Kff_unaugmented = self.Kff
        self.Kff = numpy.matrix(numpy.zeros((1, 2)))
        self.Kff[0, 1:2] = self.Kff_unaugmented

        self.InitializeState()


class IntegralFlywheel(Flywheel):
    def __init__(self, params, name="IntegralFlywheel"):
        super(IntegralFlywheel, self).__init__(params, name=name)

        self.A_continuous_unaugmented = self.A_continuous
        self.B_continuous_unaugmented = self.B_continuous

        self.A_continuous = numpy.matrix(numpy.zeros((3, 3)))
        self.A_continuous[0:2, 0:2] = self.A_continuous_unaugmented
        self.A_continuous[0:2, 2] = self.B_continuous_unaugmented

        self.B_continuous = numpy.matrix(numpy.zeros((3, 1)))
        self.B_continuous[0:2, 0] = self.B_continuous_unaugmented


        # states
        # [position, velocity, voltage_error]
        self.C_unaugmented = self.C
        self.C = numpy.matrix(numpy.zeros((1, 3)))
        self.C[0:1, 0:2] = self.C_unaugmented

        glog.debug('A_continuous %s' % str(self.A_continuous))
        glog.debug('B_continuous %s' % str(self.B_continuous))
        glog.debug('C %s' % str(self.C))

        self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                                   self.B_continuous, self.dt)

        glog.debug('A %s' % str(self.A))
        glog.debug('B %s' % str(self.B))

        q_pos = self.params.q_pos
        q_vel = self.params.q_vel
        q_voltage = self.params.q_voltage
        self.Q_continuous = numpy.matrix([[(q_pos**2.0), 0.0, 0.0],
                               [0.0, (q_vel**2.0), 0.0],
                               [0.0, 0.0, (q_voltage**2.0)]])

        r_pos = self.params.r_pos
        self.R_continuous = numpy.matrix([[(r_pos**2.0)]])

        _, _, self.Q, self.R = controls.kalmd(
            A_continuous=self.A_continuous,
            B_continuous=self.B_continuous,
            Q_continuous=self.Q_continuous,
            R_continuous=self.R_continuous,
            dt=self.dt)

        glog.debug('Q_discrete %s' % (str(self.Q)))
        glog.debug('R_discrete %s' % (str(self.R)))

        self.KalmanGain, self.P_steady_state = controls.kalman(
            A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)
        self.L = self.A * self.KalmanGain

        self.K_unaugmented = self.K
        self.K = numpy.matrix(numpy.zeros((1, 3)))
        self.K[0, 0:2] = self.K_unaugmented
        self.K[0, 2] = 1
        self.Kff_unaugmented = self.Kff
        self.Kff = numpy.matrix(numpy.zeros((1, 3)))
        self.Kff[0, 0:2] = self.Kff_unaugmented

        self.InitializeState()


def PlotSpinup(params, goal, iterations=400):
    """Runs the flywheel plant with an initial condition and goal.

    Args:
        flywheel: Flywheel object to use.
        goal: goal state.
        iterations: Number of timesteps to run the model for.
        controller_flywheel: Flywheel object to get K from, or None if we should
             use flywheel.
        observer_flywheel: Flywheel object to use for the observer, or None if we
            should use the actual state.
    """

    # Various lists for graphing things.
    t = []
    x = []
    v = []
    a = []
    x_hat = []
    u = []
    offset = []

    flywheel = Flywheel(params, params.name)
    controller_flywheel = IntegralFlywheel(params, params.name)
    observer_flywheel = IntegralFlywheel(params, params.name)
    vbat = 12.0

    if t:
        initial_t = t[-1] + flywheel.dt
    else:
        initial_t = 0

    for i in range(iterations):
        X_hat = flywheel.X

        if observer_flywheel is not None:
            X_hat = observer_flywheel.X_hat
            x_hat.append(observer_flywheel.X_hat[1, 0])

        ff_U = controller_flywheel.Kff * (goal - observer_flywheel.A * goal)

        U = controller_flywheel.K * (goal - X_hat) + ff_U
        U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)
        x.append(flywheel.X[0, 0])

        if v:
            last_v = v[-1]
        else:
            last_v = 0

        v.append(flywheel.X[1, 0])
        a.append((v[-1] - last_v) / flywheel.dt)

        if observer_flywheel is not None:
            observer_flywheel.Y = flywheel.Y
            observer_flywheel.CorrectHybridObserver(U)
            offset.append(observer_flywheel.X_hat[2, 0])

        applied_U = U.copy()
        if i > 200:
            applied_U += 2
        flywheel.Update(applied_U)

        if observer_flywheel is not None:
            observer_flywheel.PredictHybridObserver(U, flywheel.dt)

        t.append(initial_t + i * flywheel.dt)
        u.append(U[0, 0])

    pylab.subplot(3, 1, 1)
    pylab.plot(t, v, label='x')
    pylab.plot(t, x_hat, label='x_hat')
    pylab.legend()

    pylab.subplot(3, 1, 2)
    pylab.plot(t, u, label='u')
    pylab.plot(t, offset, label='voltage_offset')
    pylab.legend()

    pylab.subplot(3, 1, 3)
    pylab.plot(t, a, label='a')
    pylab.legend()

    pylab.show()


def WriteFlywheel(params, plant_files, controller_files, namespace):
    """Writes out the constants for a flywheel to a file.

    Args:
      params: list of Flywheel Params, the
        parameters defining the system.
      plant_files: list of strings, the cc and h files for the plant.
      controller_files: list of strings, the cc and h files for the integral
        controller.
      namespaces: list of strings, the namespace list to use.
    """
    # Write the generated constants out to a file.
    flywheels = []
    integral_flywheels = []

    if type(params) is list:
        name = params[0].name
        for index, param in enumerate(params):
            flywheels.append(Flywheel(param, name=param.name + str(index)))
            integral_flywheels.append(
                IntegralFlywheel(
                    param, name='Integral' + param.name + str(index)))
    else:
        name = params.name
        flywheels.append(Flywheel(params, params.name))
        integral_flywheels.append(
            IntegralFlywheel(params, name='Integral' + params.name))

    loop_writer = control_loop.ControlLoopWriter(
        name, flywheels, namespaces=namespace)
    loop_writer.AddConstant(
        control_loop.Constant('kOutputRatio', '%f',
                              flywheels[0].G))
    loop_writer.AddConstant(
        control_loop.Constant('kFreeSpeed', '%f',
                              flywheels[0].motor.free_speed))
    loop_writer.AddConstant(
        control_loop.Constant(
            'kBemf',
            '%f',
            flywheels[0].motor.Kv * flywheels[0].G,
            comment="// Radians/sec / volt"))
    loop_writer.AddConstant(
        control_loop.Constant(
            'kResistance',
            '%f',
            flywheels[0].motor.resistance,
            comment="// Ohms"))
    loop_writer.Write(plant_files[0], plant_files[1])

    integral_loop_writer = control_loop.ControlLoopWriter(
        'Integral' + name,
        integral_flywheels,
        namespaces=namespace,
        plant_type='StateFeedbackHybridPlant',
        observer_type='HybridKalman')
    integral_loop_writer.Write(controller_files[0], controller_files[1])
