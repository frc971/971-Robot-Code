from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
from matplotlib import pylab

import gflags
import glog


class CatapultParams(object):
    def __init__(self,
                 name,
                 motor,
                 G,
                 J,
                 lever,
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
        self.lever = lever
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.q_voltage = q_voltage
        self.r_pos = r_pos
        self.dt = dt
        self.controller_poles = controller_poles


class VelocityCatapult(control_loop.HybridControlLoop):
    def __init__(self, params, name="Catapult"):
        super(VelocityCatapult, self).__init__(name=name)
        self.params = params
        # Set Motor
        self.motor = self.params.motor
        # Gear ratio
        self.G = self.params.G
        # Moment of inertia of the catapult wheel in kg m^2
        self.J = self.params.J + self.motor.motor_inertia / (self.G**2.0)
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

        glog.debug('K: %s', str(self.K))
        glog.debug('Poles: %s',
                   str(numpy.linalg.eig(self.A - self.B * self.K)[0]))


class Catapult(VelocityCatapult):
    def __init__(self, params, name="Catapult"):
        super(Catapult, self).__init__(params, name=name)

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


class IntegralCatapult(Catapult):
    def __init__(self, params, name="IntegralCatapult"):
        super(IntegralCatapult, self).__init__(params, name=name)

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
        self.Q_continuous = numpy.matrix([[(q_pos**2.0), 0.0,
                                           0.0], [0.0, (q_vel**2.0), 0.0],
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


def MaxSpeed(params, U, final_position):
    """Runs the catapult plant with an initial condition and goal.

    Args:
        catapult: Catapult object to use.
        goal: goal state.
        iterations: Number of timesteps to run the model for.
        controller_catapult: Catapult object to get K from, or None if we should
             use catapult.
        observer_catapult: Catapult object to use for the observer, or None if we
            should use the actual state.
    """

    # Various lists for graphing things.
    catapult = Catapult(params, params.name)
    controller_catapult = IntegralCatapult(params, params.name)
    observer_catapult = IntegralCatapult(params, params.name)
    vbat = 12.0

    while True:
        X_hat = catapult.X
        if catapult.X[0, 0] > final_position:
            return catapult.X[1, 0] * params.lever

        if observer_catapult is not None:
            X_hat = observer_catapult.X_hat

        U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)

        if observer_catapult is not None:
            observer_catapult.Y = catapult.Y
            observer_catapult.CorrectHybridObserver(U)

        applied_U = U.copy()
        catapult.Update(applied_U)

        if observer_catapult is not None:
            observer_catapult.PredictHybridObserver(U, catapult.dt)


def PlotShot(params, U, final_position):
    """Runs the catapult plant with an initial condition and goal.

    Args:
        catapult: Catapult object to use.
        goal: goal state.
        iterations: Number of timesteps to run the model for.
        controller_catapult: Catapult object to get K from, or None if we should
             use catapult.
        observer_catapult: Catapult object to use for the observer, or None if we
            should use the actual state.
    """

    # Various lists for graphing things.
    t = []
    x = []
    x_hat = []
    v = []
    w_hat = []
    v_hat = []
    a = []
    u = []
    offset = []

    catapult = Catapult(params, params.name)
    controller_catapult = IntegralCatapult(params, params.name)
    observer_catapult = IntegralCatapult(params, params.name)
    vbat = 12.0

    if t:
        initial_t = t[-1] + catapult.dt
    else:
        initial_t = 0

    for i in range(10000):
        X_hat = catapult.X
        if catapult.X[0, 0] > final_position:
            break

        if observer_catapult is not None:
            X_hat = observer_catapult.X_hat
            x_hat.append(observer_catapult.X_hat[0, 0])
            w_hat.append(observer_catapult.X_hat[1, 0])
            v_hat.append(observer_catapult.X_hat[1, 0] * params.lever)

        U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)
        x.append(catapult.X[0, 0])

        if v:
            last_v = v[-1]
        else:
            last_v = 0

        v.append(catapult.X[1, 0])
        a.append((v[-1] - last_v) / catapult.dt)

        if observer_catapult is not None:
            observer_catapult.Y = catapult.Y
            observer_catapult.CorrectHybridObserver(U)
            offset.append(observer_catapult.X_hat[2, 0])

        catapult.Update(U)

        if observer_catapult is not None:
            observer_catapult.PredictHybridObserver(U, catapult.dt)

        t.append(initial_t + i * catapult.dt)
        u.append(U[0, 0])

    pylab.subplot(3, 1, 1)
    pylab.plot(t, v, label='v')
    pylab.plot(t, x_hat, label='x_hat')
    pylab.plot(t, v, label='v')
    pylab.plot(t, v_hat, label='v_hat')
    pylab.plot(t, w_hat, label='w_hat')
    pylab.legend()

    pylab.subplot(3, 1, 2)
    pylab.plot(t, u, label='u')
    pylab.plot(t, offset, label='voltage_offset')
    pylab.legend()

    pylab.subplot(3, 1, 3)
    pylab.plot(t, a, label='a')
    pylab.legend()

    pylab.show()
