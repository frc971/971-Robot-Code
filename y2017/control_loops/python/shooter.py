#!/usr/bin/python

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
import scipy
import sys
from matplotlib import pylab

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')


def PlotDiff(list1, list2, time):
  pylab.subplot(1, 1, 1)
  pylab.plot(time, numpy.subtract(list1, list2), label='diff')
  pylab.legend()

class VelocityShooter(control_loop.HybridControlLoop):
  def __init__(self, name='VelocityShooter'):
    super(VelocityShooter, self).__init__(name)
    # Number of motors
    self.num_motors = 2.0
    # Stall Torque in N m
    self.stall_torque = 0.71 * self.num_motors
    # Stall Current in Amps
    self.stall_current = 134.0 * self.num_motors
    # Free Speed in RPM
    self.free_speed_rpm = 18730.0
    # Free Speed in rotations/second.
    self.free_speed = self.free_speed_rpm / 60.0
    # Free Current in Amps
    self.free_current = 0.7 * self.num_motors
    # Moment of inertia of the shooter wheel in kg m^2
    # 1400.6 grams/cm^2
    # 1.407 *1e-4 kg m^2
    self.J = 0.00120
    # Resistance of the motor, divided by 2 to account for the 2 motors
    self.R = 12.0 / self.stall_current
    # Motor velocity constant
    self.Kv = ((self.free_speed * 2.0 * numpy.pi) /
              (12.0 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratio
    self.G = 12.0 / 36.0
    # Control loop time step
    self.dt = 0.00505

    # State feedback matrices
    # [angular velocity]
    self.A_continuous = numpy.matrix(
        [[-self.Kt / (self.Kv * self.J * self.G * self.G * self.R)]])
    self.B_continuous = numpy.matrix(
        [[self.Kt / (self.J * self.G * self.R)]])
    self.C = numpy.matrix([[1]])
    self.D = numpy.matrix([[0]])

    # The states are [unfiltered_velocity]
    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    self.PlaceControllerPoles([.75])

    glog.debug('K %s', repr(self.K))
    glog.debug('System poles are %s',
               repr(numpy.linalg.eig(self.A_continuous)[0]))
    glog.debug('Poles are %s',
               repr(numpy.linalg.eig(self.A - self.B * self.K)[0]))

    self.PlaceObserverPoles([0.3])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    qff_vel = 8.0
    self.Qff = numpy.matrix([[1.0 / (qff_vel ** 2.0)]])

    self.Kff = controls.TwoStateFeedForwards(self.B, self.Qff)
    self.InitializeState()

class SecondOrderVelocityShooter(VelocityShooter):
  def __init__(self, name='SecondOrderVelocityShooter'):
    super(SecondOrderVelocityShooter, self).__init__(name)

    self.A_continuous_unaugmented = self.A_continuous
    self.B_continuous_unaugmented = self.B_continuous

    self.A_continuous = numpy.matrix(numpy.zeros((2, 2)))
    self.A_continuous[0:1, 0:1] = self.A_continuous_unaugmented
    self.A_continuous[1, 0] = 175.0
    self.A_continuous[1, 1] = -self.A_continuous[1, 0]

    self.B_continuous = numpy.matrix(numpy.zeros((2, 1)))
    self.B_continuous[0:1, 0] = self.B_continuous_unaugmented

    self.C = numpy.matrix([[0, 1]])
    self.D = numpy.matrix([[0]])

    # The states are [unfiltered_velocity, velocity]
    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    self.PlaceControllerPoles([.70, 0.60])

    q_vel = 40.0
    q_filteredvel = 30.0
    self.Q = numpy.matrix([[(1.0 / (q_vel ** 2.0)), 0.0],
                           [0.0, (1.0 / (q_filteredvel ** 2.0))]])

    self.R = numpy.matrix([[(1.0 / (3.0 ** 2.0))]])
    self.K = controls.dlqr(self.A, self.B, self.Q, self.R)

    glog.debug('K %s', repr(self.K))
    glog.debug('System poles are %s',
               repr(numpy.linalg.eig(self.A_continuous)[0]))
    glog.debug('Poles are %s',
               repr(numpy.linalg.eig(self.A - self.B * self.K)[0]))

    self.PlaceObserverPoles([0.3, 0.3])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    qff_vel = 8.0
    self.Qff = numpy.matrix([[1.0 / (qff_vel ** 2.0), 0.0],
                             [0.0, 1.0 / (qff_vel ** 2.0)]])

    self.Kff = controls.TwoStateFeedForwards(self.B, self.Qff)
    self.InitializeState()


class Shooter(SecondOrderVelocityShooter):
  def __init__(self, name='Shooter'):
    super(Shooter, self).__init__(name)

    self.A_continuous_unaugmented = self.A_continuous
    self.B_continuous_unaugmented = self.B_continuous

    self.A_continuous = numpy.matrix(numpy.zeros((3, 3)))
    self.A_continuous[1:3, 1:3] = self.A_continuous_unaugmented
    self.A_continuous[0, 2] = 1

    self.B_continuous = numpy.matrix(numpy.zeros((3, 1)))
    self.B_continuous[1:3, 0] = self.B_continuous_unaugmented

    # State feedback matrices
    # [position, unfiltered_velocity, angular velocity]
    self.C = numpy.matrix([[1, 0, 0]])
    self.D = numpy.matrix([[0]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)
    glog.debug(repr(self.A_continuous))
    glog.debug(repr(self.B_continuous))

    observeability = controls.ctrb(self.A.T, self.C.T)
    glog.debug('Rank of augmented observability matrix. %d', numpy.linalg.matrix_rank(
            observeability))


    self.PlaceObserverPoles([0.9, 0.8, 0.7])

    self.K_unaugmented = self.K
    self.K = numpy.matrix(numpy.zeros((1, 3)))
    self.K[0, 1:3] = self.K_unaugmented
    self.Kff_unaugmented = self.Kff
    self.Kff = numpy.matrix(numpy.zeros((1, 3)))
    self.Kff[0, 1:3] = self.Kff_unaugmented

    self.InitializeState()


class IntegralShooter(Shooter):
  def __init__(self, name='IntegralShooter'):
    super(IntegralShooter, self).__init__(name=name)

    self.A_continuous_unaugmented = self.A_continuous
    self.B_continuous_unaugmented = self.B_continuous

    self.A_continuous = numpy.matrix(numpy.zeros((4, 4)))
    self.A_continuous[0:3, 0:3] = self.A_continuous_unaugmented
    self.A_continuous[0:3, 3] = self.B_continuous_unaugmented

    self.B_continuous = numpy.matrix(numpy.zeros((4, 1)))
    self.B_continuous[0:3, 0] = self.B_continuous_unaugmented

    self.C_unaugmented = self.C
    self.C = numpy.matrix(numpy.zeros((1, 4)))
    self.C[0:1, 0:3] = self.C_unaugmented

    # The states are [position, unfiltered_velocity, velocity, torque_error]
    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    glog.debug('A: \n%s', repr(self.A_continuous))
    glog.debug('eig(A): \n%s', repr(scipy.linalg.eig(self.A_continuous)))
    glog.debug('schur(A): \n%s', repr(scipy.linalg.schur(self.A_continuous)))
    glog.debug('A_dt(A): \n%s', repr(self.A))

    q_pos = 0.01
    q_vel = 5.0
    q_velfilt = 1.5
    q_voltage = 2.0
    self.Q_continuous = numpy.matrix([[(q_pos ** 2.0), 0.0, 0.0, 0.0],
                                      [0.0, (q_vel ** 2.0), 0.0, 0.0],
                                      [0.0, 0.0, (q_velfilt ** 2.0), 0.0],
                                      [0.0, 0.0, 0.0, (q_voltage ** 2.0)]])

    r_pos = 0.0003
    self.R_continuous = numpy.matrix([[(r_pos ** 2.0)]])

    _, _, self.Q, self.R = controls.kalmd(
        A_continuous=self.A_continuous, B_continuous=self.B_continuous,
        Q_continuous=self.Q_continuous, R_continuous=self.R_continuous,
        dt=self.dt)

    self.KalmanGain, self.P_steady_state = controls.kalman(
        A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)
    self.L = self.A * self.KalmanGain

    self.K_unaugmented = self.K
    self.K = numpy.matrix(numpy.zeros((1, 4)))
    self.K[0, 0:3] = self.K_unaugmented
    self.K[0, 3] = 1
    self.Kff_unaugmented = self.Kff
    self.Kff = numpy.matrix(numpy.zeros((1, 4)))
    self.Kff[0, 0:3] = self.Kff_unaugmented

    self.InitializeState()


class ScenarioPlotter(object):
  def __init__(self):
    # Various lists for graphing things.
    self.t = []
    self.x = []
    self.v = []
    self.a = []
    self.x_hat = []
    self.u = []
    self.offset = []
    self.diff = []

  def run_test(self, shooter, goal, iterations=200, controller_shooter=None,
             observer_shooter=None, hybrid_obs = False):
    """Runs the shooter plant with an initial condition and goal.

      Args:
        shooter: Shooter object to use.
        goal: goal state.
        iterations: Number of timesteps to run the model for.
        controller_shooter: Shooter object to get K from, or None if we should
            use shooter.
        observer_shooter: Shooter object to use for the observer, or None if we
            should use the actual state.
    """

    if controller_shooter is None:
      controller_shooter = shooter

    vbat = 12.0

    if self.t:
      initial_t = self.t[-1] + shooter.dt
    else:
      initial_t = 0

    last_U = numpy.matrix([[0.0]])
    for i in xrange(iterations):
      X_hat = shooter.X

      if observer_shooter is not None:
        X_hat = observer_shooter.X_hat
        self.x_hat.append(observer_shooter.X_hat[2, 0])

      ff_U = controller_shooter.Kff * (goal - observer_shooter.A * goal)

      U = controller_shooter.K * (goal - X_hat) + ff_U
      U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)
      self.x.append(shooter.X[0, 0])

      self.diff.append(shooter.X[2, 0] - observer_shooter.X_hat[2, 0])

      if self.v:
        last_v = self.v[-1]
      else:
        last_v = 0

      self.v.append(shooter.X[2, 0])
      self.a.append((self.v[-1] - last_v) / shooter.dt)

      if observer_shooter is not None:
        if i != 0:
          observer_shooter.Y = shooter.Y
          observer_shooter.CorrectObserver(U)
        self.offset.append(observer_shooter.X_hat[3, 0])

      applied_U = last_U.copy()
      if i > 60:
        applied_U += 2
      shooter.Update(applied_U)

      if observer_shooter is not None:
        if hybrid_obs:
          observer_shooter.PredictHybridObserver(last_U, shooter.dt)
        else:
          observer_shooter.PredictObserver(last_U)
      last_U = U.copy()


      self.t.append(initial_t + i * shooter.dt)
      self.u.append(U[0, 0])

  def Plot(self):
    pylab.figure()
    pylab.subplot(3, 1, 1)
    pylab.plot(self.t, self.v, label='x')
    pylab.plot(self.t, self.x_hat, label='x_hat')
    pylab.legend()

    pylab.subplot(3, 1, 2)
    pylab.plot(self.t, self.u, label='u')
    pylab.plot(self.t, self.offset, label='voltage_offset')
    pylab.legend()

    pylab.subplot(3, 1, 3)
    pylab.plot(self.t, self.a, label='a')
    pylab.legend()

    pylab.draw()


def main(argv):
  scenario_plotter = ScenarioPlotter()

  if FLAGS.plot:
    iterations = 200

    initial_X = numpy.matrix([[0.0], [0.0], [0.0]])
    R = numpy.matrix([[0.0], [100.0], [100.0], [0.0]])

    scenario_plotter_int = ScenarioPlotter()

    shooter = Shooter()
    shooter_controller = IntegralShooter()
    observer_shooter_hybrid = IntegralShooter()

    scenario_plotter_int.run_test(shooter, goal=R, controller_shooter=shooter_controller,
      observer_shooter=observer_shooter_hybrid, iterations=iterations,
      hybrid_obs = True)

    scenario_plotter_int.Plot()

    pylab.show()

  if len(argv) != 5:
    glog.fatal('Expected .h file name and .cc file name')
  else:
    namespaces = ['y2017', 'control_loops', 'superstructure', 'shooter']
    shooter = Shooter('Shooter')
    loop_writer = control_loop.ControlLoopWriter('Shooter', [shooter],
                                                 namespaces=namespaces)
    loop_writer.AddConstant(control_loop.Constant(
        'kFreeSpeed', '%f', shooter.free_speed))
    loop_writer.AddConstant(control_loop.Constant(
        'kOutputRatio', '%f', shooter.G))
    loop_writer.Write(argv[1], argv[2])

    integral_shooter = IntegralShooter('IntegralShooter')
    integral_loop_writer = control_loop.ControlLoopWriter(
        'IntegralShooter', [integral_shooter], namespaces=namespaces,
        plant_type='StateFeedbackHybridPlant',
        observer_type='HybridKalman')
    integral_loop_writer.Write(argv[3], argv[4])


if __name__ == '__main__':
  argv = FLAGS(sys.argv)
  glog.init()
  sys.exit(main(argv))
