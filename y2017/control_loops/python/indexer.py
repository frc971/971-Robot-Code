#!/usr/bin/python

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
import sys
from matplotlib import pylab

import gflags
import glog

FLAGS = gflags.FLAGS

try:
  gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
  pass

gflags.DEFINE_bool('stall', False, 'If true, stall the indexer.')

class VelocityIndexer(control_loop.ControlLoop):
  def __init__(self, name='VelocityIndexer'):
    super(VelocityIndexer, self).__init__(name)
    # Stall Torque in N m
    self.stall_torque = 0.71
    # Stall Current in Amps
    self.stall_current = 134
    self.free_speed_rpm = 18730.0
    # Free Speed in rotations/second.
    self.free_speed = self.free_speed_rpm / 60.0
    # Free Current in Amps
    self.free_current = 0.7
    # Moment of inertia of the indexer halves in kg m^2
    # This is measured as Iyy in CAD (the moment of inertia around the Y axis).
    # Inner part of indexer -> Iyy = 59500 lb * mm * mm
    # Inner spins with 12 / 48 * 18 / 48 * 24 / 36 * 16 / 72
    # Outer part of indexer -> Iyy = 210000 lb * mm * mm
    # 1 775 pro -> 12 / 48 * 18 / 48 * 30 / 422

    self.J_inner = 0.0269
    self.J_outer = 0.0952
    # Gear ratios for the inner and outer parts.
    self.G_inner = (12.0 / 48.0) * (20.0 / 34.0) * (18.0 / 36.0) * (12.0 / 84.0)
    self.G_outer = (12.0 / 48.0) * (20.0 / 34.0) * (18.0 / 36.0) * (24.0 / 420.0)

    # Motor inertia in kg m^2
    self.motor_inertia = 0.00001187

    # The output coordinate system is in radians for the inner part of the
    # indexer.
    # Compute the effective moment of inertia assuming all the mass is in that
    # coordinate system.
    self.J = (
        self.J_inner * self.G_inner * self.G_inner +
        self.J_outer * self.G_outer * self.G_outer) / (self.G_inner * self.G_inner) + \
        self.motor_inertia * ((1.0 / self.G_inner) ** 2.0)
    glog.debug('Indexer J is %f', self.J)
    self.G = self.G_inner

    # Resistance of the motor, divided by 2 to account for the 2 motors
    self.resistance = 12.0 / self.stall_current
    # Motor velocity constant
    self.Kv = ((self.free_speed * 2.0 * numpy.pi) /
              (12.0 - self.resistance * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Control loop time step
    self.dt = 0.005

    # State feedback matrices
    # [angular velocity]
    self.A_continuous = numpy.matrix(
        [[-self.Kt / self.Kv / (self.J * self.G * self.G * self.resistance)]])
    self.B_continuous = numpy.matrix(
        [[self.Kt / (self.J * self.G * self.resistance)]])
    self.C = numpy.matrix([[1]])
    self.D = numpy.matrix([[0]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    self.PlaceControllerPoles([.75])

    self.PlaceObserverPoles([0.3])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    qff_vel = 8.0
    self.Qff = numpy.matrix([[1.0 / (qff_vel ** 2.0)]])

    self.Kff = controls.TwoStateFeedForwards(self.B, self.Qff)
    self.InitializeState()


class Indexer(VelocityIndexer):
  def __init__(self, name='Indexer'):
    super(Indexer, self).__init__(name)

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

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    self.rpl = .45
    self.ipl = 0.07
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    self.K_unaugmented = self.K
    self.K = numpy.matrix(numpy.zeros((1, 2)))
    self.K[0, 1:2] = self.K_unaugmented
    self.Kff_unaugmented = self.Kff
    self.Kff = numpy.matrix(numpy.zeros((1, 2)))
    self.Kff[0, 1:2] = self.Kff_unaugmented

    self.InitializeState()


class IntegralIndexer(Indexer):
  def __init__(self, name="IntegralIndexer", voltage_error_noise=None):
    super(IntegralIndexer, self).__init__(name=name)

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

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    q_pos = 0.01
    q_vel = 2.0
    q_voltage = 0.6
    if voltage_error_noise is not None:
      q_voltage = voltage_error_noise

    self.Q = numpy.matrix([[(q_pos ** 2.0), 0.0, 0.0],
                           [0.0, (q_vel ** 2.0), 0.0],
                           [0.0, 0.0, (q_voltage ** 2.0)]])

    r_pos = 0.001
    self.R = numpy.matrix([[(r_pos ** 2.0)]])

    self.KalmanGain, self.Q_steady = controls.kalman(
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


class ScenarioPlotter(object):
  def __init__(self):
    # Various lists for graphing things.
    self.t = []
    self.x = []
    self.v = []
    self.a = []
    self.stall_ratio = []
    self.x_hat = []
    self.u = []
    self.offset = []

  def run_test(self, indexer, goal, iterations=200, controller_indexer=None,
             observer_indexer=None):
    """Runs the indexer plant with an initial condition and goal.

      Args:
        indexer: Indexer object to use.
        goal: goal state.
        iterations: Number of timesteps to run the model for.
        controller_indexer: Indexer object to get K from, or None if we should
            use indexer.
        observer_indexer: Indexer object to use for the observer, or None if we
            should use the actual state.
    """

    if controller_indexer is None:
      controller_indexer = indexer

    vbat = 12.0

    if self.t:
      initial_t = self.t[-1] + indexer.dt
    else:
      initial_t = 0

    for i in xrange(iterations):
      X_hat = indexer.X

      if observer_indexer is not None:
        X_hat = observer_indexer.X_hat
        observer_indexer.Y = indexer.Y
        observer_indexer.CorrectObserver(numpy.matrix([[0.0]]))
        self.x_hat.append(observer_indexer.X_hat[1, 0])
        self.offset.append(observer_indexer.X_hat[2, 0])

      ff_U = controller_indexer.Kff * (goal - observer_indexer.A * goal)

      U = controller_indexer.K * (goal - X_hat) + ff_U
      U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)
      self.x.append(indexer.X[0, 0])

      if self.v:
        last_v = self.v[-1]
      else:
        last_v = 0

      self.v.append(indexer.X[1, 0])
      self.a.append((self.v[-1] - last_v) / indexer.dt)

      applied_U = U.copy()
      if i >= 40:
        applied_U -= 2

      if FLAGS.stall and i >= 40:
        indexer.X[1, 0] = 0.0
      else:
        indexer.Update(applied_U)

      if observer_indexer is not None:
        clipped_u = U[0, 0]
        clip_u_value = 3.0
        if clipped_u < 0:
          clipped_u = min(clipped_u, -clip_u_value)
        else:
          clipped_u = max(clipped_u, clip_u_value)

        self.stall_ratio.append(10 * (-self.offset[-1] / clipped_u))

        observer_indexer.PredictObserver(U)

      self.t.append(initial_t + i * indexer.dt)
      self.u.append(U[0, 0])

  def Plot(self):
    pylab.subplot(3, 1, 1)
    pylab.plot(self.t, self.v, label='x')
    pylab.plot(self.t, self.x_hat, label='x_hat')
    pylab.legend()

    pylab.subplot(3, 1, 2)
    pylab.plot(self.t, self.u, label='u')
    pylab.plot(self.t, self.offset, label='voltage_offset')
    pylab.plot(self.t, self.stall_ratio, label='stall_ratio')
    pylab.plot(self.t,
               [10.0 if x > 6.0 else 0.0 for x in self.stall_ratio],
               label='is_stalled')
    pylab.legend()

    pylab.subplot(3, 1, 3)
    pylab.plot(self.t, self.a, label='a')
    pylab.legend()

    pylab.show()


def main(argv):
  scenario_plotter = ScenarioPlotter()

  indexer = Indexer()
  indexer_controller = IntegralIndexer()
  observer_indexer = IntegralIndexer()

  initial_X = numpy.matrix([[0.0], [0.0]])
  R = numpy.matrix([[0.0], [20.0], [0.0]])
  scenario_plotter.run_test(indexer, goal=R, controller_indexer=indexer_controller,
                            observer_indexer=observer_indexer, iterations=200)

  if FLAGS.plot:
    scenario_plotter.Plot()

  scenario_plotter = ScenarioPlotter()

  indexer = Indexer()
  indexer_controller = IntegralIndexer(voltage_error_noise=1.5)
  observer_indexer = IntegralIndexer(voltage_error_noise=1.5)

  initial_X = numpy.matrix([[0.0], [0.0]])
  R = numpy.matrix([[0.0], [20.0], [0.0]])
  scenario_plotter.run_test(indexer, goal=R, controller_indexer=indexer_controller,
                            observer_indexer=observer_indexer, iterations=200)

  if FLAGS.plot:
    scenario_plotter.Plot()

  if len(argv) != 7:
    glog.fatal('Expected .h file name and .cc file names')
  else:
    namespaces = ['y2017', 'control_loops', 'superstructure', 'indexer']
    indexer = Indexer('Indexer')
    loop_writer = control_loop.ControlLoopWriter('Indexer', [indexer],
                                                 namespaces=namespaces)
    loop_writer.AddConstant(control_loop.Constant(
        'kFreeSpeed', '%f', indexer.free_speed))
    loop_writer.AddConstant(control_loop.Constant(
        'kOutputRatio', '%f', indexer.G))
    loop_writer.Write(argv[1], argv[2])

    integral_indexer = IntegralIndexer('IntegralIndexer')
    integral_loop_writer = control_loop.ControlLoopWriter(
        'IntegralIndexer', [integral_indexer], namespaces=namespaces)
    integral_loop_writer.Write(argv[3], argv[4])

    stuck_integral_indexer = IntegralIndexer('StuckIntegralIndexer',
                                             voltage_error_noise=1.5)
    stuck_integral_loop_writer = control_loop.ControlLoopWriter(
        'StuckIntegralIndexer', [stuck_integral_indexer], namespaces=namespaces)
    stuck_integral_loop_writer.Write(argv[5], argv[6])


if __name__ == '__main__':
  argv = FLAGS(sys.argv)
  glog.init()
  sys.exit(main(argv))
