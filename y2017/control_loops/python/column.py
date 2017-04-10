#!/usr/bin/python

from aos.common.util.trapezoid_profile import TrapezoidProfile
from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
from y2017.control_loops.python import turret
from y2017.control_loops.python import indexer
import numpy
import sys
import matplotlib
from matplotlib import pylab
import gflags
import glog

FLAGS = gflags.FLAGS

try:
  gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
  pass


class ColumnController(control_loop.ControlLoop):
  def __init__(self, name='Column'):
    super(ColumnController, self).__init__(name)
    self.turret = turret.Turret(name + 'Turret')
    self.indexer = indexer.Indexer(name + 'Indexer')

    # Control loop time step
    self.dt = 0.005

    # State is [position_indexer,
    #           velocity_indexer,
    #           position_shooter,
    #           velocity_shooter]
    # Input is [volts_indexer, volts_shooter]
    self.A_continuous = numpy.matrix(numpy.zeros((3, 3)))
    self.B_continuous = numpy.matrix(numpy.zeros((3, 2)))

    self.A_continuous[0, 0] = -(self.indexer.Kt / self.indexer.Kv / (self.indexer.J * self.indexer.resistance * self.indexer.G * self.indexer.G) +
                                self.turret.Kt / self.turret.Kv / (self.indexer.J * self.turret.resistance * self.turret.G * self.turret.G))
    self.A_continuous[0, 2] = self.turret.Kt / self.turret.Kv / (self.indexer.J * self.turret.resistance * self.turret.G * self.turret.G)
    self.B_continuous[0, 0] = self.indexer.Kt / (self.indexer.J * self.indexer.resistance * self.indexer.G)
    self.B_continuous[0, 1] = -self.turret.Kt / (self.indexer.J * self.turret.resistance * self.turret.G)

    self.A_continuous[1, 2] = 1

    self.A_continuous[2, 0] = self.turret.Kt / self.turret.Kv / (self.turret.J * self.turret.resistance * self.turret.G * self.turret.G)
    self.A_continuous[2, 2] = -self.turret.Kt / self.turret.Kv / (self.turret.J * self.turret.resistance * self.turret.G * self.turret.G)

    self.B_continuous[2, 1] = self.turret.Kt / (self.turret.J * self.turret.resistance * self.turret.G)

    self.C = numpy.matrix([[1, 0, 0], [0, 1, 0]])
    self.D = numpy.matrix([[0, 0], [0, 0]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    q_indexer_vel = 13.0
    q_pos = 0.05
    q_vel = 0.8
    self.Q = numpy.matrix([[(1.0 / (q_indexer_vel ** 2.0)), 0.0, 0.0],
                           [0.0, (1.0 / (q_pos ** 2.0)), 0.0],
                           [0.0, 0.0, (1.0 / (q_vel ** 2.0))]])

    self.R = numpy.matrix([[(1.0 / (12.0 ** 2.0)), 0.0],
                           [0.0, (1.0 / (12.0 ** 2.0))]])
    self.K = controls.dlqr(self.A, self.B, self.Q, self.R)

    glog.debug('Controller poles are ' + repr(numpy.linalg.eig(self.A - self.B * self.K)[0]))

    q_vel_indexer_ff = 0.000005
    q_pos_ff = 0.0000005
    q_vel_ff = 0.00008
    self.Qff = numpy.matrix([[(1.0 / (q_vel_indexer_ff ** 2.0)), 0.0, 0.0],
                             [0.0, (1.0 / (q_pos_ff ** 2.0)), 0.0],
                             [0.0, 0.0, (1.0 / (q_vel_ff ** 2.0))]])

    self.Kff = controls.TwoStateFeedForwards(self.B, self.Qff)

    self.U_max = numpy.matrix([[12.0], [12.0]])
    self.U_min = numpy.matrix([[-12.0], [-12.0]])

    self.InitializeState()


class Column(ColumnController):
  def __init__(self, name='Column', disable_indexer=False):
    super(Column, self).__init__(name)
    A_continuous = numpy.matrix(numpy.zeros((4, 4)))
    B_continuous = numpy.matrix(numpy.zeros((4, 2)))

    A_continuous[0, 1] = 1
    A_continuous[1:, 1:] = self.A_continuous
    B_continuous[1:, :] = self.B_continuous

    self.A_continuous = A_continuous
    self.B_continuous = B_continuous

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    self.C = numpy.matrix([[1, 0, 0, 0], [-1, 0, 1, 0]])
    self.D = numpy.matrix([[0, 0], [0, 0]])

    orig_K = self.K
    self.K = numpy.matrix(numpy.zeros((2, 4)))
    self.K[:, 1:] = orig_K

    glog.debug('K is ' + repr(self.K))
    # TODO(austin): Do we want to damp velocity out or not when disabled?
    #if disable_indexer:
    #  self.K[0, 1] = 0.0
    #  self.K[1, 1] = 0.0

    orig_Kff = self.Kff
    self.Kff = numpy.matrix(numpy.zeros((2, 4)))
    self.Kff[:, 1:] = orig_Kff

    q_pos = 0.12
    q_vel = 2.00
    self.Q = numpy.matrix([[(q_pos ** 2.0), 0.0, 0.0, 0.0],
                           [0.0, (q_vel ** 2.0), 0.0, 0.0],
                           [0.0, 0.0, (q_pos ** 2.0), 0.0],
                           [0.0, 0.0, 0.0, (q_vel ** 2.0)]])

    r_pos = 0.05
    self.R = numpy.matrix([[(r_pos ** 2.0), 0.0],
                           [0.0, (r_pos ** 2.0)]])

    self.KalmanGain, self.Q_steady = controls.kalman(
        A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)
    self.L = self.A * self.KalmanGain

    self.InitializeState()


class IntegralColumn(Column):
  def __init__(self, name='IntegralColumn', voltage_error_noise=None,
               disable_indexer=False):
    super(IntegralColumn, self).__init__(name)

    A_continuous = numpy.matrix(numpy.zeros((6, 6)))
    A_continuous[0:4, 0:4] = self.A_continuous
    A_continuous[0:4:, 4:6] = self.B_continuous

    B_continuous = numpy.matrix(numpy.zeros((6, 2)))
    B_continuous[0:4, :] = self.B_continuous

    self.A_continuous = A_continuous
    self.B_continuous = B_continuous

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    C = numpy.matrix(numpy.zeros((2, 6)))
    C[0:2, 0:4] = self.C
    self.C = C

    self.D = numpy.matrix([[0, 0], [0, 0]])

    orig_K = self.K
    self.K = numpy.matrix(numpy.zeros((2, 6)))
    self.K[:, 0:4] = orig_K

    # TODO(austin): I'm not certain this is ideal.  If someone spins the bottom
    # at a constant rate, we'll learn a voltage offset.  That should translate
    # directly to a voltage on the turret to hold it steady.  I'm also not
    # convinced we care that much.  If the indexer is off, it'll stop rather
    # quickly anyways, so this is mostly a moot point.
    if not disable_indexer:
      self.K[0, 4] = 1
    self.K[1, 5] = 1

    orig_Kff = self.Kff
    self.Kff = numpy.matrix(numpy.zeros((2, 6)))
    self.Kff[:, 0:4] = orig_Kff

    q_pos = 0.40
    q_vel = 2.00
    q_voltage = 8.0
    if voltage_error_noise is not None:
      q_voltage = voltage_error_noise

    self.Q = numpy.matrix([[(q_pos ** 2.0), 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, (q_vel ** 2.0), 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, (q_pos ** 2.0), 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, (q_vel ** 2.0), 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, (q_voltage ** 2.0), 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, (q_voltage ** 2.0)]])

    r_pos = 0.05
    self.R = numpy.matrix([[(r_pos ** 2.0), 0.0],
                           [0.0, (r_pos ** 2.0)]])

    self.KalmanGain, self.Q_steady = controls.kalman(
        A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)
    self.L = self.A * self.KalmanGain

    self.InitializeState()


class ScenarioPlotter(object):
  def __init__(self):
    # Various lists for graphing things.
    self.t = []
    self.xi = []
    self.xt = []
    self.vi = []
    self.vt = []
    self.ai = []
    self.at = []
    self.x_hat = []
    self.ui = []
    self.ut = []
    self.ui_fb = []
    self.ut_fb = []
    self.offseti = []
    self.offsett = []
    self.turret_error = []

  def run_test(self, column, end_goal,
             controller_column,
             observer_column=None,
             iterations=200):
    """Runs the column plant with an initial condition and goal.

      Args:
        column: column object to use.
        end_goal: end_goal state.
        controller_column: Intake object to get K from, or None if we should
            use column.
        observer_column: Intake object to use for the observer, or None if we should
            use the actual state.
        iterations: Number of timesteps to run the model for.
    """

    if controller_column is None:
      controller_column = column

    vbat = 12.0

    if self.t:
      initial_t = self.t[-1] + column.dt
    else:
      initial_t = 0

    goal = numpy.concatenate((column.X, numpy.matrix(numpy.zeros((2, 1)))), axis=0)

    profile = TrapezoidProfile(column.dt)
    profile.set_maximum_acceleration(10.0)
    profile.set_maximum_velocity(3.0)
    profile.SetGoal(goal[2, 0])

    U_last = numpy.matrix(numpy.zeros((2, 1)))
    for i in xrange(iterations):
      observer_column.Y = column.Y
      observer_column.CorrectObserver(U_last)

      self.offseti.append(observer_column.X_hat[4, 0])
      self.offsett.append(observer_column.X_hat[5, 0])
      self.x_hat.append(observer_column.X_hat[0, 0])

      next_goal = numpy.concatenate(
          (end_goal[0:2, :],
           profile.Update(end_goal[2, 0], end_goal[3, 0]),
           end_goal[4:6, :]),
          axis=0)

      ff_U = controller_column.Kff * (next_goal - observer_column.A * goal)
      fb_U = controller_column.K * (goal - observer_column.X_hat)
      self.turret_error.append((goal[2, 0] - column.X[2, 0]) * 100.0)
      self.ui_fb.append(fb_U[0, 0])
      self.ut_fb.append(fb_U[1, 0])

      U_uncapped = ff_U + fb_U

      U = U_uncapped.copy()
      U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)
      U[1, 0] = numpy.clip(U[1, 0], -vbat, vbat)
      self.xi.append(column.X[0, 0])
      self.xt.append(column.X[2, 0])

      if self.vi:
        last_vi = self.vi[-1]
      else:
        last_vi = 0
      if self.vt:
        last_vt = self.vt[-1]
      else:
        last_vt = 0

      self.vi.append(column.X[1, 0])
      self.vt.append(column.X[3, 0])
      self.ai.append((self.vi[-1] - last_vi) / column.dt)
      self.at.append((self.vt[-1] - last_vt) / column.dt)

      offset = 0.0
      if i > 100:
        offset = 1.0
      column.Update(U + numpy.matrix([[0.0], [offset]]))

      observer_column.PredictObserver(U)

      self.t.append(initial_t + i * column.dt)
      self.ui.append(U[0, 0])
      self.ut.append(U[1, 0])

      ff_U -= U_uncapped - U
      goal = controller_column.A * goal + controller_column.B * ff_U

      if U[1, 0] != U_uncapped[1, 0]:
        profile.MoveCurrentState(
            numpy.matrix([[goal[2, 0]], [goal[3, 0]]]))

    glog.debug('Time: %f', self.t[-1])
    glog.debug('goal_error %s', repr((end_goal - goal).T))
    glog.debug('error %s', repr((observer_column.X_hat - end_goal).T))

  def Plot(self):
    pylab.subplot(3, 1, 1)
    pylab.plot(self.t, self.xi, label='x_indexer')
    pylab.plot(self.t, self.xt, label='x_turret')
    pylab.plot(self.t, self.x_hat, label='x_hat')
    pylab.plot(self.t, self.turret_error, label='turret_error * 100')
    pylab.legend()

    pylab.subplot(3, 1, 2)
    pylab.plot(self.t, self.ui, label='u_indexer')
    pylab.plot(self.t, self.ui_fb, label='u_indexer_fb')
    pylab.plot(self.t, self.ut, label='u_turret')
    pylab.plot(self.t, self.ut_fb, label='u_turret_fb')
    pylab.plot(self.t, self.offseti, label='voltage_offset_indexer')
    pylab.plot(self.t, self.offsett, label='voltage_offset_turret')
    pylab.legend()

    pylab.subplot(3, 1, 3)
    pylab.plot(self.t, self.ai, label='a_indexer')
    pylab.plot(self.t, self.at, label='a_turret')
    pylab.plot(self.t, self.vi, label='v_indexer')
    pylab.plot(self.t, self.vt, label='v_turret')
    pylab.legend()

    pylab.show()


def main(argv):
  scenario_plotter = ScenarioPlotter()

  column = Column()
  column_controller = IntegralColumn()
  observer_column = IntegralColumn()

  initial_X = numpy.matrix([[0.0], [0.0], [0.0], [0.0]])
  R = numpy.matrix([[0.0], [10.0], [5.0], [0.0], [0.0], [0.0]])
  scenario_plotter.run_test(column, end_goal=R, controller_column=column_controller,
                            observer_column=observer_column, iterations=400)

  if FLAGS.plot:
    scenario_plotter.Plot()

  if len(argv) != 7:
    glog.fatal('Expected .h file name and .cc file names')
  else:
    namespaces = ['y2017', 'control_loops', 'superstructure', 'column']
    column = Column('Column')
    loop_writer = control_loop.ControlLoopWriter('Column', [column],
                                                 namespaces=namespaces)
    loop_writer.AddConstant(control_loop.Constant(
        'kIndexerFreeSpeed', '%f', column.indexer.free_speed))
    loop_writer.AddConstant(control_loop.Constant(
        'kIndexerOutputRatio', '%f', column.indexer.G))
    loop_writer.AddConstant(control_loop.Constant(
        'kTurretFreeSpeed', '%f', column.turret.free_speed))
    loop_writer.AddConstant(control_loop.Constant(
        'kTurretOutputRatio', '%f', column.turret.G))
    loop_writer.Write(argv[1], argv[2])

    # IntegralColumn controller 1 will disable the indexer.
    integral_column = IntegralColumn('IntegralColumn')
    disabled_integral_column = IntegralColumn('DisabledIntegralColumn',
                                              disable_indexer=True)
    integral_loop_writer = control_loop.ControlLoopWriter(
        'IntegralColumn', [integral_column, disabled_integral_column],
        namespaces=namespaces)
    integral_loop_writer.Write(argv[3], argv[4])

    stuck_integral_column = IntegralColumn('StuckIntegralColumn', voltage_error_noise=8.0)
    stuck_integral_loop_writer = control_loop.ControlLoopWriter(
        'StuckIntegralColumn', [stuck_integral_column], namespaces=namespaces)
    stuck_integral_loop_writer.Write(argv[5], argv[6])


if __name__ == '__main__':
  argv = FLAGS(sys.argv)
  glog.init()
  sys.exit(main(argv))
