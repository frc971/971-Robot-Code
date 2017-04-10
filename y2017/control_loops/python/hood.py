#!/usr/bin/python

from aos.common.util.trapezoid_profile import TrapezoidProfile
from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
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

class Hood(control_loop.ControlLoop):
  def __init__(self, name='Hood'):
    super(Hood, self).__init__(name)
    # Stall Torque in N m
    self.stall_torque = 0.43
    # Stall Current in Amps
    self.stall_current = 53.0
    self.free_speed_rpm = 13180.0
    # Free Speed in rotations/second.
    self.free_speed = self.free_speed_rpm / 60
    # Free Current in Amps
    self.free_current = 1.8

    # Resistance of the motor
    self.R = 12.0 / self.stall_current
    # Motor velocity constant
    self.Kv = ((self.free_speed * 2.0 * numpy.pi) /
               (12.0 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # First axle gear ratio off the motor
    self.G1 = (12.0 / 60.0)
    # Second axle gear ratio off the motor
    self.G2 = self.G1 * (14.0 / 36.0)
    # Third axle gear ratio off the motor
    self.G3 = self.G2 * (14.0 / 36.0)
    # The last gear reduction (encoder -> hood angle)
    self.last_G = (20.0 / 345.0)
    # Gear ratio
    self.G = (12.0 / 60.0) * (14.0 / 36.0) * (14.0 / 36.0) * self.last_G

    # 36 tooth gear inertia in kg * m^2
    self.big_gear_inertia = 0.5 * 0.039 * ((36.0 / 40.0 * 0.025) ** 2)

    # Motor inertia in kg * m^2
    self.motor_inertia = 0.000006
    glog.debug(self.big_gear_inertia)

    # Moment of inertia, measured in CAD.
    # Extra mass to compensate for friction is added on.
    self.J = 0.08 + 2.3 + \
             self.big_gear_inertia * ((self.G1 / self.G) ** 2) + \
             self.big_gear_inertia * ((self.G2 / self.G) ** 2) + \
             self.motor_inertia * ((1.0 / self.G) ** 2.0)
    glog.debug('J effective %f', self.J)

    # Control loop time step
    self.dt = 0.005

    # State is [position, velocity]
    # Input is [Voltage]

    C1 = self.Kt / (self.R * self.J * self.Kv * self.G * self.G)
    C2 = self.Kt / (self.J * self.R * self.G)

    self.A_continuous = numpy.matrix(
        [[0, 1],
         [0, -C1]])

    # Start with the unmodified input
    self.B_continuous = numpy.matrix(
        [[0],
         [C2]])

    self.C = numpy.matrix([[1, 0]])
    self.D = numpy.matrix([[0]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    controllability = controls.ctrb(self.A, self.B)

    glog.debug('Free speed is %f',
               -self.B_continuous[1, 0] / self.A_continuous[1, 1] * 12.0)
    glog.debug(repr(self.A_continuous))

    # Calculate the LQR controller gain
    q_pos = 0.015
    q_vel = 0.40
    self.Q = numpy.matrix([[(1.0 / (q_pos ** 2.0)), 0.0],
                           [0.0, (1.0 / (q_vel ** 2.0))]])

    self.R = numpy.matrix([[(5.0 / (12.0 ** 2.0))]])
    self.K = controls.dlqr(self.A, self.B, self.Q, self.R)

    # Calculate the feed forwards gain.
    q_pos_ff = 0.005
    q_vel_ff = 1.0
    self.Qff = numpy.matrix([[(1.0 / (q_pos_ff ** 2.0)), 0.0],
                             [0.0, (1.0 / (q_vel_ff ** 2.0))]])

    self.Kff = controls.TwoStateFeedForwards(self.B, self.Qff)

    glog.debug('K %s', repr(self.K))
    glog.debug('Poles are %s',
               repr(numpy.linalg.eig(self.A - self.B * self.K)[0]))

    q_pos = 0.10
    q_vel = 1.65
    self.Q = numpy.matrix([[(q_pos ** 2.0), 0.0],
                           [0.0, (q_vel ** 2.0)]])

    r_volts = 0.025
    self.R = numpy.matrix([[(r_volts ** 2.0)]])

    self.KalmanGain, self.Q_steady = controls.kalman(
        A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)

    glog.debug('Kal %s', repr(self.KalmanGain))
    self.L = self.A * self.KalmanGain
    glog.debug('KalL is %s', repr(self.L))

    # The box formed by U_min and U_max must encompass all possible values,
    # or else Austin's code gets angry.
    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    self.InitializeState()

class IntegralHood(Hood):
  def __init__(self, name='IntegralHood'):
    super(IntegralHood, self).__init__(name=name)

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
    q_vel = 4.0
    q_voltage = 4.0
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

    self.Kff = numpy.concatenate((self.Kff, numpy.matrix(numpy.zeros((1, 1)))), axis=1)

    self.InitializeState()

class ScenarioPlotter(object):
  def __init__(self):
    # Various lists for graphing things.
    self.t = []
    self.x = []
    self.v = []
    self.v_hat = []
    self.a = []
    self.x_hat = []
    self.u = []
    self.offset = []

  def run_test(self, hood, end_goal,
             controller_hood,
             observer_hood=None,
             iterations=200):
    """Runs the hood plant with an initial condition and goal.

      Args:
        hood: hood object to use.
        end_goal: end_goal state.
        controller_hood: Hood object to get K from, or None if we should
            use hood.
        observer_hood: Hood object to use for the observer, or None if we should
            use the actual state.
        iterations: Number of timesteps to run the model for.
    """

    if controller_hood is None:
      controller_hood = hood

    vbat = 12.0

    if self.t:
      initial_t = self.t[-1] + hood.dt
    else:
      initial_t = 0

    goal = numpy.concatenate((hood.X, numpy.matrix(numpy.zeros((1, 1)))), axis=0)

    profile = TrapezoidProfile(hood.dt)
    profile.set_maximum_acceleration(10.0)
    profile.set_maximum_velocity(1.0)
    profile.SetGoal(goal[0, 0])

    U_last = numpy.matrix(numpy.zeros((1, 1)))
    for i in xrange(iterations):
      observer_hood.Y = hood.Y
      observer_hood.CorrectObserver(U_last)

      self.offset.append(observer_hood.X_hat[2, 0])
      self.x_hat.append(observer_hood.X_hat[0, 0])

      next_goal = numpy.concatenate(
          (profile.Update(end_goal[0, 0], end_goal[1, 0]),
           numpy.matrix(numpy.zeros((1, 1)))),
          axis=0)

      ff_U = controller_hood.Kff * (next_goal - observer_hood.A * goal)

      U_uncapped = controller_hood.K * (goal - observer_hood.X_hat) + ff_U
      U = U_uncapped.copy()
      U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)
      self.x.append(hood.X[0, 0])

      if self.v:
        last_v = self.v[-1]
      else:
        last_v = 0

      self.v.append(hood.X[1, 0])
      self.a.append((self.v[-1] - last_v) / hood.dt)
      self.v_hat.append(observer_hood.X_hat[1, 0])

      offset = 0.0
      if i > 100:
        offset = 2.0
      hood.Update(U + offset)

      observer_hood.PredictObserver(U)

      self.t.append(initial_t + i * hood.dt)
      self.u.append(U[0, 0])

      ff_U -= U_uncapped - U
      goal = controller_hood.A * goal + controller_hood.B * ff_U

      if U[0, 0] != U_uncapped[0, 0]:
        profile.MoveCurrentState(
            numpy.matrix([[goal[0, 0]], [goal[1, 0]]]))

    glog.debug('Time: %f', self.t[-1])
    glog.debug('goal_error %s', repr(end_goal - goal))
    glog.debug('error %s', repr(observer_hood.X_hat - end_goal))

  def Plot(self):
    pylab.subplot(3, 1, 1)
    pylab.plot(self.t, self.x, label='x')
    pylab.plot(self.t, self.x_hat, label='x_hat')
    pylab.plot(self.t, self.v, label='v')
    pylab.plot(self.t, self.v_hat, label='v_hat')
    pylab.legend()

    pylab.subplot(3, 1, 2)
    pylab.plot(self.t, self.u, label='u')
    pylab.plot(self.t, self.offset, label='voltage_offset')
    pylab.legend()

    pylab.subplot(3, 1, 3)
    pylab.plot(self.t, self.a, label='a')
    pylab.legend()

    pylab.show()


def main(argv):

  scenario_plotter = ScenarioPlotter()

  hood = Hood()
  hood_controller = IntegralHood()
  observer_hood = IntegralHood()

  # Test moving the hood with constant separation.
  initial_X = numpy.matrix([[0.0], [0.0]])
  R = numpy.matrix([[numpy.pi/4.0], [0.0], [0.0]])
  scenario_plotter.run_test(hood, end_goal=R,
                            controller_hood=hood_controller,
                            observer_hood=observer_hood, iterations=200)

  if FLAGS.plot:
    scenario_plotter.Plot()

  # Write the generated constants out to a file.
  if len(argv) != 5:
    glog.fatal('Expected .h file name and .cc file name for the hood and integral hood.')
  else:
    namespaces = ['y2017', 'control_loops', 'superstructure', 'hood']
    hood = Hood('Hood')
    loop_writer = control_loop.ControlLoopWriter('Hood', [hood],
                                                 namespaces=namespaces)
    loop_writer.AddConstant(control_loop.Constant(
        'kFreeSpeed', '%f', hood.free_speed))
    loop_writer.AddConstant(control_loop.Constant(
        'kOutputRatio', '%f', hood.G))
    loop_writer.Write(argv[1], argv[2])

    integral_hood = IntegralHood('IntegralHood')
    integral_loop_writer = control_loop.ControlLoopWriter('IntegralHood', [integral_hood],
                                                          namespaces=namespaces)
    integral_loop_writer.AddConstant(control_loop.Constant('kLastReduction', '%f',
          integral_hood.last_G))
    integral_loop_writer.Write(argv[3], argv[4])


if __name__ == '__main__':
  argv = FLAGS(sys.argv)
  glog.init()
  sys.exit(main(argv))
