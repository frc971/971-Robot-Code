#!/usr/bin/python

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
import math
import sys
from matplotlib import pylab

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')


class Arm(control_loop.ControlLoop):
  def __init__(self, name='Arm', mass=None):
    super(Arm, self).__init__(name)
    # Stall Torque in N m
    self.stall_torque = 0.476
    # Stall Current in Amps
    self.stall_current = 80.730
    # Free Speed in RPM
    self.free_speed = 13906.0
    # Free Current in Amps
    self.free_current = 5.820
    # Mass of the arm
    if mass is None:
      self.mass = 13.0
    else:
      self.mass = mass

    # Resistance of the motor
    self.R = 12.0 / self.stall_current
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
               (12.0 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratio
    self.G = (44.0 / 12.0) * (54.0 / 14.0) * (54.0 / 14.0) * (44.0 / 20.0) * (72.0 / 16.0)
    # Fridge arm length
    self.r = 32 * 0.0254
    # Control loop time step
    self.dt = 0.005

    # Arm moment of inertia
    self.J = self.r * self.mass

    # Arm left/right spring constant (N*m / radian)
    self.spring = 100.0

    # State is [average position, average velocity,
    #           position difference/2, velocity difference/2]
    # Position difference is 1 - 2
    # Input is [Voltage 1, Voltage 2]

    self.C1 = self.spring / (self.J * 0.5)
    self.C2 = self.Kt * self.G / (self.J * 0.5 * self.R)
    self.C3 = self.G * self.G * self.Kt / (self.R  * self.J * 0.5 * self.Kv)

    self.A_continuous = numpy.matrix(
        [[0, 1, 0, 0],
         [0, -self.C3, 0, 0],
         [0, 0, 0, 1],
         [0, 0, -self.C1 * 2.0, -self.C3]])

    glog.debug('Full speed is %f', self.C2 / self.C3 * 12.0)

    glog.debug('Stall arm difference is %f', 12.0 * self.C2 / self.C1)
    glog.debug('Stall arm difference first principles is %f', self.stall_torque * self.G / self.spring)

    glog.debug('5 degrees of arm error is %f', self.spring / self.r * (math.pi * 5.0 / 180.0))

    # Start with the unmodified input
    self.B_continuous = numpy.matrix(
        [[0, 0],
         [self.C2 / 2.0, self.C2 / 2.0],
         [0, 0],
         [self.C2 / 2.0, -self.C2 / 2.0]])

    self.C = numpy.matrix([[1, 0, 1, 0],
                           [1, 0, -1, 0]])
    self.D = numpy.matrix([[0, 0],
                           [0, 0]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    controllability = controls.ctrb(self.A, self.B)
    glog.debug('Rank of augmented controllability matrix. %d', numpy.linalg.matrix_rank(
        controllability))

    q_pos = 0.02
    q_vel = 0.300
    q_pos_diff = 0.005
    q_vel_diff = 0.13
    self.Q = numpy.matrix([[(1.0 / (q_pos ** 2.0)), 0.0, 0.0, 0.0],
                           [0.0, (1.0 / (q_vel ** 2.0)), 0.0, 0.0],
                           [0.0, 0.0, (1.0 / (q_pos_diff ** 2.0)), 0.0],
                           [0.0, 0.0, 0.0, (1.0 / (q_vel_diff ** 2.0))]])

    self.R = numpy.matrix([[(1.0 / (12.0 ** 2.0)), 0.0],
                           [0.0, 1.0 / (12.0 ** 2.0)]])
    self.K = controls.dlqr(self.A, self.B, self.Q, self.R)
    glog.debug('Controller\n %s', repr(self.K))

    glog.debug('Controller Poles\n %s',
               numpy.linalg.eig(self.A - self.B * self.K)[0])

    self.rpl = 0.20
    self.ipl = 0.05
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    # The box formed by U_min and U_max must encompass all possible values,
    # or else Austin's code gets angry.
    self.U_max = numpy.matrix([[12.0], [12.0]])
    self.U_min = numpy.matrix([[-12.0], [-12.0]])

    glog.debug('Observer (Converted to a KF):\n%s',
               repr(numpy.linalg.inv(self.A) * self.L))

    self.InitializeState()


class IntegralArm(Arm):
  def __init__(self, name='IntegralArm', mass=None):
    super(IntegralArm, self).__init__(name=name, mass=mass)

    self.A_continuous_unaugmented = self.A_continuous
    self.A_continuous = numpy.matrix(numpy.zeros((5, 5)))
    self.A_continuous[0:4, 0:4] = self.A_continuous_unaugmented
    self.A_continuous[1, 4] = self.C2

    # Start with the unmodified input
    self.B_continuous_unaugmented = self.B_continuous
    self.B_continuous = numpy.matrix(numpy.zeros((5, 2)))
    self.B_continuous[0:4, 0:2] = self.B_continuous_unaugmented

    self.C_unaugmented = self.C
    self.C = numpy.matrix(numpy.zeros((2, 5)))
    self.C[0:2, 0:4] = self.C_unaugmented

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)
    glog.debug('A cont: %s', repr(self.A_continuous))
    glog.debug('B cont %s', repr(self.B_continuous))
    glog.debug('A discrete %s', repr(self.A))

    q_pos = 0.08
    q_vel = 0.40

    q_pos_diff = 0.08
    q_vel_diff = 0.40
    q_voltage = 6.0
    self.Q = numpy.matrix([[(q_pos ** 2.0), 0.0, 0.0, 0.0, 0.0],
                           [0.0, (q_vel ** 2.0), 0.0, 0.0, 0.0],
                           [0.0, 0.0, (q_pos_diff ** 2.0), 0.0, 0.0],
                           [0.0, 0.0, 0.0, (q_vel_diff ** 2.0), 0.0],
                           [0.0, 0.0, 0.0, 0.0, (q_voltage ** 2.0)]])

    r_volts = 0.05
    self.R = numpy.matrix([[(r_volts ** 2.0), 0.0],
                           [0.0, (r_volts ** 2.0)]])

    self.KalmanGain, self.Q_steady = controls.kalman(
        A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)

    self.U_max = numpy.matrix([[12.0], [12.0]])
    self.U_min = numpy.matrix([[-12.0], [-12.0]])

    self.K_unaugmented = self.K
    self.K = numpy.matrix(numpy.zeros((2, 5)))
    self.K[0:2, 0:4] = self.K_unaugmented
    self.K[0, 4] = 1
    self.K[1, 4] = 1
    glog.debug('Kal: %s', repr(self.KalmanGain))
    self.L = self.A * self.KalmanGain

    self.InitializeState()


def CapU(U):
  if U[0, 0] - U[1, 0] > 24:
    return numpy.matrix([[12], [-12]])
  elif U[0, 0] - U[1, 0] < -24:
    return numpy.matrix([[-12], [12]])
  else:
    max_u = max(U[0, 0], U[1, 0])
    min_u = min(U[0, 0], U[1, 0])
    if max_u > 12:
      return U - (max_u - 12)
    if min_u < -12:
      return U - (min_u + 12)
    return U


def run_test(arm, initial_X, goal, max_separation_error=0.01,
             show_graph=True, iterations=200, controller_arm=None,
             observer_arm=None):
  """Runs the arm plant with an initial condition and goal.

    The tests themselves are not terribly sophisticated; I just test for
    whether the goal has been reached and whether the separation goes
    outside of the initial and goal values by more than max_separation_error.
    Prints out something for a failure of either condition and returns
    False if tests fail.
    Args:
      arm: arm object to use.
      initial_X: starting state.
      goal: goal state.
      show_graph: Whether or not to display a graph showing the changing
           states and voltages.
      iterations: Number of timesteps to run the model for.
      controller_arm: arm object to get K from, or None if we should
          use arm.
      observer_arm: arm object to use for the observer, or None if we should
          use the actual state.
  """

  arm.X = initial_X

  if controller_arm is None:
    controller_arm = arm

  if observer_arm is not None:
    observer_arm.X_hat = initial_X + 0.01
    observer_arm.X_hat = initial_X

  # Various lists for graphing things.
  t = []
  x_avg = []
  x_sep = []
  x_hat_avg = []
  x_hat_sep = []
  v_avg = []
  v_sep = []
  u_left = []
  u_right = []

  sep_plot_gain = 100.0

  for i in xrange(iterations):
    X_hat = arm.X
    if observer_arm is not None:
      X_hat = observer_arm.X_hat
      x_hat_avg.append(observer_arm.X_hat[0, 0])
      x_hat_sep.append(observer_arm.X_hat[2, 0] * sep_plot_gain)
    U = controller_arm.K * (goal - X_hat)
    U = CapU(U)
    x_avg.append(arm.X[0, 0])
    v_avg.append(arm.X[1, 0])
    x_sep.append(arm.X[2, 0] * sep_plot_gain)
    v_sep.append(arm.X[3, 0])
    if observer_arm is not None:
      observer_arm.PredictObserver(U)
    arm.Update(U)
    if observer_arm is not None:
      observer_arm.Y = arm.Y
      observer_arm.CorrectObserver(U)

    t.append(i * arm.dt)
    u_left.append(U[0, 0])
    u_right.append(U[1, 0])

  glog.debug(repr(numpy.linalg.inv(arm.A)))
  glog.debug('delta time is %f', arm.dt)
  glog.debug('Velocity at t=0 is %f %f %f %f', x_avg[0], v_avg[0], x_sep[0], v_sep[0])
  glog.debug('Velocity at t=1+dt is %f %f %f %f', x_avg[1], v_avg[1], x_sep[1], v_sep[1])

  if show_graph:
    pylab.subplot(2, 1, 1)
    pylab.plot(t, x_avg, label='x avg')
    pylab.plot(t, x_sep, label='x sep')
    if observer_arm is not None:
      pylab.plot(t, x_hat_avg, label='x_hat avg')
      pylab.plot(t, x_hat_sep, label='x_hat sep')
    pylab.legend()

    pylab.subplot(2, 1, 2)
    pylab.plot(t, u_left, label='u left')
    pylab.plot(t, u_right, label='u right')
    pylab.legend()
    pylab.show()


def run_integral_test(arm, initial_X, goal, observer_arm, disturbance,
                      max_separation_error=0.01, show_graph=True,
                      iterations=400):
  """Runs the integral control arm plant with an initial condition and goal.

    The tests themselves are not terribly sophisticated; I just test for
    whether the goal has been reached and whether the separation goes
    outside of the initial and goal values by more than max_separation_error.
    Prints out something for a failure of either condition and returns
    False if tests fail.
    Args:
      arm: arm object to use.
      initial_X: starting state.
      goal: goal state.
      observer_arm: arm object to use for the observer.
      show_graph: Whether or not to display a graph showing the changing
           states and voltages.
      iterations: Number of timesteps to run the model for.
      disturbance: Voltage missmatch between controller and model.
  """

  arm.X = initial_X

  # Various lists for graphing things.
  t = []
  x_avg = []
  x_sep = []
  x_hat_avg = []
  x_hat_sep = []
  v_avg = []
  v_sep = []
  u_left = []
  u_right = []
  u_error = []

  sep_plot_gain = 100.0

  unaugmented_goal = goal
  goal = numpy.matrix(numpy.zeros((5, 1)))
  goal[0:4, 0] = unaugmented_goal

  for i in xrange(iterations):
    X_hat = observer_arm.X_hat[0:4]

    x_hat_avg.append(observer_arm.X_hat[0, 0])
    x_hat_sep.append(observer_arm.X_hat[2, 0] * sep_plot_gain)

    U = observer_arm.K * (goal - observer_arm.X_hat)
    u_error.append(observer_arm.X_hat[4,0])
    U = CapU(U)
    x_avg.append(arm.X[0, 0])
    v_avg.append(arm.X[1, 0])
    x_sep.append(arm.X[2, 0] * sep_plot_gain)
    v_sep.append(arm.X[3, 0])

    observer_arm.PredictObserver(U)

    arm.Update(U + disturbance)
    observer_arm.Y = arm.Y
    observer_arm.CorrectObserver(U)

    t.append(i * arm.dt)
    u_left.append(U[0, 0])
    u_right.append(U[1, 0])

  glog.debug('End is %f', observer_arm.X_hat[4, 0])

  if show_graph:
    pylab.subplot(2, 1, 1)
    pylab.plot(t, x_avg, label='x avg')
    pylab.plot(t, x_sep, label='x sep')
    if observer_arm is not None:
      pylab.plot(t, x_hat_avg, label='x_hat avg')
      pylab.plot(t, x_hat_sep, label='x_hat sep')
    pylab.legend()

    pylab.subplot(2, 1, 2)
    pylab.plot(t, u_left, label='u left')
    pylab.plot(t, u_right, label='u right')
    pylab.plot(t, u_error, label='u error')
    pylab.legend()
    pylab.show()


def main(argv):
  if FLAGS.plot:
    loaded_mass = 25
    #loaded_mass = 0
    arm = Arm(mass=13 + loaded_mass)
    #arm_controller = Arm(mass=13 + 15)
    #observer_arm = Arm(mass=13 + 15)
    #observer_arm = None

    integral_arm = IntegralArm(mass=13 + loaded_mass)
    integral_arm.X_hat[0, 0] += 0.02
    integral_arm.X_hat[2, 0] += 0.02
    integral_arm.X_hat[4] = 0

    # Test moving the arm with constant separation.
    initial_X = numpy.matrix([[0.0], [0.0], [0.0], [0.0]])
    R = numpy.matrix([[0.0], [0.0], [0.0], [0.0]])
    run_integral_test(arm, initial_X, R, integral_arm, disturbance=2)

  # Write the generated constants out to a file.
  if len(argv) != 5:
    glog.fatal('Expected .h file name and .cc file name for the arm and augmented arm.')
  else:
    namespaces = ['y2015', 'control_loops', 'fridge']
    arm = Arm('Arm', mass=13)
    loop_writer = control_loop.ControlLoopWriter('Arm', [arm],
                                                 namespaces=namespaces)
    loop_writer.Write(argv[1], argv[2])

    integral_arm = IntegralArm('IntegralArm', mass=13)
    loop_writer = control_loop.ControlLoopWriter('IntegralArm', [integral_arm],
                                                 namespaces=namespaces)
    loop_writer.Write(argv[3], argv[4])

if __name__ == '__main__':
  argv = FLAGS(sys.argv)
  glog.init()
  sys.exit(main(argv))
