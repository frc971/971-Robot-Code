#!/usr/bin/python

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

class Wrist(control_loop.ControlLoop):
  def __init__(self, name="Wrist"):
    super(Wrist, self).__init__(name)
    # TODO(constants): Update all of these & retune poles.
    # Stall Torque in N m
    self.stall_torque = 0.71
    # Stall Current in Amps
    self.stall_current = 134
    # Free Speed in RPM
    self.free_speed = 18730
    # Free Current in Amps
    self.free_current = 0.7

    # Resistance of the motor
    self.R = 12.0 / self.stall_current
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
               (12.0 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratio
    self.G = (56.0 / 12.0) * (54.0 / 14.0) * (64.0 / 18.0) * (48.0 / 16.0)

    self.J = 0.35

    # Control loop time step
    self.dt = 0.005

    # State is [position, velocity]
    # Input is [Voltage]

    C1 = self.G * self.G * self.Kt / (self.R  * self.J * self.Kv)
    C2 = self.Kt * self.G / (self.J * self.R)

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

    q_pos = 0.20
    q_vel = 8.0
    self.Q = numpy.matrix([[(1.0 / (q_pos ** 2.0)), 0.0],
                           [0.0, (1.0 / (q_vel ** 2.0))]])

    self.R = numpy.matrix([[(1.0 / (12.0 ** 2.0))]])
    self.K = controls.dlqr(self.A, self.B, self.Q, self.R)

    glog.debug('Poles are %s for %s',
               repr(numpy.linalg.eig(self.A - self.B * self.K)[0]), self._name)

    q_pos = 0.05
    q_vel = 2.65
    self.Q = numpy.matrix([[(q_pos ** 2.0), 0.0],
                           [0.0, (q_vel ** 2.0)]])

    r_volts = 0.025
    self.R = numpy.matrix([[(r_volts ** 2.0)]])

    self.KalmanGain, self.Q_steady = controls.kalman(
        A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)

    self.L = self.A * self.KalmanGain

    # The box formed by U_min and U_max must encompass all possible values,
    # or else Austin's code gets angry.
    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    self.Kff = controls.TwoStateFeedForwards(self.B, self.Q)

    self.InitializeState()

class IntegralWrist(Wrist):
  def __init__(self, name="IntegralWrist"):
    super(IntegralWrist, self).__init__(name=name)

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

    self.A, self.B = self.ContinuousToDiscrete(self.A_continuous, self.B_continuous, self.dt)

    q_pos = 0.08
    q_vel = 4.00
    q_voltage = 1.5
    self.Q = numpy.matrix([[(q_pos ** 2.0), 0.0, 0.0],
                           [0.0, (q_vel ** 2.0), 0.0],
                           [0.0, 0.0, (q_voltage ** 2.0)]])

    r_pos = 0.05
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
    self.x_hat = []
    self.u = []
    self.offset = []

  def run_test(self, wrist, goal, iterations=200, controller_wrist=None,
             observer_wrist=None):
    """Runs the wrist plant with an initial condition and goal.

      Test for whether the goal has been reached and whether the separation
      goes  outside of the initial and goal values by more than
      max_separation_error.

      Prints out something for a failure of either condition and returns
      False if tests fail.
      Args:
        wrist: wrist object to use.
        goal: goal state.
        iterations: Number of timesteps to run the model for.
        controller_wrist: Wrist object to get K from, or None if we should
            use wrist.
        observer_wrist: Wrist object to use for the observer, or None if we should
            use the actual state.
    """

    if controller_wrist is None:
      controller_wrist = wrist

    vbat = 12.0

    if self.t:
      initial_t = self.t[-1] + wrist.dt
    else:
      initial_t = 0

    for i in xrange(iterations):
      X_hat = wrist.X

      if observer_wrist is not None:
        X_hat = observer_wrist.X_hat
        self.x_hat.append(observer_wrist.X_hat[0, 0])

      U = controller_wrist.K * (goal - X_hat)
      U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)
      self.x.append(wrist.X[0, 0])

      if self.v:
        last_v = self.v[-1]
      else:
        last_v = 0

      self.v.append(wrist.X[1, 0])
      self.a.append((self.v[-1] - last_v) / wrist.dt)

      if observer_wrist is not None:
        observer_wrist.Y = wrist.Y
        observer_wrist.CorrectObserver(U)
        self.offset.append(observer_wrist.X_hat[2, 0])

      wrist.Update(U + 2.0)

      if observer_wrist is not None:
        observer_wrist.PredictObserver(U)

      self.t.append(initial_t + i * wrist.dt)
      self.u.append(U[0, 0])

      glog.debug('Time: %f', self.t[-1])

  def Plot(self):
    pylab.subplot(3, 1, 1)
    pylab.plot(self.t, self.x, label='x')
    pylab.plot(self.t, self.x_hat, label='x_hat')
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

  wrist = Wrist()
  wrist_controller = IntegralWrist()
  observer_wrist = IntegralWrist()

  # Test moving the wrist with constant separation.
  initial_X = numpy.matrix([[0.0], [0.0]])
  R = numpy.matrix([[1.0], [0.0], [0.0]])
  scenario_plotter.run_test(wrist, goal=R, controller_wrist=wrist_controller,
                            observer_wrist=observer_wrist, iterations=200)

  if FLAGS.plot:
    scenario_plotter.Plot()

  # Write the generated constants out to a file.
  if len(argv) != 5:
    glog.fatal('Expected .h file name and .cc file name for the wrist and integral wrist.')
  else:
    namespaces = ['y2016', 'control_loops', 'superstructure']
    wrist = Wrist('Wrist')
    loop_writer = control_loop.ControlLoopWriter(
        'Wrist', [wrist], namespaces=namespaces)
    loop_writer.Write(argv[1], argv[2])

    integral_wrist = IntegralWrist('IntegralWrist')
    integral_loop_writer = control_loop.ControlLoopWriter(
        'IntegralWrist', [integral_wrist], namespaces=namespaces)
    integral_loop_writer.Write(argv[3], argv[4])

if __name__ == '__main__':
  argv = FLAGS(sys.argv)
  sys.exit(main(argv))
