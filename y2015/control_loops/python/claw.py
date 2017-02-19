#!/usr/bin/python

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
from frc971.control_loops.python import polytope
import numpy
import sys
import matplotlib
from matplotlib import pylab
import glog
import gflags

FLAGS = gflags.FLAGS

try:
  gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
  pass

class Claw(control_loop.ControlLoop):
  def __init__(self, name='Claw', mass=None):
    super(Claw, self).__init__(name)
    # Stall Torque in N m
    self.stall_torque = 0.476
    # Stall Current in Amps
    self.stall_current = 80.730
    # Free Speed in RPM
    self.free_speed = 13906.0
    # Free Current in Amps
    self.free_current = 5.820
    # Mass of the claw
    if mass is None:
      self.mass = 5.0
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
    self.G = (56.0 / 12.0) * (54.0 / 14.0) * (64.0 / 14.0) * (72.0 / 18.0)
    # Claw length
    self.r = 18 * 0.0254

    self.J = self.r * self.mass

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

    glog.debug('Free speed is %f', self.free_speed * numpy.pi * 2.0 / 60.0 / self.G)

    q_pos = 0.15
    q_vel = 2.5
    self.Q = numpy.matrix([[(1.0 / (q_pos ** 2.0)), 0.0],
                           [0.0, (1.0 / (q_vel ** 2.0))]])

    self.R = numpy.matrix([[(1.0 / (12.0 ** 2.0))]])
    self.K = controls.dlqr(self.A, self.B, self.Q, self.R)

    glog.debug('K: %s', repr(self.K))
    glog.debug('Poles are: %s', repr(numpy.linalg.eig(self.A - self.B * self.K)[0]))

    self.rpl = 0.30
    self.ipl = 0.10
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    glog.debug('L is: %s', repr(self.L))

    q_pos = 0.05
    q_vel = 2.65
    self.Q = numpy.matrix([[(q_pos ** 2.0), 0.0],
                           [0.0, (q_vel ** 2.0)]])

    r_volts = 0.025
    self.R = numpy.matrix([[(r_volts ** 2.0)]])

    self.KalmanGain, self.Q_steady = controls.kalman(
        A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)

    glog.debug('Kal: %s', repr(self.KalmanGain))
    self.L = self.A * self.KalmanGain
    glog.debug('KalL is: %s', repr(self.L))

    # The box formed by U_min and U_max must encompass all possible values,
    # or else Austin's code gets angry.
    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    self.InitializeState()


def run_test(claw, initial_X, goal, max_separation_error=0.01,
             iterations=200, controller_claw=None,
             observer_claw=None):
  """Runs the claw plant with an initial condition and goal.

    The tests themselves are not terribly sophisticated; I just test for
    whether the goal has been reached and whether the separation goes
    outside of the initial and goal values by more than max_separation_error.
    Prints out something for a failure of either condition and returns
    False if tests fail.
    Args:
      claw: claw object to use.
      initial_X: starting state.
      goal: goal state.
      iterations: Number of timesteps to run the model for.
      controller_claw: claw object to get K from, or None if we should
          use claw.
      observer_claw: claw object to use for the observer, or None if we should
          use the actual state.
  """

  claw.X = initial_X

  if controller_claw is None:
    controller_claw = claw

  if observer_claw is not None:
    observer_claw.X_hat = initial_X + 0.01
    observer_claw.X_hat = initial_X

  # Various lists for graphing things.
  t = []
  x = []
  v = []
  x_hat = []
  u = []

  sep_plot_gain = 100.0

  for i in xrange(iterations):
    X_hat = claw.X
    if observer_claw is not None:
      X_hat = observer_claw.X_hat
      x_hat.append(observer_claw.X_hat[0, 0])
    U = controller_claw.K * (goal - X_hat)
    U[0, 0] = numpy.clip(U[0, 0], -12, 12)
    x.append(claw.X[0, 0])
    v.append(claw.X[1, 0])
    if observer_claw is not None:
      observer_claw.PredictObserver(U)
    claw.Update(U)
    if observer_claw is not None:
      observer_claw.Y = claw.Y
      observer_claw.CorrectObserver(U)

    t.append(i * claw.dt)
    u.append(U[0, 0])

  pylab.subplot(2, 1, 1)
  pylab.plot(t, x, label='x')
  if observer_claw is not None:
    pylab.plot(t, x_hat, label='x_hat')
  pylab.legend()

  pylab.subplot(2, 1, 2)
  pylab.plot(t, u, label='u')
  pylab.legend()
  pylab.show()


def main(argv):
  if FLAGS.plot:
    loaded_mass = 0
    #loaded_mass = 0
    claw = Claw(mass=4 + loaded_mass)
    claw_controller = Claw(mass=5 + 0)
    observer_claw = Claw(mass=5 + 0)
    #observer_claw = None

    # Test moving the claw with constant separation.
    initial_X = numpy.matrix([[0.0], [0.0]])
    R = numpy.matrix([[1.0], [0.0]])
    run_test(claw, initial_X, R, controller_claw=claw_controller,
             observer_claw=observer_claw)

  # Write the generated constants out to a file.
  if len(argv) != 3:
    glog.fatal('Expected .h and .cc filename for claw.')
  else:
    namespaces = ['y2015', 'control_loops', 'claw']
    claw = Claw('Claw')
    loop_writer = control_loop.ControlLoopWriter('Claw', [claw],
                                                 namespaces=namespaces)
    loop_writer.Write(argv[1], argv[2])

if __name__ == '__main__':
  argv = FLAGS(sys.argv)
  glog.init()
  sys.exit(main(argv))
