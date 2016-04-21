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

class Intake(control_loop.ControlLoop):
  def __init__(self, name="Intake"):
    super(Intake, self).__init__(name)
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

    # Moment of inertia, measured in CAD.
    # Extra mass to compensate for friction is added on.
    self.J = 0.34 + 0.40

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

    glog.debug("Free speed is %f", self.free_speed * numpy.pi * 2.0 / 60.0 / self.G)

    q_pos = 0.20
    q_vel = 5.0
    self.Q = numpy.matrix([[(1.0 / (q_pos ** 2.0)), 0.0],
                           [0.0, (1.0 / (q_vel ** 2.0))]])

    self.R = numpy.matrix([[(1.0 / (12.0 ** 2.0))]])
    self.K = controls.dlqr(self.A, self.B, self.Q, self.R)

    q_pos_ff = 0.005
    q_vel_ff = 1.0
    self.Qff = numpy.matrix([[(1.0 / (q_pos_ff ** 2.0)), 0.0],
                             [0.0, (1.0 / (q_vel_ff ** 2.0))]])

    self.Kff = controls.TwoStateFeedForwards(self.B, self.Qff)

    glog.debug('K %s', repr(self.K))
    glog.debug('Poles are %s',
              repr(numpy.linalg.eig(self.A - self.B * self.K)[0]))

    self.rpl = 0.30
    self.ipl = 0.10
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    glog.debug('L is %s', repr(self.L))

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

class IntegralIntake(Intake):
  def __init__(self, name="IntegralIntake"):
    super(IntegralIntake, self).__init__(name=name)

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

    q_pos = 0.12
    q_vel = 2.00
    q_voltage = 4.0
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

    self.Kff = numpy.concatenate((self.Kff, numpy.matrix(numpy.zeros((1, 1)))), axis=1)

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

  def run_test(self, intake, end_goal,
             controller_intake,
             observer_intake=None,
             iterations=200):
    """Runs the intake plant with an initial condition and goal.

      Args:
        intake: intake object to use.
        end_goal: end_goal state.
        controller_intake: Intake object to get K from, or None if we should
            use intake.
        observer_intake: Intake object to use for the observer, or None if we should
            use the actual state.
        iterations: Number of timesteps to run the model for.
    """

    if controller_intake is None:
      controller_intake = intake

    vbat = 12.0

    if self.t:
      initial_t = self.t[-1] + intake.dt
    else:
      initial_t = 0

    goal = numpy.concatenate((intake.X, numpy.matrix(numpy.zeros((1, 1)))), axis=0)

    profile = TrapezoidProfile(intake.dt)
    profile.set_maximum_acceleration(70.0)
    profile.set_maximum_velocity(10.0)
    profile.SetGoal(goal[0, 0])

    U_last = numpy.matrix(numpy.zeros((1, 1)))
    for i in xrange(iterations):
      observer_intake.Y = intake.Y
      observer_intake.CorrectObserver(U_last)

      self.offset.append(observer_intake.X_hat[2, 0])
      self.x_hat.append(observer_intake.X_hat[0, 0])

      next_goal = numpy.concatenate(
          (profile.Update(end_goal[0, 0], end_goal[1, 0]),
           numpy.matrix(numpy.zeros((1, 1)))),
          axis=0)

      ff_U = controller_intake.Kff * (next_goal - observer_intake.A * goal)

      U_uncapped = controller_intake.K * (goal - observer_intake.X_hat) + ff_U
      U = U_uncapped.copy()
      U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)
      self.x.append(intake.X[0, 0])

      if self.v:
        last_v = self.v[-1]
      else:
        last_v = 0

      self.v.append(intake.X[1, 0])
      self.a.append((self.v[-1] - last_v) / intake.dt)

      offset = 0.0
      if i > 100:
        offset = 2.0
      intake.Update(U + offset)

      observer_intake.PredictObserver(U)

      self.t.append(initial_t + i * intake.dt)
      self.u.append(U[0, 0])

      ff_U -= U_uncapped - U
      goal = controller_intake.A * goal + controller_intake.B * ff_U

      if U[0, 0] != U_uncapped[0, 0]:
        profile.MoveCurrentState(
            numpy.matrix([[goal[0, 0]], [goal[1, 0]]]))

    glog.debug('Time: %f', self.t[-1])
    glog.debug('goal_error %s', repr(end_goal - goal))
    glog.debug('error %s', repr(observer_intake.X_hat - end_goal))

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
  argv = FLAGS(argv)
  glog.init()

  scenario_plotter = ScenarioPlotter()

  intake = Intake()
  intake_controller = IntegralIntake()
  observer_intake = IntegralIntake()

  # Test moving the intake with constant separation.
  initial_X = numpy.matrix([[0.0], [0.0]])
  R = numpy.matrix([[numpy.pi/2.0], [0.0], [0.0]])
  scenario_plotter.run_test(intake, end_goal=R,
                            controller_intake=intake_controller,
                            observer_intake=observer_intake, iterations=200)

  if FLAGS.plot:
    scenario_plotter.Plot()

  # Write the generated constants out to a file.
  if len(argv) != 5:
    glog.fatal('Expected .h file name and .cc file name for the intake and integral intake.')
  else:
    namespaces = ['y2016', 'control_loops', 'superstructure']
    intake = Intake("Intake")
    loop_writer = control_loop.ControlLoopWriter('Intake', [intake],
                                                 namespaces=namespaces)
    loop_writer.Write(argv[1], argv[2])

    integral_intake = IntegralIntake("IntegralIntake")
    integral_loop_writer = control_loop.ControlLoopWriter("IntegralIntake", [integral_intake],
                                                          namespaces=namespaces)
    integral_loop_writer.Write(argv[3], argv[4])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
