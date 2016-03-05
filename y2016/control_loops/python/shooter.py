#!/usr/bin/python

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
import sys
from matplotlib import pylab

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

class VelocityShooter(control_loop.ControlLoop):
  def __init__(self, name='VelocityShooter'):
    super(VelocityShooter, self).__init__(name)
    # Stall Torque in N m
    self.stall_torque = 0.71
    # Stall Current in Amps
    self.stall_current = 134
    # Free Speed in RPM
    self.free_speed = 18730.0
    # Free Current in Amps
    self.free_current = 0.7
    # Moment of inertia of the shooter wheel in kg m^2
    self.J = 0.00032
    # Resistance of the motor, divided by 2 to account for the 2 motors
    self.R = 12.0 / self.stall_current
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
              (12.0 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratio
    self.G = 12.0 / 18.0
    # Control loop time step
    self.dt = 0.005

    # State feedback matrices
    # [angular velocity]
    self.A_continuous = numpy.matrix(
        [[-self.Kt / self.Kv / (self.J * self.G * self.G * self.R)]])
    self.B_continuous = numpy.matrix(
        [[self.Kt / (self.J * self.G * self.R)]])
    self.C = numpy.matrix([[1]])
    self.D = numpy.matrix([[0]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    self.PlaceControllerPoles([.87])

    self.PlaceObserverPoles([0.3])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    qff_vel = 8.0
    self.Qff = numpy.matrix([[1.0 / (qff_vel ** 2.0)]])

    self.Kff = controls.TwoStateFeedForwards(self.B, self.Qff)


class Shooter(VelocityShooter):
  def __init__(self, name='Shooter'):
    super(Shooter, self).__init__(name)

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


class IntegralShooter(Shooter):
  def __init__(self, name="IntegralShooter"):
    super(IntegralShooter, self).__init__(name=name)

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

    q_pos = 0.08
    q_vel = 4.00
    q_voltage = 0.3
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

  def run_test(self, shooter, goal, iterations=200, controller_shooter=None,
             observer_shooter=None):
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

    for i in xrange(iterations):
      X_hat = shooter.X

      if observer_shooter is not None:
        X_hat = observer_shooter.X_hat
        self.x_hat.append(observer_shooter.X_hat[1, 0])

      ff_U = controller_shooter.Kff * (goal - observer_shooter.A * goal)

      U = controller_shooter.K * (goal - X_hat) + ff_U
      U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)
      self.x.append(shooter.X[0, 0])


      if self.v:
        last_v = self.v[-1]
      else:
        last_v = 0

      self.v.append(shooter.X[1, 0])
      self.a.append((self.v[-1] - last_v) / shooter.dt)

      if observer_shooter is not None:
        observer_shooter.Y = shooter.Y
        observer_shooter.CorrectObserver(U)
        self.offset.append(observer_shooter.X_hat[2, 0])

      applied_U = U.copy()
      if i > 30:
        applied_U += 2
      shooter.Update(applied_U)

      if observer_shooter is not None:
        observer_shooter.PredictObserver(U)

      self.t.append(initial_t + i * shooter.dt)
      self.u.append(U[0, 0])

      glog.debug('Time: %f', self.t[-1])

  def Plot(self):
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

    pylab.show()


def main(argv):
  scenario_plotter = ScenarioPlotter()

  shooter = Shooter()
  shooter_controller = IntegralShooter()
  observer_shooter = IntegralShooter()

  initial_X = numpy.matrix([[0.0], [0.0]])
  R = numpy.matrix([[0.0], [100.0], [0.0]])
  scenario_plotter.run_test(shooter, goal=R, controller_shooter=shooter_controller,
                            observer_shooter=observer_shooter, iterations=200)

  if FLAGS.plot:
    scenario_plotter.Plot()

  if len(argv) != 5:
    glog.fatal('Expected .h file name and .cc file name')
  else:
    namespaces = ['y2016', 'control_loops', 'shooter']
    shooter = Shooter('Shooter')
    loop_writer = control_loop.ControlLoopWriter('Shooter', [shooter],
                                                 namespaces=namespaces)
    loop_writer.Write(argv[1], argv[2])

    integral_shooter = IntegralShooter('IntegralShooter')
    integral_loop_writer = control_loop.ControlLoopWriter(
        'IntegralShooter', [integral_shooter], namespaces=namespaces)
    integral_loop_writer.Write(argv[3], argv[4])


if __name__ == '__main__':
  argv = FLAGS(sys.argv)
  sys.exit(main(argv))
