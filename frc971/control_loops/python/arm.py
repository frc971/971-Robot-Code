#!/usr/bin/python

import control_loop
import controls
import polytope
import polydrivetrain
import numpy
import math
import sys
import matplotlib
from matplotlib import pylab

class Arm(control_loop.ControlLoop):
  def __init__(self, name="Arm", mass=None):
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
    self.spring = 400.0

    # State is [average position, average velocity,
    #           position difference/2, velocity difference/2]
    # Position difference is 1 - 2
    # Input is [Voltage 1, Voltage 2]

    C1 = self.spring / (self.J * 0.5)
    C2 = self.Kt * self.G / (self.J * 0.5 * self.R)
    C3 = self.G * self.G * self.Kt / (self.R  * self.J * 0.5 * self.Kv)

    self.A_continuous = numpy.matrix(
        [[0, 1, 0, 0],
         [0, -C3, 0, 0],
         [0, 0, 0, 1],
         [0, 0, -C1 * 2.0, -C3]])

    print 'Full speed is', C2 / C3 * 12.0

    print 'Stall arm difference is', 12.0 * C2 / C1
    print 'Stall arm difference first principles is', self.stall_torque * self.G / self.spring

    print '5 degrees of arm error is', self.spring / self.r * (math.pi * 5.0 / 180.0)

    # Start with the unmodified input
    self.B_continuous = numpy.matrix(
        [[0, 0],
         [C2 / 2.0, C2 / 2.0],
         [0, 0],
         [C2 / 2.0, -C2 / 2.0]])

    self.C = numpy.matrix([[1, 0, 1, 0],
                           [1, 0, -1, 0]])
    self.D = numpy.matrix([[0, 0],
                           [0, 0]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    controlability = controls.ctrb(self.A, self.B);
    print 'Rank of augmented controlability matrix.', numpy.linalg.matrix_rank(
        controlability)

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
    print 'Controller'
    print self.K

    print 'Controller Poles'
    print numpy.linalg.eig(self.A - self.B * self.K)[0]

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

  print numpy.linalg.inv(arm.A)
  print "delta time is ", arm.dt
  print "Velocity at t=0 is ", x_avg[0], v_avg[0], x_sep[0], v_sep[0]
  print "Velocity at t=1+dt is ", x_avg[1], v_avg[1], x_sep[1], v_sep[1]

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


def main(argv):
  loaded_mass = 25
  #loaded_mass = 0
  arm = Arm(mass=13 + loaded_mass)
  arm_controller = Arm(mass=13 + 15)
  observer_arm = Arm(mass=13 + 15)
  #observer_arm = None

  # Test moving the arm with constant separation.
  initial_X = numpy.matrix([[0.0], [0.0], [0.01], [0.0]])
  #initial_X = numpy.matrix([[0.0], [0.0], [0.00], [0.0]])
  R = numpy.matrix([[1.0], [0.0], [0.0], [0.0]])
  run_test(arm, initial_X, R, controller_arm=arm_controller,
           observer_arm=observer_arm)

  # Write the generated constants out to a file.
  if len(argv) != 3:
    print "Expected .h file name and .cc file name for the arm."
  else:
    arm = Arm("Arm", 2)
    loop_writer = control_loop.ControlLoopWriter("Arm", [arm])
    if argv[1][-3:] == '.cc':
      loop_writer.Write(argv[2], argv[1])
    else:
      loop_writer.Write(argv[1], argv[2])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
