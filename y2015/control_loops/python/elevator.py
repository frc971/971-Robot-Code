#!/usr/bin/python

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
from frc971.control_loops.python import polytope
import numpy
import sys
import matplotlib
from matplotlib import pylab

class Elevator(control_loop.ControlLoop):
  def __init__(self, name="Elevator", mass=None):
    super(Elevator, self).__init__(name)
    # Stall Torque in N m
    self.stall_torque = 0.476
    # Stall Current in Amps
    self.stall_current = 80.730
    # Free Speed in RPM
    self.free_speed = 13906.0
    # Free Current in Amps
    self.free_current = 5.820
    # Mass of the elevator
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
    self.G = (56.0 / 12.0) * (84.0 / 14.0)
    # Pulley diameter
    self.r = 32 * 0.005 / numpy.pi / 2.0
    # Control loop time step
    self.dt = 0.005

    # Elevator left/right spring constant (N/m)
    self.spring = 800.0

    # State is [average position, average velocity,
    #           position difference/2, velocity difference/2]
    # Input is [V_left, V_right]

    C1 = self.spring / (self.mass * 0.5)
    C2 = self.Kt * self.G / (self.mass * 0.5 * self.r * self.R)
    C3 = self.G * self.G * self.Kt / (
        self.R  * self.r * self.r * self.mass * 0.5 * self.Kv)

    self.A_continuous = numpy.matrix(
        [[0, 1, 0, 0],
         [0, -C3, 0, 0],
         [0, 0, 0, 1],
         [0, 0, -C1 * 2.0, -C3]])

    print "Full speed is", C2 / C3 * 12.0

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

    print self.A

    controllability = controls.ctrb(self.A, self.B)
    print "Rank of augmented controllability matrix.", numpy.linalg.matrix_rank(
        controllability)

    q_pos = 0.02
    q_vel = 0.400
    q_pos_diff = 0.01
    q_vel_diff = 0.45
    self.Q = numpy.matrix([[(1.0 / (q_pos ** 2.0)), 0.0, 0.0, 0.0],
                           [0.0, (1.0 / (q_vel ** 2.0)), 0.0, 0.0],
                           [0.0, 0.0, (1.0 / (q_pos_diff ** 2.0)), 0.0],
                           [0.0, 0.0, 0.0, (1.0 / (q_vel_diff ** 2.0))]])

    self.R = numpy.matrix([[(1.0 / (12.0 ** 2.0)), 0.0],
                           [0.0, 1.0 / (12.0 ** 2.0)]])
    self.K = controls.dlqr(self.A, self.B, self.Q, self.R)
    print self.K

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


def run_test(elevator, initial_X, goal, max_separation_error=0.01,
             show_graph=False, iterations=200, controller_elevator=None,
             observer_elevator=None):
  """Runs the elevator plant with an initial condition and goal.

    The tests themselves are not terribly sophisticated; I just test for
    whether the goal has been reached and whether the separation goes
    outside of the initial and goal values by more than max_separation_error.
    Prints out something for a failure of either condition and returns
    False if tests fail.
    Args:
      elevator: elevator object to use.
      initial_X: starting state.
      goal: goal state.
      show_graph: Whether or not to display a graph showing the changing
           states and voltages.
      iterations: Number of timesteps to run the model for.
      controller_elevator: elevator object to get K from, or None if we should
          use elevator.
      observer_elevator: elevator object to use for the observer, or None if we
          should use the actual state.
  """

  elevator.X = initial_X

  if controller_elevator is None:
    controller_elevator = elevator

  if observer_elevator is not None:
    observer_elevator.X_hat = initial_X + 0.01
    observer_elevator.X_hat = initial_X

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
    X_hat = elevator.X
    if observer_elevator is not None:
      X_hat = observer_elevator.X_hat
      x_hat_avg.append(observer_elevator.X_hat[0, 0])
      x_hat_sep.append(observer_elevator.X_hat[2, 0] * sep_plot_gain)
    U = controller_elevator.K * (goal - X_hat)
    U = CapU(U)
    x_avg.append(elevator.X[0, 0])
    v_avg.append(elevator.X[1, 0])
    x_sep.append(elevator.X[2, 0] * sep_plot_gain)
    v_sep.append(elevator.X[3, 0])
    if observer_elevator is not None:
      observer_elevator.PredictObserver(U)
    elevator.Update(U)
    if observer_elevator is not None:
      observer_elevator.Y = elevator.Y
      observer_elevator.CorrectObserver(U)

    t.append(i * elevator.dt)
    u_left.append(U[0, 0])
    u_right.append(U[1, 0])

  print numpy.linalg.inv(elevator.A)
  print "delta time is ", elevator.dt
  print "Velocity at t=0 is ", x_avg[0], v_avg[0], x_sep[0], v_sep[0]
  print "Velocity at t=1+dt is ", x_avg[1], v_avg[1], x_sep[1], v_sep[1]

  if show_graph:
    pylab.subplot(2, 1, 1)
    pylab.plot(t, x_avg, label='x avg')
    pylab.plot(t, x_sep, label='x sep')
    if observer_elevator is not None:
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
  elevator = Elevator(mass=13 + loaded_mass)
  elevator_controller = Elevator(mass=13 + 15)
  observer_elevator = Elevator(mass=13 + 15)
  #observer_elevator = None

  # Test moving the elevator with constant separation.
  initial_X = numpy.matrix([[0.0], [0.0], [0.01], [0.0]])
  #initial_X = numpy.matrix([[0.0], [0.0], [0.00], [0.0]])
  R = numpy.matrix([[1.0], [0.0], [0.0], [0.0]])
  run_test(elevator, initial_X, R, controller_elevator=elevator_controller,
           observer_elevator=observer_elevator)

  # Write the generated constants out to a file.
  if len(argv) != 3:
    print "Expected .h file name and .cc file name for the elevator."
  else:
    namespaces = ['y2015', 'control_loops', 'fridge']
    elevator = Elevator("Elevator")
    loop_writer = control_loop.ControlLoopWriter("Elevator", [elevator],
                                                 namespaces=namespaces)
  if argv[1][-3:] == '.cc':
    loop_writer.Write(argv[2], argv[1])
  else:
    loop_writer.Write(argv[1], argv[2])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
