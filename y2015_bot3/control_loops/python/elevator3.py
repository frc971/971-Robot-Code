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

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')


class Elevator(control_loop.ControlLoop):
  def __init__(self, name="Elevator", mass=None):
    super(Elevator, self).__init__(name)
    # Stall Torque in N m
    self.stall_torque = 2.402
    # Stall Current in Amps
    self.stall_current = 126.145
    # Free Speed in RPM
    self.free_speed = 5015.562
    # Free Current in Amps
    self.free_current = 1.170
    # Mass of the Elevator
    if mass is None:
      self.mass = 5.0
    else:
      self.mass = mass

    # Number of motors
    self.num_motors = 2.0
    # Resistance of the motor
    self.resistance = 12.0 / self.stall_current
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
               (12.0 - self.resistance * self.free_current))
    # Torque constant
    self.Kt = (self.num_motors * self.stall_torque) / self.stall_current
    # Gear ratio
    self.G = 8
    # Radius of pulley
    self.r = 0.0254

    # Control loop time step
    self.dt = 0.005

    # State is [position, velocity]
    # Input is [Voltage]

    C1 = self.Kt * self.G * self.G / (self.Kv * self.resistance * self.r * self.r * self.mass)
    C2 = self.G * self.Kt / (self.resistance * self.r * self.mass)

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

    q_pos = 0.015
    q_vel = 0.5
    self.Q = numpy.matrix([[(1.0 / (q_pos ** 2.0)), 0.0],
                           [0.0, (1.0 / (q_vel ** 2.0))]])

    self.R = numpy.matrix([[(1.0 / (12.0 ** 2.0))]])
    self.K = controls.dlqr(self.A, self.B, self.Q, self.R)

    glog.info('K %s', str(self.K))
    glog.info('Poles are %s', str(numpy.linalg.eig(self.A - self.B * self.K)[0]))

    self.rpl = 0.30
    self.ipl = 0.10
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    q_pos = 0.05
    q_vel = 2.65
    self.Q = numpy.matrix([[(q_pos ** 2.0), 0.0],
                           [0.0, (q_vel ** 2.0)]])

    r_volts = 0.025
    self.R = numpy.matrix([[(r_volts ** 2.0)]])

    self.KalmanGain, self.Q_steady = controls.kalman(
        A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)

    self.L = self.A * self.KalmanGain
    glog.info('KalL is %s', str(self.L))

    # The box formed by U_min and U_max must encompass all possible values,
    # or else Austin's code gets angry.
    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    self.InitializeState()

class IntegralElevator(Elevator):
  def __init__(self, name="IntegralElevator", mass=None):
    super(IntegralElevator, self).__init__(name=name, mass=mass)

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
    q_voltage = 6.0
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

  def run_test(self, elevator, goal,
               iterations=200, controller_elevator=None,
               observer_elevator=None):
    """Runs the Elevator plant with an initial condition and goal.

      Args:
        Elevator: elevator object to use.
        initial_X: starting state.
        goal: goal state.
        iterations: Number of timesteps to run the model for.
        controller_Elevator: elevator object to get K from, or None if we should
            use Elevator.
        observer_Elevator: elevator object to use for the observer, or None if we should
            use the actual state.
    """

    if controller_elevator is None:
      controller_elevator = elevator

    vbat = 10.0
    if self.t:
      initial_t = self.t[-1] + elevator.dt
    else:
      initial_t = 0
    for i in xrange(iterations):
      X_hat = elevator.X
      if observer_elevator is not None:
        X_hat = observer_elevator.X_hat
        self.x_hat.append(observer_elevator.X_hat[0, 0])
      gravity_compensation =  9.8 * elevator.mass * elevator.r / elevator.G / elevator.Kt * elevator.resistance

      U = controller_elevator.K * (goal - X_hat)
      U[0, 0] = numpy.clip(U[0, 0], -vbat , vbat )
      self.x.append(elevator.X[0, 0])
      if self.v:
        last_v = self.v[-1]
      else:
        last_v = 0
      self.v.append(elevator.X[1, 0])
      self.a.append((self.v[-1] - last_v) / elevator.dt)

      if observer_elevator is not None:
        observer_elevator.Y = elevator.Y
        observer_elevator.CorrectObserver(U)

      elevator.Update(U - gravity_compensation)

      if observer_elevator is not None:
        observer_elevator.PredictObserver(U)

      self.t.append(initial_t + i * elevator.dt)
      self.u.append(U[0, 0])
#      if numpy.abs((goal - X_hat)[0:2, 0]).sum() < .025:
#        print "Time: ", self.t[-1]
#        break

    glog.debug('Time: %f', self.t[-1])


  def Plot(self):
    pylab.subplot(3, 1, 1)
    pylab.plot(self.t, self.x, label='x')
    pylab.plot(self.t, self.x_hat, label='x_hat')
    pylab.legend()

    pylab.subplot(3, 1, 2)
    pylab.plot(self.t, self.u, label='u')

    pylab.subplot(3, 1, 3)
    pylab.plot(self.t, self.a, label='a')

    pylab.legend()
    pylab.show()


def main(argv):
  argv = FLAGS(argv)

  loaded_mass = 7+4.0
  #loaded_mass = 0
  #observer_elevator = None

  # Test moving the Elevator
  initial_X = numpy.matrix([[0.0], [0.0]])
  up_R = numpy.matrix([[0.4572], [0.0], [0.0]])
  down_R = numpy.matrix([[0.0], [0.0], [0.0]])
  totemass = 3.54
  scenario_plotter = ScenarioPlotter()

  elevator_controller = IntegralElevator(mass=4*totemass + loaded_mass)
  observer_elevator = IntegralElevator(mass=4*totemass + loaded_mass)

  for i in xrange(0, 7):
    elevator = Elevator(mass=i*totemass + loaded_mass)
    glog.info('Actual poles are %s', str(numpy.linalg.eig(elevator.A - elevator.B * elevator_controller.K[0, 0:2])[0]))

    elevator.X = initial_X
    scenario_plotter.run_test(elevator, goal=up_R, controller_elevator=elevator_controller,
                              observer_elevator=observer_elevator, iterations=200)
    scenario_plotter.run_test(elevator, goal=down_R, controller_elevator=elevator_controller,
                              observer_elevator=observer_elevator, iterations=200)

  if FLAGS.plot:
    scenario_plotter.Plot()

  # Write the generated constants out to a file.
  if len(argv) != 5:
    glog.fatal('Expected .h file name and .cc file name for the Elevator and integral elevator.')
  else:
    design_mass = 4*totemass + loaded_mass
    elevator = Elevator("Elevator", mass=design_mass)
    loop_writer = control_loop.ControlLoopWriter("Elevator", [elevator],
                                                 namespaces=['y2015_bot3', 'control_loops', 'elevator'])
    loop_writer.Write(argv[1], argv[2])

    integral_elevator = IntegralElevator("IntegralElevator", mass=design_mass)
    integral_loop_writer = control_loop.ControlLoopWriter("IntegralElevator", [integral_elevator],
                                                          namespaces=['y2015_bot3', 'control_loops', 'elevator'])
    integral_loop_writer.Write(argv[3], argv[4])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
