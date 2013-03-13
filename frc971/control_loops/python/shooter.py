#!/usr/bin/python

import numpy
import sys
from matplotlib import pylab
import control_loop

class Shooter(control_loop.ControlLoop):
  def __init__(self):
    super(Shooter, self).__init__("Shooter")
    # Stall Torque in N m
    self.stall_torque = 0.49819248
    # Stall Current in Amps
    self.stall_current = 85
    # Free Speed in RPM
    self.free_speed = 19300.0 - 1500.0
    # Free Current in Amps
    self.free_current = 1.4
    # Moment of inertia of the shooter wheel in kg m^2
    self.J = 0.0032
    # Resistance of the motor, divided by 2 to account for the 2 motors
    self.R = 12.0 / self.stall_current / 2
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
              (12.0 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratio
    self.G = 11.0 / 34.0
    # Control loop time step
    self.dt = 0.01

    # State feedback matrices
    self.A_continuous = numpy.matrix(
        [[0, 1],
         [0, -self.Kt / self.Kv / (self.J * self.G * self.G * self.R)]])
    self.B_continuous = numpy.matrix(
        [[0],
         [self.Kt / (self.J * self.G * self.R)]])
    self.C = numpy.matrix([[1, 0]])
    self.D = numpy.matrix([[0]])

    self.ContinuousToDiscrete(self.A_continuous, self.B_continuous,
                              self.dt, self.C)

    self.PlaceControllerPoles([.6, .981])

    self.rpl = .45
    self.ipl = 0.07
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])


def main(argv):
  # Simulate the response of the system to a step input.
  shooter_data = numpy.genfromtxt('shooter/shooter_data.csv', delimiter=',')
  shooter = Shooter()
  simulated_x = []
  real_x = []
  x_vel = []
  initial_x = shooter_data[0, 2]
  last_x = initial_x
  for i in xrange(shooter_data.shape[0]):
    shooter.Update(numpy.matrix([[shooter_data[i, 1]]]))
    simulated_x.append(shooter.X[0, 0])
    x_offset = shooter_data[i, 2] - initial_x
    real_x.append(x_offset)
    x_vel.append((shooter_data[i, 2] - last_x) * 100.0)
    last_x = shooter_data[i, 2]

  sim_delay = 1
  pylab.plot(range(sim_delay, shooter_data.shape[0] + sim_delay),
             simulated_x, label='Simulation')
  pylab.plot(range(shooter_data.shape[0]), real_x, label='Reality')
  pylab.plot(range(shooter_data.shape[0]), x_vel, label='Velocity')
  pylab.legend()
  pylab.show()

  # Simulate the closed loop response of the system to a step input.
  shooter = Shooter()
  close_loop_x = []
  close_loop_U = []
  velocity_goal = 300
  R = numpy.matrix([[0.0], [velocity_goal]])
  for _ in pylab.linspace(0,1.99,200):
    # Iterate the position up.
    R = numpy.matrix([[R[0, 0] + 10.5], [velocity_goal]])
    # Prevents the position goal from going beyond what is necessary.
    velocity_weight_scalar = 0.35
    max_reference = (
        (shooter.U_max[0, 0] - velocity_weight_scalar *
         (velocity_goal - shooter.X_hat[1, 0]) * shooter.K[0, 1]) /
         shooter.K[0, 0] +
         shooter.X_hat[0, 0])
    min_reference = (
        (shooter.U_min[0, 0] - velocity_weight_scalar *
         (velocity_goal - shooter.X_hat[1, 0]) * shooter.K[0, 1]) /
         shooter.K[0, 0] +
         shooter.X_hat[0, 0])
    R[0, 0] = numpy.clip(R[0, 0], min_reference, max_reference)
    U = numpy.clip(shooter.K * (R - shooter.X_hat),
                   shooter.U_min, shooter.U_max)
    shooter.UpdateObserver(U)
    shooter.Update(U)
    close_loop_x.append(shooter.X[1, 0])
    close_loop_U.append(U[0, 0])

  #pylab.plotfile("shooter.csv", (0,1))
  #pylab.plot(pylab.linspace(0,1.99,200), close_loop_U, 'ro')
  #pylab.plotfile("shooter.csv", (0,2))
  pylab.plot(pylab.linspace(0,1.99,200), close_loop_x, 'ro')
  pylab.show()

  # Simulate spin down.
  spin_down_x = [];
  R = numpy.matrix([[50.0], [0.0]])
  for _ in xrange(150):
    U = 0
    shooter.UpdateObserver(U)
    shooter.Update(U)
    spin_down_x.append(shooter.X[1, 0])

  #pylab.plot(range(150), spin_down_x)
  #pylab.show()

  if len(argv) != 3:
    print "Expected .h file name and .cc file name"
  else:
    loop_writer = control_loop.ControlLoopWriter("Shooter", [shooter])
    if argv[1][-3:] == '.cc':
      loop_writer.Write(argv[2], argv[1])
    else:
      loop_writer.Write(argv[1], argv[2])


if __name__ == '__main__':
  sys.exit(main(sys.argv))
