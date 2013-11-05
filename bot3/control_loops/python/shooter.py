#!/usr/bin/python

import numpy
import sys
from matplotlib import pylab
import control_loop
import slycot

class Shooter(control_loop.ControlLoop):
  def __init__(self):
    super(Shooter, self).__init__("Shooter")
    # Stall Torque in N m
    self.stall_torque = 2.42211227883219
    # Stall Current in Amps
    self.stall_current = 133
    # Free Speed in RPM
    self.free_speed = 4650.0
    # Free Current in Amps
    self.free_current = 2.7
    # Moment of inertia of the shooter wheel in kg m^2
    self.J = 0.0032
    # Resistance of the motor, divided by 2 to account for the 2 motors
    self.R = 12.0 / self.stall_current
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
              (12.0 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratio
    self.G = 40.0 / 34.0
    # Control loop time step
    self.dt = 0.01

    # State feedback matrices
    self.A_continuous = numpy.matrix(
        [[-self.Kt / self.Kv / (self.J * self.G * self.G * self.R)]])
    self.B_continuous = numpy.matrix(
        [[self.Kt / (self.J * self.G * self.R)]])
    self.C = numpy.matrix([[1]])
    self.D = numpy.matrix([[0]])

    self.A, self.B = self.ContinuousToDiscrete(self.A_continuous, self.B_continuous,
                              self.dt)

    self.InitializeState()

    self.PlaceControllerPoles([.881])
    print self.K
    self.R_LQR = numpy.matrix([[0]])
    self.P = slycot.sb02od(1, 1, self.A, self.B, self.C * self.C.T, self.R, 'D')[0]
    self.K = (numpy.linalg.inv(self.R_LQR + self.B.T * self.P * self.B)
             * self.B.T * self.P * self.A)
    print self.K


    self.PlaceObserverPoles([0.45])

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
# pylab.plot(range(sim_delay, shooter_data.shape[0] + sim_delay),
#            simulated_x, label='Simulation')
# pylab.plot(range(shooter_data.shape[0]), real_x, label='Reality')
# pylab.plot(range(shooter_data.shape[0]), x_vel, label='Velocity')
# pylab.legend()
# pylab.show()

  # Simulate the closed loop response of the system to a step input.
  shooter = Shooter()
  close_loop_x = []
  close_loop_U = []
  velocity_goal = 400
  R = numpy.matrix([[velocity_goal]])
  goal = False
  for i in pylab.linspace(0,1.99,200):
    # Iterate the position up.
    R = numpy.matrix([[velocity_goal]])
    U = numpy.clip(shooter.K * (R - shooter.X_hat) +
                   (numpy.identity(shooter.A.shape[0]) - shooter.A) * R / shooter.B,
                   shooter.U_min, shooter.U_max)
    shooter.UpdateObserver(U)
    shooter.Update(U)
    close_loop_x.append(shooter.X[0, 0])
    close_loop_U.append(U[0, 0])
    if (abs(R[0, 0] - shooter.X[0, 0]) < R[0, 0]* 0.01 and (not goal)):
      goal = True
      print i

  #pylab.plotfile("shooter.csv", (0,1))
  pylab.plot(pylab.linspace(0,1.99,200), close_loop_U)
  #pylab.plotfile("shooter.csv", (0,2))
  pylab.plot(pylab.linspace(0,1.99,200), close_loop_x)
  pylab.show()

  # Simulate spin down.
  spin_down_x = [];
  for _ in xrange(150):
    U = 0
    shooter.UpdateObserver(U)
    shooter.Update(U)
    spin_down_x.append(shooter.X[0, 0])

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
