#!/usr/bin/python

import control_loop
import numpy
import sys
from matplotlib import pylab

class Drivetrain(control_loop.ControlLoop):
  def __init__(self):
    super(Drivetrain, self).__init__("Drivetrain")
    # Stall Torque in N m
    self.stall_torque = 2.42
    # Stall Current in Amps
    self.stall_current = 133
    # Free Speed in RPM. Used number from last year.
    self.free_speed = 4650.0
    # Free Current in Amps
    self.free_current = 2.7
    # Moment of inertia of the drivetrain in kg m^2
    # Just borrowed from last year.
    self.J = 7.0
    # Mass of the robot, in kg.
    self.m = 68
    # Radius of the robot, in meters (from last year).
    self.rb = 0.617998644 / 2.0
    # Radius of the wheels, in meters.
    self.r = .04445
    # Resistance of the motor, divided by the number of motors.
    self.R = 12.0 / self.stall_current / 6
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
               (12.0 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratios
    self.G_low = 16.0 / 60.0 * 19.0 / 50.0
    self.G_high = 28.0 / 48.0 * 19.0 / 50.0
    self.G = self.G_low
    # Control loop time step
    self.dt = 0.01

    # These describe the way that a given side of a robot will be influenced
    # by the other side. Units of 1 / kg.
    self.msp = 1.0 / self.m + self.rb * self.rb / self.J
    self.msn = 1.0 / self.m - self.rb * self.rb / self.J
    # The calculations which we will need for A and B.
    self.tc = -self.Kt / self.Kv / (self.G * self.G * self.R * self.r * self.r)
    self.mp = self.Kt / (self.G * self.R * self.r)

    # State feedback matrices
    # X will be of the format
    # [[position1], [velocity1], [position2], velocity2]]
    self.A_continuous = numpy.matrix(
        [[0, 1, 0, 0],
         [0, self.msp * self.tc, 0, self.msn * self.tc],
         [0, 0, 0, 1],
         [0, self.msn * self.tc, 0, self.msp * self.tc]])
    self.B_continuous = numpy.matrix(
        [[0, 0],
         [self.msp * self.mp, self.msn * self.mp],
         [0, 0],
         [self.msn * self.mp, self.msp * self.mp]])
    self.C = numpy.matrix([[1, 0, 0, 0],
                           [0, 0, 1, 0]])
    self.D = numpy.matrix([[0, 0],
                           [0, 0]])

    self.ContinuousToDiscrete(self.A_continuous, self.B_continuous,
                              self.dt, self.C)

    # Poles from last year.
    self.hp = 0.8
    self.lp = 0.85
    self.PlaceControllerPoles([self.hp, self.hp, self.lp, self.lp])

    print self.K

    self.hlp = 0.07
    self.llp = 0.09
    self.PlaceObserverPoles([self.hlp, self.hlp, self.llp, self.llp])

    self.U_max = numpy.matrix([[12.0], [12.0]])
    self.U_min = numpy.matrix([[-12.0], [-12.0]])

def main(argv):
  # Simulate the response of the system to a step input.
  drivetrain = Drivetrain()
  simulated_left = []
  simulated_right = []
  for _ in xrange(100):
    drivetrain.Update(numpy.matrix([[12.0], [12.0]]))
    simulated_left.append(drivetrain.X[0, 0])
    simulated_right.append(drivetrain.X[2, 0])

  pylab.plot(range(100), simulated_left)
  pylab.plot(range(100), simulated_right)
  pylab.show()

  # Simulate forwards motion.
  drivetrain = Drivetrain()
  close_loop_left = []
  close_loop_right = []
  R = numpy.matrix([[1.0], [0.0], [1.0], [0.0]])
  for _ in xrange(100):
    U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat),
                   drivetrain.U_min, drivetrain.U_max)
    drivetrain.UpdateObserver(U)
    drivetrain.Update(U)
    close_loop_left.append(drivetrain.X[0, 0])
    close_loop_right.append(drivetrain.X[2, 0])

  pylab.plot(range(100), close_loop_left)
  pylab.plot(range(100), close_loop_right)
  pylab.show()

  # Try turning in place
  drivetrain = Drivetrain()
  close_loop_left = []
  close_loop_right = []
  R = numpy.matrix([[-1.0], [0.0], [1.0], [0.0]])
  for _ in xrange(100):
    U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat),
                   drivetrain.U_min, drivetrain.U_max)
    drivetrain.UpdateObserver(U)
    drivetrain.Update(U)
    close_loop_left.append(drivetrain.X[0, 0])
    close_loop_right.append(drivetrain.X[2, 0])

  pylab.plot(range(100), close_loop_left)
  pylab.plot(range(100), close_loop_right)
  pylab.show()

  # Try turning just one side.
  drivetrain = Drivetrain()
  close_loop_left = []
  close_loop_right = []
  R = numpy.matrix([[0.0], [0.0], [1.0], [0.0]])
  for _ in xrange(100):
    U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat),
                   drivetrain.U_min, drivetrain.U_max)
    drivetrain.UpdateObserver(U)
    drivetrain.Update(U)
    close_loop_left.append(drivetrain.X[0, 0])
    close_loop_right.append(drivetrain.X[2, 0])

  pylab.plot(range(100), close_loop_left)
  pylab.plot(range(100), close_loop_right)
  pylab.show()

  # Write the generated constants out to a file.
  if len(argv) != 3:
    print "Expected .h file name and .cc file name"
  else:
    if argv[1][-3:] == '.cc':
      print '.cc file is second'
    else:
      drivetrain.DumpHeaderFile(argv[1])
      drivetrain.DumpCppFile(argv[2], argv[1])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
