#!/usr/bin/python

import control_loop
import numpy
import sys
from matplotlib import pylab

class Wrist(control_loop.ControlLoop):
  def __init__(self):
    super(Wrist, self).__init__("Wrist")
    # Stall Torque in N m
    self.stall_torque = 1.4
    # Stall Current in Amps
    self.stall_current = 86
    # Free Speed in RPM
    self.free_speed = 6200.0
    # Free Current in Amps
    self.free_current = 1.5
    # Moment of inertia of the wrist in kg m^2
    # TODO(aschuh): Measure this in reality.  It doesn't seem high enough.
    # James measured 0.51, but that can't be right given what I am seeing.
    self.J = 1.51
    # Resistance of the motor
    self.R = 12.0 / self.stall_current + 0.024 + .003
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
               (13.5 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratio
    self.G = 1.0 / ((84.0 / 20.0) * (50.0 / 14.0) * (40.0 / 14.0) * (40.0 / 12.0))
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

    self.PlaceControllerPoles([.84, .84])

    print self.K

    self.rpl = .05
    self.ipl = 0.008
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

def main(argv):
  # Simulate the response of the system to a step input.
  wrist = Wrist()
  simulated_x = []
  for _ in xrange(100):
    wrist.Update(numpy.matrix([[12.0]]))
    simulated_x.append(wrist.X[0, 0])

  pylab.plot(range(100), simulated_x)
  pylab.show()

  # Simulate the closed loop response of the system to a step input.
  wrist = Wrist()
  close_loop_x = []
  R = numpy.matrix([[1.0], [0.0]])
  for _ in xrange(100):
    U = numpy.clip(wrist.K * (R - wrist.X_hat), wrist.U_min, wrist.U_max)
    wrist.UpdateObserver(U)
    wrist.Update(U)
    close_loop_x.append(wrist.X[0, 0])

  pylab.plot(range(100), close_loop_x)
  pylab.show()

  # Write the generated constants out to a file.
  if len(argv) != 3:
    print "Expected .h file name and .cc file name"
  else:
    if argv[1][-3:] == '.cc':
      print '.cc file is second'
    else:
      wrist.DumpHeaderFile(argv[1])
      wrist.DumpCppFile(argv[2], argv[1])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
