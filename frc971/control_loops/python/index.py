#!/usr/bin/python

import control_loop
import numpy
import sys
from matplotlib import pylab

class Index(control_loop.ControlLoop):
  def __init__(self):
    super(Index, self).__init__("Index")
    # Stall Torque in N m
    self.stall_torque = 0.4862
    # Stall Current in Amps
    self.stall_current = 85
    # Free Speed in RPM
    self.free_speed = 19300.0
    # Free Current in Amps
    self.free_current = 1.5
    # Moment of inertia of the index in kg m^2
    self.J = 0.00013
    # Resistance of the motor
    self.R = 12.0 / self.stall_current + 0.024 + .003
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
               (13.5 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratio
    self.G = 1.0 / ((40.0 / 11.0) * (34.0 / 30.0))
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

    self.PlaceControllerPoles([.5, .55])

    self.rpl = .05
    self.ipl = 0.008
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])


def main(argv):
  # Simulate the response of the system to a step input.
  index = Index()
  simulated_x = []
  simulated_v = []
  for _ in xrange(100):
    index.Update(numpy.matrix([[12.0]]))
    simulated_x.append(index.X[0, 0])
    simulated_v.append(index.X[1, 0])

  pylab.plot(range(100), simulated_v)
  pylab.show()

  # Simulate the closed loop response of the system to a step input.
  index = Index()
  close_loop_x = []
  R = numpy.matrix([[1.0], [0.0]])
  for _ in xrange(100):
    U = numpy.clip(index.K * (R - index.X_hat), index.U_min, index.U_max)
    index.UpdateObserver(U)
    index.Update(U)
    close_loop_x.append(index.X[0, 0])

  pylab.plot(range(100), close_loop_x)
  pylab.show()

  # Write the generated constants out to a file.
  if len(argv) != 3:
    print "Expected .h file name and .c file name"
  else:
    if argv[1][-3:] == '.cc':
      print '.cc file is second'
    else:
      index.DumpHeaderFile(argv[1])
      index.DumpCppFile(argv[2], argv[1])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
