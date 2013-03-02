#!/usr/bin/python

import control_loop
import numpy
import sys
from matplotlib import pylab

class Transfer(control_loop.ControlLoop):
  def __init__(self):
    super(Transfer, self).__init__("Transfer")
    # Stall Torque in N m
    self.stall_torque = 0.4862
    # Stall Current in Amps
    self.stall_current = 85
    # Free Speed in RPM
    self.free_speed = 19300.0
    # Free Current in Amps
    self.free_current = 1.5
    # Moment of inertia of the transfer in kg m^2
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

    self.PlaceControllerPoles([.75, .6])

    self.rpl = .05
    self.ipl = 0.008
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])


def main(argv):
  # Simulate the response of the system to a step input.
  transfer = Transfer()
  simulated_x = []
  simulated_v = []
  for _ in xrange(100):
    transfer.Update(numpy.matrix([[12.0]]))
    simulated_x.append(transfer.X[0, 0])
    simulated_v.append(transfer.X[1, 0])

  pylab.plot(range(100), simulated_v)
  pylab.show()

  # Simulate the closed loop response of the system to a step input.
  transfer = Transfer()
  close_loop_x = []
  R = numpy.matrix([[1.0], [0.0]])
  for _ in xrange(100):
    U = numpy.clip(transfer.K * (R - transfer.X_hat), transfer.U_min, transfer.U_max)
    transfer.UpdateObserver(U)
    transfer.Update(U)
    close_loop_x.append(transfer.X[0, 0])

  #pylab.plot(range(100), close_loop_x)
  #pylab.show()

  # Write the generated constants out to a file.
  if len(argv) != 3:
    print "Expected .cc file name and .h file name"
  else:
    transfer.DumpHeaderFile(argv[1])
    transfer.DumpCppFile(argv[2], argv[1])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
