#!/usr/bin/python

from frc971.control_loops.python import control_loop
import numpy
import sys
from matplotlib import pylab

class Shooter(control_loop.ControlLoop):
  def __init__(self):
    super(Shooter, self).__init__("Shooter")
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
    # [position, angular velocity]
    self.A_continuous = numpy.matrix(
        [[0, 1],
         [0, -self.Kt / self.Kv / (self.J * self.G * self.G * self.R)]])
    self.B_continuous = numpy.matrix(
        [[0],
         [self.Kt / (self.J * self.G * self.R)]])
    self.C = numpy.matrix([[1, 0]])
    self.D = numpy.matrix([[0]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    self.PlaceControllerPoles([.6, .981])

    self.rpl = .45
    self.ipl = 0.07
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])


def main(argv):
  if len(argv) != 3:
    print "Expected .h file name and .cc file name"
  else:
    namespaces = ['y2016', 'control_loops', 'shooter']
    shooter = Shooter()
    loop_writer = control_loop.ControlLoopWriter("Shooter", [shooter],
                                                 namespaces=namespaces)
    if argv[1][-3:] == '.cc':
      loop_writer.Write(argv[2], argv[1])
    else:
      loop_writer.Write(argv[1], argv[2])


if __name__ == '__main__':
  sys.exit(main(sys.argv))
