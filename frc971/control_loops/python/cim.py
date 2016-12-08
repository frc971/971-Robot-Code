#!/usr/bin/python

from frc971.control_loops.python import control_loop
import numpy

class CIM(control_loop.ControlLoop):
  def __init__(self):
    super(CIM, self).__init__("CIM")
    # Stall Torque in N m
    self.stall_torque = 2.42
    # Stall Current in Amps
    self.stall_current = 133
    # Free Speed in RPM
    self.free_speed = 4650.0
    # Free Current in Amps
    self.free_current = 2.7
    # Moment of inertia of the CIM in kg m^2
    self.J = 0.0001
    # Resistance of the motor, divided by 2 to account for the 2 motors
    self.resistance = 12.0 / self.stall_current
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
              (12.0 - self.resistance * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Control loop time step
    self.dt = 0.005

    # State feedback matrices
    self.A_continuous = numpy.matrix(
        [[-self.Kt / self.Kv / (self.J * self.resistance)]])
    self.B_continuous = numpy.matrix(
        [[self.Kt / (self.J * self.resistance)]])
    self.C = numpy.matrix([[1]])
    self.D = numpy.matrix([[0]])

    self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                               self.B_continuous, self.dt)

    self.PlaceControllerPoles([0.01])
    self.PlaceObserverPoles([0.01])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    self.InitializeState()
