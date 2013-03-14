#!/usr/bin/python

import control_loop
import numpy
import sys
from matplotlib import pylab

class AngleAdjust(control_loop.ControlLoop):
  def __init__(self):
    super(AngleAdjust, self).__init__("AngleAdjust")
    # Stall Torque in N m
    self.stall_torque = .428
    # Stall Current in Amps
    self.stall_current = 63.8
    # Free Speed in RPM
    self.free_speed = 14900.0
    # Free Current in Amps
    self.free_current = 1.2
    # Moment of inertia of the angle adjust about the shooter's pivot in kg m^2
    self.J = 9.4
    # Resistance of the motor
    self.R = 12.0 / self.stall_current
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
               (12.0 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratio of the gearbox multiplied by the ratio of the radii of
    # the output and the angle adjust curve, which is essentially another gear.
    self.G = (1.0 / 50.0) * (0.01905 / 0.41964)
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

    self.PlaceControllerPoles([.45, .8])

    self.rpl = .05
    self.ipl = 0.008
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

def main(argv):
  # Simulate the response of the system to a step input.
  angle_adjust_data = numpy.genfromtxt(
      'angle_adjust/angle_adjust_data.csv', delimiter=',')
  angle_adjust = AngleAdjust()
  simulated_x = []
  real_x = []
  initial_x = angle_adjust_data[0, 2]
  for i in xrange(angle_adjust_data.shape[0]):
    angle_adjust.Update(numpy.matrix([[angle_adjust_data[i, 1] - 0.7]]))
    simulated_x.append(angle_adjust.X[0, 0])
    x_offset = angle_adjust_data[i, 2] - initial_x
    real_x.append(x_offset)

  sim_delay = 2
  pylab.plot(range(sim_delay, angle_adjust_data.shape[0] + sim_delay),
             simulated_x, label='Simulation')
  pylab.plot(range(angle_adjust_data.shape[0]), real_x, label='Reality')
  pylab.legend()
  pylab.show()

  # Simulate the closed loop response of the system to a step input.
  angle_adjust = AngleAdjust()
  close_loop_x = []
  R = numpy.matrix([[1.0], [0.0]])
  for _ in xrange(100):
    U = numpy.clip(angle_adjust.K * (R - angle_adjust.X_hat), angle_adjust.U_min, angle_adjust.U_max)
    angle_adjust.UpdateObserver(U)
    angle_adjust.Update(U)
    close_loop_x.append(angle_adjust.X[0, 0])

  pylab.plot(range(100), close_loop_x)
  pylab.show()

  # Write the generated constants out to a file.
  if len(argv) != 3:
    print "Expected .cc file name and .h file name"
  else:
    loop_writer = control_loop.ControlLoopWriter("AngleAdjust", [angle_adjust])
    if argv[1][-3:] == '.cc':
      loop_writer.Write(argv[2], argv[1])
    else:
      loop_writer.Write(argv[1], argv[2])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
