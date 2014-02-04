#!/usr/bin/python

import control_loop
import numpy
import sys
from matplotlib import pylab

class Shooter(control_loop.ControlLoop):
  def __init__(self, name="RawShooter"):
    super(Shooter, self).__init__(name)
    # Stall Torque in N m
    self.stall_torque = .4982
    # Stall Current in Amps
    self.stall_current = 85
    # Free Speed in RPM
    self.free_speed = 19300.0
    # Free Current in Amps
    self.free_current = 1.2
    # Moment of inertia of the shooter in kg m^2
    # Calculate Moment of Irtia
    self.J = 0.3
    # Resistance of the motor, divided by the number of motors.
    self.R = 12.0 / self.stall_current / 2.0
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
               (13.5 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Spring constant for the springs, N/m
    self.Ks = 3600.0
    # Gear ratio
    self.G = 1.0 / ((84.0 / 20.0) * (50.0 / 14.0) * (40.0 / 14.0) * (40.0 / 12.0))
    # Control loop time step
    self.dt = 0.01


    # State feedback matrices
    # TODO(james): Make this work with origins other than at kx = 0.
    self.A_continuous = numpy.matrix(
        [[0, 1],
         [-self.Ks * 0.01 / self.J,
          -self.Kt / self.Kv / (self.J * self.G * self.G * self.R)]])
    self.B_continuous = numpy.matrix(
        [[0],
         [self.Kt / (self.J * self.G * self.R)]])
    self.C = numpy.matrix([[1, 0]])
    self.D = numpy.matrix([[0]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    self.PlaceControllerPoles([0.85, 0.45])

    self.rpl = .05
    self.ipl = 0.008
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    self.InitializeState()


class ShooterDeltaU(Shooter):
  def __init__(self, name="Shooter"):
    super(ShooterDeltaU, self).__init__(name)
    A_unaugmented = self.A
    B_unaugmented = self.B

    self.A = numpy.matrix([[0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0]])
    self.A[0:2, 0:2] = A_unaugmented
    self.A[0:2, 2] = B_unaugmented

    self.B = numpy.matrix([[0.0],
                           [0.0],
                           [1.0]])

    self.C = numpy.matrix([[1.0, 0.0, 0.0]])
    self.D = numpy.matrix([[0.0]])

    self.PlaceControllerPoles([0.55, 0.45, 0.80])

    print "K"
    print self.K
    print "Placed controller poles are"
    print numpy.linalg.eig(self.A - self.B * self.K)[0]

    self.rpl = .05
    self.ipl = 0.008
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl, 0.90])
    print "Placed observer poles are"
    print numpy.linalg.eig(self.A - self.L * self.C)[0]

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    self.InitializeState()


def ClipDeltaU(shooter, delta_u):
  old_u = numpy.matrix([[shooter.X[2, 0]]])
  new_u = numpy.clip(old_u + delta_u, shooter.U_min, shooter.U_max)
  return new_u - old_u

def main(argv):
  # Simulate the response of the system to a step input.
  shooter = Shooter()
  simulated_x = []
  for _ in xrange(1000):
    shooter.Update(numpy.matrix([[2.0]]))
    simulated_x.append(shooter.X[0, 0])

  pylab.plot(range(1000), simulated_x)
  pylab.show()

  # Simulate the response of the system to a goal.
  shooter = Shooter()
  close_loop_x = []
  close_loop_u = []
  R = numpy.matrix([[1.0], [0.0]])
  for _ in xrange(100):
    feed_forward = (-numpy.linalg.lstsq(shooter.B_continuous, numpy.identity(
                         shooter.B_continuous.shape[0]))[0] *
                   shooter.A_continuous * R)
    U = numpy.clip(shooter.K * (R - shooter.X_hat) + feed_forward,
                   shooter.U_min, shooter.U_max)
#U = ClipDeltaU(shooter, U)
    shooter.UpdateObserver(U)
    shooter.Update(U)
    close_loop_x.append(shooter.X[0, 0] * 10)
    close_loop_u.append(U[0, 0])

  pylab.plot(range(100), close_loop_x)
  pylab.plot(range(100), close_loop_u)
  pylab.show()

  # Write the generated constants out to a file.
  if len(argv) != 3:
    print "Expected .h file name and .cc file name for"
    print "both the plant and unaugmented plant"
  else:
    unaug_shooter = Shooter("RawShooter")
    unaug_loop_writer = control_loop.ControlLoopWriter("RawShooter",
                                                       [unaug_shooter])
    #if argv[3][-3:] == '.cc':
    #  unaug_loop_writer.Write(argv[4], argv[3])
    #else:
    #  unaug_loop_writer.Write(argv[3], argv[4])

    loop_writer = control_loop.ControlLoopWriter("Shooter", [shooter])
    if argv[1][-3:] == '.cc':
      loop_writer.Write(argv[2], argv[1])
    else:
      loop_writer.Write(argv[1], argv[2])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
