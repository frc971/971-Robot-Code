#!/usr/bin/python

import control_loop
import numpy
import sys
from matplotlib import pylab

class Claw(control_loop.ControlLoop):
  def __init__(self, name="RawClaw"):
    super(Claw, self).__init__(name)
    # Stall Torque in N m
    self.stall_torque = 2.42
    # Stall Current in Amps
    self.stall_current = 133
    # Free Speed in RPM, pulled from drivetrain
    self.free_speed = 4650.0
    # Free Current in Amps
    self.free_current = 2.7
    # Moment of inertia of the claw in kg m^2
    # approzimately 0.76 (without ball) in CAD
    self.J = 0.76
    # Resistance of the motor
    self.R = 12.0 / self.stall_current + 0.024 + .003
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
               (13.5 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratio
    self.G = 14.0 / 48.0 * 18.0 / 32.0 * 18.0 / 66.0 * 12.0 / 60.0
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


class ClawDeltaU(Claw):
  def __init__(self, name="Claw"):
    super(ClawDeltaU, self).__init__(name)
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

    self.PlaceControllerPoles([0.55, 0.35, 0.80])

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


def ClipDeltaU(claw, delta_u):
  old_u = numpy.matrix([[claw.X[2, 0]]])
  new_u = numpy.clip(old_u + delta_u, claw.U_min, claw.U_max)
  return new_u - old_u

def main(argv):
  # Simulate the response of the system to a step input.
  claw = ClawDeltaU()
  simulated_x = []
  for _ in xrange(100):
    claw.Update(numpy.matrix([[12.0]]))
    simulated_x.append(claw.X[0, 0])

  pylab.plot(range(100), simulated_x)
  pylab.show()

  # Simulate the closed loop response of the system to a step input.
  top_claw = ClawDeltaU("TopClaw")
  close_loop_x = []
  close_loop_u = []
  R = numpy.matrix([[1.0], [0.0], [0.0]])
  top_claw.X[2, 0] = -5
  for _ in xrange(100):
    U = numpy.clip(top_claw.K * (R - top_claw.X_hat), top_claw.U_min, top_claw.U_max)
    U = ClipDeltaU(top_claw, U)
    top_claw.UpdateObserver(U)
    top_claw.Update(U)
    close_loop_x.append(top_claw.X[0, 0] * 10)
    close_loop_u.append(top_claw.X[2, 0])

  pylab.plot(range(100), close_loop_x)
  pylab.plot(range(100), close_loop_u)
  pylab.show()

  # Write the generated constants out to a file.
  if len(argv) != 9:
    print "Expected .h file name and .cc file name for"
    print "both the plant and unaugmented plant"
  else:
    top_unaug_claw = Claw("RawTopClaw")
    top_unaug_loop_writer = control_loop.ControlLoopWriter("RawTopClaw",
                                                           [top_unaug_claw])
    if argv[1][-3:] == '.cc':
      top_unaug_loop_writer.Write(argv[2], argv[1])
    else:
      top_unaug_loop_writer.Write(argv[1], argv[2])

    top_loop_writer = control_loop.ControlLoopWriter("TopClaw", [top_claw])
    if argv[3][-3:] == '.cc':
      top_loop_writer.Write(argv[4], argv[3])
    else:
      top_loop_writer.Write(argv[3], argv[4])

    bottom_claw = ClawDeltaU("BottomClaw")
    bottom_unaug_claw = Claw("RawBottomClaw")
    bottom_unaug_loop_writer = control_loop.ControlLoopWriter(
        "RawBottomClaw", [bottom_unaug_claw])
    if argv[5][-3:] == '.cc':
      bottom_unaug_loop_writer.Write(argv[6], argv[5])
    else:
      bottom_unaug_loop_writer.Write(argv[5], argv[6])

    bottom_loop_writer = control_loop.ControlLoopWriter("BottomClaw",
                                                        [bottom_claw])
    if argv[7][-3:] == '.cc':
      bottom_loop_writer.Write(argv[8], argv[7])
    else:
      bottom_loop_writer.Write(argv[7], argv[8])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
