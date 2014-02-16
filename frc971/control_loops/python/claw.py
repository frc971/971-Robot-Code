#!/usr/bin/python

import control_loop
import controls
import numpy
import sys
from matplotlib import pylab

class Claw(control_loop.ControlLoop):
  def __init__(self, name="RawClaw"):
    super(Claw, self).__init__(name)
    # Stall Torque in N m
    self.stall_torque = 1.4
    # Stall Current in Amps
    self.stall_current = 86
    # Free Speed in RPM
    self.free_speed = 6200.0
    # Free Current in Amps
    self.free_current = 1.5
    # Moment of inertia of the claw in kg m^2
    # TODO(aschuh): Measure this in reality.  It doesn't seem high enough.
    # James measured 0.51, but that can't be right given what I am seeing.
    self.J = 2.0
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

    # State is [bottom position, top - bottom position,
    #           bottom velocity, top - bottom velocity]
    # Input is [bottom power, top power]
    # Motor time constant.
    self.motor_timeconstant = self.Kt / self.Kv / (self.J * self.G * self.G * self.R)
    # State feedback matrices
    self.A_continuous = numpy.matrix(
        [[0, 0, 1, 0],
         [0, 0, 0, 1],
         [0, 0, -self.motor_timeconstant, 0],
         [0, 0, 0, -self.motor_timeconstant]])

    self.motor_feedforward = self.Kt / (self.J * self.G * self.R)

    self.B_continuous = numpy.matrix(
        [[0, 0],
         [0, 0],
         [self.motor_feedforward, 0],
         [-self.motor_feedforward, self.motor_feedforward]])
    self.C = numpy.matrix([[1, 0, 0, 0],
                           [1, 1, 0, 0]])
    self.D = numpy.matrix([[0, 0],
                           [0, 0]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    #controlability = controls.ctrb(self.A, self.B);
    #print "Rank of controlability matrix.", numpy.linalg.matrix_rank(controlability)

    self.PlaceControllerPoles([0.85, 0.45, 0.45, 0.85])

    self.rpl = .05
    self.ipl = 0.008
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl,
                             self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    self.U_max = numpy.matrix([[12.0], [12.0]])
    self.U_min = numpy.matrix([[-12.0], [-12.0]])

    self.InitializeState()


class ClawDeltaU(Claw):
  def __init__(self, name="Claw"):
    super(ClawDeltaU, self).__init__(name)
    A_unaugmented = self.A
    B_unaugmented = self.B
    C_unaugmented = self.C

    self.A = numpy.matrix([[0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 1.0]])
    self.A[0:4, 0:4] = A_unaugmented
    self.A[0:4, 4] = B_unaugmented[0:4, 0]

    self.B = numpy.matrix([[0.0, 0.0],
                           [0.0, 0.0],
                           [0.0, 0.0],
                           [0.0, 0.0],
                           [1.0, 0.0]])
    self.B[0:4, 1] = B_unaugmented[0:4, 1]

    self.C = numpy.concatenate((C_unaugmented, numpy.matrix([[0.0], [0.0]])),
                               axis=1)
    self.D = numpy.matrix([[0.0, 0.0],
                           [0.0, 0.0]])

    #self.PlaceControllerPoles([0.55, 0.35, 0.55, 0.35, 0.80])
    self.Q = numpy.matrix([[(1.0 / (0.04 ** 2.0)), 0.0, 0.0, 0.0, 0.0],
                           [0.0, (1.0 / (0.01 ** 2)), 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.01, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.08, 0.0],
                           [0.0, 0.0, 0.0, 0.0, (1.0 / (10.0 ** 2))]])

    self.R = numpy.matrix([[0.000001, 0.0],
                           [0.0, 1.0 / (10.0 ** 2.0)]])
    self.K = controls.dlqr(self.A, self.B, self.Q, self.R)

    self.K = numpy.matrix([[50.0, 0.0, 10.0, 0.0, 1.0],
                           [50.0, 0.0, 10.0, 0.0, 1.0]])
    #self.K = numpy.matrix([[50.0, -100.0, 0, -10, 0],
    #                       [50.0,  100.0, 0, 10, 0]])

    controlability = controls.ctrb(self.A, self.B);
    print "Rank of augmented controlability matrix.", numpy.linalg.matrix_rank(controlability)

    print "K"
    print self.K
    print "Placed controller poles are"
    print numpy.linalg.eig(self.A - self.B * self.K)[0]
    print [numpy.abs(x) for x in numpy.linalg.eig(self.A - self.B * self.K)[0]]

    self.rpl = .05
    self.ipl = 0.008
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl, 0.10, 0.09,
                             self.rpl - 1j * self.ipl, 0.90])
    #print "A is"
    #print self.A
    #print "L is"
    #print self.L
    #print "C is"
    #print self.C
    #print "A - LC is"
    #print self.A - self.L * self.C

    #print "Placed observer poles are"
    #print numpy.linalg.eig(self.A - self.L * self.C)[0]

    self.U_max = numpy.matrix([[12.0], [12.0]])
    self.U_min = numpy.matrix([[-12.0], [-12.0]])

    self.InitializeState()


def ClipDeltaU(claw, U):
  delta_u = U[0, 0]
  top_u = U[1, 0]
  old_bottom_u = claw.X[4, 0]

  # TODO(austin): Preserve the difference between the top and bottom power.
  new_unclipped_bottom_u = old_bottom_u + delta_u

  #print "Bottom is", new_unclipped_bottom_u, "Top is", top_u
  if new_unclipped_bottom_u > claw.U_max[0, 0]:
    #print "Bottom is too big.  Was", new_unclipped_bottom_u, "changing top by", new_unclipped_bottom_u - claw.U_max[0, 0]
    top_u -= new_unclipped_bottom_u - claw.U_max[0, 0]
    new_unclipped_bottom_u = claw.U_max[0, 0]
  if top_u > claw.U_max[1, 0]:
    new_unclipped_bottom_u -= top_u - claw.U_max[1, 0]
    top_u = claw.U_max[1, 0]
  if top_u < claw.U_min[1, 0]:
    new_unclipped_bottom_u -= top_u - claw.U_min[1, 0]
    top_u = claw.U_min[1, 0]
  if new_unclipped_bottom_u < claw.U_min[0, 0]:
    top_u -= new_unclipped_bottom_u - claw.U_min[0, 0]
    new_unclipped_bottom_u = claw.U_min[0, 0]

  new_bottom_u = numpy.clip(new_unclipped_bottom_u, claw.U_min[0, 0], claw.U_max[0, 0])
  new_top_u = numpy.clip(top_u, claw.U_min[1, 0], claw.U_max[1, 0])

  return numpy.matrix([[new_bottom_u - old_bottom_u], [new_top_u]])

def main(argv):
  # Simulate the response of the system to a step input.
  #claw = ClawDeltaU()
  #simulated_x = []
  #for _ in xrange(100):
  #  claw.Update(numpy.matrix([[12.0]]))
  #  simulated_x.append(claw.X[0, 0])

  #pylab.plot(range(100), simulated_x)
  #pylab.show()

  # Simulate the closed loop response of the system to a step input.
  top_claw = ClawDeltaU("TopClaw")
  close_loop_x_bottom = []
  close_loop_x_sep = []
  close_loop_u_bottom = []
  close_loop_u_top = []
  R = numpy.matrix([[1.0], [0.0], [0.0], [0.0], [0.0]])
  top_claw.X[0, 0] = 0
  for _ in xrange(50):
    #print "Error is", (R - top_claw.X_hat)
    U = top_claw.K * (R - top_claw.X_hat)
    U = ClipDeltaU(top_claw, U)
    top_claw.UpdateObserver(U)
    top_claw.Update(U)
    close_loop_x_bottom.append(top_claw.X[0, 0] * 10)
    close_loop_u_bottom.append(top_claw.X[4, 0])
    close_loop_x_sep.append(top_claw.X[1, 0] * 10)
    close_loop_u_top.append(U[1, 0])

  pylab.plot(range(50), close_loop_x_bottom, label='x bottom')
  pylab.plot(range(50), close_loop_u_bottom, label='u bottom')
  pylab.plot(range(50), close_loop_x_sep, label='seperation')
  pylab.plot(range(50), close_loop_u_top, label='u top')
  pylab.legend()
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
