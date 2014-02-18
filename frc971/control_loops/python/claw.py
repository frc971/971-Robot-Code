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
    self.stall_torque = 2.42
    # Stall Current in Amps
    self.stall_current = 133
    # Free Speed in RPM
    self.free_speed = 5500.0
    # Free Current in Amps
    self.free_current = 2.7
    # Moment of inertia of the claw in kg m^2
    # measured from CAD
    self.J_top = 0.3
    self.J_bottom = 0.9
    # Resistance of the motor
    self.R = 12.0 / self.stall_current
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
               (13.5 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratio
    self.G = 14.0 / 48.0 * 18.0 / 32.0 * 18.0 / 66.0 * 12.0 / 60.0
    # Control loop time step
    self.dt = 0.01

    # State is [bottom position, bottom velocity, top - bottom position,
    #           top - bottom velocity]
    # Input is [bottom power, top power - bottom power * J_top / J_bottom]
    # Motor time constants. difference_bottom refers to the constant for how the
    # bottom velocity affects the difference of the top and bottom velocities.
    self.common_motor_constant = -self.Kt / self.Kv / (self.G * self.G * self.R)
    self.bottom_bottom = self.common_motor_constant / self.J_bottom
    self.difference_bottom = -self.common_motor_constant * (1 / self.J_bottom
                                                        - 1 / self.J_top)
    self.difference_difference = self.common_motor_constant / self.J_top
    # State feedback matrices

    self.A_continuous = numpy.matrix(
        [[0, 0, 1, 0],
         [0, 0, 0, 1],
         [0, 0, self.bottom_bottom, 0],
         [0, 0, self.difference_bottom, self.difference_difference]])

    self.A_bottom_cont = numpy.matrix(
        [[0, 1],
         [0, self.bottom_bottom]])

    self.A_diff_cont = numpy.matrix(
        [[0, 1],
         [0, self.difference_difference]])

  # self.A_continuous[0:2, 0:2] = self.A_bottom_cont
  # self.A_continuous[2:4, 2:4] = self.A_diff_cont
  # self.A_continuous[3, 1] = self.difference_bottom

    self.motor_feedforward = self.Kt / (self.G * self.R)
    self.motor_feedforward_bottom = self.motor_feedforward / self.J_bottom
    self.motor_feedforward_difference = self.motor_feedforward / self.J_top
    self.motor_feedforward_difference_bottom = (
        self.motor_feedforward * (1 / self.J_bottom - 1 / self.J_top))
    self.B_continuous = numpy.matrix(
        [[0, 0],
         [0, 0],
         [self.motor_feedforward_bottom, 0],
         [-self.motor_feedforward_bottom,
          self.motor_feedforward_difference]])

    print "Cont X_ss", self.motor_feedforward / -self.common_motor_constant

    self.B_bottom_cont = numpy.matrix(
        [[0],
         [self.motor_feedforward_bottom]])

    self.B_diff_cont = numpy.matrix(
        [[0],
         [self.motor_feedforward_difference]])

    self.C = numpy.matrix([[1, 0, 0, 0],
                           [1, 1, 0, 0]])
    self.D = numpy.matrix([[0, 0],
                           [0, 0]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    self.A_bottom, self.B_bottom = controls.c2d(
        self.A_bottom_cont, self.B_bottom_cont, self.dt)
    self.A_diff, self.B_diff = controls.c2d(
        self.A_diff_cont, self.B_diff_cont, self.dt)

    print "A"
    print self.A
    print "B"
    print self.B

    X_ss = numpy.matrix([[0], [0], [0.0], [0]])
    
    U = numpy.matrix([[1.0],[1.0]])
    A = self.A
    B = self.B
   #X_ss[2, 0] = X_ss[2, 0] * A[2, 2] + B[2, 0] * U[0, 0]
    X_ss[2, 0] = 1 / (1 - A[2, 2]) * B[2, 0] * U[0, 0]
   #X_ss[3, 0] = X_ss[3, 0] * A[3, 3] + X_ss[2, 0] * A[3, 2] + B[3, 0] * U[0, 0] + B[3, 1] * U[1, 0]
   #X_ss[3, 0] * (1 - A[3, 3]) = X_ss[2, 0] * A[3, 2] + B[3, 0] * U[0, 0] + B[3, 1] * U[1, 0]
    X_ss[3, 0] = 1 / (1 - A[3, 3]) * (X_ss[2, 0] * A[3, 2] + B[3, 1] * U[1, 0] + B[3, 0] * U[0, 0])
   #X_ss[3, 0] = 1 / (1 - A[3, 3]) / (1 - A[2, 2]) * B[2, 0] * U[0, 0] * A[3, 2] + B[3, 0] * U[0, 0] + B[3, 1] * U[1, 0]
    X_ss[0, 0] = A[0, 2] * X_ss[2, 0] + B[0, 0] * U[0, 0]
    X_ss[1, 0] = A[1, 2] * X_ss[2, 0] + A[1, 3] * X_ss[3, 0] + B[1, 0] * U[0, 0] + B[1, 1] * U[1, 0]

    print "X_ss", X_ss
    
    #controlability = controls.ctrb(self.A, self.B);
    #print "Rank of controlability matrix.", numpy.linalg.matrix_rank(controlability)

    self.Q = numpy.matrix([[(1.0 / (0.40 ** 2.0)), 0.0, 0.0, 0.0],
                           [0.0, (1.0 / (0.007 ** 2.0)), 0.0, 0.0],
                           [0.0, 0.0, 0.2, 0.0],
                           [0.0, 0.0, 0.0, 2.00]])

    self.R = numpy.matrix([[(1.0 / (40.0 ** 2.0)), 0.0],
                           [0.0, (1.0 / (5.0 ** 2.0))]])
   #self.K = controls.dlqr(self.A, self.B, self.Q, self.R)

    self.K = numpy.matrix([[50, 0.0, 1, 0.0],
                           [0.0, 300, 0.0, 1.1]])
    lstsq_A = numpy.identity(2)
    lstsq_A[0] = self.B[1]
    lstsq_A[1] = self.B[3]
    print "System of Equations coefficients:"
    print  lstsq_A
    print "det", numpy.linalg.det(lstsq_A)
    self.K[1, 0] = -lstsq_A[0, 0] * self.K[0, 0] / lstsq_A[0, 1]
   #self.K[0:2, 0] = numpy.linalg.lstsq(lstsq_A, numpy.matrix([[0.0], [0.0]]))[0]
    out_x = numpy.linalg.lstsq(
                         lstsq_A,
                         numpy.matrix([[self.A[1, 2]], [self.A[3, 2]]]))[0]
    self.K[1, 2] = -lstsq_A[0, 0] * (self.K[0, 2] - out_x[0]) / lstsq_A[0, 1] + out_x[1]

    print "K unaugmented"
    print self.K
    print "B * K unaugmented"
    print self.B * self.K
    F = self.A - self.B * self.K
    print "A - B * K unaugmented"
    print F
    print "eigenvalues"
    print numpy.linalg.eig(F)[0]

    self.rpl = .05
    self.ipl = 0.008
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    # The box formed by U_min and U_max must encompass all possible values,
    # or else Austin's code gets angry.
    self.U_max = numpy.matrix([[12.0], [24.0]])
    self.U_min = numpy.matrix([[-12.0], [-24.0]])

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


def FullSeparationPriority(claw, U):
  bottom_u = U[0, 0]
  top_u = U[1, 0] + bottom_u

  #print "Bottom is", new_unclipped_bottom_u, "Top is", top_u
  if bottom_u > claw.U_max[0, 0]:
    #print "Bottom is too big.  Was", new_unclipped_bottom_u, "changing top by", new_unclipped_bottom_u - claw.U_max[0, 0]
    top_u -= bottom_u - claw.U_max[0, 0]
    if top_u < claw.U_min[1, 0]:
      top_u = claw.U_min[1, 0]

    bottom_u = claw.U_max[0, 0]
  if top_u > claw.U_max[1, 0]:
    bottom_u -= top_u - claw.U_max[1, 0]
    if bottom_u < claw.U_min[0, 0]:
      bottom_u = claw.U_min[0, 0]

    top_u = claw.U_max[1, 0]
  if top_u < claw.U_min[1, 0]:
    bottom_u -= top_u - claw.U_min[1, 0]
    if bottom_u > claw.U_max[0, 0]:
      bottom_u = claw.U_max[0, 0]

    top_u = claw.U_min[1, 0]
  if bottom_u < claw.U_min[0, 0]:
    top_u -= bottom_u - claw.U_min[0, 0]
    if top_u > claw.U_max[1, 0]:
      top_u = claw.U_max[1, 0]

    bottom_u = claw.U_min[0, 0]

  return numpy.matrix([[bottom_u], [top_u - bottom_u]])

def AverageUFix(claw, U, preserve_v3=False):
  """Clips U as necessary.

    Args:
      claw: claw object containing moments of inertia and U limits.
      U: Input matrix to clip as necessary.
      preserve_v3: There are two ways to attempt to clip the voltages:
        -If you preserve the imaginary v3, then it will attempt to 
          preserve the effect on the separation of the two claws.
          If it is not able to do this, then it calls itself with preserve_v3
          set to False.
        -If you preserve the ratio of the voltage of the bottom and the top,
          then it will attempt to preserve the ratio of those two. This is
          equivalent to preserving the ratio of v3 and the bottom voltage.
  """
  bottom_u = U[0, 0]
  top_u = U[1, 0]
  seperation_u = top_u - bottom_u * claw.J_top / claw.J_bottom

  top_big = top_u > claw.U_max[0, 0]
  top_small = top_u < claw.U_min[0, 0]
  bot_big = bottom_u > claw.U_max[0, 0]
  bot_small = top_u < claw.U_min[0, 0]
  bottom_bad = bot_big or bot_small
  top_bad = top_big or top_small
  scalar = claw.U_max[0, 0] / max(numpy.abs(top_u), numpy.abs(bottom_u))
  if bottom_bad and preserve_v3:
    bottom_u *= scalar
    top_u = seperation_u + bottom_u * claw.J_top / claw.J_bottom
    if abs(top_u) > claw.U_max[0, 0]:
      return AverageUFix(claw, U, preserve_v3=False)
  elif top_bad and preserve_v3:
    top_u *= scalar
    bottom_u = (top_u - seperation_u) * claw.J_bottom / claw.J_top
    if abs(bottom_u) > claw.U_max[0, 0]:
      return AverageUFix(claw, U, preserve_v3=False)
  elif (bottom_bad or top_bad) and not preserve_v3:
    top_u *= scalar
    bottom_u *= scalar

  return numpy.matrix([[bottom_u], [top_u]])

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

  # Simulate the closed loop response of the system.
  claw = Claw("TopClaw")
  t = []
  close_loop_x_bottom = []
  close_loop_x_sep = []
  actual_sep = []
  actual_x_bottom = []
  close_loop_x_top = []
  close_loop_u_bottom = []
  close_loop_u_top = []
  R = numpy.matrix([[0.0], [0.00], [0.0], [0.0]])
  claw.X[0, 0] = 1
  claw.X[1, 0] = .0
  claw.X_hat = claw.X
 #X_actual = claw.X
  for i in xrange(100):
    #print "Error is", (R - claw.X_hat)
    U = claw.K * (R - claw.X)
    #U = numpy.clip(claw.K * (R - claw.X_hat), claw.U_min, claw.U_max)
    #U = FullSeparationPriority(claw, U)
   #U = AverageUFix(claw, U, preserve_v3=True)
    #U = claw.K * (R - claw.X_hat)
    #U = ClipDeltaU(claw, U)
    claw.UpdateObserver(U)
    claw.Update(U)
   #X_actual = claw.A_actual * X_actual + claw.B_actual * U
   #claw.Y = claw.C * X_actual
    close_loop_x_bottom.append(claw.X[0, 0] * 10)
    close_loop_u_bottom.append(U[0, 0])
   #actual_sep.append(X_actual[2, 0] * 100)
   #actual_x_bottom.append(X_actual[0, 0] * 10)
    close_loop_x_sep.append(claw.X[1, 0] * 100)
    close_loop_x_top.append((claw.X[1, 0] + claw.X[0, 0]) * 10)
    close_loop_u_top.append(U[1, 0])
    t.append(0.01 * i)

  pylab.plot(t, close_loop_x_bottom, label='x bottom')
  pylab.plot(t, close_loop_x_sep, label='separation')
 #pylab.plot(t, actual_x_bottom, label='true x bottom')
 #pylab.plot(t, actual_sep, label='true separation')
  pylab.plot(t, close_loop_x_top, label='x top')
  pylab.plot(t, close_loop_u_bottom, label='u bottom')
  pylab.plot(t, close_loop_u_top, label='u top')
  pylab.legend()
  pylab.show()

  # Write the generated constants out to a file.
  if len(argv) != 3:
    print "Expected .h file name and .cc file name for the claw."
  else:
    claw = Claw("Claw")
    loop_writer = control_loop.ControlLoopWriter("Claw", [claw])
    if argv[1][-3:] == '.cc':
      loop_writer.Write(argv[2], argv[1])
    else:
      loop_writer.Write(argv[1], argv[2])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
