#!/usr/bin/python

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
from frc971.control_loops.python import polytope
from y2014.control_loops.python import polydrivetrain
import numpy
import sys
from matplotlib import pylab
import gflags
import glog

FLAGS = gflags.FLAGS

try:
  gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
  pass

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
    self.J_top = 2.8
    self.J_bottom = 3.0

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
    self.dt = 0.005

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

    self.motor_feedforward = self.Kt / (self.G * self.R)
    self.motor_feedforward_bottom = self.motor_feedforward / self.J_bottom
    self.motor_feedforward_difference = self.motor_feedforward / self.J_top
    self.motor_feedforward_difference_bottom = (
        self.motor_feedforward * (1 / self.J_bottom - 1 / self.J_top))
    self.B_continuous = numpy.matrix(
        [[0, 0],
         [0, 0],
         [self.motor_feedforward_bottom, 0],
         [-self.motor_feedforward_bottom, self.motor_feedforward_difference]])

    glog.debug('Cont X_ss %f', self.motor_feedforward / -self.common_motor_constant)

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

    self.K_bottom = controls.dplace(self.A_bottom, self.B_bottom,
                                    [0.87 + 0.05j, 0.87 - 0.05j])
    self.K_diff = controls.dplace(self.A_diff, self.B_diff,
                                  [0.85 + 0.05j, 0.85 - 0.05j])

    glog.debug('K_diff %s', str(self.K_diff))
    glog.debug('K_bottom %s', str(self.K_bottom))

    glog.debug('A')
    glog.debug(self.A)
    glog.debug('B')
    glog.debug(self.B)

    
    self.Q = numpy.matrix([[(1.0 / (0.10 ** 2.0)), 0.0, 0.0, 0.0],
                           [0.0, (1.0 / (0.06 ** 2.0)), 0.0, 0.0],
                           [0.0, 0.0, 0.10, 0.0],
                           [0.0, 0.0, 0.0, 0.1]])

    self.R = numpy.matrix([[(1.0 / (40.0 ** 2.0)), 0.0],
                           [0.0, (1.0 / (5.0 ** 2.0))]])
    #self.K = controls.dlqr(self.A, self.B, self.Q, self.R)

    self.K = numpy.matrix([[self.K_bottom[0, 0], 0.0, self.K_bottom[0, 1], 0.0],
                           [0.0, self.K_diff[0, 0], 0.0, self.K_diff[0, 1]]])

    # Compute the feed forwards aceleration term.
    self.K[1, 0] = -self.B[1, 0] * self.K[0, 0] / self.B[1, 1]

    lstsq_A = numpy.identity(2)
    lstsq_A[0, :] = self.B[1, :]
    lstsq_A[1, :] = self.B[3, :]
    glog.debug('System of Equations coefficients:')
    glog.debug(str(lstsq_A))
    glog.debug('det %s', str(numpy.linalg.det(lstsq_A)))

    out_x = numpy.linalg.lstsq(
                         lstsq_A,
                         numpy.matrix([[self.A[1, 2]], [self.A[3, 2]]]))[0]
    self.K[1, 2] = -lstsq_A[0, 0] * (self.K[0, 2] - out_x[0]) / lstsq_A[0, 1] + out_x[1]

    glog.debug('K unaugmented')
    glog.debug(str(self.K))
    glog.debug('B * K unaugmented')
    glog.debug(str(self.B * self.K))
    F = self.A - self.B * self.K
    glog.debug('A - B * K unaugmented')
    glog.debug(str(F))
    glog.debug('eigenvalues')
    glog.debug(str(numpy.linalg.eig(F)[0]))

    self.rpl = .09
    self.ipl = 0.030
    self.PlaceObserverPoles([self.rpl + 1j * self.ipl,
                             self.rpl + 1j * self.ipl,
                             self.rpl - 1j * self.ipl,
                             self.rpl - 1j * self.ipl])

    # The box formed by U_min and U_max must encompass all possible values,
    # or else Austin's code gets angry.
    self.U_max = numpy.matrix([[12.0], [12.0]])
    self.U_min = numpy.matrix([[-12.0], [-12.0]])

    # For the tests that check the limits, these are (upper, lower) for both
    # claws.
    self.hard_pos_limits = None
    self.pos_limits = None

    # Compute the steady state velocities for a given applied voltage.
    # The top and bottom of the claw should spin at the same rate if the
    # physics is right.
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

    glog.debug('X_ss %s', str(X_ss))

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

    controllability = controls.ctrb(self.A, self.B)
    glog.debug('Rank of augmented controllability matrix: %d',
              numpy.linalg.matrix_rank(controllability))

    glog.debug('K')
    glog.debug(str(self.K))
    glog.debug('Placed controller poles are')
    glog.debug(str(numpy.linalg.eig(self.A - self.B * self.K)[0]))
    glog.debug(str([numpy.abs(x) for x in
                       numpy.linalg.eig(self.A - self.B * self.K)[0]]))

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

def ScaleU(claw, U, K, error):
  """Clips U as necessary.

    Args:
      claw: claw object containing moments of inertia and U limits.
      U: Input matrix to clip as necessary.
  """

  bottom_u = U[0, 0]
  top_u = U[1, 0]
  position_error = error[0:2, 0]
  velocity_error = error[2:, 0]

  U_poly = polytope.HPolytope(
      numpy.matrix([[1, 0],
                    [-1, 0],
                    [0, 1],
                    [0, -1]]),
      numpy.matrix([[12],
                    [12],
                    [12],
                    [12]]))

  if (bottom_u > claw.U_max[0, 0] or bottom_u < claw.U_min[0, 0] or
      top_u > claw.U_max[0, 0] or top_u < claw.U_min[0, 0]):

    position_K = K[:, 0:2]
    velocity_K = K[:, 2:]

    # H * U <= k
    # U = UPos + UVel
    # H * (UPos + UVel) <= k
    # H * UPos <= k - H * UVel
    #
    # Now, we can do a coordinate transformation and say the following.
    #
    # UPos = position_K * position_error
    # (H * position_K) * position_error <= k - H * UVel
    #
    # Add in the constraint that 0 <= t <= 1
    # Now, there are 2 ways this can go.  Either we have a region, or we don't
    # have a region.  If we have a region, then pick the largest t and go for it.
    # If we don't have a region, we need to pick a good comprimise.

    pos_poly = polytope.HPolytope(
        U_poly.H * position_K,
        U_poly.k - U_poly.H * velocity_K * velocity_error)

    # The actual angle for the line we call 45.
    angle_45 = numpy.matrix([[numpy.sqrt(3), 1]])
    if claw.pos_limits and claw.hard_pos_limits and claw.X[0, 0] + claw.X[1, 0] > claw.pos_limits[1]:
      angle_45 = numpy.matrix([[1, 1]])

    P = position_error
    L45 = numpy.multiply(numpy.matrix([[numpy.sign(P[1, 0]), -numpy.sign(P[0, 0])]]), angle_45)
    if L45[0, 1] == 0:
      L45[0, 1] = 1
    if L45[0, 0] == 0:
      L45[0, 0] = 1
    w45 = numpy.matrix([[0]])

    if numpy.abs(P[0, 0]) > numpy.abs(P[1, 0]):
      LH = numpy.matrix([[0, 1]])
    else:
      LH = numpy.matrix([[1, 0]])
    wh = LH * P
    standard = numpy.concatenate((L45, LH))
    W = numpy.concatenate((w45, wh))
    intersection = numpy.linalg.inv(standard) * W
    adjusted_pos_error_h, is_inside_h = polydrivetrain.DoCoerceGoal(pos_poly,
        LH, wh, position_error)
    adjusted_pos_error_45, is_inside_45 = polydrivetrain.DoCoerceGoal(pos_poly,
        L45, w45, intersection)
    if pos_poly.IsInside(intersection):
      adjusted_pos_error = adjusted_pos_error_h
    else:
      if is_inside_h:
        if numpy.linalg.norm(adjusted_pos_error_h) > numpy.linalg.norm(adjusted_pos_error_45):
          adjusted_pos_error = adjusted_pos_error_h
        else:
          adjusted_pos_error = adjusted_pos_error_45
      else:
        adjusted_pos_error = adjusted_pos_error_45
    #print adjusted_pos_error

    #print "Actual power is ", velocity_K * velocity_error + position_K * adjusted_pos_error
    return velocity_K * velocity_error + position_K * adjusted_pos_error

    #U = Kpos * poserror + Kvel * velerror
      
    #scalar = claw.U_max[0, 0] / max(numpy.abs(top_u), numpy.abs(bottom_u))

    #top_u *= scalar
    #bottom_u *= scalar

  return numpy.matrix([[bottom_u], [top_u]])

def run_test(claw, initial_X, goal, max_separation_error=0.01, show_graph=True, iterations=200):
  """Runs the claw plant on a given claw (claw) with an initial condition (initial_X) and goal (goal).

    The tests themselves are not terribly sophisticated; I just test for 
    whether the goal has been reached and whether the separation goes
    outside of the initial and goal values by more than max_separation_error.
    Prints out something for a failure of either condition and returns
    False if tests fail.
    Args:
      claw: claw object to use.
      initial_X: starting state.
      goal: goal state.
      show_graph: Whether or not to display a graph showing the changing
           states and voltages.
      iterations: Number of timesteps to run the model for."""

  claw.X = initial_X

  # Various lists for graphing things.
  t = []
  x_bottom = []
  x_top = []
  u_bottom = []
  u_top = []
  x_separation = []

  tests_passed = True

  # Bounds which separation should not exceed.
  lower_bound = (initial_X[1, 0] if initial_X[1, 0] < goal[1, 0]
                 else goal[1, 0]) - max_separation_error
  upper_bound = (initial_X[1, 0] if initial_X[1, 0] > goal[1, 0]
                 else goal[1, 0]) + max_separation_error

  for i in xrange(iterations):
    U = claw.K * (goal - claw.X)
    U = ScaleU(claw, U, claw.K, goal - claw.X)
    claw.Update(U)

    if claw.X[1, 0] > upper_bound or claw.X[1, 0] < lower_bound:
      tests_passed = False
      glog.info('Claw separation was %f', claw.X[1, 0])
      glog.info("Should have been between", lower_bound, "and", upper_bound)

    if claw.hard_pos_limits and \
      (claw.X[0, 0] > claw.hard_pos_limits[1] or
          claw.X[0, 0] < claw.hard_pos_limits[0] or
          claw.X[0, 0] + claw.X[1, 0] > claw.hard_pos_limits[1] or
          claw.X[0, 0] + claw.X[1, 0] < claw.hard_pos_limits[0]):
      tests_passed = False
      glog.info('Claws at %f and %f', claw.X[0, 0], claw.X[0, 0] + claw.X[1, 0])
      glog.info("Both should be in %s, definitely %s",
                claw.pos_limits, claw.hard_pos_limits)

    t.append(i * claw.dt)
    x_bottom.append(claw.X[0, 0] * 10.0)
    x_top.append((claw.X[1, 0] + claw.X[0, 0]) * 10.0)
    u_bottom.append(U[0, 0])
    u_top.append(U[1, 0])
    x_separation.append(claw.X[1, 0] * 10.0)

  if show_graph:
    pylab.plot(t, x_bottom, label='x bottom * 10')
    pylab.plot(t, x_top, label='x top * 10')
    pylab.plot(t, u_bottom, label='u bottom')
    pylab.plot(t, u_top, label='u top')
    pylab.plot(t, x_separation, label='separation * 10')
    pylab.legend()
    pylab.show()

  # Test to make sure that we are near the goal.
  if numpy.max(abs(claw.X - goal)) > 1e-4:
    tests_passed = False
    glog.error('X was %s Expected %s', str(claw.X), str(goal))

  return tests_passed

def main(argv):
  argv = FLAGS(argv)

  claw = Claw()
  if FLAGS.plot:
    # Test moving the claw with constant separation.
    initial_X = numpy.matrix([[-1.0], [0.0], [0.0], [0.0]])
    R = numpy.matrix([[1.0], [0.0], [0.0], [0.0]])
    run_test(claw, initial_X, R)

    # Test just changing separation.
    initial_X = numpy.matrix([[0.0], [0.0], [0.0], [0.0]])
    R = numpy.matrix([[0.0], [1.0], [0.0], [0.0]])
    run_test(claw, initial_X, R)

    # Test changing both separation and position at once.
    initial_X = numpy.matrix([[0.0], [0.0], [0.0], [0.0]])
    R = numpy.matrix([[1.0], [1.0], [0.0], [0.0]])
    run_test(claw, initial_X, R)

    # Test a small separation error and a large position one.
    initial_X = numpy.matrix([[0.0], [0.0], [0.0], [0.0]])
    R = numpy.matrix([[2.0], [0.05], [0.0], [0.0]])
    run_test(claw, initial_X, R)

    # Test a small separation error and a large position one.
    initial_X = numpy.matrix([[0.0], [0.0], [0.0], [0.0]])
    R = numpy.matrix([[-0.5], [1.0], [0.0], [0.0]])
    run_test(claw, initial_X, R)

    # Test opening with the top claw at the limit.
    initial_X = numpy.matrix([[0.0], [0.0], [0.0], [0.0]])
    R = numpy.matrix([[-1.5], [1.5], [0.0], [0.0]])
    claw.hard_pos_limits = (-1.6, 0.1)
    claw.pos_limits = (-1.5, 0.0)
    run_test(claw, initial_X, R)
    claw.pos_limits = None

    # Test opening with the bottom claw at the limit.
    initial_X = numpy.matrix([[0.0], [0.0], [0.0], [0.0]])
    R = numpy.matrix([[0], [1.5], [0.0], [0.0]])
    claw.hard_pos_limits = (-0.1, 1.6)
    claw.pos_limits = (0.0, 1.6)
    run_test(claw, initial_X, R)
    claw.pos_limits = None

  # Write the generated constants out to a file.
  if len(argv) != 3:
    glog.fatal('Expected .h file name and .cc file name for the claw.')
  else:
    namespaces = ['y2014', 'control_loops', 'claw']
    claw = Claw('Claw')
    loop_writer = control_loop.ControlLoopWriter('Claw', [claw],
                                                 namespaces=namespaces)
    loop_writer.AddConstant(control_loop.Constant('kClawMomentOfInertiaRatio',
      '%f', claw.J_top / claw.J_bottom))
    loop_writer.AddConstant(control_loop.Constant('kDt', '%f',
          claw.dt))
    loop_writer.Write(argv[1], argv[2])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
