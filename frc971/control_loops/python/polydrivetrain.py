#!/usr/bin/python

import numpy
import sys
import polytope
import drivetrain
import controls
from matplotlib import pylab

__author__ = 'Austin Schuh (austin.linux@gmail.com)'


def CoerceGoal(region, K, w, R):
  """Intersects a line with a region, and finds the closest point to R.

  Finds a point that is closest to R inside the region, and on the line
  defined by K X = w.  If it is not possible to find a point on the line,
  finds a point that is inside the region and closest to the line.  This
  function assumes that

  Args:
    region: HPolytope, the valid goal region.
    K: numpy.matrix (2 x 1), the matrix for the equation [K1, K2] [x1; x2] = w
    w: float, the offset in the equation above.
    R: numpy.matrix (2 x 1), the point to be closest to.

  Returns:
    numpy.matrix (2 x 1), the point.
  """

  if region.IsInside(R):
    return R

  perpendicular_vector = K.T / numpy.linalg.norm(K)
  parallel_vector = numpy.matrix([[perpendicular_vector[1, 0]],
                                  [-perpendicular_vector[0, 0]]])
  
  # We want to impose the constraint K * X = w on the polytope H * X <= k.
  # We do this by breaking X up into parallel and perpendicular components to
  # the half plane.  This gives us the following equation.
  #
  #  parallel * (parallel.T \dot X) + perpendicular * (perpendicular \dot X)) = X
  #
  # Then, substitute this into the polytope.
  #
  #  H * (parallel * (parallel.T \dot X) + perpendicular * (perpendicular \dot X)) <= k
  #
  # Substitute K * X = w
  #
  # H * parallel * (parallel.T \dot X) + H * perpendicular * w <= k
  #
  # Move all the knowns to the right side.
  #
  # H * parallel * ([parallel1 parallel2] * X) <= k - H * perpendicular * w
  #
  # Let t = parallel.T \dot X, the component parallel to the surface.
  #
  # H * parallel * t <= k - H * perpendicular * w
  #
  # This is a polytope which we can solve, and use to figure out the range of X
  # that we care about!

  t_poly = polytope.HPolytope(
      region.H * parallel_vector,
      region.k - region.H * perpendicular_vector * w)

  vertices = t_poly.Vertices()

  if vertices.shape[0]:
    # The region exists!
    # Find the closest vertex
    min_distance = numpy.infty
    closest_point = None
    for vertex in vertices:
      point = parallel_vector * vertex + perpendicular_vector * w
      length = numpy.linalg.norm(R - point)
      if length < min_distance:
        min_distance = length
        closest_point = point

    return closest_point
  else:
    # Find the vertex of the space that is closest to the line.
    region_vertices = region.Vertices()
    min_distance = numpy.infty
    closest_point = None
    for vertex in region_vertices:
      point = vertex.T
      length = numpy.abs((perpendicular_vector.T * point)[0, 0])
      if length < min_distance:
        min_distance = length
        closest_point = point

    return closest_point


class VelocityDrivetrainModel(object):
  def __init__(self, left_low=True, right_low=True):
    self._drivetrain = drivetrain.Drivetrain(left_low=left_low,
                                             right_low=right_low)
    self.A = numpy.matrix(
        [[self._drivetrain.A[1, 1], self._drivetrain.A[1, 3]],
         [self._drivetrain.A[3, 1], self._drivetrain.A[3, 3]]])

    self.B = numpy.matrix(
        [[self._drivetrain.B[1, 0], self._drivetrain.B[1, 1]],
         [self._drivetrain.B[3, 0], self._drivetrain.B[3, 1]]])

    # FF * X = U (steady state)
    self.FF = self.B.I * (numpy.eye(2) - self.A)

    self.K = controls.dplace(self.A, self.B, [0.3, 0.3])


class VelocityDrivetrain(object):
  def __init__(self):
    self.drivetrain_low_low = VelocityDrivetrainModel(left_low=True, right_low=True)
    self.drivetrain_low_high = VelocityDrivetrainModel(left_low=True, right_low=False)
    self.drivetrain_high_low = VelocityDrivetrainModel(left_low=False, right_low=True)
    self.drivetrain_high_high = VelocityDrivetrainModel(left_low=False, right_low=False)

    # X is [lvel, rvel]
    self.X = numpy.matrix(
        [[0.0],
         [0.0]])

    self.U_poly = polytope.HPolytope(
        numpy.matrix([[1, 0],
                      [-1, 0],
                      [0, 1],
                      [0, -1]]),
        numpy.matrix([[12],
                      [12],
                      [12],
                      [12]]))

    self.U_max = numpy.matrix(
        [[12.0],
         [12.0]])
    self.U_min = numpy.matrix(
        [[-12.0000000000],
         [-12.0000000000]])

    self.dt = 0.01

    self.R = numpy.matrix(
        [[0.0],
         [0.0]])

    self.xfiltered = 0.0

    # U = self.K[0, :].sum() * (R - xfiltered) + self.FF[0, :].sum() * R
    # throttle * 12.0 = (self.K[0, :].sum() + self.FF[0, :].sum()) * R
    #                   - self.K[0, :].sum() * xfiltered

    # R = (throttle * 12.0 + self.K[0, :].sum() * xfiltered) /
    #     (self.K[0, :].sum() + self.FF[0, :].sum())

    # U = (K + FF) * R - K * X
    # (K + FF) ^-1 * (U + K * X) = R

    # (K + FF) ^-1 * (throttle * 12.0 + K * throttle * self.vmax) = R
    # Xn+1 = A * X + B * (throttle * 12.0)

    # xfiltered = self.A[0, :].sum() + B[0, :].sum() * throttle * 12.0
    self.ttrust = 1.0

    self.left_high = False
    self.right_high = False

  def CurrentDrivetrain(self):
    if self.left_high:
      if self.right_high:
        return self.drivetrain_high_high
      else:
        return self.drivetrain_high_low
    else:
      if self.right_high:
        return self.drivetrain_low_high
      else:
        return self.drivetrain_low_low

  def Update(self, throttle, steering):
    # Invert the plant to figure out how the velocity filter would have to work
    # out in order to filter out the forwards negative inertia.
    # This math assumes that the left and right power and velocity are equals,
    # and that the plant is the same on the left and right.

    # TODO(aschuh): Rederive this assuming G_left != G_right.
    # TODO(aschuh): Figure out the correct throttle if we start in low gear and
    # then let it shift up.  This isn't it.
    fvel = ((throttle * 12.0 + self.ttrust * self.CurrentDrivetrain().K[0, :].sum() * self.xfiltered)
            / (self.ttrust * self.CurrentDrivetrain().K[0, :].sum() + self.CurrentDrivetrain().FF[0, :].sum()))
    self.xfiltered = (self.CurrentDrivetrain().A[0, :].sum() * self.xfiltered +
                      self.CurrentDrivetrain().B[0, :].sum() * throttle * 12.0)

    # Constant radius means that angualar_velocity / linear_velocity = constant.
    # Compute the left and right velocities.
    left_velocity = fvel - steering * numpy.abs(fvel)
    right_velocity = fvel + steering * numpy.abs(fvel)

    # Write this constraint in the form of K * R = w
    # angular velocity / linear velocity = constant
    # (left - right) / (left + right) = constant
    # left - right = constant * left + constant * right

    # (fvel - steering * numpy.abs(fvel) - fvel - steering * numpy.abs(fvel)) /
    #  (fvel - steering * numpy.abs(fvel) + fvel + steering * numpy.abs(fvel)) =
    #       constant
    # (- 2 * steering * numpy.abs(fvel)) / (2 * fvel) = constant
    # (-steering * sign(fvel)) = constant
    # (-steering * sign(fvel)) * (left + right) = left - right
    # (steering * sign(fvel) + 1) * left + (steering * sign(fvel) - 1) * right = 0

    equality_k = numpy.matrix(
        [[1 + steering * numpy.sign(fvel), -(1 - steering * numpy.sign(fvel))]])
    equality_w = 0.0

    self.R[0, 0] = left_velocity
    self.R[1, 0] = right_velocity

    # Construct a constraint on R by manipulating the constraint on U
    # Start out with H * U <= k
    # U = FF * R + K * (R - X)
    # H * (FF * R + K * R - K * X) <= k
    # H * (FF + K) * R <= k + H * K * X
    R_poly = polytope.HPolytope(
        self.U_poly.H * (self.CurrentDrivetrain().K + self.CurrentDrivetrain().FF),
        self.U_poly.k + self.U_poly.H * self.CurrentDrivetrain().K * self.X)

    # Limit R back inside the box.
    self.boxed_R = CoerceGoal(R_poly, equality_k, equality_w, self.R)

    FF_volts = self.CurrentDrivetrain().FF * self.boxed_R
    self.U_ideal = self.CurrentDrivetrain().K * (self.boxed_R - self.X) + FF_volts

    self.U = numpy.clip(self.U_ideal, self.U_min, self.U_max)
    self.X = self.CurrentDrivetrain().A * self.X + self.CurrentDrivetrain().B * self.U


def main(argv):
  drivetrain = VelocityDrivetrain()

  vl_plot = []
  vr_plot = []
  ul_plot = []
  ur_plot = []
  radius_plot = []
  t_plot = []
  for t in numpy.arange(0, 1.5, drivetrain.dt):
    if t < 0.5:
      drivetrain.Update(throttle=0.60, steering=0.3)
    elif t < 1.0:
      if t > 0.7:
        drivetrain.left_high = False
        drivetrain.right_high = True

      drivetrain.Update(throttle=0.60, steering=-0.3)
    else:
      drivetrain.Update(throttle=0.00, steering=0.3)
    t_plot.append(t)
    vl_plot.append(drivetrain.X[0, 0])
    vr_plot.append(drivetrain.X[1, 0])
    ul_plot.append(drivetrain.U[0, 0])
    ur_plot.append(drivetrain.U[1, 0])

    fwd_velocity = (drivetrain.X[1, 0] + drivetrain.X[0, 0]) / 2
    turn_velocity = (drivetrain.X[1, 0] - drivetrain.X[0, 0])
    if fwd_velocity < 0.0000001:
      radius_plot.append(turn_velocity)
    else:
      radius_plot.append(turn_velocity / fwd_velocity)

  pylab.plot(t_plot, vl_plot, label='left velocity')
  pylab.plot(t_plot, vr_plot, label='right velocity')
  pylab.plot(t_plot, ul_plot, label='left power')
  pylab.plot(t_plot, ur_plot, label='right power')
  pylab.plot(t_plot, radius_plot, label='radius')
  pylab.legend()
  pylab.show()
  return 0

if __name__ == '__main__':
  sys.exit(main(sys.argv))
