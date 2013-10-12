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

    self.G_high = self._drivetrain.G_high
    self.G_low = self._drivetrain.G_low
    self.R = self._drivetrain.R
    self.r = self._drivetrain.r
    self.Kv = self._drivetrain.Kv
    self.Kt = self._drivetrain.Kt


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

    # ttrust is the comprimise between having full throttle negative inertia,
    # and having no throttle negative inertia.  A value of 0 is full throttle
    # inertia.  A value of 1 is no throttle negative inertia.
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

  def ComputeGear(self, wheel_velocity, should_print=False, current_gear=False, gear_name=None):
    high_omega = (wheel_velocity / self.CurrentDrivetrain().G_high /
                  self.CurrentDrivetrain().r)
    low_omega = (wheel_velocity / self.CurrentDrivetrain().G_low /
                 self.CurrentDrivetrain().r)
    high_torque = ((12.0 - high_omega / self.CurrentDrivetrain().Kv) *
                   self.CurrentDrivetrain().Kt / self.CurrentDrivetrain().R)
    low_torque = ((12.0 - low_omega / self.CurrentDrivetrain().Kv) *
                  self.CurrentDrivetrain().Kt / self.CurrentDrivetrain().R)
    high_power = high_torque * high_omega
    low_power = low_torque * low_omega
    if should_print:
      print gear_name, "High omega", high_omega, "Low omega", low_omega
      print gear_name, "High torque", high_torque, "Low torque", low_torque
      print gear_name, "High power", high_power, "Low power", low_power
      if (high_power > low_power) != current_gear:
        if high_power > low_power:
          print gear_name, "Shifting to high"
        else:
          print gear_name, "Shifting to low"

    return high_power > low_power

  def FilterVelocity(self, throttle):
    # Invert the plant to figure out how the velocity filter would have to work
    # out in order to filter out the forwards negative inertia.
    # This math assumes that the left and right power and velocity are equal.

    # The throttle filter should filter such that the motor in the highest gear
    # should be controlling the time constant.
    # Do this by finding the index of FF that has the lowest value, and computing
    # the sums using that index.
    FF_sum = self.CurrentDrivetrain().FF.sum(axis=1)
    max_FF_sum_index = numpy.argmin(FF_sum)
    max_FF_sum = FF_sum[max_FF_sum_index, 0]
    max_K_sum = self.CurrentDrivetrain().K[max_FF_sum_index, :].sum()
    max_A_sum = self.CurrentDrivetrain().A[max_FF_sum_index, :].sum()
    max_B_sum = self.CurrentDrivetrain().B[max_FF_sum_index, :].sum()
    # Compute the FF sum for high gear.
    high_max_FF_sum = self.drivetrain_high_high.FF[0, :].sum()

    # U = self.K[0, :].sum() * (R - x_avg) + self.FF[0, :].sum() * R
    # throttle * 12.0 = (self.K[0, :].sum() + self.FF[0, :].sum()) * R
    #                   - self.K[0, :].sum() * x_avg

    # R = (throttle * 12.0 + self.K[0, :].sum() * x_avg) /
    #     (self.K[0, :].sum() + self.FF[0, :].sum())

    # U = (K + FF) * R - K * X
    # (K + FF) ^-1 * (U + K * X) = R

    # Scale throttle by max_FF_sum / high_max_FF_sum.  This will make low gear
    # have the same velocity goal as high gear, and so that the robot will hold
    # the same speed for the same throttle for all gears.
    adjusted_ff_voltage = numpy.clip(throttle * 12.0 * max_FF_sum / high_max_FF_sum, -12.0, 12.0)
    return ((adjusted_ff_voltage + self.ttrust * max_K_sum * (self.X[0, 0] + self.X[1, 0]) / 2.0)
            / (self.ttrust * max_K_sum + max_FF_sum))

  def Update(self, throttle, steering):
    # Shift into the gear which sends the most power to the floor.
    # This is the same as sending the most torque down to the floor at the
    # wheel.

    self.left_high = self.ComputeGear(self.X[0, 0], should_print=True, current_gear=self.left_high, gear_name="left")
    self.right_high = self.ComputeGear(self.X[1, 0], should_print=True, current_gear=self.right_high, gear_name="right")

    FF_sum = self.CurrentDrivetrain().FF.sum(axis=1)

    # Filter the throttle to provide a nicer response.

    # TODO(austin): fn
    fvel = self.FilterVelocity(throttle)

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
    print "U is", self.U[0, 0], self.U[1, 0]


def main(argv):
  drivetrain = VelocityDrivetrain()

  vl_plot = []
  vr_plot = []
  ul_plot = []
  ur_plot = []
  radius_plot = []
  t_plot = []
  left_gear_plot = []
  right_gear_plot = []
  drivetrain.left_high = True
  drivetrain.right_high = True

  if drivetrain.left_high:
    print "Left is high"
  else:
    print "Left is low"
  if drivetrain.right_high:
    print "Right is high"
  else:
    print "Right is low"

  for t in numpy.arange(0, 2.0, drivetrain.dt):
    if t < 1.0:
      drivetrain.Update(throttle=0.60, steering=0.3)
    elif t < 1.5:
      drivetrain.Update(throttle=0.60, steering=-0.3)
    else:
      drivetrain.Update(throttle=0.0, steering=0.3)
    t_plot.append(t)
    vl_plot.append(drivetrain.X[0, 0])
    vr_plot.append(drivetrain.X[1, 0])
    ul_plot.append(drivetrain.U[0, 0])
    ur_plot.append(drivetrain.U[1, 0])
    left_gear_plot.append(drivetrain.left_high * 2.0 - 10.0)
    right_gear_plot.append(drivetrain.right_high * 2.0 - 10.0)

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
  pylab.plot(t_plot, left_gear_plot, label='left_gear')
  pylab.plot(t_plot, right_gear_plot, label='right_gear')
  pylab.legend()
  pylab.show()
  return 0

if __name__ == '__main__':
  sys.exit(main(sys.argv))
