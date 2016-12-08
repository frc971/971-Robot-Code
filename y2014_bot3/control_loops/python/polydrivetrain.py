#!/usr/bin/python

import numpy
import sys
from frc971.control_loops.python import polytope
from y2014_bot3.control_loops.python import drivetrain
from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
from frc971.control_loops.python.cim import CIM
from matplotlib import pylab

import gflags
import glog

__author__ = 'Austin Schuh (austin.linux@gmail.com)'

FLAGS = gflags.FLAGS

try:
  gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
  pass

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
  return DoCoerceGoal(region, K, w, R)[0]

def DoCoerceGoal(region, K, w, R):
  if region.IsInside(R):
    return (R, True)

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

    return (closest_point, True)
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

    return (closest_point, False)


class VelocityDrivetrainModel(control_loop.ControlLoop):
  def __init__(self, left_low=True, right_low=True, name="VelocityDrivetrainModel"):
    super(VelocityDrivetrainModel, self).__init__(name)
    self._drivetrain = drivetrain.Drivetrain(left_low=left_low,
                                             right_low=right_low)
    self.dt = 0.005
    self.A_continuous = numpy.matrix(
        [[self._drivetrain.A_continuous[1, 1], self._drivetrain.A_continuous[1, 3]],
         [self._drivetrain.A_continuous[3, 1], self._drivetrain.A_continuous[3, 3]]])

    self.B_continuous = numpy.matrix(
        [[self._drivetrain.B_continuous[1, 0], self._drivetrain.B_continuous[1, 1]],
         [self._drivetrain.B_continuous[3, 0], self._drivetrain.B_continuous[3, 1]]])
    self.C = numpy.matrix(numpy.eye(2))
    self.D = numpy.matrix(numpy.zeros((2, 2)))

    self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                               self.B_continuous, self.dt)

    # FF * X = U (steady state)
    self.FF = self.B.I * (numpy.eye(2) - self.A)

    self.PlaceControllerPoles([0.67, 0.67])
    self.PlaceObserverPoles([0.02, 0.02])

    self.G_high = self._drivetrain.G_high
    self.G_low = self._drivetrain.G_low
    self.resistance = self._drivetrain.resistance
    self.r = self._drivetrain.r
    self.Kv = self._drivetrain.Kv
    self.Kt = self._drivetrain.Kt

    self.U_max = self._drivetrain.U_max
    self.U_min = self._drivetrain.U_min


class VelocityDrivetrain(object):
  HIGH = 'high'
  LOW = 'low'
  SHIFTING_UP = 'up'
  SHIFTING_DOWN = 'down'

  def __init__(self):
    self.drivetrain_low_low = VelocityDrivetrainModel(
        left_low=True, right_low=True, name='VelocityDrivetrainLowLow')
    self.drivetrain_low_high = VelocityDrivetrainModel(left_low=True, right_low=False, name='VelocityDrivetrainLowHigh')
    self.drivetrain_high_low = VelocityDrivetrainModel(left_low=False, right_low=True, name = 'VelocityDrivetrainHighLow')
    self.drivetrain_high_high = VelocityDrivetrainModel(left_low=False, right_low=False, name = 'VelocityDrivetrainHighHigh')

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

    self.dt = 0.005

    self.R = numpy.matrix(
        [[0.0],
         [0.0]])

    self.U_ideal = numpy.matrix(
        [[0.0],
         [0.0]])

    # ttrust is the comprimise between having full throttle negative inertia,
    # and having no throttle negative inertia.  A value of 0 is full throttle
    # inertia.  A value of 1 is no throttle negative inertia.
    self.ttrust = 1.0

    self.left_gear = VelocityDrivetrain.LOW
    self.right_gear = VelocityDrivetrain.LOW
    self.left_shifter_position = 0.0
    self.right_shifter_position = 0.0
    self.left_cim = CIM()
    self.right_cim = CIM()

  def IsInGear(self, gear):
    return gear is VelocityDrivetrain.HIGH or gear is VelocityDrivetrain.LOW

  def MotorRPM(self, shifter_position, velocity):
    if shifter_position > 0.5:
      return (velocity / self.CurrentDrivetrain().G_high /
              self.CurrentDrivetrain().r)
    else:
      return (velocity / self.CurrentDrivetrain().G_low /
              self.CurrentDrivetrain().r)

  def CurrentDrivetrain(self):
    if self.left_shifter_position > 0.5:
      if self.right_shifter_position > 0.5:
        return self.drivetrain_high_high
      else:
        return self.drivetrain_high_low
    else:
      if self.right_shifter_position > 0.5:
        return self.drivetrain_low_high
      else:
        return self.drivetrain_low_low

  def SimShifter(self, gear, shifter_position):
    if gear is VelocityDrivetrain.HIGH or gear is VelocityDrivetrain.SHIFTING_UP:
      shifter_position = min(shifter_position + 0.5, 1.0)
    else:
      shifter_position = max(shifter_position - 0.5, 0.0)

    if shifter_position == 1.0:
      gear = VelocityDrivetrain.HIGH
    elif shifter_position == 0.0:
      gear = VelocityDrivetrain.LOW

    return gear, shifter_position

  def ComputeGear(self, wheel_velocity, should_print=False, current_gear=False, gear_name=None):
    high_omega = (wheel_velocity / self.CurrentDrivetrain().G_high /
                  self.CurrentDrivetrain().r)
    low_omega = (wheel_velocity / self.CurrentDrivetrain().G_low /
                 self.CurrentDrivetrain().r)
    #print gear_name, "Motor Energy Difference.", 0.5 * 0.000140032647 * (low_omega * low_omega - high_omega * high_omega), "joules"
    high_torque = ((12.0 - high_omega / self.CurrentDrivetrain().Kv) *
                   self.CurrentDrivetrain().Kt / self.CurrentDrivetrain().resistance)
    low_torque = ((12.0 - low_omega / self.CurrentDrivetrain().Kv) *
                  self.CurrentDrivetrain().Kt / self.CurrentDrivetrain().resistance)
    high_power = high_torque * high_omega
    low_power = low_torque * low_omega
    #if should_print:
    #  print gear_name, "High omega", high_omega, "Low omega", low_omega
    #  print gear_name, "High torque", high_torque, "Low torque", low_torque
    #  print gear_name, "High power", high_power, "Low power", low_power

    # Shift algorithm improvements.
    # TODO(aschuh):
    # It takes time to shift.  Shifting down for 1 cycle doesn't make sense
    # because you will end up slower than without shifting.  Figure out how
    # to include that info.
    # If the driver is still in high gear, but isn't asking for the extra power
    # from low gear, don't shift until he asks for it.
    goal_gear_is_high = high_power > low_power
    #goal_gear_is_high = True

    if not self.IsInGear(current_gear):
      glog.debug('%s Not in gear.', gear_name)
      return current_gear
    else:
      is_high = current_gear is VelocityDrivetrain.HIGH
      if is_high != goal_gear_is_high:
        if goal_gear_is_high:
          glog.debug('%s Shifting up.', gear_name)
          return VelocityDrivetrain.SHIFTING_UP
        else:
          glog.debug('%s Shifting down.', gear_name)
          return VelocityDrivetrain.SHIFTING_DOWN
      else:
        return current_gear

  def FilterVelocity(self, throttle):
    # Invert the plant to figure out how the velocity filter would have to work
    # out in order to filter out the forwards negative inertia.
    # This math assumes that the left and right power and velocity are equal.

    # The throttle filter should filter such that the motor in the highest gear
    # should be controlling the time constant.
    # Do this by finding the index of FF that has the lowest value, and computing
    # the sums using that index.
    FF_sum = self.CurrentDrivetrain().FF.sum(axis=1)
    min_FF_sum_index = numpy.argmin(FF_sum)
    min_FF_sum = FF_sum[min_FF_sum_index, 0]
    min_K_sum = self.CurrentDrivetrain().K[min_FF_sum_index, :].sum()
    # Compute the FF sum for high gear.
    high_min_FF_sum = self.drivetrain_high_high.FF[0, :].sum()

    # U = self.K[0, :].sum() * (R - x_avg) + self.FF[0, :].sum() * R
    # throttle * 12.0 = (self.K[0, :].sum() + self.FF[0, :].sum()) * R
    #                   - self.K[0, :].sum() * x_avg

    # R = (throttle * 12.0 + self.K[0, :].sum() * x_avg) /
    #     (self.K[0, :].sum() + self.FF[0, :].sum())

    # U = (K + FF) * R - K * X
    # (K + FF) ^-1 * (U + K * X) = R

    # Scale throttle by min_FF_sum / high_min_FF_sum.  This will make low gear
    # have the same velocity goal as high gear, and so that the robot will hold
    # the same speed for the same throttle for all gears.
    adjusted_ff_voltage = numpy.clip(throttle * 12.0 * min_FF_sum / high_min_FF_sum, -12.0, 12.0)
    return ((adjusted_ff_voltage + self.ttrust * min_K_sum * (self.X[0, 0] + self.X[1, 0]) / 2.0)
            / (self.ttrust * min_K_sum + min_FF_sum))

  def Update(self, throttle, steering):
    # Shift into the gear which sends the most power to the floor.
    # This is the same as sending the most torque down to the floor at the
    # wheel.

    self.left_gear = self.right_gear = True
    if True:
      self.left_gear = self.ComputeGear(self.X[0, 0], should_print=True,
                                        current_gear=self.left_gear,
                                        gear_name="left")
      self.right_gear = self.ComputeGear(self.X[1, 0], should_print=True,
                                         current_gear=self.right_gear,
                                         gear_name="right")
      if self.IsInGear(self.left_gear):
        self.left_cim.X[0, 0] = self.MotorRPM(self.left_shifter_position, self.X[0, 0])

      if self.IsInGear(self.right_gear):
        self.right_cim.X[0, 0] = self.MotorRPM(self.right_shifter_position, self.X[0, 0])

    if self.IsInGear(self.left_gear) and self.IsInGear(self.right_gear):
      # Filter the throttle to provide a nicer response.
      fvel = self.FilterVelocity(throttle)

      # Constant radius means that angualar_velocity / linear_velocity = constant.
      # Compute the left and right velocities.
      steering_velocity = numpy.abs(fvel) * steering
      left_velocity = fvel - steering_velocity
      right_velocity = fvel + steering_velocity

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
    else:
      glog.debug('Not all in gear')
      if not self.IsInGear(self.left_gear) and not self.IsInGear(self.right_gear):
        # TODO(austin): Use battery volts here.
        R_left = self.MotorRPM(self.left_shifter_position, self.X[0, 0])
        self.U_ideal[0, 0] = numpy.clip(
            self.left_cim.K * (R_left - self.left_cim.X) + R_left / self.left_cim.Kv,
            self.left_cim.U_min, self.left_cim.U_max)
        self.left_cim.Update(self.U_ideal[0, 0])

        R_right = self.MotorRPM(self.right_shifter_position, self.X[1, 0])
        self.U_ideal[1, 0] = numpy.clip(
            self.right_cim.K * (R_right - self.right_cim.X) + R_right / self.right_cim.Kv,
            self.right_cim.U_min, self.right_cim.U_max)
        self.right_cim.Update(self.U_ideal[1, 0])
      else:
        assert False

    self.U = numpy.clip(self.U_ideal, self.U_min, self.U_max)

    # TODO(austin): Model the robot as not accelerating when you shift...
    # This hack only works when you shift at the same time.
    if self.IsInGear(self.left_gear) and self.IsInGear(self.right_gear):
      self.X = self.CurrentDrivetrain().A * self.X + self.CurrentDrivetrain().B * self.U

    self.left_gear, self.left_shifter_position = self.SimShifter(
        self.left_gear, self.left_shifter_position)
    self.right_gear, self.right_shifter_position = self.SimShifter(
        self.right_gear, self.right_shifter_position)

    glog.debug('U is %s %s', str(self.U[0, 0]), str(self.U[1, 0]))
    glog.debug('Left shifter %s %d Right shifter %s %d',
               self.left_gear, self.left_shifter_position,
               self.right_gear, self.right_shifter_position)


def main(argv):
  argv = FLAGS(argv)

  vdrivetrain = VelocityDrivetrain()

  if not FLAGS.plot:
    if len(argv) != 5:
      glog.fatal('Expected .h file name and .cc file name')
    else:
      namespaces = ['y2014_bot3', 'control_loops', 'drivetrain']
      dog_loop_writer = control_loop.ControlLoopWriter(
          "VelocityDrivetrain", [vdrivetrain.drivetrain_low_low,
                         vdrivetrain.drivetrain_low_high,
                         vdrivetrain.drivetrain_high_low,
                         vdrivetrain.drivetrain_high_high],
                         namespaces=namespaces)

      dog_loop_writer.Write(argv[1], argv[2])

      cim_writer = control_loop.ControlLoopWriter("CIM", [CIM()])

      cim_writer.Write(argv[3], argv[4])
      return

  vl_plot = []
  vr_plot = []
  ul_plot = []
  ur_plot = []
  radius_plot = []
  t_plot = []
  left_gear_plot = []
  right_gear_plot = []
  vdrivetrain.left_shifter_position = 0.0
  vdrivetrain.right_shifter_position = 0.0
  vdrivetrain.left_gear = VelocityDrivetrain.LOW
  vdrivetrain.right_gear = VelocityDrivetrain.LOW

  glog.debug('K is %s', str(vdrivetrain.CurrentDrivetrain().K))

  if vdrivetrain.left_gear is VelocityDrivetrain.HIGH:
    glog.debug('Left is high')
  else:
    glog.debug('Left is low')
  if vdrivetrain.right_gear is VelocityDrivetrain.HIGH:
    glog.debug('Right is high')
  else:
    glog.debug('Right is low')

  for t in numpy.arange(0, 1.7, vdrivetrain.dt):
    if t < 0.5:
      vdrivetrain.Update(throttle=0.00, steering=1.0)
    elif t < 1.2:
      vdrivetrain.Update(throttle=0.5, steering=1.0)
    else:
      vdrivetrain.Update(throttle=0.00, steering=1.0)
    t_plot.append(t)
    vl_plot.append(vdrivetrain.X[0, 0])
    vr_plot.append(vdrivetrain.X[1, 0])
    ul_plot.append(vdrivetrain.U[0, 0])
    ur_plot.append(vdrivetrain.U[1, 0])
    left_gear_plot.append((vdrivetrain.left_gear is VelocityDrivetrain.HIGH) * 2.0 - 10.0)
    right_gear_plot.append((vdrivetrain.right_gear is VelocityDrivetrain.HIGH) * 2.0 - 10.0)

    fwd_velocity = (vdrivetrain.X[1, 0] + vdrivetrain.X[0, 0]) / 2
    turn_velocity = (vdrivetrain.X[1, 0] - vdrivetrain.X[0, 0])
    if abs(fwd_velocity) < 0.0000001:
      radius_plot.append(turn_velocity)
    else:
      radius_plot.append(turn_velocity / fwd_velocity)

  # TODO(austin):
  # Shifting compensation.

  # Tighten the turn.
  # Closed loop drive.

  pylab.plot(t_plot, vl_plot, label='left velocity')
  pylab.plot(t_plot, vr_plot, label='right velocity')
  pylab.plot(t_plot, ul_plot, label='left voltage')
  pylab.plot(t_plot, ur_plot, label='right voltage')
  pylab.plot(t_plot, radius_plot, label='radius')
  pylab.plot(t_plot, left_gear_plot, label='left gear high')
  pylab.plot(t_plot, right_gear_plot, label='right gear high')
  pylab.legend()
  pylab.show()
  return 0

if __name__ == '__main__':
  sys.exit(main(sys.argv))
