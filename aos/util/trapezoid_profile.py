#!/usr/bin/python

import numpy

class TrapezoidProfile(object):
  """Computes a trapezoidal motion profile

  Attributes:
    _acceleration_time: the amount of time the robot will travel at the
        specified acceleration (s)
    _acceleration: the acceleration the robot will use to get to the target
        (unit/s^2)
    _constant_time: amount of time to travel at a constant velocity to reach
        target (s)
    _deceleration_time: amount of time to decelerate (at specified
        deceleration) to target (s)
    _deceleration: decceleration the robot needs to get to goal velocity
        (units/s^2)
    _maximum_acceleration: the maximum acceleration (units/s^2)
    _maximum_velocity: the maximum velocity (unit/s)
    _timestep: time between calls to Update (delta_time)
    _output: output array containing distance to goal and velocity
  """
  def __init__(self, delta_time):
    """Constructs a TrapezoidProfile.

    Args:
      delta_time: time between calls to Update (seconds)
    """
    self._acceleration_time = 0
    self._acceleration = 0
    self._constant_time = 0
    self._deceleration_time = 0
    self._deceleration = 0

    self._maximum_acceleration = 0
    self._maximum_velocity = 0
    self._timestep = delta_time

    self._output = numpy.array(numpy.zeros((2,1)))

  # Updates the state
  def Update(self, goal_position, goal_velocity):
    self._CalculateTimes(goal_position - self._output[0], goal_velocity)

    next_timestep = self._timestep

    # We now have the amount of time we need to accelerate to follow the
    # profile, the amount of time we need to move at constant velocity
    # to follow the profile, and the amount of time we need to decelerate to
    # follow the profile.  Do as much of that as we have time left in dt.
    if self._acceleration_time > next_timestep:
      self._UpdateVals(self._acceleration, next_timestep)
    else:
      self._UpdateVals(self._acceleration, self._acceleration_time)

      next_timestep -= self._acceleration_time
      if self._constant_time > next_timestep:
        self._UpdateVals(0, next_timestep)
      else:
        self._UpdateVals(0, self._constant_time)
        next_timestep -= self._constant_time;
        if self._deceleration_time > next_timestep:
          self._UpdateVals(self._deceleration, next_timestep)
        else:
          self._UpdateVals(self._deceleration, self._deceleration_time)
          next_timestep -= self._deceleration_time
          self._UpdateVals(0, next_timestep)

    return self._output

  # Useful for preventing windup etc.
  def MoveCurrentState(self, current):
    self._output = current

  # Useful for preventing windup etc.
  def MoveGoal(self, dx):
    self._output[0] += dx

  def SetGoal(self, x):
    self._output[0] = x

  def set_maximum_acceleration(self, maximum_acceleration):
    self._maximum_acceleration = maximum_acceleration

  def set_maximum_velocity(self, maximum_velocity):
    self._maximum_velocity = maximum_velocity

  def _UpdateVals(self, acceleration, delta_time):
    self._output[0, 0] += (self._output[1, 0] * delta_time
        + 0.5 * acceleration * delta_time * delta_time)
    self._output[1, 0] += acceleration * delta_time

  def _CalculateTimes(self, distance_to_target, goal_velocity):
    if distance_to_target == 0:
      self._acceleration_time = 0
      self._acceleration = 0
      self._constant_time = 0
      self._deceleration_time = 0
      self._deceleration = 0
      return
    elif distance_to_target < 0:
      # Recurse with everything inverted.
      self._output[1] *= -1
      self._CalculateTimes(-distance_to_target, -goal_velocity)
      self._output[1] *= -1
      self._acceleration *= -1
      self._deceleration *= -1
      return

    self._constant_time = 0
    self._acceleration = self._maximum_acceleration
    maximum_acceleration_velocity = (
        distance_to_target * 2 * numpy.abs(self._acceleration)
        + self._output[1] * self._output[1])
    if maximum_acceleration_velocity > 0:
      maximum_acceleration_velocity = numpy.sqrt(maximum_acceleration_velocity)
    else:
      maximum_acceleration_velocity = -numpy.sqrt(-maximum_acceleration_velocity)

    # Since we know what we'd have to do if we kept after it to decelerate, we
    # know the sign of the acceleration.
    if maximum_acceleration_velocity > goal_velocity:
      self._deceleration = -self._maximum_acceleration
    else:
      self._deceleration = self._maximum_acceleration

    # We now know the top velocity we can get to.
    top_velocity = numpy.sqrt((distance_to_target +
                                (self._output[1] * self._output[1]) /
                                (2.0 * self._acceleration) +
                                (goal_velocity * goal_velocity) /
                                (2.0 * self._deceleration)) /
                               (-1.0 / (2.0 * self._deceleration) +
                                1.0 / (2.0 * self._acceleration)))

    # If it can go too fast, we now know how long we get to accelerate for and
    # how long to go at constant velocity.
    if top_velocity > self._maximum_velocity:
      self._acceleration_time = ((self._maximum_velocity - self._output[1]) /
                                 self._maximum_acceleration)
      self._constant_time = (distance_to_target +
                        (goal_velocity * goal_velocity -
                         self._maximum_velocity * self._maximum_velocity) /
                        (2.0 * self._maximum_acceleration)) / self._maximum_velocity
    else:
      self._acceleration_time = (
          (top_velocity - self._output[1]) / self._acceleration)

    if self._output[1] > self._maximum_velocity:
      self._constant_time = 0
      self._acceleration_time = 0

    self._deceleration_time = (
        (goal_velocity - top_velocity) / self._deceleration)

