#!/usr/bin/python

import math
import sys
import random

import numpy
import gflags
import glog
from matplotlib import pylab

from frc971.control_loops.python import controls
from frc971.control_loops.python import control_loop

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

class DownEstimator(control_loop.ControlLoop):
  def __init__(self, name='DownEstimator'):
    super(DownEstimator, self).__init__(name)
    self.dt = 0.005

    # State is [gyro_angle, bias].
    # U is [gyro_x_velocity].

    self.A_continuous = numpy.matrix([[0, 1],
                                      [0, 0]])

    self.B_continuous = numpy.matrix([[1],
                                      [0]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    q_gyro_noise = 1e-6
    q_gyro_bias_noise = 1e-3
    self.Q = numpy.matrix([[1.0 / (q_gyro_noise ** 2.0), 0.0],
                           [0.0, 1.0 / (q_gyro_bias_noise ** 2.0)]])

    r_accelerometer_angle_noise = 5e+7
    self.R = numpy.matrix([[(r_accelerometer_angle_noise ** 2.0)]])

    self.C = numpy.matrix([[1.0, 0.0]])
    self.D = numpy.matrix([[0]])

    self.U_max = numpy.matrix([[numpy.pi]])
    self.U_min = numpy.matrix([[-numpy.pi]])
    self.K = numpy.matrix(numpy.zeros((1, 2)))

    self.KalmanGain, self.Q_steady = controls.kalman(
        A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)

    self.L = self.A * self.KalmanGain

    self.InitializeState()

  def Update(self, accelerometer_x, accelerometer_y, accelerometer_z, gyro_x):
    acceleration = math.sqrt(
        accelerometer_x**2 + accelerometer_y**2 + accelerometer_z**2)
    if acceleration < 0.9 or acceleration > 1.1:
      glog.error('bad total acceleration %f' % acceleration)
      # TODO(Brian): Tune this?
      return
    accelerometer_angle = math.atan2(accelerometer_x, accelerometer_z)
    Z = numpy.matrix([[accelerometer_angle], [gyro_x]])

    Y = Z - self.H * self.X_hat
    S = self.H * self.P * self.H.transpose() + self.R
    K = self.P * self.H.transpose() * numpy.linalg.inv(S)
    self.X_hat += K * Y
    self.P = (numpy.identity(K.shape[0]) - K * self.H) * self.P

def main(argv):
  argv = FLAGS(argv)
  glog.init()

  estimator = DownEstimator()

  if FLAGS.plot:
    real_angles = [0]
    real_velocities = [0]
    estimated_angles = [0]
    estimated_velocities = [0]
    for _ in xrange(100):
      estimator.Predict(0)
      estimator.Update(numpy.sqrt(2) / 2.0, numpy.sqrt(2) / 2.0, 0, 0)
      real_angles.append(math.pi / 2)
      real_velocities.append(0)
      estimated_angles.append(estimator.X_hat[0, 0])
      estimated_velocities.append(estimator.X_hat[1, 0])
    angle = math.pi / 2
    velocity = 1
    for i in xrange(100):
      measured_velocity = velocity + (random.random() - 0.5) * 0.01 + 0.05
      estimator.Predict(measured_velocity)
      estimator.Update(math.sin(angle) + (random.random() - 0.5) * 0.02, 0,
                       math.cos(angle) + (random.random() - 0.5) * 0.02,
                       measured_velocity)
      real_angles.append(angle)
      real_velocities.append(velocity)
      estimated_angles.append(estimator.X_hat[0, 0])
      estimated_velocities.append(estimator.X_hat[1, 0])
      angle += velocity * estimator.dt
    pylab.plot(range(len(real_angles)), real_angles)
    pylab.plot(range(len(real_velocities)), real_velocities)
    pylab.plot(range(len(estimated_angles)), estimated_angles)
    pylab.plot(range(len(estimated_velocities)), estimated_velocities)
    pylab.show()

  if len(argv) != 3:
    print "Expected .h file name and .cc file name"
  else:
    namespaces = ['frc971', 'control_loops', 'drivetrain']
    kf_loop_writer = control_loop.ControlLoopWriter(
        "DownEstimator", [estimator],
        namespaces = namespaces)
    kf_loop_writer.Write(argv[1], argv[2])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
