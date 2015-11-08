#!/usr/bin/python

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
import sys
from matplotlib import pylab


class CIM(control_loop.ControlLoop):
  def __init__(self):
    super(CIM, self).__init__("CIM")
    # Stall Torque in N m
    self.stall_torque = 2.42
    # Stall Current in Amps
    self.stall_current = 133
    # Free Speed in RPM
    self.free_speed = 4650.0
    # Free Current in Amps
    self.free_current = 2.7
    # Moment of inertia of the CIM in kg m^2
    self.J = 0.0001
    # Resistance of the motor, divided by 2 to account for the 2 motors
    self.R = 12.0 / self.stall_current
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
              (12.0 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Control loop time step
    self.dt = 0.010

    # State feedback matrices
    self.A_continuous = numpy.matrix(
        [[-self.Kt / self.Kv / (self.J * self.R)]])
    self.B_continuous = numpy.matrix(
        [[self.Kt / (self.J * self.R)]])
    self.C = numpy.matrix([[1]])
    self.D = numpy.matrix([[0]])

    self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                               self.B_continuous, self.dt)

    self.PlaceControllerPoles([0.01])
    self.PlaceObserverPoles([0.01])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    self.InitializeState()


class Drivetrain(control_loop.ControlLoop):
  def __init__(self, name="Drivetrain", left_low=True, right_low=True):
    super(Drivetrain, self).__init__(name)
    # Stall Torque in N m
    self.stall_torque = 2.42
    # Stall Current in Amps
    self.stall_current = 133.0
    # Free Speed in RPM. Used number from last year.
    self.free_speed = 4650.0
    # Free Current in Amps
    self.free_current = 2.7
    # Moment of inertia of the drivetrain in kg m^2
    # Just borrowed from last year.
    self.J = 4.5
    # Mass of the robot, in kg.
    self.m = 68
    # Radius of the robot, in meters (from last year).
    self.rb = 0.617998644 / 2.0
    # Radius of the wheels, in meters.
    self.r = .04445
    # Resistance of the motor, divided by the number of motors.
    self.R = 12.0 / self.stall_current / 2
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
               (12.0 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratios
    self.G_low = 18.0 / 60.0 * 18.0 / 50.0
    self.G_high = 28.0 / 50.0 * 18.0 / 50.0
    if left_low:
      self.Gl = self.G_low
    else:
      self.Gl = self.G_high
    if right_low:
      self.Gr = self.G_low
    else:
      self.Gr = self.G_high

    # Control loop time step
    self.dt = 0.010

    # These describe the way that a given side of a robot will be influenced
    # by the other side. Units of 1 / kg.
    self.msp = 1.0 / self.m + self.rb * self.rb / self.J
    self.msn = 1.0 / self.m - self.rb * self.rb / self.J
    # The calculations which we will need for A and B.
    self.tcl = -self.Kt / self.Kv / (self.Gl * self.Gl * self.R * self.r * self.r)
    self.tcr = -self.Kt / self.Kv / (self.Gr * self.Gr * self.R * self.r * self.r)
    self.mpl = self.Kt / (self.Gl * self.R * self.r)
    self.mpr = self.Kt / (self.Gr * self.R * self.r)

    # State feedback matrices
    # X will be of the format
    # [[positionl], [velocityl], [positionr], velocityr]]
    self.A_continuous = numpy.matrix(
        [[0, 1, 0, 0],
         [0, self.msp * self.tcl, 0, self.msn * self.tcr],
         [0, 0, 0, 1],
         [0, self.msn * self.tcl, 0, self.msp * self.tcr]])
    self.B_continuous = numpy.matrix(
        [[0, 0],
         [self.msp * self.mpl, self.msn * self.mpr],
         [0, 0],
         [self.msn * self.mpl, self.msp * self.mpr]])
    self.C = numpy.matrix([[1, 0, 0, 0],
                           [0, 0, 1, 0]])
    self.D = numpy.matrix([[0, 0],
                           [0, 0]])

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    # Poles from last year.
    self.hp = 0.65
    self.lp = 0.83
    self.PlaceControllerPoles([self.hp, self.lp, self.hp, self.lp])
    #print self.K
    q_pos = 0.07
    q_vel = 1.0
    self.Q = numpy.matrix([[(1.0 / (q_pos ** 2.0)), 0.0, 0.0, 0.0],
                           [0.0, (1.0 / (q_vel ** 2.0)), 0.0, 0.0],
                           [0.0, 0.0, (1.0 / (q_pos ** 2.0)), 0.0],
                           [0.0, 0.0, 0.0, (1.0 / (q_vel ** 2.0))]])

    self.R = numpy.matrix([[(1.0 / (12.0 ** 2.0)), 0.0],
                           [0.0, (1.0 / (12.0 ** 2.0))]])
    self.K = controls.dlqr(self.A, self.B, self.Q, self.R)
    #print self.A
    #print self.B
    #print self.K
    #print numpy.linalg.eig(self.A - self.B * self.K)[0]

    self.hlp = 0.3
    self.llp = 0.4
    self.PlaceObserverPoles([self.hlp, self.hlp, self.llp, self.llp])

    self.U_max = numpy.matrix([[12.0], [12.0]])
    self.U_min = numpy.matrix([[-12.0], [-12.0]])
    self.InitializeState()


class KFDrivetrain(Drivetrain):
  def __init__(self, name="KFDrivetrain", left_low=True, right_low=True):
    super(KFDrivetrain, self).__init__(name, left_low, right_low)

    self.unaugmented_A_continuous = self.A_continuous
    self.unaugmented_B_continuous = self.B_continuous

    # The states are
    # The practical voltage applied to the wheels is
    #   V_left = U_left + left_voltage_error
    #
    # [left position, left velocity, right position, right velocity,
    #  left voltage error, right voltage error, angular_error]
    self.A_continuous = numpy.matrix(numpy.zeros((7, 7)))
    self.B_continuous = numpy.matrix(numpy.zeros((7, 2)))
    self.A_continuous[0:4,0:4] = self.unaugmented_A_continuous
    self.A_continuous[0:4,4:6] = self.unaugmented_B_continuous
    self.B_continuous[0:4,0:2] = self.unaugmented_B_continuous

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    self.C = numpy.matrix([[1, 0, 0, 0, 0, 0, self.rb],
                           [0, 0, 1, 0, 0, 0, -self.rb],
                           [0, -2.0 / self.rb, 0, 2.0 / self.rb, 0, 0, 0]])

    self.D = numpy.matrix([[0, 0],
                           [0, 0],
                           [0, 0]])

    q_pos = 0.08
    q_vel = 0.40
    q_voltage = 6.0
    q_gyro = 0.1

    self.Q = numpy.matrix([[(q_pos ** 2.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, (q_vel ** 2.0), 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, (q_pos ** 2.0), 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, (q_vel ** 2.0), 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, (q_voltage ** 2.0), 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, (q_voltage ** 2.0), 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (q_gyro ** 2.0)]])

    r_pos = 0.05
    r_gyro = 0.001
    self.R = numpy.matrix([[(r_pos ** 2.0), 0.0, 0.0],
                           [0.0, (r_pos ** 2.0), 0.0],
                           [0.0, 0.0, (r_gyro ** 2.0)]])

    # Solving for kf gains.
    self.KalmanGain, self.Q_steady = controls.kalman(
        A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)

    self.L = self.A * self.KalmanGain

    # We need a nothing controller for the autogen code to be happy.
    self.K = numpy.matrix(numpy.zeros((self.B.shape[1], self.A.shape[0])))


def main(argv):
  # Simulate the response of the system to a step input.
  drivetrain = Drivetrain()
  simulated_left = []
  simulated_right = []
  for _ in xrange(100):
    drivetrain.Update(numpy.matrix([[12.0], [12.0]]))
    simulated_left.append(drivetrain.X[0, 0])
    simulated_right.append(drivetrain.X[2, 0])

  #pylab.plot(range(100), simulated_left)
  #pylab.plot(range(100), simulated_right)
  #pylab.show()

  # Simulate forwards motion.
  drivetrain = Drivetrain()
  close_loop_left = []
  close_loop_right = []
  R = numpy.matrix([[1.0], [0.0], [1.0], [0.0]])
  for _ in xrange(100):
    U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat),
                   drivetrain.U_min, drivetrain.U_max)
    drivetrain.UpdateObserver(U)
    drivetrain.Update(U)
    close_loop_left.append(drivetrain.X[0, 0])
    close_loop_right.append(drivetrain.X[2, 0])

  #pylab.plot(range(100), close_loop_left)
  #pylab.plot(range(100), close_loop_right)
  #pylab.show()

  # Try turning in place
  drivetrain = Drivetrain()
  close_loop_left = []
  close_loop_right = []
  R = numpy.matrix([[-1.0], [0.0], [1.0], [0.0]])
  for _ in xrange(100):
    U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat),
                   drivetrain.U_min, drivetrain.U_max)
    drivetrain.UpdateObserver(U)
    drivetrain.Update(U)
    close_loop_left.append(drivetrain.X[0, 0])
    close_loop_right.append(drivetrain.X[2, 0])

  #pylab.plot(range(100), close_loop_left)
  #pylab.plot(range(100), close_loop_right)
  #pylab.show()

  # Try turning just one side.
  drivetrain = Drivetrain()
  close_loop_left = []
  close_loop_right = []
  R = numpy.matrix([[0.0], [0.0], [1.0], [0.0]])
  for _ in xrange(100):
    U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat),
                   drivetrain.U_min, drivetrain.U_max)
    drivetrain.UpdateObserver(U)
    drivetrain.Update(U)
    close_loop_left.append(drivetrain.X[0, 0])
    close_loop_right.append(drivetrain.X[2, 0])

  #pylab.plot(range(100), close_loop_left)
  #pylab.plot(range(100), close_loop_right)
  #pylab.show()

  # Write the generated constants out to a file.
  drivetrain_low_low = Drivetrain(name="DrivetrainLowLow", left_low=True, right_low=True)
  drivetrain_low_high = Drivetrain(name="DrivetrainLowHigh", left_low=True, right_low=False)
  drivetrain_high_low = Drivetrain(name="DrivetrainHighLow", left_low=False, right_low=True)
  drivetrain_high_high = Drivetrain(name="DrivetrainHighHigh", left_low=False, right_low=False)

  kf_drivetrain_low_low = KFDrivetrain(name="KFDrivetrainLowLow", left_low=True, right_low=True)
  kf_drivetrain_low_high = KFDrivetrain(name="KFDrivetrainLowHigh", left_low=True, right_low=False)
  kf_drivetrain_high_low = KFDrivetrain(name="KFDrivetrainHighLow", left_low=False, right_low=True)
  kf_drivetrain_high_high = KFDrivetrain(name="KFDrivetrainHighHigh", left_low=False, right_low=False)

  if len(argv) != 5:
    print "Expected .h file name and .cc file name"
  else:
    namespaces = ['y2014', 'control_loops', 'drivetrain']
    dog_loop_writer = control_loop.ControlLoopWriter(
        "Drivetrain", [drivetrain_low_low, drivetrain_low_high,
                       drivetrain_high_low, drivetrain_high_high],
        namespaces = namespaces)
    dog_loop_writer.AddConstant(control_loop.Constant("kDt", "%f",
          drivetrain_low_low.dt))
    if argv[1][-3:] == '.cc':
      dog_loop_writer.Write(argv[2], argv[1])
    else:
      dog_loop_writer.Write(argv[1], argv[2])

    kf_loop_writer = control_loop.ControlLoopWriter(
        "KFDrivetrain", [kf_drivetrain_low_low, kf_drivetrain_low_high,
                       kf_drivetrain_high_low, kf_drivetrain_high_high],
        namespaces = namespaces)
    if argv[3][-3:] == '.cc':
      kf_loop_writer.Write(argv[4], argv[3])
    else:
      kf_loop_writer.Write(argv[3], argv[4])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
