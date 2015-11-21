#!/usr/bin/python

import control_loop
import controls
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
    self.dt = 0.005

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
    self.J = 10
    # Mass of the robot, in kg.
    self.m = 68
    # Radius of the robot, in meters (from last year).
    self.rb = 0.9603 / 2.0
    # Radius of the wheels, in meters.
    self.r = 0.0508
    # Resistance of the motor, divided by the number of motors.
    self.R = 12.0 / self.stall_current / 2
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
               (12.0 - self.R * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratios
    self.G_const = 18.0 / 44.0 * 18.0 / 60.0

    self.G_low = self.G_const
    self.G_high = self.G_const

    if left_low:
      self.Gl = self.G_low
    else:
      self.Gl = self.G_high
    if right_low:
      self.Gr = self.G_low
    else:
      self.Gr = self.G_high

    # Control loop time step
    self.dt = 0.005

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

    #print "THE NUMBER I WANT" + str(numpy.linalg.inv(self.A_continuous) * -self.B_continuous * numpy.matrix([[12.0], [12.0]]))
    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    # Poles from last year.
    self.hp = 0.65
    self.lp = 0.83
    self.PlaceControllerPoles([self.hp, self.lp, self.hp, self.lp])
    print self.K
    q_pos = 0.07
    q_vel = 1.0
    self.Q = numpy.matrix([[(1.0 / (q_pos ** 2.0)), 0.0, 0.0, 0.0],
                           [0.0, (1.0 / (q_vel ** 2.0)), 0.0, 0.0],
                           [0.0, 0.0, (1.0 / (q_pos ** 2.0)), 0.0],
                           [0.0, 0.0, 0.0, (1.0 / (q_vel ** 2.0))]])

    self.R = numpy.matrix([[(1.0 / (12.0 ** 2.0)), 0.0],
                           [0.0, (1.0 / (12.0 ** 2.0))]])
    self.K = controls.dlqr(self.A, self.B, self.Q, self.R)
    print self.A
    print self.B
    print self.K
    print numpy.linalg.eig(self.A - self.B * self.K)[0]

    self.hlp = 0.3
    self.llp = 0.4
    self.PlaceObserverPoles([self.hlp, self.hlp, self.llp, self.llp])

    self.U_max = numpy.matrix([[12.0], [12.0]])
    self.U_min = numpy.matrix([[-12.0], [-12.0]])
    self.InitializeState()

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
  print "Output one"
  drivetrain_low_low = Drivetrain(name="DrivetrainLowLow", left_low=True, right_low=True)
  drivetrain_low_high = Drivetrain(name="DrivetrainLowHigh", left_low=True, right_low=False)
  drivetrain_high_low = Drivetrain(name="DrivetrainHighLow", left_low=False, right_low=True)
  drivetrain_high_high = Drivetrain(name="DrivetrainHighHigh", left_low=False, right_low=False)

  if len(argv) != 5:
    print "Expected .h file name and .cc file name"
  else:
    dog_loop_writer = control_loop.ControlLoopWriter(
        "Drivetrain", [drivetrain_low_low, drivetrain_low_high,
                       drivetrain_high_low, drivetrain_high_high],
        namespaces=['y2015_bot3', 'control_loops'])
    if argv[1][-3:] == '.cc':
      dog_loop_writer.Write(argv[2], argv[1])
    else:
      dog_loop_writer.Write(argv[1], argv[2])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
