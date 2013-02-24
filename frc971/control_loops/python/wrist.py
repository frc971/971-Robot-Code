#!/usr/bin/python

import numpy
import string
import sys
import polytope
from matplotlib import pylab
import controls


class Wrist(object):
  def __init__(self):
    # Stall Torque in N m
    self.stall_torque = 1.4
    # Stall Current in Amps
    self.stall_current = 86
    # Free Speed in RPM
    self.free_speed = 6200.0
    # Moment of inertia of the wrist in kg m^2
    self.J = 0.51
    # Resistance of the motor
    self.R = 12.0 / self.stall_current + 0.024 + .003
    # Motor velocity constant
    self.Kv = (self.free_speed / 60.0 * 2.0 * numpy.pi) / (13.5 - self.R * 1.5)
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratio
    self.G = 1.0 / ((84.0 / 20.0) * (50.0 / 14.0) * (40.0 / 14.0) * (40.0 / 12.0))
    # Control loop time step
    self.dt = 0.01

    # State feedback matrices
    self.A_continuous = numpy.matrix(
        [[0, 1],
         [0, -self.Kt / self.Kv / (self.J * self.G * self.G * self.R)]])
    self.B_continuous = numpy.matrix(
        [[0],
         [self.Kt / (self.J * self.G * self.R)]])
    self.C = numpy.matrix([[1, 0]])
    self.D = numpy.matrix([[0]])

    self.A, self.B = controls.c2d(
        self.A_continuous, self.B_continuous, self.dt)

    self.K = controls.dplace(self.A, self.B, [.95, .92])

    self.rpl = .05
    self.ipl = 0.008
    self.L = controls.dplace(self.A.T, self.C.T,
                             [self.rpl + 1j * self.ipl,
                              self.rpl - 1j * self.ipl]).T

    self.X = numpy.matrix([[0],
                           [0]])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])
    self.Y = self.C * self.X

  def Update(self, U):
    U = numpy.clip(U, self.U_min, self.U_max)
    self.X = self.A * self.X + self.B * U
    self.Y = self.C * self.X + self.D * U

  def _DumpMatrix(self, matrix_name, matrix):
    ans = ["  Eigen::Matrix<double, %d, %d> %s;\n" % (
        matrix.shape[0], matrix.shape[1], matrix_name)]
    first = True
    for element in numpy.nditer(matrix, order='C'):
      if first:
        ans.append("  %s << " % matrix_name)
        first = False
      else:
        ans.append(", ")
      ans.append(str(element))

    ans.append(";\n")
    return "".join(ans)

  def DumpPlantHeader(self, plant_name):
    """Writes out a c++ header declaration which will create a Plant object.

    Args:
      plant_name: string, the name of the plant.  Used to create the name of the
        function.  The function name will be Make<plant_name>Plant().
    """
    num_states = self.A.shape[0]
    num_inputs = self.B.shape[1]
    num_outputs = self.C.shape[0]
    return "StateFeedbackPlant<%d, %d, %d> Make%sPlant();\n" % (
        num_states, num_inputs, num_outputs, plant_name)

  def DumpPlant(self, plant_name):
    """Writes out a c++ function which will create a Plant object.

    Args:
      plant_name: string, the name of the plant.  Used to create the name of the
        function.  The function name will be Make<plant_name>Plant().
    """
    num_states = self.A.shape[0]
    num_inputs = self.B.shape[1]
    num_outputs = self.C.shape[0]
    ans = ["StateFeedbackPlant<%d, %d, %d> Make%sPlant() {\n" % (
        num_states, num_inputs, num_outputs, plant_name)]

    ans.append(self._DumpMatrix("A", self.A))
    ans.append(self._DumpMatrix("B", self.B))
    ans.append(self._DumpMatrix("C", self.C))
    ans.append(self._DumpMatrix("D", self.D))
    ans.append(self._DumpMatrix("U_max", self.U_max))
    ans.append(self._DumpMatrix("U_min", self.U_min))

    ans.append("  return StateFeedbackPlant<%d, %d, %d>"
               "(A, B, C, D, U_max, U_min);\n" % (num_states, num_inputs,
                                                  num_outputs))
    ans.append("}\n")
    return "".join(ans)

  def DumpLoopHeader(self, loop_name):
    """Writes out a c++ header declaration which will create a Loop object.

    Args:
      loop_name: string, the name of the loop.  Used to create the name of the
        function.  The function name will be Make<loop_name>Loop().
    """
    num_states = self.A.shape[0]
    num_inputs = self.B.shape[1]
    num_outputs = self.C.shape[0]
    return "StateFeedbackLoop<%d, %d, %d> Make%sLoop();\n" % (
        num_states, num_inputs, num_outputs, loop_name)

  def DumpLoop(self, loop_name):
    """Writes out a c++ function which will create a Loop object.

    Args:
      loop_name: string, the name of the loop.  Used to create the name of the
        function and create the plant.  The function name will be
        Make<loop_name>Loop().
    """
    num_states = self.A.shape[0]
    num_inputs = self.B.shape[1]
    num_outputs = self.C.shape[0]
    ans = ["StateFeedbackLoop<%d, %d, %d> Make%sLoop() {\n" % (
        num_states, num_inputs, num_outputs, loop_name)]

    ans.append(self._DumpMatrix("L", self.L))
    ans.append(self._DumpMatrix("K", self.K))

    ans.append("  return StateFeedbackLoop<%d, %d, %d>"
               "(Make%sPlant(), L, K);\n" % (num_states, num_inputs,
                                             num_outputs, loop_name))
    ans.append("}\n")
    return "".join(ans)


def main(argv):
  wrist = Wrist()
  simulated_x = []
  for _ in xrange(100):
    wrist.Update(numpy.matrix([[12.0]]))
    simulated_x.append(wrist.X[0, 0])

  #pylab.plot(range(100), simulated_x)
  #pylab.show()

  wrist = Wrist()
  close_loop_x = []
  X_hat = numpy.matrix([[0.0], [0.0]])
  R = numpy.matrix([[1.0], [0.0]])
  for _ in xrange(100):
    U = numpy.clip(wrist.K * (R - X_hat), wrist.U_min, wrist.U_max)
    X_hat = wrist.A * X_hat + wrist.B * U + wrist.L * (wrist.Y - wrist.C * X_hat - wrist.D * U)
    wrist.Update(U)
    close_loop_x.append(wrist.X[0, 0])

  #pylab.plot(range(100), close_loop_x)
  #pylab.show()

  if len(argv) != 3:
    print "Expected .cc file name and .h file name"
  else:
    namespace_start = ("namespace frc971 {\n"
                       "namespace control_loops {\n\n");

    namespace_end = ("}  // namespace frc971\n"
                     "}  // namespace control_loops\n");

    header_start = ("#ifndef FRC971_CONTROL_LOOPS_WRIST_MOTOR_PLANT_H_\n"
                    "#define FRC971_CONTROL_LOOPS_WRIST_MOTOR_PLANT_H_\n\n")
    header_end = "#endif  // FRC971_CONTROL_LOOPS_WRIST_MOTOR_PLANT_H_\n";

    with open(argv[1], "w") as fd:
      fd.write("#include \"frc971/control_loops/wrist_motor_plant.h\"\n")
      fd.write('\n')
      fd.write("#include \"frc971/control_loops/state_feedback_loop.h\"\n")
      fd.write('\n')
      fd.write(namespace_start)
      fd.write('\n')
      fd.write(wrist.DumpPlant("Wrist"))
      fd.write('\n')
      fd.write(wrist.DumpLoop("Wrist"))
      fd.write('\n')
      fd.write(namespace_end)

    with open(argv[2], "w") as fd:
      fd.write(header_start)
      fd.write("#include \"frc971/control_loops/state_feedback_loop.h\"\n")
      fd.write('\n')
      fd.write(namespace_start)
      fd.write(wrist.DumpPlantHeader("Wrist"))
      fd.write('\n')
      fd.write(wrist.DumpLoopHeader("Wrist"))
      fd.write('\n')
      fd.write(namespace_end)
      fd.write(header_end)


if __name__ == '__main__':
  sys.exit(main(sys.argv))
