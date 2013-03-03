import controls
import numpy

class ControlLoop(object):
  def __init__(self, name):
    """Constructs a control loop object.

    Args:
      name: string, The name of the loop to use when writing the C++ files.
    """
    self._name = name

    self._namespace_start = ("namespace frc971 {\n"
                             "namespace control_loops {\n\n")

    self._namespace_end = ("}  // namespace frc971\n"
                           "}  // namespace control_loops\n")

    self._header_start = ("#ifndef FRC971_CONTROL_LOOPS_%s_%s_MOTOR_PLANT_H_\n"
                          "#define FRC971_CONTROL_LOOPS_%s_%s_MOTOR_PLANT_H_\n\n"
                          % (self._name.upper(), self._name.upper(),
                             self._name.upper(), self._name.upper()))

    self._header_end = ("#endif  // FRC971_CONTROL_LOOPS_%s_%s_MOTOR_PLANT_H_\n"
                        % (self._name.upper(), self._name.upper()))

  def ContinuousToDiscrete(self, A_continuous, B_continuous, dt, C):
    """Calculates the discrete time values for A and B as well as initializing
      X and Y to the correct sizes.

      Args:
        A_continuous: numpy.matrix, The continuous time A matrix
        B_continuous: numpy.matrix, The continuous time B matrix
        dt: float, The time step of the control loop
        C: C
    """
    self.A, self.B = controls.c2d(
        A_continuous, B_continuous, dt)
    self.X = numpy.zeros((self.A.shape[0], 1))
    self.Y = C * self.X
    self.X_hat = numpy.zeros((self.A.shape[0], 1))

  def PlaceControllerPoles(self, poles):
    """Places the controller poles.

    Args:
      poles: array, An array of poles.  Must be complex conjegates if they have
        any imaginary portions.
    """
    self.K = controls.dplace(self.A, self.B, poles)

  def PlaceObserverPoles(self, poles):
    """Places the observer poles.

    Args:
      poles: array, An array of poles.  Must be complex conjegates if they have
        any imaginary portions.
    """
    self.L = controls.dplace(self.A.T, self.C.T, poles).T

  def Update(self, U):
    """Simulates one time step with the provided U."""
    U = numpy.clip(U, self.U_min, self.U_max)
    self.X = self.A * self.X + self.B * U
    self.Y = self.C * self.X + self.D * U

  def UpdateObserver(self, U):
    """Updates the observer given the provided U."""
    self.X_hat = (self.A * self.X_hat + self.B * U +
                  self.L * (self.Y - self.C * self.X_hat - self.D * U))

  def _DumpMatrix(self, matrix_name, matrix):
    """Dumps the provided matrix into a variable called matrix_name.

    Args:
      matrix_name: string, The variable name to save the matrix to.
      matrix: The matrix to dump.

    Returns:
      string, The C++ commands required to populate a variable named matrix_name
        with the contents of matrix.
    """
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

  def _DumpPlantHeader(self, plant_name):
    """Writes out a c++ header declaration which will create a Plant object.

    Args:
      plant_name: string, the name of the plant.  Used to create the name of the
        function.  The function name will be Make<plant_name>Plant().

    Returns:
      string, The header declaration for the function.
    """
    num_states = self.A.shape[0]
    num_inputs = self.B.shape[1]
    num_outputs = self.C.shape[0]
    return "StateFeedbackPlant<%d, %d, %d> Make%sPlant();\n" % (
        num_states, num_inputs, num_outputs, plant_name)

  def _DumpPlant(self, plant_name):
    """Writes out a c++ function which will create a Plant object.

    Args:
      plant_name: string, the name of the plant.  Used to create the name of the
        function.  The function name will be Make<plant_name>Plant().

    Returns:
      string, The function which will create the object.
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

  def _DumpLoopHeader(self, loop_name):
    """Writes out a c++ header declaration which will create a Loop object.

    Args:
      loop_name: string, the name of the loop.  Used to create the name of the
        function.  The function name will be Make<loop_name>Loop().

    Returns:
      string, The header declaration for the function.
    """
    num_states = self.A.shape[0]
    num_inputs = self.B.shape[1]
    num_outputs = self.C.shape[0]
    return "StateFeedbackLoop<%d, %d, %d> Make%sLoop();\n" % (
        num_states, num_inputs, num_outputs, loop_name)

  def _DumpLoop(self, loop_name):
    """Returns a c++ function which will create a Loop object.

    Args:
      loop_name: string, the name of the loop.  Used to create the name of the
        function and create the plant.  The function name will be
        Make<loop_name>Loop().

    Returns:
      string, The function which will create the object.
    """
    num_states = self.A.shape[0]
    num_inputs = self.B.shape[1]
    num_outputs = self.C.shape[0]
    ans = ["StateFeedbackLoop<%d, %d, %d> Make%sLoop() {\n" % (
        num_states, num_inputs, num_outputs, loop_name)]

    ans.append(self._DumpMatrix("L", self.L))
    ans.append(self._DumpMatrix("K", self.K))

    ans.append("  return StateFeedbackLoop<%d, %d, %d>"
               "(L, K, Make%sPlant());\n" % (num_states, num_inputs,
                                             num_outputs, loop_name))
    ans.append("}\n")
    return "".join(ans)

  def DumpHeaderFile(self, file_name):
    """Writes the header file for creating a Plant and Loop object.

    Args:
      file_name: string, name of the file to write the header file to.
    """
    with open(file_name, "w") as fd:
      fd.write(self._header_start)
      fd.write("#include \"frc971/control_loops/state_feedback_loop.h\"\n")
      fd.write('\n')
      fd.write(self._namespace_start)
      fd.write(self._DumpPlantHeader(self._name))
      fd.write('\n')
      fd.write(self._DumpLoopHeader(self._name))
      fd.write('\n')
      fd.write(self._namespace_end)
      fd.write('\n')
      fd.write(self._header_end)

  def DumpCppFile(self, file_name, header_file_name):
    """Writes the C++ file for creating a Plant and Loop object.

    Args:
      file_name: string, name of the file to write the header file to.
    """
    with open(file_name, "w") as fd:
      fd.write("#include \"frc971/control_loops/%s\"\n" % header_file_name)
      fd.write('\n')
      fd.write("#include \"frc971/control_loops/state_feedback_loop.h\"\n")
      fd.write('\n')
      fd.write(self._namespace_start)
      fd.write('\n')
      fd.write(self._DumpPlant(self._name))
      fd.write('\n')
      fd.write(self._DumpLoop(self._name))
      fd.write('\n')
      fd.write(self._namespace_end)
