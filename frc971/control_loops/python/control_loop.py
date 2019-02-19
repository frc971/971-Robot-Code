import controls
import numpy
import os


class Constant(object):

    def __init__(self, name, formatt, value):
        self.name = name
        self.formatt = formatt
        self.value = value
        self.formatToType = {}
        self.formatToType['%f'] = "double"
        self.formatToType['%d'] = "int"

    def Render(self, loop_type):
        typestring = self.formatToType[self.formatt]
        if loop_type == 'float' and typestring == 'double':
            typestring = loop_type
        return str("\nstatic constexpr %s %s = "+ self.formatt +";\n") % \
            (typestring, self.name, self.value)


class ControlLoopWriter(object):

    def __init__(self,
                 gain_schedule_name,
                 loops,
                 namespaces=None,
                 write_constants=False,
                 plant_type='StateFeedbackPlant',
                 observer_type='StateFeedbackObserver',
                 scalar_type='double'):
        """Constructs a control loop writer.

        Args:
            gain_schedule_name: string, Name of the overall controller.
            loops: array[ControlLoop], a list of control loops to gain schedule
                in order.
            namespaces: array[string], a list of names of namespaces to nest in
                order.  If None, the default will be used.
            plant_type: string, The C++ type of the plant.
            observer_type: string, The C++ type of the observer.
            scalar_type: string, The C++ type of the base scalar.
        """
        self._gain_schedule_name = gain_schedule_name
        self._loops = loops
        if namespaces:
            self._namespaces = namespaces
        else:
            self._namespaces = ['frc971', 'control_loops']

        self._namespace_start = '\n'.join(
            ['namespace %s {' % name for name in self._namespaces])

        self._namespace_end = '\n'.join([
            '}  // namespace %s' % name for name in reversed(self._namespaces)
        ])

        self._constant_list = []
        self._plant_type = plant_type
        self._observer_type = observer_type
        self._scalar_type = scalar_type

    def AddConstant(self, constant):
        """Adds a constant to write.

        Args:
            constant: Constant, the constant to add to the header.
        """
        self._constant_list.append(constant)

    def _TopDirectory(self):
        return self._namespaces[0]

    def _HeaderGuard(self, header_file):
        return ('_'.join([namespace.upper() for namespace in self._namespaces])
                + '_' + os.path.basename(header_file).upper().replace(
                    '.', '_').replace('/', '_') + '_')

    def Write(self, header_file, cc_file):
        """Writes the loops to the specified files."""
        self.WriteHeader(header_file)
        self.WriteCC(os.path.basename(header_file), cc_file)

    def _GenericType(self, typename, extra_args=None):
        """Returns a loop template using typename for the type."""
        num_states = self._loops[0].A.shape[0]
        num_inputs = self._loops[0].B.shape[1]
        num_outputs = self._loops[0].C.shape[0]
        if extra_args is not None:
            extra_args = ', ' + extra_args
        else:
            extra_args = ''
        if self._scalar_type != 'double':
            extra_args += ', ' + self._scalar_type
        return '%s<%d, %d, %d%s>' % (typename, num_states, num_inputs,
                                     num_outputs, extra_args)

    def _ControllerType(self):
        """Returns a template name for StateFeedbackController."""
        return self._GenericType('StateFeedbackController')

    def _ObserverType(self):
        """Returns a template name for StateFeedbackObserver."""
        return self._GenericType(self._observer_type)

    def _LoopType(self):
        """Returns a template name for StateFeedbackLoop."""
        num_states = self._loops[0].A.shape[0]
        num_inputs = self._loops[0].B.shape[1]
        num_outputs = self._loops[0].C.shape[0]

        return 'StateFeedbackLoop<%d, %d, %d, %s, %s, %s>' % (
            num_states, num_inputs, num_outputs, self._scalar_type,
            self._PlantType(), self._ObserverType())

    def _PlantType(self):
        """Returns a template name for StateFeedbackPlant."""
        return self._GenericType(self._plant_type)

    def _PlantCoeffType(self):
        """Returns a template name for StateFeedbackPlantCoefficients."""
        return self._GenericType(self._plant_type + 'Coefficients')

    def _ControllerCoeffType(self):
        """Returns a template name for StateFeedbackControllerCoefficients."""
        return self._GenericType('StateFeedbackControllerCoefficients')

    def _ObserverCoeffType(self):
        """Returns a template name for StateFeedbackObserverCoefficients."""
        return self._GenericType(self._observer_type + 'Coefficients')

    def WriteHeader(self, header_file):
        """Writes the header file to the file named header_file."""
        with open(header_file, 'w') as fd:
            header_guard = self._HeaderGuard(header_file)
            fd.write('#ifndef %s\n'
                     '#define %s\n\n' % (header_guard, header_guard))
            fd.write(
                '#include \"frc971/control_loops/state_feedback_loop.h\"\n')
            if (self._plant_type == 'StateFeedbackHybridPlant' or
                    self._observer_type == 'HybridKalman'):
                fd.write(
                    '#include \"frc971/control_loops/hybrid_state_feedback_loop.h\"\n'
                )

            fd.write('\n')

            fd.write(self._namespace_start)

            for const in self._constant_list:
                fd.write(const.Render(self._scalar_type))

            fd.write('\n\n')
            for loop in self._loops:
                fd.write(loop.DumpPlantHeader(self._PlantCoeffType()))
                fd.write('\n')
                fd.write(loop.DumpControllerHeader(self._scalar_type))
                fd.write('\n')
                fd.write(loop.DumpObserverHeader(self._ObserverCoeffType()))
                fd.write('\n')

            fd.write('%s Make%sPlant();\n\n' % (self._PlantType(),
                                                self._gain_schedule_name))

            fd.write('%s Make%sController();\n\n' % (self._ControllerType(),
                                                     self._gain_schedule_name))

            fd.write('%s Make%sObserver();\n\n' % (self._ObserverType(),
                                                   self._gain_schedule_name))

            fd.write('%s Make%sLoop();\n\n' % (self._LoopType(),
                                               self._gain_schedule_name))

            fd.write(self._namespace_end)
            fd.write('\n\n')
            fd.write("#endif  // %s\n" % header_guard)

    def WriteCC(self, header_file_name, cc_file):
        """Writes the cc file to the file named cc_file."""
        with open(cc_file, 'w') as fd:
            fd.write('#include \"%s/%s\"\n' % (os.path.join(*self._namespaces),
                                               header_file_name))
            fd.write('\n')
            fd.write('#include <vector>\n')
            fd.write('\n')
            fd.write(
                '#include \"frc971/control_loops/state_feedback_loop.h\"\n')
            fd.write('\n')
            fd.write(self._namespace_start)
            fd.write('\n\n')
            for loop in self._loops:
                fd.write(
                    loop.DumpPlant(self._PlantCoeffType(), self._scalar_type))
                fd.write('\n')

            for loop in self._loops:
                fd.write(loop.DumpController(self._scalar_type))
                fd.write('\n')

            for loop in self._loops:
                fd.write(
                    loop.DumpObserver(self._ObserverCoeffType(),
                                      self._scalar_type))
                fd.write('\n')

            fd.write('%s Make%sPlant() {\n' % (self._PlantType(),
                                               self._gain_schedule_name))
            fd.write('  ::std::vector< ::std::unique_ptr<%s>> plants(%d);\n' %
                     (self._PlantCoeffType(), len(self._loops)))
            for index, loop in enumerate(self._loops):
                fd.write('  plants[%d] = ::std::unique_ptr<%s>(new %s(%s));\n' %
                         (index, self._PlantCoeffType(), self._PlantCoeffType(),
                          loop.PlantFunction()))
            fd.write('  return %s(&plants);\n' % self._PlantType())
            fd.write('}\n\n')

            fd.write('%s Make%sController() {\n' % (self._ControllerType(),
                                                    self._gain_schedule_name))
            fd.write(
                '  ::std::vector< ::std::unique_ptr<%s>> controllers(%d);\n' %
                (self._ControllerCoeffType(), len(self._loops)))
            for index, loop in enumerate(self._loops):
                fd.write(
                    '  controllers[%d] = ::std::unique_ptr<%s>(new %s(%s));\n' %
                    (index, self._ControllerCoeffType(),
                     self._ControllerCoeffType(), loop.ControllerFunction()))
            fd.write('  return %s(&controllers);\n' % self._ControllerType())
            fd.write('}\n\n')

            fd.write('%s Make%sObserver() {\n' % (self._ObserverType(),
                                                  self._gain_schedule_name))
            fd.write('  ::std::vector< ::std::unique_ptr<%s>> observers(%d);\n'
                     % (self._ObserverCoeffType(), len(self._loops)))
            for index, loop in enumerate(self._loops):
                fd.write(
                    '  observers[%d] = ::std::unique_ptr<%s>(new %s(%s));\n'
                    % (index, self._ObserverCoeffType(),
                       self._ObserverCoeffType(), loop.ObserverFunction()))
            fd.write('  return %s(&observers);\n' % self._ObserverType())
            fd.write('}\n\n')

            fd.write('%s Make%sLoop() {\n' % (self._LoopType(),
                                              self._gain_schedule_name))
            fd.write(
                '  return %s(Make%sPlant(), Make%sController(), Make%sObserver());\n'
                % (self._LoopType(), self._gain_schedule_name,
                   self._gain_schedule_name, self._gain_schedule_name))
            fd.write('}\n\n')

            fd.write(self._namespace_end)
            fd.write('\n')


class ControlLoop(object):

    def __init__(self, name):
        """Constructs a control loop object.

        Args:
            name: string, The name of the loop to use when writing the C++ files.
        """
        self._name = name

    @property
    def name(self):
        """Returns the name"""
        return self._name

    def ContinuousToDiscrete(self, A_continuous, B_continuous, dt):
        """Calculates the discrete time values for A and B.

        Args:
            A_continuous: numpy.matrix, The continuous time A matrix
            B_continuous: numpy.matrix, The continuous time B matrix
            dt: float, The time step of the control loop

        Returns:
            (A, B), numpy.matrix, the control matricies.
        """
        return controls.c2d(A_continuous, B_continuous, dt)

    def InitializeState(self):
        """Sets X, Y, and X_hat to zero defaults."""
        self.X = numpy.zeros((self.A.shape[0], 1))
        self.Y = self.C * self.X
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
        #U = numpy.clip(U, self.U_min, self.U_max)
        self.X = self.A * self.X + self.B * U
        self.Y = self.C * self.X + self.D * U

    def PredictObserver(self, U):
        """Runs the predict step of the observer update."""
        self.X_hat = (self.A * self.X_hat + self.B * U)

    def CorrectObserver(self, U):
        """Runs the correct step of the observer update."""
        if hasattr(self, 'KalmanGain'):
            KalmanGain = self.KalmanGain
        else:
            KalmanGain = numpy.linalg.inv(self.A) * self.L
        self.X_hat += KalmanGain * (self.Y - self.C * self.X_hat - self.D * U)

    def UpdateObserver(self, U):
        """Updates the observer given the provided U."""
        if hasattr(self, 'KalmanGain'):
            KalmanGain = self.KalmanGain
        else:
            KalmanGain = numpy.linalg.inv(self.A) * self.L
        self.X_hat = (self.A * self.X_hat + self.B * U + self.A * KalmanGain *
                      (self.Y - self.C * self.X_hat - self.D * U))

    def _DumpMatrix(self, matrix_name, matrix, scalar_type):
        """Dumps the provided matrix into a variable called matrix_name.

        Args:
            matrix_name: string, The variable name to save the matrix to.
            matrix: The matrix to dump.
            scalar_type: The C++ type to use for the scalar in the matrix.

        Returns:
            string, The C++ commands required to populate a variable named matrix_name
                with the contents of matrix.
        """
        ans = [
            '  Eigen::Matrix<%s, %d, %d> %s;\n' % (scalar_type, matrix.shape[0],
                                                   matrix.shape[1], matrix_name)
        ]
        for x in xrange(matrix.shape[0]):
            for y in xrange(matrix.shape[1]):
                write_type = repr(matrix[x, y])
                if scalar_type == 'float':
                    if '.' not in write_type:
                        write_type += '.0'
                    write_type += 'f'
                ans.append(
                    '  %s(%d, %d) = %s;\n' % (matrix_name, x, y, write_type))

        return ''.join(ans)

    def DumpPlantHeader(self, plant_coefficient_type):
        """Writes out a c++ header declaration which will create a Plant object.

        Returns:
            string, The header declaration for the function.
        """
        return '%s Make%sPlantCoefficients();\n' % (plant_coefficient_type,
                                                    self._name)

    def DumpPlant(self, plant_coefficient_type, scalar_type):
        """Writes out a c++ function which will create a PlantCoefficients object.

        Returns:
            string, The function which will create the object.
        """
        ans = [
            '%s Make%sPlantCoefficients() {\n' % (plant_coefficient_type,
                                                  self._name)
        ]

        ans.append(self._DumpMatrix('C', self.C, scalar_type))
        ans.append(self._DumpMatrix('D', self.D, scalar_type))
        ans.append(self._DumpMatrix('U_max', self.U_max, scalar_type))
        ans.append(self._DumpMatrix('U_min', self.U_min, scalar_type))

        if plant_coefficient_type.startswith('StateFeedbackPlant'):
            ans.append(self._DumpMatrix('A', self.A, scalar_type))
            ans.append(self._DumpMatrix('B', self.B, scalar_type))
            ans.append(
                '  return %s'
                '(A, B, C, D, U_max, U_min);\n' % (plant_coefficient_type))
        elif plant_coefficient_type.startswith('StateFeedbackHybridPlant'):
            ans.append(
                self._DumpMatrix('A_continuous', self.A_continuous,
                                 scalar_type))
            ans.append(
                self._DumpMatrix('B_continuous', self.B_continuous,
                                 scalar_type))
            ans.append('  return %s'
                       '(A_continuous, B_continuous, C, D, U_max, U_min);\n' %
                       (plant_coefficient_type))
        else:
            glog.fatal('Unsupported plant type %s', plant_coefficient_type)

        ans.append('}\n')
        return ''.join(ans)

    def PlantFunction(self):
        """Returns the name of the plant coefficient function."""
        return 'Make%sPlantCoefficients()' % self._name

    def ControllerFunction(self):
        """Returns the name of the controller function."""
        return 'Make%sControllerCoefficients()' % self._name

    def ObserverFunction(self):
        """Returns the name of the controller function."""
        return 'Make%sObserverCoefficients()' % self._name

    def DumpControllerHeader(self, scalar_type):
        """Writes out a c++ header declaration which will create a Controller object.

        Returns:
            string, The header declaration for the function.
        """
        num_states = self.A.shape[0]
        num_inputs = self.B.shape[1]
        num_outputs = self.C.shape[0]
        return 'StateFeedbackControllerCoefficients<%d, %d, %d, %s> %s;\n' % (
            num_states, num_inputs, num_outputs, scalar_type,
            self.ControllerFunction())

    def DumpController(self, scalar_type):
        """Returns a c++ function which will create a Controller object.

        Returns:
            string, The function which will create the object.
        """
        num_states = self.A.shape[0]
        num_inputs = self.B.shape[1]
        num_outputs = self.C.shape[0]
        ans = [
            'StateFeedbackControllerCoefficients<%d, %d, %d, %s> %s {\n' %
            (num_states, num_inputs, num_outputs, scalar_type,
             self.ControllerFunction())
        ]

        ans.append(self._DumpMatrix('K', self.K, scalar_type))
        if not hasattr(self, 'Kff'):
            self.Kff = numpy.matrix(numpy.zeros(self.K.shape))

        ans.append(self._DumpMatrix('Kff', self.Kff, scalar_type))

        ans.append(
            '  return StateFeedbackControllerCoefficients<%d, %d, %d, %s>'
            '(K, Kff);\n' % (num_states, num_inputs, num_outputs, scalar_type))
        ans.append('}\n')
        return ''.join(ans)

    def DumpObserverHeader(self, observer_coefficient_type):
        """Writes out a c++ header declaration which will create a Observer object.

        Returns:
            string, The header declaration for the function.
        """
        return '%s %s;\n' % (observer_coefficient_type, self.ObserverFunction())

    def DumpObserver(self, observer_coefficient_type, scalar_type):
        """Returns a c++ function which will create a Observer object.

        Returns:
            string, The function which will create the object.
        """
        ans = [
            '%s %s {\n' % (observer_coefficient_type, self.ObserverFunction())
        ]

        if observer_coefficient_type.startswith('StateFeedbackObserver'):
            if hasattr(self, 'KalmanGain'):
                KalmanGain = self.KalmanGain
                Q = self.Q
                R = self.R
            else:
                KalmanGain = numpy.linalg.inv(self.A) * self.L
                Q = numpy.zeros(self.A.shape)
                R = numpy.zeros((self.C.shape[0], self.C.shape[0]))
            ans.append(self._DumpMatrix('KalmanGain', KalmanGain, scalar_type))
            ans.append(self._DumpMatrix('Q', Q, scalar_type))
            ans.append(self._DumpMatrix('R', R, scalar_type))
            ans.append('  return %s(KalmanGain, Q, R);\n' %
                       (observer_coefficient_type,))

        elif observer_coefficient_type.startswith('HybridKalman'):
            ans.append(
                self._DumpMatrix('Q_continuous', self.Q_continuous,
                                 scalar_type))
            ans.append(
                self._DumpMatrix('R_continuous', self.R_continuous,
                                 scalar_type))
            ans.append(
                self._DumpMatrix('P_steady_state', self.P_steady_state,
                                 scalar_type))
            ans.append(
                '  return %s(Q_continuous, R_continuous, P_steady_state);\n' %
                (observer_coefficient_type,))
        else:
            glog.fatal('Unsupported observer type %s',
                       observer_coefficient_type)

        ans.append('}\n')
        return ''.join(ans)


class HybridControlLoop(ControlLoop):

    def __init__(self, name):
        super(HybridControlLoop, self).__init__(name=name)

    def Discretize(self, dt):
        [self.A, self.B, self.Q, self.R] = \
            controls.kalmd(self.A_continuous, self.B_continuous,
                           self.Q_continuous, self.R_continuous, dt)

    def PredictHybridObserver(self, U, dt):
        self.Discretize(dt)
        self.X_hat = self.A * self.X_hat + self.B * U
        self.P = (self.A * self.P * self.A.T + self.Q)

    def CorrectHybridObserver(self, U):
        Y_bar = self.Y - self.C * self.X_hat
        C_t = self.C.T
        S = self.C * self.P * C_t + self.R
        self.KalmanGain = self.P * C_t * numpy.linalg.inv(S)
        self.X_hat = self.X_hat + self.KalmanGain * Y_bar
        self.P = (numpy.eye(len(self.A)) - self.KalmanGain * self.C) * self.P

    def InitializeState(self):
        super(HybridControlLoop, self).InitializeState()
        if hasattr(self, 'Q_steady_state'):
            self.P = self.Q_steady_state
        else:
            self.P = numpy.matrix(
                numpy.zeros((self.A.shape[0], self.A.shape[0])))


class CIM(object):

    def __init__(self):
        # Stall Torque in N m
        self.stall_torque = 2.42
        # Stall Current in Amps
        self.stall_current = 133.0
        # Free Speed in rad/s
        self.free_speed = 5500.0 / 60.0 * 2.0 * numpy.pi
        # Free Current in Amps
        self.free_current = 4.7
        # Resistance of the motor
        self.resistance = 12.0 / self.stall_current
        # Motor velocity constant
        self.Kv = (
            self.free_speed / (12.0 - self.resistance * self.free_current))
        # Torque constant
        self.Kt = self.stall_torque / self.stall_current


class MiniCIM(object):

    def __init__(self):
        # Stall Torque in N m
        self.stall_torque = 1.41
        # Stall Current in Amps
        self.stall_current = 89.0
        # Free Speed in rad/s
        self.free_speed = 5840.0 / 60.0 * 2.0 * numpy.pi
        # Free Current in Amps
        self.free_current = 3.0
        # Resistance of the motor
        self.resistance = 12.0 / self.stall_current
        # Motor velocity constant
        self.Kv = (
            self.free_speed / (12.0 - self.resistance * self.free_current))
        # Torque constant
        self.Kt = self.stall_torque / self.stall_current


class NMotor(object):

    def __init__(self, motor, n):
        """Gangs together n motors."""
        self.motor = motor
        self.stall_torque = motor.stall_torque * n
        self.stall_current = motor.stall_current * n
        self.free_speed = motor.free_speed

        self.free_current = motor.free_current * n
        self.resistance = motor.resistance / n
        self.Kv = motor.Kv
        self.Kt = motor.Kt
        self.motor_inertia = motor.motor_inertia * n


class Vex775Pro(object):

    def __init__(self):
        # Stall Torque in N m
        self.stall_torque = 0.71
        # Stall Current in Amps
        self.stall_current = 134.0
        # Free Speed in rad/s
        self.free_speed = 18730.0 / 60.0 * 2.0 * numpy.pi
        # Free Current in Amps
        self.free_current = 0.7
        # Resistance of the motor
        self.resistance = 12.0 / self.stall_current
        # Motor velocity constant
        self.Kv = (
            self.free_speed / (12.0 - self.resistance * self.free_current))
        # Torque constant
        self.Kt = self.stall_torque / self.stall_current
        # Motor inertia in kg m^2
        self.motor_inertia = 0.00001187


class BAG(object):
    # BAG motor specs available at http://motors.vex.com/vexpro-motors/bag-motor
    def __init__(self):
        # Stall Torque in (N m)
        self.stall_torque = 0.43
        # Stall Current in (Amps)
        self.stall_current = 53.0
        # Free Speed in (rad/s)
        self.free_speed = 13180.0 / 60.0 * 2.0 * numpy.pi
        # Free Current in (Amps)
        self.free_current = 1.8
        # Resistance of the motor (Ohms)
        self.resistance = 12.0 / self.stall_current
        # Motor velocity constant (radians / (sec * volt))
        self.Kv = (
            self.free_speed / (12.0 - self.resistance * self.free_current))
        # Torque constant (N * m / A)
        self.Kt = self.stall_torque / self.stall_current
        # Motor inertia in kg m^2
        self.motor_inertia = 0.000006


class MN3510(object):

    def __init__(self):
        # http://www.robotshop.com/en/t-motor-navigator-mn3510-360kv-brushless-motor.html#Specifications
        # Free Current in Amps
        self.free_current = 0.0
        # Resistance of the motor
        self.resistance = 0.188
        # Stall Current in Amps
        self.stall_current = 14.0 / self.resistance
        # Motor velocity constant
        self.Kv = 360.0 / 60.0 * (2.0 * numpy.pi)
        # Torque constant Nm / A
        self.Kt = 1.0 / self.Kv
        # Stall Torque in N m
        self.stall_torque = self.Kt * self.stall_current
