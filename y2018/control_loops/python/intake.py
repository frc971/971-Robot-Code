#!/usr/bin/python

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
import sys
from matplotlib import pylab
import gflags
import glog

FLAGS = gflags.FLAGS

try:
    gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
    pass


class Intake(control_loop.ControlLoop):

    def __init__(self, name="Intake"):
        super(Intake, self).__init__(name)
        self.motor = control_loop.BAG()
        # Stall Torque in N m
        self.stall_torque = self.motor.stall_torque
        # Stall Current in Amps
        self.stall_current = self.motor.stall_current
        # Free Speed in RPM
        self.free_speed = self.motor.free_speed
        # Free Current in Amps
        self.free_current = self.motor.free_current

        # Resistance of the motor
        self.resistance = self.motor.resistance
        # Motor velocity constant
        self.Kv = self.motor.Kv
        # Torque constant
        self.Kt = self.motor.Kt
        # Gear ratio
        self.G = 1.0 / 102.6

        self.motor_inertia = 0.00000589 * 1.2

        # Series elastic moment of inertia
        self.Je = self.motor_inertia / (self.G * self.G)
        # Grabber moment of inertia
        self.Jo = 0.0363

        # Bot has a time constant of 0.22
        # Current physics has a time constant of 0.18

        # Spring constant (N m / radian)
        self.Ks = 32.74

        # Damper constant (N m s/ radian)
        # 0.01 is small and 1 is big
        self.b = 0.1

        # Control loop time step
        self.dt = 0.00505

        # State is [output_position, output_velocity,
        #           elastic_position, elastic_velocity]
        # The output position is the absolute position of the intake arm.
        # The elastic position is the absolute position of the motor side of the
        # series elastic.
        # Input is [voltage]

        self.A_continuous = numpy.matrix(
            [[0.0, 1.0, 0.0, 0.0],
             [(-self.Ks / self.Jo), (-self.b / self.Jo), (self.Ks / self.Jo),
              (self.b / self.Jo)], [0.0, 0.0, 0.0, 1.0],
             [(self.Ks / self.Je), (self.b / self.Je), (-self.Ks / self.Je),
              (-self.b / self.Je) - self.Kt /
              (self.Je * self.resistance * self.Kv * self.G * self.G)]])

        # Start with the unmodified input
        self.B_continuous = numpy.matrix(
            [[0.0], [0.0], [0.0],
             [self.Kt / (self.G * self.Je * self.resistance)]])

        self.C = numpy.matrix([[1.0, 0.0, -1.0, 0.0], [0.0, 0.0, 1.0, 0.0]])
        self.D = numpy.matrix([[0.0], [0.0]])

        self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                                   self.B_continuous, self.dt)

        #controllability = controls.ctrb(self.A, self.B)
        #glog.debug('ctrb: ' + repr(numpy.linalg.matrix_rank(controllability)))

        #observability = controls.ctrb(self.A.T, self.C.T)
        #glog.debug('obs: ' + repr(numpy.linalg.matrix_rank(observability)))

        glog.debug('A_continuous ' + repr(self.A_continuous))
        glog.debug('B_continuous ' + repr(self.B_continuous))

        self.K = numpy.matrix(numpy.zeros((1, 4)))

        q_pos = 0.05
        q_vel = 2.65
        self.Q = numpy.matrix(
            numpy.diag([(q_pos**2.0), (q_vel**2.0), (q_pos**2.0),
                        (q_vel**2.0)]))

        r_nm = 0.025
        self.R = numpy.matrix(numpy.diag([(r_nm**2.0), (r_nm**2.0)]))

        self.KalmanGain, self.Q_steady = controls.kalman(
            A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)

        # The box formed by U_min and U_max must encompass all possible values,
        # or else Austin's code gets angry.
        self.U_max = numpy.matrix([[12.0]])
        self.U_min = numpy.matrix([[-12.0]])

        self.InitializeState()


class DelayedIntake(Intake):

    def __init__(self, name="DelayedIntake"):
        super(DelayedIntake, self).__init__(name=name)

        self.A_undelayed = self.A
        self.B_undelayed = self.B

        self.C_unaugmented = self.C
        self.C = numpy.matrix(numpy.zeros((2, 5)))
        self.C[0:2, 0:4] = self.C_unaugmented

        # Model this as X[4] is the last power.  And then B applies to the last
        # power.  This lets us model the 1 cycle PWM delay accurately.
        self.A = numpy.matrix(numpy.zeros((5, 5)))
        self.A[0:4, 0:4] = self.A_undelayed
        self.A[0:4, 4] = self.B_undelayed
        self.B = numpy.matrix(numpy.zeros((5, 1)))
        self.B[4, 0] = 1.0

        # Coordinate transform fom absolute angles to relative angles.
        # [output_position, output_velocity, spring_angle, spring_velocity, voltage]
        abs_to_rel = numpy.matrix([[1.0, 0.0, 0.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0, 0.0, 0.0],
                                   [1.0, 0.0, -1.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0, -1.0, 0.0],
                                   [0.0, 0.0, 0.0, 0.0, 1.0]])
        # and back again.
        rel_to_abs = numpy.matrix(numpy.linalg.inv(abs_to_rel))

        # Now, get A and B in the relative coordinate system.
        self.A_transformed_full = numpy.matrix(numpy.zeros((5, 5)))
        self.B_transformed_full = numpy.matrix(numpy.zeros((5, 1)))
        (self.A_transformed_full[0:4, 0:4],
         self.A_transformed_full[0:4, 4]) = self.ContinuousToDiscrete(
             abs_to_rel[0:4, 0:4] * self.A_continuous * rel_to_abs[0:4, 0:4],
             abs_to_rel[0:4, 0:4] * self.B_continuous, self.dt)
        self.B_transformed_full[4, 0] = 1.0

        # Pull out the components of the dynamics which don't include the spring
        # output position so we can do partial state feedback on what we care about.
        self.A_transformed = self.A_transformed_full[1:5, 1:5]
        self.B_transformed = self.B_transformed_full[1:5, 0]

        glog.debug('A_transformed_full ' + str(self.A_transformed_full))
        glog.debug('B_transformed_full ' + str(self.B_transformed_full))
        glog.debug('A_transformed ' + str(self.A_transformed))
        glog.debug('B_transformed ' + str(self.B_transformed))

        # Now, let's design a controller in
        #   [output_velocity, spring_position, spring_velocity, delayed_voltage]
        # space.

        q_output_vel = 1.0
        q_spring_pos = 0.10
        q_spring_vel = 2.0
        q_voltage = 1000000000000.0
        self.Q_lqr = numpy.matrix(
            numpy.diag([
                1.0 / (q_output_vel**2.0), 1.0 / (q_spring_pos**2.0),
                1.0 / (q_spring_vel**2.0), 1.0 / (q_voltage**2.0)
            ]))

        self.R = numpy.matrix([[(1.0 / (12.0**2.0))]])

        self.K_transformed = controls.dlqr(
            self.A_transformed, self.B_transformed, self.Q_lqr, self.R)

        # Write the controller back out in the absolute coordinate system.
        self.K = numpy.hstack(
            (numpy.matrix([[0.0]]), self.K_transformed)) * abs_to_rel

        controllability = controls.ctrb(self.A_transformed, self.B_transformed)
        glog.debug('ctrb: ' + repr(numpy.linalg.matrix_rank(controllability)))

        w, v = numpy.linalg.eig(self.A_transformed -
                                self.B_transformed * self.K_transformed)
        glog.debug('Poles are %s, for %s', repr(w), self._name)

        for i in range(len(w)):
            glog.debug('  Pole %s -> %s', repr(w[i]), v[:, i])

        glog.debug('K is %s', repr(self.K_transformed))

        # Design a kalman filter here as well.
        q_pos = 0.05
        q_vel = 2.65
        q_volts = 0.005
        self.Q = numpy.matrix(
            numpy.diag([(q_pos**2.0), (q_vel**2.0), (q_pos**2.0), (q_vel**2.0),
                        (q_volts**2.0)]))

        r_nm = 0.025
        self.R = numpy.matrix(numpy.diag([(r_nm**2.0), (r_nm**2.0)]))

        glog.debug('Overall poles are %s, for %s',
                   repr(numpy.linalg.eig(self.A - self.B * self.K)[0]),
                   self._name)

        self.KalmanGain, self.Q_steady = controls.kalman(
            A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)

        self.InitializeState()


class ScenarioPlotter(object):

    def __init__(self):
        # Various lists for graphing things.
        self.t = []
        self.x_motor = []
        self.x_output = []
        self.v = []
        self.goal_v = []
        self.a = []
        self.spring = []
        self.x_hat = []
        self.u = []

    def run_test(self,
                 intake,
                 iterations=400,
                 controller_intake=None,
                 observer_intake=None):
        """Runs the intake plant with an initial condition and goal.

      Test for whether the goal has been reached and whether the separation
      goes outside of the initial and goal values by more than
      max_separation_error.

      Prints out something for a failure of either condition and returns
      False if tests fail.
      Args:
        intake: intake object to use.
        iterations: Number of timesteps to run the model for.
        controller_intake: Intake object to get K from, or None if we should
            use intake.
        observer_intake: Intake object to use for the observer, or None if we
            should use the actual state.
    """

        if controller_intake is None:
            controller_intake = intake

        vbat = 12.0

        if self.t:
            initial_t = self.t[-1] + intake.dt
        else:
            initial_t = 0

        # Delay U by 1 cycle in our simulation to make it more realistic.
        last_U = numpy.matrix([[0.0]])
        intake.Y = intake.C * intake.X

        # Start with the intake deflected by 0.2 radians
        # intake.X[0,0] = 0.2
        # intake.Y[0,0] = intake.X[0,0]
        # observer_intake.X_hat[0,0] = intake.X[0,0]

        for i in xrange(iterations):
            X_hat = intake.X

            if observer_intake is not None:
                X_hat = observer_intake.X_hat
                self.x_hat.append(observer_intake.X_hat[0, 0])

            goal_angle = 3.0
            goal_velocity = numpy.clip((goal_angle - X_hat[0, 0]) * 6.0, -1.0,
                                       1.0)

            self.goal_v.append(goal_velocity)

            # Nominal: 1.8 N at 0.25 m -> 0.45 N m
            # Nominal: 13 N at 0.25 m at 0.5 radians -> 3.25 N m -> 6 N m / radian

            R = numpy.matrix([[0.0], [goal_velocity], [0.0], [goal_velocity],
                              [goal_velocity / (intake.G * intake.Kv)]])
            U = controller_intake.K * (R - X_hat) + R[4, 0]

            U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)  # * 0.0

            self.x_output.append(intake.X[0, 0])
            self.x_motor.append(intake.X[2, 0])
            self.spring.append(intake.X[0, 0] - intake.X[2, 0])

            if self.v:
                last_v = self.v[-1]
            else:
                last_v = 0

            self.v.append(intake.X[1, 0])
            self.a.append((self.v[-1] - last_v) / intake.dt)

            if observer_intake is not None:
                observer_intake.Y = intake.Y
                observer_intake.CorrectObserver(U)

            intake.Update(last_U + 0.0)

            if observer_intake is not None:
                observer_intake.PredictObserver(U)

            self.t.append(initial_t + i * intake.dt)
            self.u.append(U[0, 0])
            last_U = U

    def Plot(self):
        pylab.subplot(3, 1, 1)
        pylab.plot(self.t, self.x_output, label='x output')
        pylab.plot(self.t, self.x_motor, label='x motor')
        pylab.plot(self.t, self.x_hat, label='x_hat')
        pylab.legend()

        spring_ax1 = pylab.subplot(3, 1, 2)
        spring_ax1.plot(self.t, self.u, 'k', label='u')
        spring_ax2 = spring_ax1.twinx()
        spring_ax2.plot(self.t, self.spring, label='spring_angle')
        spring_ax1.legend(loc=2)
        spring_ax2.legend()

        accel_ax1 = pylab.subplot(3, 1, 3)
        accel_ax1.plot(self.t, self.a, 'r', label='a')

        accel_ax2 = accel_ax1.twinx()
        accel_ax2.plot(self.t, self.v, label='v')
        accel_ax2.plot(self.t, self.goal_v, label='goal_v')
        accel_ax1.legend(loc=2)
        accel_ax2.legend()

        pylab.show()


def main(argv):
    scenario_plotter = ScenarioPlotter()

    intake = Intake()
    intake.X[0, 0] = 0.0
    intake_controller = DelayedIntake()
    observer_intake = DelayedIntake()
    observer_intake.X_hat[0, 0] = intake.X[0, 0]

    # Test moving the intake with constant separation.
    scenario_plotter.run_test(
        intake,
        controller_intake=intake_controller,
        observer_intake=observer_intake,
        iterations=200)

    if FLAGS.plot:
        scenario_plotter.Plot()

    # Write the generated constants out to a file.
    if len(argv) != 5:
        glog.fatal(
            'Expected .h file name and .cc file name for intake and delayed_intake.'
        )
    else:
        namespaces = ['y2018', 'control_loops', 'superstructure', 'intake']
        intake = Intake('Intake')
        loop_writer = control_loop.ControlLoopWriter(
            'Intake', [intake], namespaces=namespaces)
        loop_writer.AddConstant(
            control_loop.Constant('kGearRatio', '%f', intake.G))
        loop_writer.AddConstant(
            control_loop.Constant('kMotorVelocityConstant', '%f', intake.Kv))
        loop_writer.AddConstant(
            control_loop.Constant('kFreeSpeed', '%f', intake.free_speed))
        loop_writer.Write(argv[1], argv[2])

        delayed_intake = DelayedIntake('DelayedIntake')
        loop_writer = control_loop.ControlLoopWriter(
            'DelayedIntake', [delayed_intake], namespaces=namespaces)
        loop_writer.Write(argv[3], argv[4])


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
