#!/usr/bin/python

import numpy
import sys
import operator

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls

from y2016.control_loops.python.shoulder import Shoulder, IntegralShoulder
from y2016.control_loops.python.wrist import Wrist, IntegralWrist
from aos.common.util.trapezoid_profile import TrapezoidProfile

from matplotlib import pylab
import gflags
import glog

FLAGS = gflags.FLAGS

try:
  gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
  pass


class Arm(control_loop.ControlLoop):
  def __init__(self, name="Arm", J=None):
    super(Arm, self).__init__(name=name)
    self._shoulder = Shoulder(name=name, J=J)
    self._shooter = Wrist(name=name)
    self.shoulder_Kv = self._shoulder.Kv / self._shoulder.G

    # Do a coordinate transformation.
    # X_shooter_grounded = X_shooter + X_shoulder
    # dX_shooter_grounded/dt = A_shooter * X_shooter + A_shoulder * X_shoulder +
    #                          B_shoulder * U_shoulder + B_shooter * U_shooter
    # dX_shooter_grounded/dt = A_shooter * (X_shooter_grounded - X_shoulder) +
    #                          A_shoulder * X_shoulder + B_shooter * U_shooter + B_shoulder * U_shoulder
    # X = [X_shoulder; X_shooter + X_shoulder]
    # dX/dt = [A_shoulder                       0] [X_shoulder        ] + [B_shoulder         0] [U_shoulder]
    #         [(A_shoulder - A_shooter) A_shooter] [X_shooter_grounded] + [B_shoulder B_shooter] [ U_shooter]
    # Y_shooter_grounded = Y_shooter + Y_shoulder

    self.A_continuous = numpy.matrix(numpy.zeros((4, 4)))
    self.A_continuous[0:2, 0:2] = self._shoulder.A_continuous
    self.A_continuous[2:4, 0:2] = (self._shoulder.A_continuous -
                                   self._shooter.A_continuous)
    self.A_continuous[2:4, 2:4] = self._shooter.A_continuous

    self.B_continuous = numpy.matrix(numpy.zeros((4, 2)))
    self.B_continuous[0:2, 0:1] = self._shoulder.B_continuous
    self.B_continuous[2:4, 1:2] = self._shooter.B_continuous
    self.B_continuous[2:4, 0:1] = self._shoulder.B_continuous

    self.C = numpy.matrix(numpy.zeros((2, 4)))
    self.C[0:1, 0:2] = self._shoulder.C
    self.C[1:2, 0:2] = -self._shoulder.C
    self.C[1:2, 2:4] = self._shooter.C

    # D is 0 for all our loops.
    self.D = numpy.matrix(numpy.zeros((2, 2)))

    self.dt = 0.005

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    # Cost of error
    self.Q = numpy.matrix(numpy.zeros((4, 4)))
    q_pos_shoulder = 0.014
    q_vel_shoulder = 4.00
    q_pos_shooter = 0.014
    q_vel_shooter = 4.00
    self.Q[0, 0] = 1.0 / q_pos_shoulder ** 2.0
    self.Q[1, 1] = 1.0 / q_vel_shoulder ** 2.0
    self.Q[2, 2] = 1.0 / q_pos_shooter ** 2.0
    self.Q[3, 3] = 1.0 / q_vel_shooter ** 2.0

    self.Qff = numpy.matrix(numpy.zeros((4, 4)))
    qff_pos_shoulder = 0.005
    qff_vel_shoulder = 1.00
    qff_pos_shooter = 0.005
    qff_vel_shooter = 1.00
    self.Qff[0, 0] = 1.0 / qff_pos_shoulder ** 2.0
    self.Qff[1, 1] = 1.0 / qff_vel_shoulder ** 2.0
    self.Qff[2, 2] = 1.0 / qff_pos_shooter ** 2.0
    self.Qff[3, 3] = 1.0 / qff_vel_shooter ** 2.0

    # Cost of control effort
    self.R = numpy.matrix(numpy.zeros((2, 2)))
    r_voltage = 1.0 / 12.0
    self.R[0, 0] = r_voltage ** 2.0
    self.R[1, 1] = r_voltage ** 2.0

    self.Kff = controls.TwoStateFeedForwards(self.B, self.Qff)

    glog.debug('Shoulder K')
    glog.debug(repr(self._shoulder.K))
    glog.debug('Poles are %s',
        repr(numpy.linalg.eig(self._shoulder.A -
                              self._shoulder.B * self._shoulder.K)[0]))

    # Compute controller gains.
    # self.K = controls.dlqr(self.A, self.B, self.Q, self.R)
    self.K = numpy.matrix(numpy.zeros((2, 4)))
    self.K[0:1, 0:2] = self._shoulder.K
    self.K[1:2, 0:2] = (
        -self.Kff[1:2, 2:4] * self.B[2:4, 0:1] * self._shoulder.K
        + self.Kff[1:2, 2:4] * self.A[2:4, 0:2])
    self.K[1:2, 2:4] = self._shooter.K

    glog.debug('Arm controller %s', repr(self.K))

    # Cost of error
    self.Q = numpy.matrix(numpy.zeros((4, 4)))
    q_pos_shoulder = 0.05
    q_vel_shoulder = 2.65
    q_pos_shooter = 0.05
    q_vel_shooter = 2.65
    self.Q[0, 0] = q_pos_shoulder ** 2.0
    self.Q[1, 1] = q_vel_shoulder ** 2.0
    self.Q[2, 2] = q_pos_shooter ** 2.0
    self.Q[3, 3] = q_vel_shooter ** 2.0

    # Cost of control effort
    self.R = numpy.matrix(numpy.zeros((2, 2)))
    r_voltage = 0.025
    self.R[0, 0] = r_voltage ** 2.0
    self.R[1, 1] = r_voltage ** 2.0

    self.KalmanGain, self.Q_steady = controls.kalman(
        A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)
    self.L = self.A * self.KalmanGain

    self.U_max = numpy.matrix([[12.0], [12.0]])
    self.U_min = numpy.matrix([[-12.0], [-12.0]])

    self.InitializeState()


class IntegralArm(Arm):
  def __init__(self, name="IntegralArm", J=None):
    super(IntegralArm, self).__init__(name=name, J=J)

    self.A_continuous_unaugmented = self.A_continuous
    self.B_continuous_unaugmented = self.B_continuous

    self.A_continuous = numpy.matrix(numpy.zeros((6, 6)))
    self.A_continuous[0:4, 0:4] = self.A_continuous_unaugmented
    self.A_continuous[0:4, 4:6] = self.B_continuous_unaugmented

    self.B_continuous = numpy.matrix(numpy.zeros((6, 2)))
    self.B_continuous[0:4, 0:2] = self.B_continuous_unaugmented

    self.C_unaugmented = self.C
    self.C = numpy.matrix(numpy.zeros((2, 6)))
    self.C[0:2, 0:4] = self.C_unaugmented

    self.A, self.B = self.ContinuousToDiscrete(self.A_continuous, self.B_continuous, self.dt)

    q_pos_shoulder = 0.10
    q_vel_shoulder = 0.005
    q_voltage_shoulder = 3.5
    q_pos_shooter = 0.08
    q_vel_shooter = 2.00
    q_voltage_shooter = 1.0
    self.Q = numpy.matrix(numpy.zeros((6, 6)))
    self.Q[0, 0] = q_pos_shoulder ** 2.0
    self.Q[1, 1] = q_vel_shoulder ** 2.0
    self.Q[2, 2] = q_pos_shooter ** 2.0
    self.Q[3, 3] = q_vel_shooter ** 2.0
    self.Q[4, 4] = q_voltage_shoulder ** 2.0
    self.Q[5, 5] = q_voltage_shooter ** 2.0

    self.R = numpy.matrix(numpy.zeros((2, 2)))
    r_pos = 0.05
    self.R[0, 0] = r_pos ** 2.0
    self.R[1, 1] = r_pos ** 2.0

    self.KalmanGain, self.Q_steady = controls.kalman(
        A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)
    self.L = self.A * self.KalmanGain

    self.K_unaugmented = self.K
    self.K = numpy.matrix(numpy.zeros((2, 6)))
    self.K[0:2, 0:4] = self.K_unaugmented
    self.K[0, 4] = 1
    self.K[1, 5] = 1

    self.Kff = numpy.concatenate((self.Kff, numpy.matrix(numpy.zeros((2, 2)))), axis=1)

    self.InitializeState()


class ScenarioPlotter(object):
  def __init__(self):
    # Various lists for graphing things.
    self.t = []
    self.x_shoulder = []
    self.v_shoulder = []
    self.a_shoulder = []
    self.x_hat_shoulder = []
    self.u_shoulder = []
    self.offset_shoulder = []
    self.x_shooter = []
    self.v_shooter = []
    self.a_shooter = []
    self.x_hat_shooter = []
    self.u_shooter = []
    self.offset_shooter = []
    self.goal_x_shoulder = []
    self.goal_v_shoulder = []
    self.goal_x_shooter = []
    self.goal_v_shooter = []

  def run_test(self, arm, end_goal,
               iterations=200, controller=None, observer=None):
    """Runs the plant with an initial condition and goal.

      Args:
        arm: Arm object to use.
        end_goal: numpy.Matrix[6, 1], end goal state.
        iterations: Number of timesteps to run the model for.
        controller: Arm object to get K from, or None if we should
            use arm.
        observer: Arm object to use for the observer, or None if we should
            use the actual state.
    """

    if controller is None:
      controller = arm

    vbat = 12.0

    if self.t:
      initial_t = self.t[-1] + arm.dt
    else:
      initial_t = 0

    goal = numpy.concatenate((arm.X, numpy.matrix(numpy.zeros((2, 1)))), axis=0)

    shoulder_profile = TrapezoidProfile(arm.dt)
    shoulder_profile.set_maximum_acceleration(12.0)
    shoulder_profile.set_maximum_velocity(10.0)
    shoulder_profile.SetGoal(goal[0, 0])
    shooter_profile = TrapezoidProfile(arm.dt)
    shooter_profile.set_maximum_acceleration(50.0)
    shooter_profile.set_maximum_velocity(10.0)
    shooter_profile.SetGoal(goal[2, 0])

    U_last = numpy.matrix(numpy.zeros((2, 1)))
    for i in xrange(iterations):
      X_hat = arm.X

      if observer is not None:
        observer.Y = arm.Y
        observer.CorrectObserver(U_last)
        self.offset_shoulder.append(observer.X_hat[4, 0])
        self.offset_shooter.append(observer.X_hat[5, 0])

        X_hat = observer.X_hat
        self.x_hat_shoulder.append(observer.X_hat[0, 0])
        self.x_hat_shooter.append(observer.X_hat[2, 0])

      next_shoulder_goal = shoulder_profile.Update(end_goal[0, 0], end_goal[1, 0])
      next_shooter_goal = shooter_profile.Update(end_goal[2, 0], end_goal[3, 0])

      next_goal = numpy.concatenate(
          (next_shoulder_goal,
           next_shooter_goal,
           numpy.matrix(numpy.zeros((2, 1)))),
          axis=0)
      self.goal_x_shoulder.append(goal[0, 0])
      self.goal_v_shoulder.append(goal[1, 0])
      self.goal_x_shooter.append(goal[2, 0])
      self.goal_v_shooter.append(goal[3, 0])

      ff_U = controller.Kff * (next_goal - observer.A * goal)

      U_uncapped = controller.K * (goal - X_hat) + ff_U
      U = U_uncapped.copy()

      U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)
      U[1, 0] = numpy.clip(U[1, 0], -vbat, vbat)
      self.x_shoulder.append(arm.X[0, 0])
      self.x_shooter.append(arm.X[2, 0])

      if self.v_shoulder:
        last_v_shoulder = self.v_shoulder[-1]
      else:
        last_v_shoulder = 0
      self.v_shoulder.append(arm.X[1, 0])
      self.a_shoulder.append(
          (self.v_shoulder[-1] - last_v_shoulder) / arm.dt)

      if self.v_shooter:
        last_v_shooter = self.v_shooter[-1]
      else:
        last_v_shooter = 0
      self.v_shooter.append(arm.X[3, 0])
      self.a_shooter.append(
          (self.v_shooter[-1] - last_v_shooter) / arm.dt)

      if i % 40 == 0:
        # Test that if we move the shoulder, the shooter stays perfect.
        #observer.X_hat[0, 0] += 0.20
        #arm.X[0, 0] += 0.20
        pass
      U_error = numpy.matrix([[2.0], [2.0]])
      # Kick it and see what happens.
      #if (initial_t + i * arm.dt) % 0.4 > 0.2:
        #U_error = numpy.matrix([[4.0], [0.0]])
      #else:
        #U_error = numpy.matrix([[-4.0], [0.0]])

      arm.Update(U + U_error)

      if observer is not None:
        observer.PredictObserver(U)

      self.t.append(initial_t + i * arm.dt)
      self.u_shoulder.append(U[0, 0])
      self.u_shooter.append(U[1, 0])

      ff_U -= U_uncapped - U
      goal = controller.A * goal + controller.B * ff_U

      if U[0, 0] != U_uncapped[0, 0]:
        glog.debug('Moving shoulder %s', repr(initial_t + i * arm.dt))
        glog.debug('U error %s', repr(U_uncapped - U))
        glog.debug('goal change is %s',
                   repr(next_shoulder_goal -
                        numpy.matrix([[goal[0, 0]], [goal[1, 0]]])))
        shoulder_profile.MoveCurrentState(
            numpy.matrix([[goal[0, 0]], [goal[1, 0]]]))
      if U[1, 0] != U_uncapped[1, 0]:
        glog.debug('Moving shooter %s', repr(initial_t + i * arm.dt))
        glog.debug('U error %s', repr(U_uncapped - U))
        shooter_profile.MoveCurrentState(
            numpy.matrix([[goal[2, 0]], [goal[3, 0]]]))
      U_last = U
    glog.debug('goal_error %s', repr(end_goal - goal))
    glog.debug('error %s', repr(observer.X_hat - end_goal))


  def Plot(self):
    pylab.subplot(3, 1, 1)
    pylab.plot(self.t, self.x_shoulder, label='x shoulder')
    pylab.plot(self.t, self.goal_x_shoulder, label='goal x shoulder')
    pylab.plot(self.t, self.x_hat_shoulder, label='x_hat shoulder')

    pylab.plot(self.t, self.x_shooter, label='x shooter')
    pylab.plot(self.t, self.x_hat_shooter, label='x_hat shooter')
    pylab.plot(self.t, self.goal_x_shooter, label='goal x shooter')
    pylab.plot(self.t, map(operator.add, self.x_shooter, self.x_shoulder),
               label='x shooter ground')
    pylab.plot(self.t, map(operator.add, self.x_hat_shooter, self.x_hat_shoulder),
               label='x_hat shooter ground')
    pylab.legend()

    pylab.subplot(3, 1, 2)
    pylab.plot(self.t, self.u_shoulder, label='u shoulder')
    pylab.plot(self.t, self.offset_shoulder, label='voltage_offset shoulder')
    pylab.plot(self.t, self.u_shooter, label='u shooter')
    pylab.plot(self.t, self.offset_shooter, label='voltage_offset shooter')
    pylab.legend()

    pylab.subplot(3, 1, 3)
    pylab.plot(self.t, self.a_shoulder, label='a_shoulder')
    pylab.plot(self.t, self.a_shooter, label='a_shooter')
    pylab.legend()

    pylab.show()


def main(argv):
  argv = FLAGS(argv)
  glog.init()

  scenario_plotter = ScenarioPlotter()

  J_accelerating = 18
  J_decelerating = 7

  arm = Arm(name='AcceleratingArm', J=J_accelerating)
  arm_integral_controller = IntegralArm(
      name='AcceleratingIntegralArm', J=J_accelerating)
  arm_observer = IntegralArm()

  # Test moving the shoulder with constant separation.
  initial_X = numpy.matrix([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
  R = numpy.matrix([[numpy.pi / 2.0],
                    [0.0],
                    [0.0], #[numpy.pi / 2.0],
                    [0.0],
                    [0.0],
                    [0.0]])
  arm.X = initial_X[0:4, 0]
  arm_observer.X = initial_X

  scenario_plotter.run_test(arm=arm,
                            end_goal=R,
                            iterations=300,
                            controller=arm_integral_controller,
                            observer=arm_observer)

  if len(argv) != 5:
    glog.fatal('Expected .h file name and .cc file name for the wrist and integral wrist.')
  else:
    namespaces = ['y2016', 'control_loops', 'superstructure']
    decelerating_arm = Arm(name='DeceleratingArm', J=J_decelerating)
    loop_writer = control_loop.ControlLoopWriter(
        'Arm', [arm, decelerating_arm], namespaces=namespaces)
    loop_writer.Write(argv[1], argv[2])

    decelerating_integral_arm_controller = IntegralArm(
        name='DeceleratingIntegralArm', J=J_decelerating)

    integral_loop_writer = control_loop.ControlLoopWriter(
        'IntegralArm',
        [arm_integral_controller, decelerating_integral_arm_controller],
        namespaces=namespaces)
    integral_loop_writer.AddConstant(control_loop.Constant("kV_shoulder", "%f",
          arm_integral_controller.shoulder_Kv))
    integral_loop_writer.Write(argv[3], argv[4])

  if FLAGS.plot:
    scenario_plotter.Plot()

if __name__ == '__main__':
  sys.exit(main(sys.argv))
