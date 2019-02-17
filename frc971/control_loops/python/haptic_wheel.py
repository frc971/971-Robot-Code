#!/usr/bin/python

from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
import numpy
import sys
import copy
import scipy.interpolate
from matplotlib import pylab

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
gflags.DEFINE_string('data', None, 'If defined, plot the provided CAN data')
gflags.DEFINE_bool(
    'rerun_kf', False,
    'If true, rerun the KF.  The torque in the data file will be interpreted as the commanded current.'
)


class SystemParams(object):

    def __init__(self, J, G, kP, kD, kCompensationTimeconstant, q_pos, q_vel,
                 q_torque, current_limit):
        self.J = J
        self.G = G
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.q_torque = q_torque
        self.kP = kP
        self.kD = kD
        self.kCompensationTimeconstant = kCompensationTimeconstant
        self.r_pos = 0.001
        self.current_limit = current_limit

        #[15.0, 0.25],
        #[10.0, 0.2],
        #[5.0, 0.13],
        #[3.0, 0.10],
        #[2.0, 0.08],
        #[1.0, 0.06],
        #[0.5, 0.05],
        #[0.25, 0.025],


kWheel = SystemParams(
    J=0.0008,
    G=(1.25 + 0.02) / 0.35,
    q_pos=0.001,
    q_vel=0.20,
    q_torque=0.005,
    kP=7.0,
    kD=0.0,
    kCompensationTimeconstant=0.95,
    current_limit=4.5)
kTrigger = SystemParams(
    J=0.00025,
    G=(0.925 * 2.0 + 0.02) / 0.35,
    q_pos=0.001,
    q_vel=0.1,
    q_torque=0.005,
    kP=120.0,
    kD=1.8,
    kCompensationTimeconstant=0.95,
    current_limit=3.0)


class HapticInput(control_loop.ControlLoop):

    def __init__(self, params=None, name='HapticInput'):
        # The defaults are for the steering wheel.
        super(HapticInput, self).__init__(name)
        motor = self.motor = control_loop.MN3510()

        # Moment of inertia of the wheel in kg m^2
        self.J = params.J

        # Control loop time step
        self.dt = 0.001

        # Gear ratio from the motor to the input.
        self.G = params.G

        self.A_continuous = numpy.matrix(numpy.zeros((2, 2)))
        self.A_continuous[1, 1] = 0
        self.A_continuous[0, 1] = 1

        self.B_continuous = numpy.matrix(numpy.zeros((2, 1)))
        self.B_continuous[1, 0] = motor.Kt * self.G / self.J

        # State feedback matrices
        # [position, angular velocity]
        self.C = numpy.matrix([[1.0, 0.0]])
        self.D = numpy.matrix([[0.0]])

        self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                                   self.B_continuous, self.dt)

        self.U_max = numpy.matrix([[2.5]])
        self.U_min = numpy.matrix([[-2.5]])

        self.L = numpy.matrix([[0.0], [0.0]])
        self.K = numpy.matrix([[0.0, 0.0]])

        self.InitializeState()


class IntegralHapticInput(HapticInput):

    def __init__(self, params=None, name="IntegralHapticInput"):
        super(IntegralHapticInput, self).__init__(name=name, params=params)

        self.A_continuous_unaugmented = self.A_continuous
        self.B_continuous_unaugmented = self.B_continuous

        self.A_continuous = numpy.matrix(numpy.zeros((3, 3)))
        self.A_continuous[0:2, 0:2] = self.A_continuous_unaugmented
        self.A_continuous[1, 2] = (1 / self.J)

        self.B_continuous = numpy.matrix(numpy.zeros((3, 1)))
        self.B_continuous[0:2, 0] = self.B_continuous_unaugmented

        self.C_unaugmented = self.C
        self.C = numpy.matrix(numpy.zeros((1, 3)))
        self.C[0:1, 0:2] = self.C_unaugmented

        self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                                   self.B_continuous, self.dt)

        self.Q = numpy.matrix([[(params.q_pos**2.0), 0.0, 0.0],
                               [0.0, (params.q_vel**2.0), 0.0],
                               [0.0, 0.0, (params.q_torque**2.0)]])

        self.R = numpy.matrix([[(params.r_pos**2.0)]])

        self.KalmanGain, self.Q_steady = controls.kalman(
            A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)
        self.L = self.A * self.KalmanGain

        self.K_unaugmented = self.K
        self.K = numpy.matrix(numpy.zeros((1, 3)))
        self.K[0, 0:2] = self.K_unaugmented
        self.K[0, 2] = 1.0 / (self.motor.Kt / (self.motor.resistance))

        self.InitializeState()


def ReadCan(filename):
    """Reads the candump in filename and returns the 4 fields."""
    trigger = []
    trigger_velocity = []
    trigger_torque = []
    trigger_current = []
    wheel = []
    wheel_velocity = []
    wheel_torque = []
    wheel_current = []

    trigger_request_time = [0.0]
    trigger_request_current = [0.0]
    wheel_request_time = [0.0]
    wheel_request_current = [0.0]

    with open(filename, 'r') as fd:
        for line in fd:
            data = line.split()
            can_id = int(data[1], 16)
            if can_id == 0:
                data = [int(d, 16) for d in data[3:]]
                trigger.append(((data[0] + (data[1] << 8)) - 32768) / 32768.0)
                trigger_velocity.append(
                    ((data[2] + (data[3] << 8)) - 32768) / 32768.0)
                trigger_torque.append(
                    ((data[4] + (data[5] << 8)) - 32768) / 32768.0)
                trigger_current.append(
                    ((data[6] + ((data[7] & 0x3f) << 8)) - 8192) / 8192.0)
            elif can_id == 1:
                data = [int(d, 16) for d in data[3:]]
                wheel.append(((data[0] + (data[1] << 8)) - 32768) / 32768.0)
                wheel_velocity.append(
                    ((data[2] + (data[3] << 8)) - 32768) / 32768.0)
                wheel_torque.append(
                    ((data[4] + (data[5] << 8)) - 32768) / 32768.0)
                wheel_current.append(
                    ((data[6] + ((data[7] & 0x3f) << 8)) - 8192) / 8192.0)
            elif can_id == 2:
                data = [int(d, 16) for d in data[3:]]
                trigger_request_current.append(
                    ((data[4] + (data[5] << 8)) - 32768) / 32768.0)
                trigger_request_time.append(len(trigger) * 0.001)
            elif can_id == 3:
                data = [int(d, 16) for d in data[3:]]
                wheel_request_current.append(
                    ((data[4] + (data[5] << 8)) - 32768) / 32768.0)
                wheel_request_time.append(len(wheel) * 0.001)

    trigger_data_time = numpy.arange(0, len(trigger)) * 0.001
    wheel_data_time = numpy.arange(0, len(wheel)) * 0.001

    # Extend out the data in the interpolation table.
    trigger_request_time.append(trigger_data_time[-1])
    trigger_request_current.append(trigger_request_current[-1])
    wheel_request_time.append(wheel_data_time[-1])
    wheel_request_current.append(wheel_request_current[-1])

    return (trigger_data_time, wheel_data_time, trigger, wheel,
            trigger_velocity, wheel_velocity, trigger_torque, wheel_torque,
            trigger_current, wheel_current, trigger_request_time,
            trigger_request_current, wheel_request_time, wheel_request_current)


def rerun_and_plot_kf(data_time,
                      data_radians,
                      data_current,
                      data_request_current,
                      params,
                      run_correct=True):
    kf_velocity = []
    dt_velocity = []
    kf_position = []
    adjusted_position = []
    last_angle = None
    haptic_observer = IntegralHapticInput(params=params)

    # Parameter sweep J.
    num_kf = 1
    min_J = max_J = params.J

    # J = 0.0022
    #num_kf = 15
    #min_J = min_J / 2.0
    #max_J = max_J * 2.0
    initial_velocity = (data_radians[1] - data_radians[0]) * 1000.0

    def DupParamsWithJ(params, J):
        p = copy.copy(params)
        p.J = J
        return p

    haptic_observers = [
        IntegralHapticInput(params=DupParamsWithJ(params, j))
        for j in numpy.logspace(
            numpy.log10(min_J), numpy.log10(max_J), num=num_kf)
    ]
    # Initialize all the KF's.
    haptic_observer.X_hat[1, 0] = initial_velocity
    haptic_observer.X_hat[0, 0] = data_radians[0]
    for observer in haptic_observers:
        observer.X_hat[1, 0] = initial_velocity
        observer.X_hat[0, 0] = data_radians[0]

    last_request_current = data_request_current[0]
    kf_torques = [[] for i in xrange(num_kf)]
    for angle, current, request_current in zip(data_radians, data_current,
                                               data_request_current):
        # Predict and correct all the parameter swept observers.
        for i, observer in enumerate(haptic_observers):
            observer.Y = numpy.matrix([[angle]])
            if run_correct:
                observer.CorrectObserver(numpy.matrix([[current]]))
            kf_torques[i].append(-observer.X_hat[2, 0])
            observer.PredictObserver(numpy.matrix([[current]]))
            observer.PredictObserver(numpy.matrix([[current]]))

        # Predict and correct the main observer.
        haptic_observer.Y = numpy.matrix([[angle]])
        if run_correct:
            haptic_observer.CorrectObserver(numpy.matrix([[current]]))
        kf_position.append(haptic_observer.X_hat[0, 0])
        adjusted_position.append(kf_position[-1] -
                                 last_request_current / params.kP)
        last_request_current = last_request_current * params.kCompensationTimeconstant + request_current * (
            1.0 - params.kCompensationTimeconstant)
        kf_velocity.append(haptic_observer.X_hat[1, 0])
        if last_angle is None:
            last_angle = angle
        dt_velocity.append((angle - last_angle) / 0.001)

        haptic_observer.PredictObserver(numpy.matrix([[current]]))
        last_angle = angle

    # Plot the wheel observers.
    fig, ax1 = pylab.subplots()
    ax1.plot(data_time, data_radians, '.', label='wheel')
    ax1.plot(data_time, dt_velocity, '.', label='dt_velocity')
    ax1.plot(data_time, kf_velocity, '.', label='kf_velocity')
    ax1.plot(data_time, kf_position, '.', label='kf_position')
    ax1.plot(data_time, adjusted_position, '.', label='adjusted_position')

    ax2 = ax1.twinx()
    ax2.plot(data_time, data_current, label='data_current')
    ax2.plot(data_time, data_request_current, label='request_current')

    for i, kf_torque in enumerate(kf_torques):
        ax2.plot(
            data_time,
            kf_torque,
            label='-kf_torque[%f]' % haptic_observers[i].J)
    fig.tight_layout()
    ax1.legend(loc=3)
    ax2.legend(loc=4)


def plot_input(data_time,
               data_radians,
               data_velocity,
               data_torque,
               data_current,
               params,
               run_correct=True):
    dt_velocity = []
    last_angle = None
    initial_velocity = (data_radians[1] - data_radians[0]) * 1000.0

    for angle in data_radians:
        if last_angle is None:
            last_angle = angle
        dt_velocity.append((angle - last_angle) / 0.001)

        last_angle = angle

    # Plot the wheel observers.
    fig, ax1 = pylab.subplots()
    ax1.plot(data_time, data_radians, '.', label='angle')
    ax1.plot(data_time, data_velocity, '-', label='velocity')
    ax1.plot(data_time, dt_velocity, '.', label='dt_velocity')

    ax2 = ax1.twinx()
    ax2.plot(data_time, data_torque, label='data_torque')
    ax2.plot(data_time, data_current, label='data_current')
    fig.tight_layout()
    ax1.legend(loc=3)
    ax2.legend(loc=4)


def main(argv):
    if FLAGS.plot:
        if FLAGS.data is None:
            haptic_wheel = HapticInput()
            haptic_wheel_controller = IntegralHapticInput()
            observer_haptic_wheel = IntegralHapticInput()
            observer_haptic_wheel.X_hat[2, 0] = 0.01

            R = numpy.matrix([[0.0], [0.0], [0.0]])

            control_loop.TestSingleIntegralAxisSquareWave(
                R, 1.0, haptic_wheel, haptic_wheel_controller,
                observer_haptic_wheel)
        else:
            # Read the CAN trace in.
            trigger_data_time, wheel_data_time, trigger, wheel, trigger_velocity, \
                wheel_velocity, trigger_torque, wheel_torque, trigger_current, \
                wheel_current, trigger_request_time, trigger_request_current, \
                wheel_request_time, wheel_request_current = ReadCan(FLAGS.data)

            wheel_radians = [w * numpy.pi * (338.16 / 360.0) for w in wheel]
            wheel_velocity = [w * 50.0 for w in wheel_velocity]
            wheel_torque = [w / 2.0 for w in wheel_torque]
            wheel_current = [w * 10.0 for w in wheel_current]
            wheel_request_current = [w * 2.0 for w in wheel_request_current]
            resampled_wheel_request_current = scipy.interpolate.interp1d(
                wheel_request_time, wheel_request_current,
                kind="zero")(wheel_data_time)

            trigger_radians = [t * numpy.pi * (45.0 / 360.0) for t in trigger]
            trigger_velocity = [t * 50.0 for t in trigger_velocity]
            trigger_torque = [t / 2.0 for t in trigger_torque]
            trigger_current = [t * 10.0 for t in trigger_current]
            trigger_request_current = [t * 2.0 for t in trigger_request_current]
            resampled_trigger_request_current = scipy.interpolate.interp1d(
                trigger_request_time, trigger_request_current,
                kind="zero")(trigger_data_time)

            if FLAGS.rerun_kf:
                rerun_and_plot_kf(
                    trigger_data_time,
                    trigger_radians,
                    trigger_current,
                    resampled_trigger_request_current,
                    kTrigger,
                    run_correct=True)
                rerun_and_plot_kf(
                    wheel_data_time,
                    wheel_radians,
                    wheel_current,
                    resampled_wheel_request_current,
                    kWheel,
                    run_correct=True)
            else:
                plot_input(trigger_data_time, trigger_radians, trigger_velocity,
                           trigger_torque, trigger_current, kTrigger)
                plot_input(wheel_data_time, wheel_radians, wheel_velocity,
                           wheel_torque, wheel_current, kWheel)
            pylab.show()

        return

    if len(argv) != 9:
        glog.fatal('Expected .h file name and .cc file name')
    else:
        namespaces = ['frc971', 'control_loops', 'drivetrain']
        for name, params, filenames in [('HapticWheel', kWheel, argv[1:5]),
                                        ('HapticTrigger', kTrigger, argv[5:9])]:
            haptic_input = HapticInput(params=params, name=name)
            loop_writer = control_loop.ControlLoopWriter(
                name, [haptic_input],
                namespaces=namespaces,
                scalar_type='float')
            loop_writer.Write(filenames[0], filenames[1])

            integral_haptic_input = IntegralHapticInput(
                params=params, name='Integral' + name)
            integral_loop_writer = control_loop.ControlLoopWriter(
                'Integral' + name, [integral_haptic_input],
                namespaces=namespaces,
                scalar_type='float')

            integral_loop_writer.AddConstant(
                control_loop.Constant("k" + name + "Dt", "%f",
                                      integral_haptic_input.dt))
            integral_loop_writer.AddConstant(
                control_loop.Constant("k" + name + "FreeCurrent", "%f",
                                      integral_haptic_input.motor.free_current))
            integral_loop_writer.AddConstant(
                control_loop.Constant("k" + name + "StallTorque", "%f",
                                      integral_haptic_input.motor.stall_torque))
            integral_loop_writer.AddConstant(
                control_loop.Constant("k" + name + "J", "%f",
                                      integral_haptic_input.J))
            integral_loop_writer.AddConstant(
                control_loop.Constant("k" + name + "R", "%f",
                                      integral_haptic_input.motor.resistance))
            integral_loop_writer.AddConstant(
                control_loop.Constant("k" + name + "T", "%f",
                                      integral_haptic_input.motor.Kt))
            integral_loop_writer.AddConstant(
                control_loop.Constant("k" + name + "V", "%f",
                                      integral_haptic_input.motor.Kv))
            integral_loop_writer.AddConstant(
                control_loop.Constant("k" + name + "P", "%f", params.kP))
            integral_loop_writer.AddConstant(
                control_loop.Constant("k" + name + "D", "%f", params.kD))
            integral_loop_writer.AddConstant(
                control_loop.Constant("k" + name + "G", "%f", params.G))
            integral_loop_writer.AddConstant(
                control_loop.Constant("k" + name + "CurrentLimit", "%f",
                                      params.current_limit))

            integral_loop_writer.Write(filenames[2], filenames[3])


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    sys.exit(main(argv))
