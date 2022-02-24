from frc971.control_loops.python import angular_system
from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls
from aos.util.trapezoid_profile import TrapezoidProfile
import numpy
from matplotlib import pylab

import gflags
import glog

CatapultParams = angular_system.AngularSystemParams


# TODO(austin): This is mostly the same as angular_system.  Can we either wrap an angular_system or assign it?
class Catapult(angular_system.AngularSystem):
    def __init__(self, params, name="Catapult"):
        super(Catapult, self).__init__(params, name)
        # Signal that we have a single cycle output delay to compensate for in
        # our observer.
        self.delayed_u = True

        self.InitializeState()


class IntegralCatapult(angular_system.IntegralAngularSystem):
    def __init__(self, params, name="IntegralCatapult"):
        super(IntegralCatapult, self).__init__(params, name=name)
        # Signal that we have a single cycle output delay to compensate for in
        # our observer.
        self.delayed_u = True

        self.InitializeState()


def MaxSpeed(params, U, final_position):
    """Runs the catapult plant with an initial condition and goal.

    Args:
        catapult: Catapult object to use.
        goal: goal state.
        iterations: Number of timesteps to run the model for.
        controller_catapult: Catapult object to get K from, or None if we should
             use catapult.
        observer_catapult: Catapult object to use for the observer, or None if we
            should use the actual state.
    """

    # Various lists for graphing things.
    catapult = Catapult(params, params.name)
    controller_catapult = IntegralCatapult(params, params.name)
    observer_catapult = IntegralCatapult(params, params.name)
    vbat = 12.0

    while True:
        X_hat = catapult.X
        if catapult.X[0, 0] > final_position:
            return catapult.X[1, 0] * params.radius

        if observer_catapult is not None:
            X_hat = observer_catapult.X_hat

        U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)

        if observer_catapult is not None:
            observer_catapult.Y = catapult.Y
            observer_catapult.CorrectObserver(U)

        applied_U = U.copy()
        catapult.Update(applied_U)

        if observer_catapult is not None:
            observer_catapult.PredictObserver(U)


def PlotShot(params, U, final_position):
    """Runs the catapult plant with an initial condition and goal.

    Args:
        catapult: Catapult object to use.
        goal: goal state.
        iterations: Number of timesteps to run the model for.
        controller_catapult: Catapult object to get K from, or None if we should
             use catapult.
        observer_catapult: Catapult object to use for the observer, or None if we
            should use the actual state.
    """

    # Various lists for graphing things.
    t = []
    x = []
    x_hat = []
    v = []
    w_hat = []
    v_hat = []
    a = []
    u = []
    offset = []

    catapult = Catapult(params, params.name)
    controller_catapult = IntegralCatapult(params, params.name)
    observer_catapult = IntegralCatapult(params, params.name)
    vbat = 12.0

    if t:
        initial_t = t[-1] + catapult.dt
    else:
        initial_t = 0

    for i in range(10000):
        X_hat = catapult.X
        if catapult.X[0, 0] > final_position:
            break

        if observer_catapult is not None:
            X_hat = observer_catapult.X_hat
            x_hat.append(observer_catapult.X_hat[0, 0])
            w_hat.append(observer_catapult.X_hat[1, 0])
            v_hat.append(observer_catapult.X_hat[1, 0] * params.radius)

        U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)
        x.append(catapult.X[0, 0])

        if v:
            last_v = v[-1]
        else:
            last_v = 0

        v.append(catapult.X[1, 0])
        a.append((v[-1] - last_v) / catapult.dt)

        if observer_catapult is not None:
            observer_catapult.Y = catapult.Y
            observer_catapult.CorrectObserver(U)
            offset.append(observer_catapult.X_hat[2, 0])

        catapult.Update(U)

        if observer_catapult is not None:
            observer_catapult.PredictObserver(U)

        t.append(initial_t + i * catapult.dt)
        u.append(U[0, 0])

    pylab.subplot(3, 1, 1)
    pylab.plot(t, v, label='v')
    pylab.plot(t, x_hat, label='x_hat')
    pylab.plot(t, v, label='v')
    pylab.plot(t, v_hat, label='v_hat')
    pylab.plot(t, w_hat, label='w_hat')
    pylab.legend()

    pylab.subplot(3, 1, 2)
    pylab.plot(t, u, label='u')
    pylab.plot(t, offset, label='voltage_offset')
    pylab.legend()

    pylab.subplot(3, 1, 3)
    pylab.plot(t, a, label='a')
    pylab.legend()

    pylab.show()


RunTest = angular_system.RunTest


def PlotStep(params, R, plant_params=None):
    """Plots a step move to the goal.

    Args:
      params: CatapultParams for the controller and observer
      plant_params: CatapultParams for the plant.  Defaults to params if
        plant_params is None.
      R: numpy.matrix(2, 1), the goal"""
    plant = Catapult(plant_params or params, params.name)
    controller = IntegralCatapult(params, params.name)
    observer = IntegralCatapult(params, params.name)

    # Test moving the system.
    initial_X = numpy.matrix([[0.0], [0.0]])
    augmented_R = numpy.matrix(numpy.zeros((3, 1)))
    augmented_R[0:2, :] = R
    RunTest(plant,
            end_goal=augmented_R,
            controller=controller,
            observer=observer,
            duration=2.0,
            use_profile=False,
            kick_time=1.0,
            kick_magnitude=0.0)


def PlotKick(params, R, plant_params=None):
    """Plots a step motion with a kick at 1.0 seconds.

    Args:
      params: CatapultParams for the controller and observer
      plant_params: CatapultParams for the plant.  Defaults to params if
        plant_params is None.
      R: numpy.matrix(2, 1), the goal"""
    plant = Catapult(plant_params or params, params.name)
    controller = IntegralCatapult(params, params.name)
    observer = IntegralCatapult(params, params.name)

    # Test moving the system.
    initial_X = numpy.matrix([[0.0], [0.0]])
    augmented_R = numpy.matrix(numpy.zeros((3, 1)))
    augmented_R[0:2, :] = R
    RunTest(plant,
            end_goal=augmented_R,
            controller=controller,
            observer=observer,
            duration=2.0,
            use_profile=False,
            kick_time=1.0,
            kick_magnitude=2.0)


def PlotMotion(params,
               R,
               max_velocity=10.0,
               max_acceleration=70.0,
               plant_params=None):
    """Plots a trapezoidal motion.

    Args:
      params: CatapultParams for the controller and observer
      plant_params: CatapultParams for the plant.  Defaults to params if
        plant_params is None.
      R: numpy.matrix(2, 1), the goal,
      max_velocity: float, The max velocity of the profile.
      max_acceleration: float, The max acceleration of the profile.
    """
    plant = Catapult(plant_params or params, params.name)
    controller = IntegralCatapult(params, params.name)
    observer = IntegralCatapult(params, params.name)

    # Test moving the system.
    initial_X = numpy.matrix([[0.0], [0.0]])
    augmented_R = numpy.matrix(numpy.zeros((3, 1)))
    augmented_R[0:2, :] = R
    RunTest(plant,
            end_goal=augmented_R,
            controller=controller,
            observer=observer,
            duration=2.0,
            use_profile=True,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration)


def WriteCatapult(params, plant_files, controller_files, year_namespaces):
    """Writes out the constants for a catapult to a file.

    Args:
      params: list of CatapultParams or CatapultParams, the
        parameters defining the system.
      plant_files: list of strings, the cc and h files for the plant.
      controller_files: list of strings, the cc and h files for the integral
        controller.
      year_namespaces: list of strings, the namespace list to use.
    """
    # Write the generated constants out to a file.
    catapults = []
    integral_catapults = []

    if type(params) is list:
        name = params[0].name
        for index, param in enumerate(params):
            catapults.append(Catapult(param, param.name + str(index)))
            integral_catapults.append(
                IntegralCatapult(param, 'Integral' + param.name + str(index)))
    else:
        name = params.name
        catapults.append(Catapult(params, params.name))
        integral_catapults.append(
            IntegralCatapult(params, 'Integral' + params.name))

    loop_writer = control_loop.ControlLoopWriter(name,
                                                 catapults,
                                                 namespaces=year_namespaces)
    loop_writer.AddConstant(
        control_loop.Constant('kOutputRatio', '%f', catapults[0].G))
    loop_writer.AddConstant(
        control_loop.Constant('kFreeSpeed', '%f',
                              catapults[0].motor.free_speed))
    loop_writer.Write(plant_files[0], plant_files[1])

    integral_loop_writer = control_loop.ControlLoopWriter(
        'Integral' + name, integral_catapults, namespaces=year_namespaces)
    integral_loop_writer.Write(controller_files[0], controller_files[1])
