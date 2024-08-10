#!/usr/bin/python3

import numpy
import sympy
import scipy.integrate
from frc971.control_loops.python import control_loop
from frc971.control_loops.python import controls

import matplotlib
from matplotlib import pylab
import sys
import gflags
import glog

FLAGS = gflags.FLAGS

matplotlib.use("GTK3Agg")


class SwerveSimulation(object):

    def __init__(self):
        self.motor = control_loop.KrakenFOC()

        vx, vy, omega = sympy.symbols('vx vy omega')
        fx, fy = sympy.symbols('fx fy')
        t = sympy.symbols('t')

        # 5kg of force got us a slip angle of 0.05 radians with 4 tires.
        self.C = 5 * 9.8 / 0.05 / 4.0

        self.r = 2 * 0.0254

        # Base is 20kg without battery.
        self.m = 25.0
        self.G = 1.0 / 6.75

        I = sympy.symbols('I')

        # Absolute linear velocity in x and y of the robot.
        self.dvx = (self.C * (self.r * omega - vx) / vx + fx) / self.m
        self.dvy = (-self.C * sympy.atan2(vy, vx) + fy) / self.m
        # Angular velocity of the wheel.
        self.domega = (-self.G * self.C * (self.r * omega - vx) / vx * self.r +
                       self.motor.Kt * I) * self.G / self.motor.motor_inertia

        self.x0 = sympy.lambdify((vx, vy, omega, fx, fy, I), self.dvx)
        self.x1 = sympy.lambdify((vx, vy, omega, fx, fy, I), self.dvy)
        self.x2 = sympy.lambdify((vx, vy, omega, fx, fy, I), self.domega)

        self.f = lambda X, fx, fy, I: numpy.array([
            self.x0(X[0], X[1], X[2], fx, fy, I),
            self.x1(X[0], X[1], X[2], fx, fy, I),
            self.x2(X[0], X[1], X[2], fx, fy, I)
        ])

        print(self.f)
        print(
            'f',
            self.f(numpy.matrix([[1.0], [0.0], [1.0 / self.r]]), 0.0, 0.0,
                   0.0))

        print(self.dvx)
        print(self.dvy)
        print(self.domega)

    def run(self, X_initial):
        print(X_initial)

        fx = -9.8
        fy = 0.0
        I = -(fx * self.r * self.G / self.motor.Kt)
        print(f"Fx: {fx}, Fy: {fy}, I: {I}")

        def f_const(t, X):
            return self.f(
                X=X,
                fx=fx,
                fy=fy,
                I=I,
            )

        result = scipy.integrate.solve_ivp(
            f_const, (0, 2.0),
            numpy.squeeze(numpy.array(X_initial.transpose())),
            max_step=0.01)

        pylab.plot(result.t, result.y[0, :], label="y0")
        pylab.plot(result.t, result.y[1, :], label="y1")
        pylab.plot(result.t, result.y[2, :], label="y2")

        pylab.legend()
        pylab.show()


def main(argv):
    s = SwerveSimulation()
    s.run(numpy.matrix([[1.0], [0.0], [1.0 / s.r]]))

    return 0


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
