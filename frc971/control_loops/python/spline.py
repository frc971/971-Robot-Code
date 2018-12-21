#!/usr/bin/python

from __future__ import print_function

from matplotlib import pylab
import gflags
import glog
import numpy
import scipy
import scipy.integrate
import sys

from frc971.control_loops.python import polydrivetrain
from frc971.control_loops.python import drivetrain
import y2018.control_loops.python.drivetrain

"""This file is my playground for implementing spline following.

All splines here are cubic bezier splines.  See
  https://en.wikipedia.org/wiki/B%C3%A9zier_curve for more details.
"""

FLAGS = gflags.FLAGS


def RungeKutta(f, y0, t, h, count=1):
    """4th order RungeKutta integration of dy/dt = f(t, y) starting at X."""
    y1 = y0
    dh = h / float(count)
    for x in xrange(count):
        k1 = dh * f(t + dh * x, y1)
        k2 = dh * f(t + dh * x + dh / 2.0, y1 + k1 / 2.0)
        k3 = dh * f(t + dh * x + dh / 2.0, y1 + k2 / 2.0)
        k4 = dh * f(t + dh * x + dh, y1 + k3)
        y1 += (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    return y1


def spline(alpha, control_points):
    """Computes a Cubic Bezier curve.

    Args:
        alpha: scalar or list of spline parameters to calculate the curve at.
        control_points: n x 4 matrix of control points.  n[:, 0] is the
            starting point, and n[:, 3] is the ending point.

    Returns:
        n x m matrix of spline points.  n is the dimension of the control
        points, and m is the number of points in 'alpha'.
    """
    if numpy.isscalar(alpha):
        alpha = [alpha]
    alpha_matrix = [[(1.0 - a)**3.0, 3.0 * (1.0 - a)**2.0 * a,
                     3.0 * (1.0 - a) * a**2.0, a**3.0] for a in alpha]

    return control_points * numpy.matrix(alpha_matrix).T


def dspline(alpha, control_points):
    """Computes the derivative of a Cubic Bezier curve wrt alpha.

    Args:
        alpha: scalar or list of spline parameters to calculate the curve at.
        control_points: n x 4 matrix of control points.  n[:, 0] is the
            starting point, and n[:, 3] is the ending point.

    Returns:
        n x m matrix of spline point derivatives.  n is the dimension of the
        control points, and m is the number of points in 'alpha'.
    """
    if numpy.isscalar(alpha):
        alpha = [alpha]
    dalpha_matrix = [[
        -3.0 * (1.0 - a)**2.0, 3.0 * (1.0 - a)**2.0 + -2.0 * 3.0 *
        (1.0 - a) * a, -3.0 * a**2.0 + 2.0 * 3.0 * (1.0 - a) * a, 3.0 * a**2.0
    ] for a in alpha]

    return control_points * numpy.matrix(dalpha_matrix).T


def ddspline(alpha, control_points):
    """Computes the second derivative of a Cubic Bezier curve wrt alpha.

    Args:
        alpha: scalar or list of spline parameters to calculate the curve at.
        control_points: n x 4 matrix of control points.  n[:, 0] is the
            starting point, and n[:, 3] is the ending point.

    Returns:
        n x m matrix of spline point second derivatives.  n is the dimension of
        the control points, and m is the number of points in 'alpha'.
    """
    if numpy.isscalar(alpha):
        alpha = [alpha]
    ddalpha_matrix = [[
        2.0 * 3.0 * (1.0 - a),
        -2.0 * 3.0 * (1.0 - a) + -2.0 * 3.0 * (1.0 - a) + 2.0 * 3.0 * a,
        -2.0 * 3.0 * a + 2.0 * 3.0 * (1.0 - a) - 2.0 * 3.0 * a,
        2.0 * 3.0 * a
    ] for a in alpha]

    return control_points * numpy.matrix(ddalpha_matrix).T


def dddspline(alpha, control_points):
    """Computes the third derivative of a Cubic Bezier curve wrt alpha.

    Args:
        alpha: scalar or list of spline parameters to calculate the curve at.
        control_points: n x 4 matrix of control points.  n[:, 0] is the
            starting point, and n[:, 3] is the ending point.

    Returns:
        n x m matrix of spline point second derivatives.  n is the dimension of
        the control points, and m is the number of points in 'alpha'.
    """
    if numpy.isscalar(alpha):
        alpha = [alpha]
    ddalpha_matrix = [[
        -2.0 * 3.0,
        2.0 * 3.0 + 2.0 * 3.0 + 2.0 * 3.0,
        -2.0 * 3.0 - 2.0 * 3.0 - 2.0 * 3.0,
        2.0 * 3.0
    ] for a in alpha]

    return control_points * numpy.matrix(ddalpha_matrix).T


def spline_theta(alpha, control_points, dspline_points=None):
    """Computes the heading of a robot following a Cubic Bezier curve at alpha.

    Args:
        alpha: scalar or list of spline parameters to calculate the heading at.
        control_points: n x 4 matrix of control points.  n[:, 0] is the
            starting point, and n[:, 3] is the ending point.

    Returns:
        m array of spline point headings.  m is the number of points in 'alpha'.
    """
    if dspline_points is None:
        dspline_points = dspline(alpha, control_points)

    return numpy.arctan2(
        numpy.array(dspline_points)[1, :], numpy.array(dspline_points)[0, :])


def dspline_theta(alpha,
                  control_points,
                  dspline_points=None,
                  ddspline_points=None):
    """Computes the derivative of the heading at alpha.

    This is the derivative of spline_theta wrt alpha.

    Args:
        alpha: scalar or list of spline parameters to calculate the derivative
            of the heading at.
        control_points: n x 4 matrix of control points.  n[:, 0] is the
            starting point, and n[:, 3] is the ending point.

    Returns:
        m array of spline point heading derivatives.  m is the number of points
        in 'alpha'.
    """
    if dspline_points is None:
        dspline_points = dspline(alpha, control_points)

    if ddspline_points is None:
        ddspline_points = ddspline(alpha, control_points)

    dx = numpy.array(dspline_points)[0, :]
    dy = numpy.array(dspline_points)[1, :]

    ddx = numpy.array(ddspline_points)[0, :]
    ddy = numpy.array(ddspline_points)[1, :]

    return 1.0 / (dx**2.0 + dy**2.0) * (dx * ddy - dy * ddx)


def ddspline_theta(alpha,
                   control_points,
                   dspline_points=None,
                   ddspline_points=None,
                   dddspline_points=None):
    """Computes the second derivative of the heading at alpha.

    This is the second derivative of spline_theta wrt alpha.

    Args:
        alpha: scalar or list of spline parameters to calculate the second
            derivative of the heading at.
        control_points: n x 4 matrix of control points.  n[:, 0] is the
            starting point, and n[:, 3] is the ending point.

    Returns:
        m array of spline point heading second derivatives.  m is the number of
        points in 'alpha'.
    """
    if dspline_points is None:
        dspline_points = dspline(alpha, control_points)

    if ddspline_points is None:
        ddspline_points = ddspline(alpha, control_points)

    if dddspline_points is None:
        dddspline_points = dddspline(alpha, control_points)

    dddspline_points = dddspline(alpha, control_points)

    dx = numpy.array(dspline_points)[0, :]
    dy = numpy.array(dspline_points)[1, :]

    ddx = numpy.array(ddspline_points)[0, :]
    ddy = numpy.array(ddspline_points)[1, :]

    dddx = numpy.array(dddspline_points)[0, :]
    dddy = numpy.array(dddspline_points)[1, :]

    return -1.0 / ((dx**2.0 + dy**2.0)**2.0) * (dx * ddy - dy * ddx) * 2.0 * (
        dy * ddy + dx * ddx) + 1.0 / (dx**2.0 + dy**2.0) * (dx * dddy - dy *
                                                            dddx)


class Path(object):
    """Represents a path to follow."""
    def __init__(self, control_points):
        """Constructs a path given the control points."""
        self._control_points = control_points

        def spline_velocity(alpha):
            return numpy.linalg.norm(dspline(alpha, self._control_points), axis=0)

        self._point_distances = [0.0]
        num_alpha = 100
        # Integrate the xy velocity as a function of alpha for each step in the
        # table to get an alpha -> distance calculation.  Gaussian Quadrature
        # is quite accurate, so we can get away with fewer points here than we
        # might think.
        for alpha in numpy.linspace(0.0, 1.0, num_alpha)[:-1]:
            self._point_distances.append(
                scipy.integrate.fixed_quad(spline_velocity, alpha, alpha + 1.0
                                           / (num_alpha - 1.0))[0] +
                self._point_distances[-1])

    def distance_to_alpha(self, distance):
        """Converts distances along the spline to alphas.

        Args:
            distance: A scalar or array of distances to convert

        Returns:
            An array of distances, (1 big if the input was a scalar)
        """
        if numpy.isscalar(distance):
            return numpy.array([self._distance_to_alpha_scalar(distance)])
        else:
            return numpy.array([self._distance_to_alpha_scalar(d) for d in distance])

    def _distance_to_alpha_scalar(self, distance):
        """Helper to compute alpha for a distance for a single scalar."""
        if distance <= 0.0:
            return 0.0
        elif distance >= self.length():
            return 1.0
        after_index = numpy.searchsorted(
            self._point_distances, distance, side='right')
        before_index = after_index - 1

        # Linearly interpolate alpha from our (sorted) distance table.
        return (distance - self._point_distances[before_index]) / (
            self._point_distances[after_index] -
            self._point_distances[before_index]) * (1.0 / (
                len(self._point_distances) - 1.0)) + float(before_index) / (
                    len(self._point_distances) - 1.0)

    def length(self):
        """Returns the length of the spline (in meters)"""
        return self._point_distances[-1]

    # TODO(austin): need a better name...
    def xy(self, distance):
        """Returns the xy position as a function of distance."""
        return spline(self.distance_to_alpha(distance), self._control_points)

    # TODO(austin): need a better name...
    def dxy(self, distance):
        """Returns the xy velocity as a function of distance."""
        dspline_point = dspline(
            self.distance_to_alpha(distance), self._control_points)
        return dspline_point / numpy.linalg.norm(dspline_point, axis=0)

    # TODO(austin): need a better name...
    def ddxy(self, distance):
        """Returns the xy acceleration as a function of distance."""
        alpha = self.distance_to_alpha(distance)
        dspline_points = dspline(alpha, self._control_points)
        ddspline_points = ddspline(alpha, self._control_points)

        norm = numpy.linalg.norm(
            dspline_points, axis=0)**2.0

        return ddspline_points / norm - numpy.multiply(
            dspline_points, (numpy.array(dspline_points)[0, :] *
                             numpy.array(ddspline_points)[0, :] +
                             numpy.array(dspline_points)[1, :] *
                             numpy.array(ddspline_points)[1, :]) / (norm**2.0))

    def theta(self, distance, dspline_points=None):
        """Returns the heading as a function of distance."""
        return spline_theta(
            self.distance_to_alpha(distance),
            self._control_points,
            dspline_points=dspline_points)

    def dtheta(self, distance, dspline_points=None, ddspline_points=None):
        """Returns the angular velocity as a function of distance."""
        alpha = self.distance_to_alpha(distance)
        if dspline_points is None:
            dspline_points = dspline(alpha, self._control_points)
        if ddspline_points is None:
            ddspline_points = ddspline(alpha, self._control_points)

        dtheta_points = dspline_theta(alpha, self._control_points,
                                      dspline_points, ddspline_points)

        return dtheta_points / numpy.linalg.norm(dspline_points, axis=0)

    def ddtheta(self,
                distance,
                dspline_points=None,
                ddspline_points=None,
                dddspline_points=None):
        """Returns the angular acceleration as a function of distance."""
        alpha = self.distance_to_alpha(distance)
        if dspline_points is None:
            dspline_points = dspline(alpha, self._control_points)
        if ddspline_points is None:
            ddspline_points = ddspline(alpha, self._control_points)
        if dddspline_points is None:
            dddspline_points = dddspline(alpha, self._control_points)

        dtheta_points = dspline_theta(alpha, self._control_points,
                                      dspline_points, ddspline_points)
        ddtheta_points = ddspline_theta(alpha, self._control_points,
                                        dspline_points, ddspline_points,
                                        dddspline_points)

        # TODO(austin): Factor out the d^alpha/dd^2.
        return ddtheta_points / numpy.linalg.norm(
            dspline_points, axis=0)**2.0 - numpy.multiply(
                dtheta_points, (numpy.array(dspline_points)[0, :] *
                                numpy.array(ddspline_points)[0, :] +
                                numpy.array(dspline_points)[1, :] *
                                numpy.array(ddspline_points)[1, :]) /
                ((numpy.array(dspline_points)[0, :]**2.0 +
                  numpy.array(dspline_points)[1, :]**2.0)**2.0))


def integrate_accel_for_distance(f, v, x, dx):
    # Use a trick from
    # https://www.johndcook.com/blog/2012/02/21/care-and-treatment-of-singularities/
    #
    # We want to calculate:
    #   v0 + (integral of dv/dt = f(x, v) from x to x + dx); noting that dv/dt
    #   is expressed in t, not distance, so we want to do the integral of
    #   dv/dx = f(x, v) / v.
    #
    # Because v can be near zero at the start of the integral (but because f is
    # nonnegative, v will never go to zero), but the integral should still be
    # valid, we follow the suggestion and instead calculate
    # v0 + integral((f(x, v) - f(x0, v0)) / v) + integral(f(x0, v0) / v).
    #
    # Using a0 = f(x0, v0), we get the second term as
    #   integral((f(x, v) - a0) / v)
    # where when v is zero we will also be at x0/v0 (because v can only start
    # at zero, not go to zero).
    #
    # The second term, integral(a0 / v) requires an approximation.--in
    # this case, that dv/dt is constant. Thus, we have
    #  integral(a0 / sqrt(v0^2 + 2*a0*x)) = sqrt(2*a0*dx + v0^2) - sqrt(v0^2)
    #                                     = sqrt(2 * a0 * dx * v0^2) - v0.
    #
    # Because the RungeKutta function returns v0 + the integral, this
    # gives the statements below.

    a0 = f(x, v)

    def integrablef(t, y):
        # Since we know that a0 == a(0) and that they are asymptotically the
        # same at 0, we know that the limit is 0 at 0.  This is true because
        # when starting from a stop, under sane accelerations, we can assume
        # that we will start with a constant acceleration.  So, hard-code it.
        if numpy.abs(y) < 1e-6:
            return 0.0
        return (f(t, y) - a0) / y

    return (RungeKutta(integrablef, v, x, dx) - v) + numpy.sqrt(2.0 * a0 * dx + v * v)


class Trajectory(object):
    def __init__(self, path, drivetrain, longitudal_accel, lateral_accel, distance_count):
        self._path = path
        self._drivetrain = drivetrain
        self.distances = numpy.linspace(0.0,
                                        self._path.length(), distance_count)
        self._longitudal_accel = longitudal_accel
        self._lateral_accel = lateral_accel

        self._B_inverse = numpy.linalg.inv(self._drivetrain.B_continuous)

    def create_plan(self, vmax):
        vmax = 10.0
        plan = numpy.array(numpy.zeros((len(self.distances), )))
        plan.fill(vmax)
        return plan

    def lateral_velocity_curvature(self, distance):
        return numpy.sqrt(self._lateral_accel /
                          numpy.linalg.norm(self._path.ddxy(distance)))

    def lateral_accel_pass(self, plan):
        plan = plan.copy()
        # TODO(austin): This appears to be doing nothing.
        for i, distance in enumerate(self.distances):
            plan[i] = min(plan[i], self.lateral_velocity_curvature(distance))
        return plan

    def compute_K345(self, current_dtheta, current_ddtheta):
        # We've now got the equation:
        #     K2 * d^2x/dt^2 + K1 (dx/dt)^2 = A * K2 * dx/dt + B * U
        K1 = numpy.matrix(
            [[-self._drivetrain.robot_radius_l * current_ddtheta],
             [self._drivetrain.robot_radius_r * current_ddtheta]])
        K2 = numpy.matrix(
            [[1.0 - self._drivetrain.robot_radius_l * current_dtheta],
             [1.0 + self._drivetrain.robot_radius_r * current_dtheta]])

        # Now, rephrase it as K5 a + K3 v^2 + K4 v = U
        K3 = self._B_inverse * K1
        K4 = -self._B_inverse * self._drivetrain.A_continuous * K2
        K5 = self._B_inverse * K2
        return K3, K4, K5

    def curvature_voltage_pass(self, plan):
        plan = plan.copy()
        for i, distance in enumerate(self.distances):
            current_ddtheta = self._path.ddtheta(distance)[0]
            current_dtheta = self._path.dtheta(distance)[0]
            # We've now got the equation:
            #     K2 * d^2x/dt^2 + K1 (dx/dt)^2 = A * K2 * dx/dt + B * U
            # Now, rephrase it as K3 v^2 + K4 v = U
            K3, K4, K5 = self.compute_K345(current_dtheta, current_ddtheta)
            # But, we are going to assume that d^2x/dt^2 = 0

            x = []
            for a, b in [(K3[0, 0], K4[0, 0]), (K3[1, 0], K4[1, 0])]:
                for c in [12.0, -12.0]:
                    middle = b * b - 4.0 * a * c
                    if middle >= 0.0:
                        x.append((-b + numpy.sqrt(middle)) / (2.0 * a))
                        x.append((-b - numpy.sqrt(middle)) / (2.0 * a))

            maxx = 0.0
            for newx in x:
                if newx < 0.0:
                    continue
                U = (K3 * newx * newx + K4 * newx).T
                # TODO(austin): We know that one of these *will* be +-12.0.  Only
                # check the other one.
                if not (numpy.abs(U) > 12.0 + 1e-6).any():
                    maxx = max(newx, maxx)

            if maxx == 0.0:
                print('Could not solve')
                return None
            plan[i] = min(plan[i], maxx)
            pass
        return plan

    def forward_acceleration(self, x, v):
        current_ddtheta = self._path.ddtheta(x)[0]
        current_dtheta = self._path.dtheta(x)[0]
        # We've now got the equation:
        #     K2 * d^2x/dt^2 + K1 (dx/dt)^2 = A * K2 * dx/dt + B * U
        # Now, rephrase it as K5 a + K3 v^2 + K4 v = U
        K3, K4, K5 = self.compute_K345(current_dtheta, current_ddtheta)

        C = K3 * v * v + K4 * v
        # Note: K345 are not quite constant over the step, but we are going
        # to assume they are for now.
        accelerations = [(12.0 - C[0, 0]) / K5[0, 0], (12.0 - C[1, 0]) /
                         K5[1, 0], (-12.0 - C[0, 0]) / K5[0, 0],
                         (-12.0 - C[1, 0]) / K5[1, 0]]
        maxa = 0.0
        for a in accelerations:
            if a < 0.0:
                continue

            U = K5 * a + K3 * v * v + K4 * v
            if not (numpy.abs(U) > 12.0 + 1e-6).any():
                maxa = max(maxa, a)

        lateral_accel = v * v * numpy.linalg.norm(self._path.ddxy(x))
        # Constrain the longitudinal acceleration to keep in a pseudo friction
        # circle.  This will make it so we don't floor it while in a turn and
        # cause extra wheel slip.
        long_accel = numpy.sqrt(1.0 - (lateral_accel / self._lateral_accel)**
                                2.0) * self._longitudal_accel
        return min(long_accel, maxa)

    def forward_pass(self, plan):
        plan = plan.copy()
        for i, distance in enumerate(self.distances):
            if i == len(self.distances) - 1:
                break

            plan[i + 1] = min(
                plan[i + 1],
                integrate_accel_for_distance(
                    self.forward_acceleration, plan[i], self.distances[i],
                    self.distances[i + 1] - self.distances[i]))
        return plan

    def backward_acceleration(self, x, v):
        # TODO(austin): Forwards and backwards are quite similar.  Can we
        # factor this out?
        current_ddtheta = self._path.ddtheta(x)[0]
        current_dtheta = self._path.dtheta(x)[0]
        # We've now got the equation:
        #     K2 * d^2x/dt^2 + K1 (dx/dt)^2 = A * K2 * dx/dt + B * U
        # Now, rephrase it as K5 a + K3 v^2 + K4 v = U
        K3, K4, K5 = self.compute_K345(current_dtheta, current_ddtheta)

        C = K3 * v * v + K4 * v
        # Note: K345 are not quite constant over the step, but we are going
        # to assume they are for now.
        accelerations = [(12.0 - C[0, 0]) / K5[0, 0], (12.0 - C[1, 0]) /
                         K5[1, 0], (-12.0 - C[0, 0]) / K5[0, 0],
                         (-12.0 - C[1, 0]) / K5[1, 0]]
        mina = 0.0
        for a in accelerations:
            if a > 0.0:
                continue

            U = K5 * a + K3 * v * v + K4 * v
            if not (numpy.abs(U) > 12.0 + 1e-6).any():
                mina = min(mina, a)

        lateral_accel = v * v * numpy.linalg.norm(self._path.ddxy(x))
        # Constrain the longitudinal acceleration to keep in a pseudo friction
        # circle.  This will make it so we don't floor it while in a turn and
        # cause extra wheel slip.
        long_accel = -numpy.sqrt(1.0 - (lateral_accel / self._lateral_accel)**
                                 2.0) * self._longitudal_accel
        return max(long_accel, mina)

    def backward_pass(self, plan):
        plan = plan.copy()
        for i, distance in reversed(list(enumerate(self.distances))):
            if i == 0:
                break

            plan[i - 1] = min(
                plan[i - 1],
                integrate_accel_for_distance(
                    self.backward_acceleration, plan[i], self.distances[i],
                    self.distances[i - 1] - self.distances[i]))
        return plan

    # TODO(austin): The plan should probably not be passed in...
    def ff_voltage(self, plan, distance):
        if distance < self.distances[1]:
            after_index = 1
            before_index = after_index - 1
            if distance < self.distances[0]:
                distance = 0.0
        elif distance > self.distances[-2]:
            after_index = len(self.distances) - 1
            before_index = after_index - 1
            if distance > self.distances[-1]:
                distance = self.distances[-1]
        else:
            after_index = numpy.searchsorted(
                self.distances, distance, side='right')
            before_index = after_index - 1

        vforwards = integrate_accel_for_distance(
            self.forward_acceleration, plan[before_index],
            self.distances[before_index],
            distance - self.distances[before_index])
        vbackward = integrate_accel_for_distance(
            self.backward_acceleration, plan[after_index],
            self.distances[after_index],
            distance - self.distances[after_index])

        vcurvature = self.lateral_velocity_curvature(distance)

        if vcurvature < vforwards and vcurvature < vbackward:
            accel = 0
            velocity = vcurvature
        elif vforwards < vbackward:
            velocity = vforwards
            accel = self.forward_acceleration(distance, velocity)
        else:
            velocity = vbackward
            accel = self.backward_acceleration(distance, velocity)

        current_ddtheta = self._path.ddtheta(distance)[0]
        current_dtheta = self._path.dtheta(distance)[0]
        # TODO(austin): Factor these out.
        # We've now got the equation:
        #     K2 * d^2x/dt^2 + K1 (dx/dt)^2 = A * K2 * dx/dt + B * U
        # Now, rephrase it as K5 a + K3 v^2 + K4 v = U
        K3, K4, K5 = self.compute_K345(current_dtheta, current_ddtheta)

        U = K5 * accel + K3 * velocity * velocity + K4 * velocity
        return U

def main(argv):
    # Build up the control point matrix
    start = numpy.matrix([[0.0, 0.0]]).T
    c1 = numpy.matrix([[0.5, 0.0]]).T
    c2 = numpy.matrix([[0.5, 1.0]]).T
    end = numpy.matrix([[1.0, 1.0]]).T
    control_points = numpy.hstack((start, c1, c2, end))

    # The alphas to plot
    alphas = numpy.linspace(0.0, 1.0, 1000)

    # Compute x, y and the 3 derivatives
    spline_points = spline(alphas, control_points)
    dspline_points = dspline(alphas, control_points)
    ddspline_points = ddspline(alphas, control_points)
    dddspline_points = dddspline(alphas, control_points)

    # Compute theta and the two derivatives
    theta = spline_theta(alphas, control_points, dspline_points=dspline_points)
    dtheta = dspline_theta(
        alphas, control_points, dspline_points=dspline_points)
    ddtheta = ddspline_theta(
        alphas,
        control_points,
        dspline_points=dspline_points,
        dddspline_points=dddspline_points)

    # Plot the control points and the spline.
    pylab.figure()
    pylab.plot(
        numpy.array(control_points)[0, :],
        numpy.array(control_points)[1, :],
        '-o',
        label='control')
    pylab.plot(
        numpy.array(spline_points)[0, :],
        numpy.array(spline_points)[1, :],
        label='spline')
    pylab.legend()

    # For grins, confirm that the double integral of the acceleration (with
    # respect to the spline parameter) matches the position.  This lets us
    # confirm that the derivatives are consistent.
    xint_plot = numpy.matrix(numpy.zeros((2, len(alphas))))
    dxint_plot = xint_plot.copy()
    xint = spline_points[:, 0].copy()
    dxint = dspline_points[:, 0].copy()
    xint_plot[:, 0] = xint
    dxint_plot[:, 0] = dxint
    for i in range(len(alphas) - 1):
        xint += (alphas[i + 1] - alphas[i]) * dxint
        dxint += (alphas[i + 1] - alphas[i]) * ddspline_points[:, i]
        xint_plot[:, i + 1] = xint
        dxint_plot[:, i + 1] = dxint

    # Integrate up the spline velocity and heading to confirm that given a
    # velocity (as a function of the spline parameter) and angle, we will move
    # from the starting point to the ending point.
    thetaint_plot = numpy.zeros((len(alphas),))
    thetaint = theta[0]
    dthetaint_plot = numpy.zeros((len(alphas),))
    dthetaint = dtheta[0]
    thetaint_plot[0] = thetaint
    dthetaint_plot[0] = dthetaint

    txint_plot = numpy.matrix(numpy.zeros((2, len(alphas))))
    txint = spline_points[:, 0].copy()
    txint_plot[:, 0] = txint
    for i in range(len(alphas) - 1):
        dalpha = alphas[i + 1] - alphas[i]
        txint += dalpha * numpy.linalg.norm(
            dspline_points[:, i]) * numpy.matrix(
                [[numpy.cos(theta[i])], [numpy.sin(theta[i])]])
        txint_plot[:, i + 1] = txint
        thetaint += dalpha * dtheta[i]
        dthetaint += dalpha * ddtheta[i]
        thetaint_plot[i + 1] = thetaint
        dthetaint_plot[i + 1] = dthetaint


    # Now plot x, dx/dalpha, ddx/ddalpha, dddx/dddalpha, and integrals thereof
    # to perform consistency checks.
    pylab.figure()
    pylab.plot(alphas, numpy.array(spline_points)[0, :], label='x')
    pylab.plot(alphas, numpy.array(xint_plot)[0, :], label='ix')
    pylab.plot(alphas, numpy.array(dspline_points)[0, :], label='dx')
    pylab.plot(alphas, numpy.array(dxint_plot)[0, :], label='idx')
    pylab.plot(alphas, numpy.array(txint_plot)[0, :], label='tix')
    pylab.plot(alphas, numpy.array(ddspline_points)[0, :], label='ddx')
    pylab.plot(alphas, numpy.array(dddspline_points)[0, :], label='dddx')
    pylab.legend()

    # Now do the same for y.
    pylab.figure()
    pylab.plot(alphas, numpy.array(spline_points)[1, :], label='y')
    pylab.plot(alphas, numpy.array(xint_plot)[1, :], label='iy')
    pylab.plot(alphas, numpy.array(dspline_points)[1, :], label='dy')
    pylab.plot(alphas, numpy.array(dxint_plot)[1, :], label='idy')
    pylab.plot(alphas, numpy.array(txint_plot)[1, :], label='tiy')
    pylab.plot(alphas, numpy.array(ddspline_points)[1, :], label='ddy')
    pylab.plot(alphas, numpy.array(dddspline_points)[1, :], label='dddy')
    pylab.legend()

    # And for theta.
    pylab.figure()
    pylab.plot(alphas, theta, label='theta')
    pylab.plot(alphas, dtheta, label='dtheta')
    pylab.plot(alphas, ddtheta, label='ddtheta')
    pylab.plot(alphas, thetaint_plot, label='thetai')
    pylab.plot(alphas, dthetaint_plot, label='dthetai')
    pylab.plot(
        alphas,
        numpy.linalg.norm(
            numpy.array(dspline_points), axis=0),
        label='velocity')

    # Now, repeat as a function of path length as opposed to alpha
    path = Path(control_points)
    distance_count = 1000
    position = path.xy(0.0)
    velocity = path.dxy(0.0)
    theta = path.theta(0.0)
    omega = path.dtheta(0.0)

    iposition_plot = numpy.matrix(numpy.zeros((2, distance_count)))
    ivelocity_plot = numpy.matrix(numpy.zeros((2, distance_count)))
    iposition_plot[:, 0] = position.copy()
    ivelocity_plot[:, 0] = velocity.copy()
    itheta_plot = numpy.zeros((distance_count, ))
    iomega_plot = numpy.zeros((distance_count, ))
    itheta_plot[0] = theta
    iomega_plot[0] = omega

    distances = numpy.linspace(0.0, path.length(), distance_count)

    for i in xrange(len(distances) - 1):
        position += velocity * (distances[i + 1] - distances[i])
        velocity += path.ddxy(distances[i]) * (distances[i + 1] - distances[i])
        iposition_plot[:, i + 1] = position
        ivelocity_plot[:, i + 1] = velocity

        theta += omega * (distances[i + 1] - distances[i])
        omega += path.ddtheta(distances[i]) * (distances[i + 1] - distances[i])
        itheta_plot[i + 1] = theta
        iomega_plot[i + 1] = omega

    pylab.figure()
    pylab.plot(distances, numpy.array(path.xy(distances))[0, :], label='x')
    pylab.plot(distances, numpy.array(iposition_plot)[0, :], label='ix')
    pylab.plot(distances, numpy.array(path.dxy(distances))[0, :], label='dx')
    pylab.plot(distances, numpy.array(ivelocity_plot)[0, :], label='idx')
    pylab.plot(distances, numpy.array(path.ddxy(distances))[0, :], label='ddx')
    pylab.legend()

    pylab.figure()
    pylab.plot(distances, numpy.array(path.xy(distances))[1, :], label='y')
    pylab.plot(distances, numpy.array(iposition_plot)[1, :], label='iy')
    pylab.plot(distances, numpy.array(path.dxy(distances))[1, :], label='dy')
    pylab.plot(distances, numpy.array(ivelocity_plot)[1, :], label='idy')
    pylab.plot(distances, numpy.array(path.ddxy(distances))[1, :], label='ddy')
    pylab.legend()

    pylab.figure()
    pylab.plot(distances, path.theta(distances), label='theta')
    pylab.plot(distances, itheta_plot, label='itheta')
    pylab.plot(distances, path.dtheta(distances), label='omega')
    pylab.plot(distances, iomega_plot, label='iomega')
    pylab.plot(distances, path.ddtheta(distances), label='alpha')
    pylab.legend()

    # TODO(austin): Start creating a velocity plan now that we have all the
    # derivitives of our spline.

    velocity_drivetrain = polydrivetrain.VelocityDrivetrainModel(
        y2018.control_loops.python.drivetrain.kDrivetrain)
    position_drivetrain = drivetrain.Drivetrain(
        y2018.control_loops.python.drivetrain.kDrivetrain)

    longitudal_accel = 3.0
    lateral_accel = 2.0

    trajectory = Trajectory(
        path,
        drivetrain=velocity_drivetrain,
        longitudal_accel=longitudal_accel,
        lateral_accel=lateral_accel,
        distance_count=500)

    vmax = numpy.inf
    vmax = 10.0
    lateral_accel_plan = trajectory.lateral_accel_pass(
        trajectory.create_plan(vmax))

    forward_accel_plan = lateral_accel_plan.copy()
    # Start and end the path stopped.
    forward_accel_plan[0] = 0.0
    forward_accel_plan[-1] = 0.0

    forward_accel_plan = trajectory.forward_pass(forward_accel_plan)

    backward_accel_plan = trajectory.backward_pass(forward_accel_plan)

    # And now, calculate the left, right voltage as a function of distance.

    # TODO(austin): Factor out the accel and decel functions so we can use them
    # to calculate voltage as a function of distance.

    pylab.figure()
    pylab.plot(trajectory.distances, lateral_accel_plan, label='accel pass')
    pylab.plot(trajectory.distances, forward_accel_plan, label='forward pass')
    pylab.plot(trajectory.distances, backward_accel_plan, label='forward pass')
    pylab.xlabel("distance along spline (m)")
    pylab.ylabel("velocity (m/s)")
    pylab.legend()

    voltages = numpy.matrix(numpy.zeros((2, len(distances))))
    for i in range(len(distances)):
        voltages[:, i] = trajectory.ff_voltage(backward_accel_plan,
                                               distances[i])

    # Now, let's integrate up the feed forwards voltage to see where we end up.
    spline_state = numpy.matrix(numpy.zeros((4, 1)))
    # x, y, theta, vl, vr
    state = numpy.matrix(numpy.zeros((5, 1)))
    dt = 0.005
    num_timesteps = 400
    spline_states = numpy.matrix(numpy.zeros((4, num_timesteps)))
    states = numpy.matrix(numpy.zeros((5, num_timesteps)))
    Us = numpy.matrix(numpy.zeros((2, num_timesteps)))
    t = numpy.array(range(num_timesteps)) * dt
    for i in range(num_timesteps):
        spline_distance = 0.5 * (spline_state[0, 0] + spline_state[2, 0])
        spline_velocity = 0.5 * (spline_state[1, 0] + spline_state[3, 0])
        def distance_spline_diffeq(t, x):
            spline_distance = 0.5 * (x[0, 0] + x[2, 0])
            spline_velocity = 0.5 * (x[1, 0] + x[3, 0])
            U = trajectory.ff_voltage(
                backward_accel_plan,
                spline_distance)
            dXdt = (position_drivetrain.A_continuous * x +
                      position_drivetrain.B_continuous * U)
            return dXdt
        spline_state = RungeKutta(distance_spline_diffeq, spline_state, i * dt, dt)

        def spline_diffeq(t, x):
            spline_distance = 0.5 * (spline_state[0, 0] + spline_state[2, 0])
            spline_velocity = 0.5 * (spline_state[1, 0] + spline_state[3, 0])
            velocity = x[3:, :]
            theta = x[2, 0]
            linear_velocity = (velocity[0, 0] + velocity[1, 0]) / 2.0
            angular_velocity = (velocity[1, 0] - velocity[0, 0]) / (
                velocity_drivetrain.robot_radius_l +
                velocity_drivetrain.robot_radius_r)
            U = trajectory.ff_voltage(
                backward_accel_plan,
                spline_distance + numpy.linalg.norm(x[0:2, 0] - state[0:2, 0]))
            accel = (velocity_drivetrain.A_continuous * velocity +
                     velocity_drivetrain.B_continuous * U)
            return numpy.matrix(
                [[numpy.cos(theta) * linear_velocity],
                 [numpy.sin(theta) * linear_velocity], [angular_velocity],
                 [accel[0, 0]], [accel[1, 0]]])

        U = trajectory.ff_voltage(backward_accel_plan, spline_distance)
        state = RungeKutta(spline_diffeq, state, i * dt, dt)
        spline_states[:, i] = spline_state
        states[:, i] = state
        Us[:, i] = U

    spline_distances = numpy.array(
        (numpy.array(spline_states)[0, :] + numpy.array(spline_states)[2, :]) /
        2.0)
    pylab.figure()
    pylab.plot(distances, numpy.array(voltages)[0, :], label='vl')
    pylab.plot(distances, numpy.array(voltages)[1, :], label='vr')
    pylab.legend()

    pylab.figure()
    pylab.plot(
        numpy.array(spline_points)[0, :],
        numpy.array(spline_points)[1, :],
        label='spline')
    pylab.plot(
        numpy.array(states)[0, :], numpy.array(states)[1, :], label="robot")
    pylab.legend()

    pylab.figure()
    pylab.plot(
        spline_distances, (numpy.array(states)[0, :] - numpy.array(
            path.xy(spline_distances))[0, :]) * 100.0,
        label='robotx_error * 100')
    pylab.plot(
        spline_distances, (numpy.array(states)[1, :] - numpy.array(
            path.xy(spline_distances))[1, :]) * 100.0,
        label='roboty_error * 100')
    pylab.plot(
        spline_distances, (numpy.array(states)[2, :] - numpy.array(
            path.theta(spline_distances))) * 100.0,
        label='robottheta_error * 100')
    pylab.plot(distances, numpy.array(voltages)[0, :], label='voltsl')
    pylab.plot(distances, numpy.array(voltages)[1, :], label='voltsr')
    pylab.legend()

    def a(_, x):
        return 2.0
        return 2.0 + 0.0001 * x

    v = 0.0
    for _ in xrange(10):
        dx = 4.0 / 10.0
        v = integrate_accel_for_distance(a, v, 0.0, dx)
    print('v', v)

    pylab.show()


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    sys.exit(main(argv))
