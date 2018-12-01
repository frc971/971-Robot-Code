#!/usr/bin/python

from __future__ import print_function

from matplotlib import pylab
import gflags
import glog
import numpy
import scipy
import scipy.integrate
import sys

"""This file is my playground for implementing spline following.

All splines here are cubic bezier splines.  See
  https://en.wikipedia.org/wiki/B%C3%A9zier_curve for more details.
"""

FLAGS = gflags.FLAGS


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
    itheta_plot = numpy.zeros((distance_count,))
    iomega_plot = numpy.zeros((distance_count,))
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

    pylab.show()


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    sys.exit(main(argv))
