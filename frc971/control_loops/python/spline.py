#!/usr/bin/python

from __future__ import print_function

import numpy
import sys
from matplotlib import pylab
import glog
import gflags

"""This file is my playground for implementing spline following."""

FLAGS = gflags.FLAGS


def spline(alpha, control_points):
    """Computes a Bezier curve.

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
    """Computes the derivitive of a Bezier curve wrt alpha.

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
    """Computes the second derivitive of a Bezier curve wrt alpha.

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
    """Computes the third derivitive of a Bezier curve wrt alpha.

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
    """Computes the heading of a robot following a Bezier curve at alpha.

    Args:
        alpha: scalar or list of spline parameters to calculate the heading at.
        control_points: n x 4 matrix of control points.  n[:, 0] is the
            starting point, and n[:, 3] is the ending point.

    Returns:
        m array of spline point headings.  m is the number of points in 'alpha'.
    """
    if dspline_points is None:
        dspline_points = dspline(alphas, control_points)

    return numpy.arctan2(
        numpy.array(dspline_points)[1, :], numpy.array(dspline_points)[0, :])


def dspline_theta(alphas,
                  control_points,
                  dspline_points=None,
                  ddspline_points=None):
    """Computes the derivitive of the heading at alpha.

    This is the derivitive of spline_theta wrt alpha.

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
        dspline_points = dspline(alphas, control_points)

    if ddspline_points is None:
        ddspline_points = ddspline(alphas, control_points)

    dx = numpy.array(dspline_points)[0, :]
    dy = numpy.array(dspline_points)[1, :]

    ddx = numpy.array(ddspline_points)[0, :]
    ddy = numpy.array(ddspline_points)[1, :]

    return 1.0 / (dx**2.0 + dy**2.0) * (dx * ddy - dy * ddx)


def ddspline_theta(alphas,
                   control_points,
                   dspline_points=None,
                   ddspline_points=None,
                   dddspline_points=None):
    """Computes the second derivitive of the heading at alpha.

    This is the second derivitive of spline_theta wrt alpha.

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
        dspline_points = dspline(alphas, control_points)

    if ddspline_points is None:
        ddspline_points = ddspline(alphas, control_points)

    if dddspline_points is None:
        dddspline_points = dddspline(alphas, control_points)

    dddspline_points = dddspline(alphas, control_points)

    dx = numpy.array(dspline_points)[0, :]
    dy = numpy.array(dspline_points)[1, :]

    ddx = numpy.array(ddspline_points)[0, :]
    ddy = numpy.array(ddspline_points)[1, :]

    dddx = numpy.array(dddspline_points)[0, :]
    dddy = numpy.array(dddspline_points)[1, :]

    return -1.0 / ((dx**2.0 + dy**2.0)**2.0) * (dx * ddy - dy * ddx) * 2.0 * (
        dy * ddy + dx * ddx) + 1.0 / (dx**2.0 + dy**2.0) * (dx * dddy - dy *
                                                            dddx)


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
    dtheta = dspline_theta(alphas, control_points, dspline_points=dspline_points)
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

    # TODO(austin): Start creating a velocity plan now that we have all the
    # derivitives of our spline.

    pylab.legend()
    pylab.show()


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    sys.exit(main(argv))
