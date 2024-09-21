#!/usr/bin/python3
"""
Control loop pole placement library.

This library will grow to support many different pole placement methods.
Currently it only supports direct pole placement.
"""

__author__ = 'Austin Schuh (austin.linux@gmail.com)'

import numpy
import scipy.linalg
import scipy.signal
import glog


class Error(Exception):
    """Base class for all control loop exceptions."""


# TODO(aschuh): dplace should take a control system object.
# There should also exist a function to manipulate laplace expressions, and
# something to plot bode plots and all that.
def dplace(A, B, poles):
    """Set the poles of (A - BF) to poles.

  Args:
    A: numpy.matrix(n x n), The A matrix.
    B: numpy.matrix(n x m), The B matrix.
    poles: array(imaginary numbers), The poles to use.  Complex conjugates poles
      must be in pairs.

  Returns:
    numpy.matrix(m x n), K
  """
    return scipy.signal.place_poles(A=A, B=B,
                                    poles=numpy.array(poles)).gain_matrix


def c2d(A, B, dt):
    """Converts from continuous time state space representation to discrete time.
     Returns (A, B).  C and D are unchanged.
     This code is copied from: scipy.signal.cont2discrete method zoh
  """

    a, b = numpy.array(A), numpy.array(B)
    # Build an exponential matrix
    em_upper = numpy.hstack((a, b))

    # Need to stack zeros under the a and b matrices
    em_lower = numpy.hstack((numpy.zeros(
        (b.shape[1], a.shape[0])), numpy.zeros((b.shape[1], b.shape[1]))))

    em = numpy.vstack((em_upper, em_lower))
    ms = scipy.linalg.expm(dt * em)

    # Dispose of the lower rows
    ms = ms[:a.shape[0], :]

    ad = ms[:, 0:a.shape[1]]
    bd = ms[:, a.shape[1]:]

    return numpy.matrix(ad), numpy.matrix(bd)


def ctrb(A, B):
    """Returns the controllability matrix.

    This matrix must have full rank for all the states to be controllable.
  """
    n = A.shape[0]
    output = B
    intermediate = B
    for i in range(0, n):
        intermediate = A * intermediate
        output = numpy.concatenate((output, intermediate), axis=1)

    return output


def dlqr(A, B, Q, R, optimal_cost_function=False):
    """Solves for the optimal lqr controller.

    x(n+1) = A * x(n) + B * u(n)
    J = sum(0, inf, x.T * Q * x + u.T * R * u)
  """

    # P = (A.T * P * A) - (A.T * P * B * numpy.linalg.inv(R + B.T * P * B) * (A.T * P.T * B).T + Q
    # X.T * P * X -> optimal cost to infinity

    P = scipy.linalg.solve_discrete_are(a=A, b=B, q=Q, r=R)
    F = numpy.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
    if optimal_cost_function:
        return F, P
    else:
        return F


def kalman(A, B, C, Q, R):
    """Solves for the steady state kalman gain and covariance matricies.

    Args:
      A, B, C: SS matricies.
      Q: The model uncertantity
      R: The measurement uncertainty

    Returns:
      KalmanGain, Covariance.
  """
    I = numpy.matrix(numpy.eye(Q.shape[0]))
    Z = numpy.matrix(numpy.zeros(Q.shape[0]))
    n = A.shape[0]
    m = C.shape[0]

    controllability_rank = numpy.linalg.matrix_rank(ctrb(A.T, C.T))
    if controllability_rank != n:
        glog.warning('Observability of %d != %d, unobservable state',
                     controllability_rank, n)

    # Compute the steady state covariance matrix.
    P_prior = scipy.linalg.solve_discrete_are(a=A.T, b=C.T, q=Q, r=R)
    S = C * P_prior * C.T + R
    K = numpy.linalg.lstsq(S.T, (P_prior * C.T).T, rcond=None)[0].T
    P = (I - K * C) * P_prior

    return K, P


def kalmd(A_continuous, B_continuous, Q_continuous, R_continuous, dt):
    """Converts a continuous time kalman filter to discrete time.

    Args:
      A_continuous: The A continuous matrix
      B_continuous: the B continuous matrix
      Q_continuous: The continuous cost matrix
      R_continuous: The R continuous matrix
      dt: Timestep

    The math for this is from:
    https://www.mathworks.com/help/control/ref/kalmd.html

    Returns:
      The discrete matrices of A, B, Q, and R.
  """
    # TODO(austin): Verify that the dimensions make sense.
    number_of_states = A_continuous.shape[0]
    number_of_inputs = B_continuous.shape[1]
    M = numpy.zeros((len(A_continuous) + number_of_inputs,
                     len(A_continuous) + number_of_inputs))
    M[0:number_of_states, 0:number_of_states] = A_continuous
    M[0:number_of_states, number_of_states:] = B_continuous
    M_exp = scipy.linalg.expm(M * dt)
    A_discrete = M_exp[0:number_of_states, 0:number_of_states]
    B_discrete = numpy.matrix(M_exp[0:number_of_states, number_of_states:])
    Q_continuous = (Q_continuous + Q_continuous.T) / 2.0
    R_continuous = (R_continuous + R_continuous.T) / 2.0
    M = numpy.concatenate((-A_continuous, Q_continuous), axis=1)
    M = numpy.concatenate(
        (M,
         numpy.concatenate(
             (numpy.matrix(numpy.zeros((number_of_states, number_of_states))),
              numpy.transpose(A_continuous)),
             axis=1)),
        axis=0)
    phi = numpy.matrix(scipy.linalg.expm(M * dt))
    phi12 = phi[0:number_of_states, number_of_states:(2 * number_of_states)]
    phi22 = phi[number_of_states:2 * number_of_states,
                number_of_states:2 * number_of_states]
    Q_discrete = phi22.T * phi12
    Q_discrete = (Q_discrete + Q_discrete.T) / 2.0
    R_discrete = R_continuous / dt
    return (A_discrete, B_discrete, Q_discrete, R_discrete)


def TwoStateFeedForwards(B, Q):
    """Computes the feed forwards constant for a 2 state controller.

  This will take the form U = Kff * (R(n + 1) - A * R(n)), where Kff is the
  feed-forwards constant.  It is important that Kff is *only* computed off
  the goal and not the feed back terms.

  Args:
    B: numpy.Matrix[num_states, num_inputs] The B matrix.
    Q: numpy.Matrix[num_states, num_states] The Q (cost) matrix.

  Returns:
    numpy.Matrix[num_inputs, num_states]
  """

    # We want to find the optimal U such that we minimize the tracking cost.
    # This means that we want to minimize
    #   (B * U - (R(n+1) - A R(n)))^T * Q * (B * U - (R(n+1) - A R(n)))
    # TODO(austin): This doesn't take into account the cost of U

    return numpy.linalg.inv(B.T * Q * B) * B.T * Q.T
