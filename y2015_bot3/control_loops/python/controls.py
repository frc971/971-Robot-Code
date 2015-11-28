#!/usr/bin/python

"""
Control loop pole placement library.

This library will grow to support many different pole placement methods.
Currently it only supports direct pole placement.
"""

__author__ = 'Austin Schuh (austin.linux@gmail.com)'

import numpy
import slycot
import scipy.signal.cont2discrete

class Error (Exception):
  """Base class for all control loop exceptions."""


class PolePlacementError(Error):
  """Exception raised when pole placement fails."""


# TODO(aschuh): dplace should take a control system object.
# There should also exist a function to manipulate laplace expressions, and
# something to plot bode plots and all that.
def dplace(A, B, poles, alpha=1e-6):
  """Set the poles of (A - BF) to poles.

  Args:
    A: numpy.matrix(n x n), The A matrix.
    B: numpy.matrix(n x m), The B matrix.
    poles: array(imaginary numbers), The poles to use.  Complex conjugates poles
      must be in pairs.

  Raises:
    ValueError: Arguments were the wrong shape or there were too many poles.
    PolePlacementError: Pole placement failed.

  Returns:
    numpy.matrix(m x n), K
  """
  # See http://www.icm.tu-bs.de/NICONET/doc/SB01BD.html for a description of the
  # fortran code that this is cleaning up the interface to.
  n = A.shape[0]
  if A.shape[1] != n:
    raise ValueError("A must be square")
  if B.shape[0] != n:
    raise ValueError("B must have the same number of states as A.")
  m = B.shape[1]

  num_poles = len(poles)
  if num_poles > n:
    raise ValueError("Trying to place more poles than states.")

  out = slycot.sb01bd(n=n,
                      m=m,
                      np=num_poles,
                      alpha=alpha,
                      A=A,
                      B=B,
                      w=numpy.array(poles),
                      dico='D')

  A_z = numpy.matrix(out[0])
  num_too_small_eigenvalues = out[2]
  num_assigned_eigenvalues = out[3]
  num_uncontrollable_eigenvalues = out[4]
  K = numpy.matrix(-out[5])
  Z = numpy.matrix(out[6])

  if num_too_small_eigenvalues != 0:
    raise PolePlacementError("Number of eigenvalues that are too small "
                             "and are therefore unmodified is %d." %
                             num_too_small_eigenvalues)
  if num_assigned_eigenvalues != num_poles:
    raise PolePlacementError("Did not place all the eigenvalues that were "
                             "requested. Only placed %d eigenvalues." %
                             num_assigned_eigenvalues)
  if num_uncontrollable_eigenvalues != 0:
    raise PolePlacementError("Found %d uncontrollable eigenvlaues." %
                             num_uncontrollable_eigenvalues)

  return K


def c2d(A, B, dt):
  """Converts from continuous time state space representation to discrete time.
     Returns (A, B).  C and D are unchanged."""

  ans_a, ans_b, _, _, _ = scipy.signal.cont2discrete((A, B, None, None), dt)
  return numpy.matrix(ans_a), numpy.matrix(ans_b)

def ctrb(A, B):
  """Returns the controllability matrix.

    This matrix must have full rank for all the states to be controllable.
  """
  n = A.shape[0]
  output = B
  intermediate = B
  for i in xrange(0, n):
    intermediate = A * intermediate
    output = numpy.concatenate((output, intermediate), axis=1)

  return output

def dlqr(A, B, Q, R):
  """Solves for the optimal lqr controller.

    x(n+1) = A * x(n) + B * u(n)
    J = sum(0, inf, x.T * Q * x + u.T * R * u)
  """

  # P = (A.T * P * A) - (A.T * P * B * numpy.linalg.inv(R + B.T * P *B) * (A.T * P.T * B).T + Q

  P, rcond, w, S, T = slycot.sb02od(
      n=A.shape[0], m=B.shape[1], A=A, B=B, Q=Q, R=R, dico='D')

  F = numpy.linalg.inv(R + B.T * P *B) * B.T * P * A
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
  P = Q

  I = numpy.matrix(numpy.eye(P.shape[0]))
  At = A.T
  Ct = C.T
  i = 0

  while True:
    last_P = P
    P_prior = A * P * At + Q
    S = C * P_prior * Ct + R
    K = P_prior * Ct * numpy.linalg.inv(S)
    P = (I - K * C) * P_prior

    diff = P - last_P
    i += 1
    if numpy.linalg.norm(diff) / numpy.linalg.norm(P) < 1e-9:
      break

  return K, P
