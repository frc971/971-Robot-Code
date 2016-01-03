#!/usr/bin/python3

import numpy
import scipy.optimize
from matplotlib import pylab

dt = 0.05

# This module computes a NMPC which solves the non-holonomic point
# stabilization problem for a mobile robot.  The algorithm is currently
# too slow to run on a robot in real-time, but is fast enough to use offline
# and to use to compare different parameters for NMPCs.
#
# Inital algorithm from http://www.ece.ufrgs.br/~fetter/icma05_608.pdf

def cartesian_to_polar(X_cartesian):
  """Converts a cartesian coordinate to polar.

  Args:
    X_cartesian, numpy.matrix[3, 1] with x, y, theta as rows.

  Returns:
    numpy.matrix[3, 1] with e, phi, alpha as rows.
  """
  phi = numpy.arctan2(X_cartesian[1, 0], X_cartesian[0, 0])
  return numpy.matrix([[numpy.hypot(X_cartesian[0, 0], X_cartesian[1, 0])],
                       [phi],
                       [X_cartesian[2, 0] - phi]])

def polar_to_cartesian(X_polar):
  """Converts a polar coordinate to cartesian.

  Args:
    X_polar, numpy.matrix[3, 1] with e, phi, alpha as rows.

  Returns:
    numpy.matrix[3, 1] with x, y, theta as rows.
  """
  return numpy.matrix([[X_polar[0, 0] * numpy.cos(X_polar[1, 0])],
                       [X_polar[0, 0] * numpy.sin(X_polar[1, 0])],
                       [X_polar[1, 0] + X_polar[2, 0]]])

def simulate_dynamics(X_cartesian, U):
  """Calculates the robot location after 1 timestep.

  Args:
    X_cartesian, numpy.matrix[3, 1] with the starting location in
        cartesian coordinates.
    U, numpy.matrix[2, 1] with velocity, angular_velocity as rows.

  Returns:
    numpy.matrix[3, 1] with the cartesian coordinate.
  """
  X_cartesian += numpy.matrix(
      [[U[0, 0] * numpy.cos(X_cartesian[2, 0]) * dt],
       [U[0, 0] * numpy.sin(X_cartesian[2, 0]) * dt],
       [U[1, 0] * dt]])

  return X_cartesian

def U_from_array(U_array):
  """Converts the U array from the optimizer to a bunch of column vectors.

  Args:
    U_array, numpy.array[N] The U coordinates in v, av, v, av, ...

  Returns:
    numpy.matrix[2, N/2] with [[v, v, v, ...], [av, av, av, ...]]
  """
  return numpy.matrix(U_array).reshape((2, -1), order='F')

def U_to_array(U_matrix):
  """Converts the U matrix to the U array for the optimizer.

  Args:
    U_matrix, numpy.matrix[2, N/2] with [[v, v, v, ...], [av, av, av, ...]]

  Returns:
    numpy.array[N] The U coordinates in v, av, v, av, ...
  """
  return numpy.array(U_matrix.reshape((1, -1), order='F'))

def cost(U_array, X_cartesian):
  """Computes the cost given the inital position and the U array.

  Args:
    U_array: numpy.array[N] The U coordinates.
    X_cartesian: numpy.matrix[3, 1] The cartesian coordinates of the starting
        location.

  Returns:
    double, The quadratic cost of evaluating U.
  """
  X_cartesian_mod = X_cartesian.copy()
  U_matrix = U_from_array(U_array)
  my_cost = 0
  Q = numpy.matrix([[0.01, 0, 0],
                    [0, 0.01, 0],
                    [0, 0, 0.01]]) / dt / dt
  R = numpy.matrix([[0.001, 0],
                    [0, 0.001]]) / dt / dt
  for U in U_matrix.T:
    # TODO(austin): Let it go to the point from either side.
    U = U.T
    X_cartesian_mod = simulate_dynamics(X_cartesian_mod, U)
    X_current_polar = cartesian_to_polar(X_cartesian_mod)
    my_cost += U.T * R * U + X_current_polar.T * Q * X_current_polar

  return my_cost

if __name__ == '__main__':
  X_cartesian = numpy.matrix([[-1.0],
                              [-1.0],
                              [0.0]])
  x_array = []
  y_array = []
  theta_array = []

  e_array = []
  phi_array = []
  alpha_array = []

  cost_array = []

  time_array = []
  u0_array = []
  u1_array = []

  num_steps = 20

  U_array = numpy.zeros((num_steps * 2))
  for i in range(400):
    print('Iteration', i)
    # Solve the NMPC
    U_array, fx, _, _, _ = scipy.optimize.fmin_slsqp(
        cost, U_array.copy(), bounds = [(-1, 1), (-7, 7)] * num_steps,
        args=(X_cartesian,), iter=500, full_output=True)
    U_matrix = U_from_array(U_array)

    # Simulate the dynamics
    X_cartesian = simulate_dynamics(X_cartesian, U_matrix[:, 0])

    # Save variables for plotting.
    cost_array.append(fx[0, 0])
    u0_array.append(U_matrix[0, 0])
    u1_array.append(U_matrix[1, 0])
    x_array.append(X_cartesian[0, 0])
    y_array.append(X_cartesian[1, 0])
    theta_array.append(X_cartesian[2, 0])
    time_array.append(i * dt)
    X_polar = cartesian_to_polar(X_cartesian)
    e_array.append(X_polar[0, 0])
    phi_array.append(X_polar[1, 0])
    alpha_array.append(X_polar[2, 0])

    U_array = U_to_array(numpy.hstack((U_matrix[:, 1:], numpy.matrix([[0], [0]]))))

    if fx < 1e-05:
      print('Cost is', fx, 'after', i, 'cycles, finishing early')
      break

  # Plot
  pylab.figure('xy')
  pylab.plot(x_array, y_array, label = 'trajectory')

  pylab.figure('time')
  pylab.plot(time_array, x_array, label='x')
  pylab.plot(time_array, y_array, label='y')
  pylab.plot(time_array, theta_array, label = 'theta')
  pylab.plot(time_array, e_array, label='e')
  pylab.plot(time_array, phi_array, label='phi')
  pylab.plot(time_array, alpha_array, label='alpha')
  pylab.plot(time_array, cost_array, label='cost')
  pylab.legend()

  pylab.figure('u')
  pylab.plot(time_array, u0_array, label='u0')
  pylab.plot(time_array, u1_array, label='u1')
  pylab.legend()

  pylab.show()
