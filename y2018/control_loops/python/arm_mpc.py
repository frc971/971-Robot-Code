#!/usr/bin/python3

import numpy
import time
import scipy.optimize
from matplotlib import pylab
from frc971.control_loops.python import controls

dt = 0.05

def RungeKutta(f, x, dt):
  """4th order RungeKutta integration of F starting at X."""
  a = f(x)
  b = f(x + dt / 2.0 * a)
  c = f(x + dt / 2.0 * b)
  d = f(x + dt * c)
  return x + dt * (a + 2.0 * b + 2.0 * c + d) / 6.0


def dynamics(X, U):
  """Calculates the dynamics for a double jointed arm.

  Args:
    X, numpy.matrix(4, 1), The state.  [theta1, omega1, theta2, omega2]
    U, numpy.matrix(2, 1), The input.  [torque1, torque2]

  Returns:
    numpy.matrix(4, 1), The derivative of the dynamics.
  """
  return numpy.matrix([[X[1, 0]],
                       [U[0, 0]],
                       [X[3, 0]],
                       [U[1, 0]]])


def discrete_dynamics(X, U):
  return RungeKutta(lambda startingX: dynamics(startingX, U), X, dt)


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

def numerical_jacobian_x(fn, X, U, epsilon=1e-4):
  """Numerically estimates the jacobian around X, U in X.

  Args:
    fn: A function of X, U.
    X: numpy.matrix(num_states, 1), The state vector to take the jacobian
      around.
    U: numpy.matrix(num_inputs, 1), The input vector to take the jacobian
      around.

  Returns:
    numpy.matrix(num_states, num_states), The jacobian of fn with X as the
      variable.
  """
  num_states = X.shape[0]
  nominal = fn(X, U)
  answer = numpy.matrix(numpy.zeros((nominal.shape[0], num_states)))
  # It's more expensive, but +- epsilon will be more reliable
  for i in range(0, num_states):
    dX_plus = X.copy()
    dX_plus[i] += epsilon
    dX_minus = X.copy()
    dX_minus[i] -= epsilon
    answer[:, i] = (fn(dX_plus, U) - fn(dX_minus, U)) / epsilon / 2.0
  return answer

def numerical_jacobian_u(fn, X, U, epsilon=1e-4):
  """Numerically estimates the jacobian around X, U in U.

  Args:
    fn: A function of X, U.
    X: numpy.matrix(num_states, 1), The state vector to take the jacobian
      around.
    U: numpy.matrix(num_inputs, 1), The input vector to take the jacobian
      around.

  Returns:
    numpy.matrix(num_states, num_inputs), The jacobian of fn with U as the
      variable.
  """
  num_inputs = U.shape[0]
  nominal = fn(X, U)
  answer = numpy.matrix(numpy.zeros((nominal.shape[0], num_inputs)))
  for i in range(0, num_inputs):
    dU_plus = U.copy()
    dU_plus[i] += epsilon
    dU_minus = U.copy()
    dU_minus[i] -= epsilon
    answer[:, i] = (fn(X, dU_plus) - fn(X, dU_minus)) / epsilon / 2.0
  return answer

class Cost(object):
  def __init__(self):
    q_pos = 0.5
    q_vel = 1.65
    self.Q = numpy.matrix(numpy.diag([
        1.0 / (q_pos ** 2.0), 1.0 / (q_vel ** 2.0),
        1.0 / (q_pos ** 2.0), 1.0 / (q_vel ** 2.0)]))

    self.R = numpy.matrix(numpy.diag([1.0 / (12.0 ** 2.0),
                                      1.0 / (12.0 ** 2.0)]))

    final_A = numerical_jacobian_x(discrete_dynamics,
                                   numpy.matrix(numpy.zeros((4, 1))),
                                   numpy.matrix(numpy.zeros((2, 1))))
    final_B = numerical_jacobian_u(discrete_dynamics,
                                   numpy.matrix(numpy.zeros((4, 1))),
                                   numpy.matrix(numpy.zeros((2, 1))))
    #print 'Final A', final_A
    #print 'Final B', final_B
    K, self.S = controls.dlqr(
        final_A, final_B, self.Q, self.R, optimal_cost_function=True)
    print K
    print self.S

  def cost(self, U_array, X):
    """Computes the cost given the inital position and the U array.

    Args:
      U_array: numpy.array[N] The U coordinates.
      X: numpy.matrix[3, 1] The cartesian coordinates of the starting
          location.

    Returns:
      double, The quadratic cost of evaluating U.
    """

    X_mod = X.copy()
    U_matrix = U_from_array(U_array)
    my_cost = 0

    for U in U_matrix.T:
      # Handle a keep out zone.
      penalized_cost = 0.0
      if X_mod[0, 0] > 0.5 and X_mod[2, 0] < 0.8:
        out_of_bound1 = 0.8 - X_mod[2, 0]
        penalized_cost += 1000.0 * (out_of_bound1 ** 2.0 + 0.1 * out_of_bound1)

        out_of_bound2 = X_mod[0, 0] - 0.5
        penalized_cost += 1000.0 * (out_of_bound2 ** 2.0 + 0.1 * out_of_bound2)

      U = U.T
      my_cost += U.T * self.R * U + X_mod.T * self.Q * X_mod + penalized_cost
      X_mod = discrete_dynamics(X_mod, U)

    return my_cost + 0.5 * X_mod.T * self.S * X_mod


# TODO(austin): Add Parker's constraints in.
# TODO(austin): Real dynamics from dad.
# TODO(austin): Look at grid resolution needed.  Grid a section in open space
#   and look at curvature of the grid.  Try motions using the grid.
#
#   https://docs.scipy.org/doc/scipy-0.16.1/reference/generated/scipy.interpolate.RegularGridInterpolator.html
# Look at a C++ version so we can build a large space.

# TODO(austin): Larger timesteps further out?


if __name__ == '__main__':
  X = numpy.matrix([[1.0],
                    [0.0],
                    [1.0],
                    [0.0]])
  theta1_array = []
  omega1_array = []
  theta2_array = []
  omega2_array = []

  cost_array = []

  time_array = []
  u0_array = []
  u1_array = []

  num_steps = 40

  cost_obj = Cost()

  U_array = numpy.zeros((num_steps * 2))
  for i in range(400):
    print('Iteration', i)
    start_time = time.time()
    # Solve the NMPC
    U_array, fx, _, _, _ = scipy.optimize.fmin_slsqp(
        cost_obj.cost, U_array.copy(), bounds = [(-12, 12), (-12, 12)] * num_steps,
        args=(X,), iter=500, full_output=True)
    U_matrix = U_from_array(U_array)

    # Save variables for plotting.
    cost_array.append(fx[0, 0])
    u0_array.append(U_matrix[0, 0])
    u1_array.append(U_matrix[1, 0])

    theta1_array.append(X[0, 0])
    omega1_array.append(X[1, 0])
    theta2_array.append(X[2, 0])
    omega2_array.append(X[3, 0])

    time_array.append(i * dt)

    # Simulate the dynamics
    X = discrete_dynamics(X, U_matrix[:, 0])

    U_array = U_to_array(numpy.hstack((U_matrix[:, 1:], numpy.matrix([[0], [0]]))))
    print 'Took %f to evaluate' % (time.time() - start_time)

    if fx < 1e-05:
      print('Cost is', fx, 'after', i, 'cycles, finishing early')
      break

  # Plot
  pylab.figure('trajectory')
  pylab.plot(theta1_array, theta2_array, label = 'trajectory')

  fig, ax1 = fig, ax1 = pylab.subplots()
  fig.suptitle('time')
  ax1.plot(time_array, theta1_array, label='theta1')
  ax1.plot(time_array, omega1_array, label='omega1')
  ax1.plot(time_array, theta2_array, label = 'theta2')
  ax1.plot(time_array, omega2_array, label='omega2')
  ax2 = ax1.twinx()
  ax2.plot(time_array, cost_array, 'k', label='cost')
  ax1.legend(loc=2)
  ax2.legend()

  pylab.figure('u')
  pylab.plot(time_array, u0_array, label='u0')
  pylab.plot(time_array, u1_array, label='u1')
  pylab.legend()

  pylab.show()
