#!/usr/bin/python

# This is an initial, hacky implementation of the extended LQR paper. It's just a proof of concept,
# so don't trust it too much.

import numpy
import scipy.optimize
from matplotlib import pylab
import sys

from frc971.control_loops.python import controls

l = 100
width = 0.2
dt = 0.05
num_states = 3
num_inputs = 2
x_hat_initial = numpy.matrix([[0.10], [1.0], [0.0]])

def dynamics(X, U):
  """Calculates the dynamics for a 2 wheeled robot.

  Args:
    X, numpy.matrix(3, 1), The state.  [x, y, theta]
    U, numpy.matrix(2, 1), The input.  [left velocity, right velocity]

  Returns:
    numpy.matrix(3, 1), The derivative of the dynamics.
  """
  #return numpy.matrix([[X[1, 0]],
  #                     [X[2, 0]],
  #                     [U[0, 0]]])
  return numpy.matrix([[(U[0, 0] + U[1, 0]) * numpy.cos(X[2, 0]) / 2.0],
                       [(U[0, 0] + U[1, 0]) * numpy.sin(X[2, 0]) / 2.0],
                       [(U[1, 0] - U[0, 0]) / width]])

def RungeKutta(f, x, dt):
  """4th order RungeKutta integration of F starting at X."""
  a = f(x)
  b = f(x + dt / 2.0 * a)
  c = f(x + dt / 2.0 * b)
  d = f(x + dt * c)
  return x + dt * (a + 2.0 * b + 2.0 * c + d) / 6.0

def discrete_dynamics(X, U):
  return RungeKutta(lambda startingX: dynamics(startingX, U), X, dt)

def inverse_discrete_dynamics(X, U):
  return RungeKutta(lambda startingX: -dynamics(startingX, U), X, dt)

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
  num_states = X.shape[0]
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

def numerical_jacobian_x_x(fn, X, U):
  return numerical_jacobian_x(
      lambda X_inner, U_inner: numerical_jacobian_x(fn, X_inner, U_inner).T, X, U)

def numerical_jacobian_x_u(fn, X, U):
  return numerical_jacobian_x(
      lambda X_inner, U_inner: numerical_jacobian_u(fn, X_inner, U_inner).T, X, U)

def numerical_jacobian_u_x(fn, X, U):
  return numerical_jacobian_u(
      lambda X_inner, U_inner: numerical_jacobian_x(fn, X_inner, U_inner).T, X, U)

def numerical_jacobian_u_u(fn, X, U):
  return numerical_jacobian_u(
      lambda X_inner, U_inner: numerical_jacobian_u(fn, X_inner, U_inner).T, X, U)

# Simple implementation for a quadratic cost function.
class CostFunction:
  def __init__(self):
    self.num_states = num_states
    self.num_inputs = num_inputs
    self.dt = dt
    self.Q = numpy.matrix([[0.1, 0, 0],
                           [0, 0.6, 0],
                           [0, 0, 0.1]]) / dt / dt
    self.R = numpy.matrix([[0.40, 0],
                           [0, 0.40]]) / dt / dt

  def estimate_Q_final(self, X_hat):
    """Returns the quadraticized final Q around X_hat.

    This is calculated by evaluating partial^2 cost(X_hat) / (partial X * partial X)

    Args:
      X_hat: numpy.matrix(self.num_states, 1), The state to quadraticize around.

    Result:
      numpy.matrix(self.num_states, self.num_states)
    """
    zero_U = numpy.matrix(numpy.zeros((self.num_inputs, 1)))
    return numerical_jacobian_x_x(self.final_cost, X_hat, zero_U)

  def estimate_partial_cost_partial_x_final(self, X_hat):
    """Returns \frac{\partial cost}{\partial X}(X_hat) for the final cost.

    Args:
      X_hat: numpy.matrix(self.num_states, 1), The state to quadraticize around.

    Result:
      numpy.matrix(self.num_states, 1)
    """
    return numerical_jacobian_x(self.final_cost, X_hat, numpy.matrix(numpy.zeros((self.num_inputs, 1)))).T

  def estimate_q_final(self, X_hat):
    """Returns q evaluated at X_hat for the final cost function."""
    return self.estimate_partial_cost_partial_x_final(X_hat) - self.estimate_Q_final(X_hat) * X_hat

  def final_cost(self, X, U):
    """Computes the final cost of being at X

    Args:
      X: numpy.matrix(self.num_states, 1)
      U: numpy.matrix(self.num_inputs, 1), ignored

    Returns:
      numpy.matrix(1, 1), The quadratic cost of being at X
    """
    return X.T * self.Q * X * 1000

  def cost(self, X, U):
    """Computes the incremental cost given a position and U.

    Args:
      X: numpy.matrix(self.num_states, 1)
      U: numpy.matrix(self.num_inputs, 1)

    Returns:
      numpy.matrix(1, 1), The quadratic cost of evaluating U.
    """
    return U.T * self.R * U + X.T * self.Q * X

cost_fn_obj = CostFunction()

S_bar_t = [numpy.matrix(numpy.zeros((num_states, num_states))) for _ in range(l + 1)]
s_bar_t = [numpy.matrix(numpy.zeros((num_states, 1))) for _ in range(l + 1)]
s_scalar_bar_t = [numpy.matrix(numpy.zeros((1, 1))) for _ in range(l + 1)]

L_t = [numpy.matrix(numpy.zeros((num_inputs, num_states))) for _ in range(l + 1)]
l_t = [numpy.matrix(numpy.zeros((num_inputs, 1))) for _ in range(l + 1)]
L_bar_t = [numpy.matrix(numpy.zeros((num_inputs, num_states))) for _ in range(l + 1)]
l_bar_t = [numpy.matrix(numpy.zeros((num_inputs, 1))) for _ in range(l + 1)]

S_t = [numpy.matrix(numpy.zeros((num_states, num_states))) for _ in range(l + 1)]
s_t = [numpy.matrix(numpy.zeros((num_states, 1))) for _ in range(l + 1)]
s_scalar_t = [numpy.matrix(numpy.zeros((1, 1))) for _ in range(l + 1)]


last_x_hat_t = [numpy.matrix(numpy.zeros((num_states, 1))) for _ in range(l + 1)]

for a in range(15):
  x_hat = x_hat_initial
  u_t = L_t[0] * x_hat + l_t[0]
  S_bar_t[0] = numpy.matrix(numpy.zeros((num_states, num_states)))
  s_bar_t[0] = numpy.matrix(numpy.zeros((num_states, 1)))
  s_scalar_bar_t[0] = numpy.matrix([[0]])

  last_x_hat_t[0] = x_hat_initial

  Q_t = numerical_jacobian_x_x(cost_fn_obj.cost, x_hat_initial, u_t)
  P_t = numerical_jacobian_x_u(cost_fn_obj.cost, x_hat_initial, u_t)
  R_t = numerical_jacobian_u_u(cost_fn_obj.cost, x_hat_initial, u_t)

  q_t = numerical_jacobian_x(cost_fn_obj.cost, x_hat_initial, u_t).T - Q_t * x_hat_initial - P_t.T * u_t
  r_t = numerical_jacobian_u(cost_fn_obj.cost, x_hat_initial, u_t).T - P_t * x_hat_initial - R_t * u_t

  q_scalar_t = cost_fn_obj.cost(x_hat_initial, u_t) - 0.5 * (x_hat_initial.T * (Q_t * x_hat_initial + P_t.T * u_t) + u_t.T * (P_t * x_hat_initial + R_t * u_t)) - x_hat_initial.T * q_t - u_t.T * r_t

  start_A_t = numerical_jacobian_x(discrete_dynamics, x_hat_initial, u_t)
  start_B_t = numerical_jacobian_u(discrete_dynamics, x_hat_initial, u_t)
  x_hat_next = discrete_dynamics(x_hat_initial, u_t)
  start_c_t = x_hat_next - start_A_t * x_hat_initial - start_B_t * u_t

  B_svd_u, B_svd_sigma_diag, B_svd_v = numpy.linalg.svd(start_B_t)
  B_svd_sigma = numpy.matrix(numpy.zeros(start_B_t.shape))
  B_svd_sigma[0:B_svd_sigma_diag.shape[0], 0:B_svd_sigma_diag.shape[0]] = numpy.diag(B_svd_sigma_diag)

  B_svd_sigma_inv = numpy.matrix(numpy.zeros(start_B_t.shape)).T
  B_svd_sigma_inv[0:B_svd_sigma_diag.shape[0], 0:B_svd_sigma_diag.shape[0]] = numpy.linalg.inv(numpy.diag(B_svd_sigma_diag))
  B_svd_inv = B_svd_v.T * B_svd_sigma_inv * B_svd_u.T

  L_bar_t[1] = B_svd_inv
  l_bar_t[1] = -B_svd_inv * (start_A_t * x_hat_initial + start_c_t)

  S_bar_t[1] = L_bar_t[1].T * R_t * L_bar_t[1]

  TotalS_1 = start_B_t.T * S_t[1] * start_B_t + R_t
  Totals_1 = start_B_t.T * S_t[1] * (start_c_t + start_A_t * x_hat_initial) + start_B_t.T * s_t[1] + P_t * x_hat_initial + r_t
  Totals_scalar_1 = 0.5 * (start_c_t.T + x_hat_initial.T * start_A_t.T) * S_t[1] * (start_c_t + start_A_t * x_hat_initial) + s_scalar_t[1] + x_hat_initial.T * q_t + q_scalar_t + 0.5 * x_hat_initial.T * Q_t * x_hat_initial + (start_c_t.T + x_hat_initial.T * start_A_t.T) * s_t[1]

  optimal_u_1 = -numpy.linalg.solve(TotalS_1, Totals_1)
  optimal_x_1 = start_A_t * x_hat_initial + start_B_t * optimal_u_1 + start_c_t

  S_bar_1_eigh_eigenvalues, S_bar_1_eigh_eigenvectors = numpy.linalg.eigh(S_bar_t[1])
  S_bar_1_eigh = numpy.matrix(numpy.diag(S_bar_1_eigh_eigenvalues))
  S_bar_1_eigh_eigenvalues_stiff = S_bar_1_eigh_eigenvalues.copy()
  for i in range(S_bar_1_eigh_eigenvalues_stiff.shape[0]):
    if abs(S_bar_1_eigh_eigenvalues_stiff[i]) < 1e-8:
      S_bar_1_eigh_eigenvalues_stiff[i] = max(S_bar_1_eigh_eigenvalues_stiff) * 1.0

  #print 'eigh eigenvalues of S bar', S_bar_1_eigh_eigenvalues
  #print 'eigh eigenvectors of S bar', S_bar_1_eigh_eigenvectors.T

  #print 'S bar eig recreate', S_bar_1_eigh_eigenvectors * numpy.matrix(numpy.diag(S_bar_1_eigh_eigenvalues)) * S_bar_1_eigh_eigenvectors.T
  #print 'S bar eig recreate error', (S_bar_1_eigh_eigenvectors * numpy.matrix(numpy.diag(S_bar_1_eigh_eigenvalues)) * S_bar_1_eigh_eigenvectors.T - S_bar_t[1])

  S_bar_stiff = S_bar_1_eigh_eigenvectors * numpy.matrix(numpy.diag(S_bar_1_eigh_eigenvalues_stiff)) * S_bar_1_eigh_eigenvectors.T

  print 'Min u', -numpy.linalg.solve(TotalS_1, Totals_1)
  print 'Min x_hat', optimal_x_1
  s_bar_t[1] = -s_t[1] - (S_bar_stiff + S_t[1]) * optimal_x_1
  s_scalar_bar_t[1] = 0.5 * (optimal_u_1.T * TotalS_1 * optimal_u_1 - optimal_x_1.T * (S_bar_stiff + S_t[1]) * optimal_x_1) + optimal_u_1.T * Totals_1 - optimal_x_1.T * (s_bar_t[1] + s_t[1]) - s_scalar_t[1] + Totals_scalar_1

  print 'optimal_u_1', optimal_u_1
  print 'TotalS_1', TotalS_1
  print 'Totals_1', Totals_1
  print 'Totals_scalar_1', Totals_scalar_1
  print 'overall cost 1',  0.5 * (optimal_u_1.T * TotalS_1 * optimal_u_1) + optimal_u_1.T * Totals_1 + Totals_scalar_1
  print 'overall cost 0',  0.5 * (x_hat_initial.T * S_t[0] * x_hat_initial) +  x_hat_initial.T * s_t[0] + s_scalar_t[0]

  print 't forward 0'
  print 'x_hat_initial[ 0]: %s' % (x_hat_initial)
  print 'x_hat[%2d]: %s' % (0, x_hat.T)
  print 'x_hat_next[%2d]: %s' % (0, x_hat_next.T)
  print 'u[%2d]: %s' % (0, u_t.T)
  print ('L[ 0]: %s' % (L_t[0],)).replace('\n', '\n        ')
  print ('l[ 0]: %s' % (l_t[0],)).replace('\n', '\n        ')

  print ('A_t[%2d]: %s' % (0, start_A_t)).replace('\n', '\n         ')
  print ('B_t[%2d]: %s' % (0, start_B_t)).replace('\n', '\n         ')
  print ('c_t[%2d]: %s' % (0, start_c_t)).replace('\n', '\n         ')

  # TODO(austin): optimal_x_1 is x_hat
  x_hat = -numpy.linalg.solve((S_t[1] + S_bar_stiff), (s_t[1] + s_bar_t[1]))
  print 'new xhat', x_hat

  S_bar_t[1] = S_bar_stiff

  last_x_hat_t[1] = x_hat

  for t in range(1, l):
    print 't forward', t
    u_t = L_t[t] * x_hat + l_t[t]

    x_hat_next = discrete_dynamics(x_hat, u_t)
    A_bar_t = numerical_jacobian_x(inverse_discrete_dynamics, x_hat_next, u_t)
    B_bar_t = numerical_jacobian_u(inverse_discrete_dynamics, x_hat_next, u_t)
    c_bar_t = x_hat - A_bar_t * x_hat_next - B_bar_t * u_t

    print 'x_hat[%2d]: %s' % (t, x_hat.T)
    print 'x_hat_next[%2d]: %s' % (t, x_hat_next.T)
    print ('L[%2d]: %s' % (t, L_t[t],)).replace('\n', '\n        ')
    print ('l[%2d]: %s' % (t, l_t[t],)).replace('\n', '\n        ')
    print 'u[%2d]: %s' % (t, u_t.T)

    print ('A_bar_t[%2d]: %s' % (t, A_bar_t)).replace('\n', '\n             ')
    print ('B_bar_t[%2d]: %s' % (t, B_bar_t)).replace('\n', '\n             ')
    print ('c_bar_t[%2d]: %s' % (t, c_bar_t)).replace('\n', '\n             ')

    Q_t = numerical_jacobian_x_x(cost_fn_obj.cost, x_hat, u_t)
    P_t = numerical_jacobian_x_u(cost_fn_obj.cost, x_hat, u_t)
    R_t = numerical_jacobian_u_u(cost_fn_obj.cost, x_hat, u_t)

    q_t = numerical_jacobian_x(cost_fn_obj.cost, x_hat, u_t).T - Q_t * x_hat - P_t.T * u_t
    r_t = numerical_jacobian_u(cost_fn_obj.cost, x_hat, u_t).T - P_t * x_hat - R_t * u_t

    q_scalar_t = cost_fn_obj.cost(x_hat, u_t) - 0.5 * (x_hat.T * (Q_t * x_hat + P_t.T * u_t) + u_t.T * (P_t * x_hat + R_t * u_t)) - x_hat.T * q_t - u_t.T * r_t

    C_bar_t = B_bar_t.T * (S_bar_t[t] + Q_t) * A_bar_t + P_t * A_bar_t
    D_bar_t = A_bar_t.T * (S_bar_t[t] + Q_t) * A_bar_t
    E_bar_t = B_bar_t.T * (S_bar_t[t] + Q_t) * B_bar_t + R_t + P_t * B_bar_t + B_bar_t.T * P_t.T
    d_bar_t = A_bar_t.T * (s_bar_t[t] + q_t) + A_bar_t.T * (S_bar_t[t] + Q_t) * c_bar_t
    e_bar_t = r_t + P_t * c_bar_t + B_bar_t.T * s_bar_t[t] + B_bar_t.T * (S_bar_t[t] + Q_t) * c_bar_t

    L_bar_t[t + 1] = -numpy.linalg.inv(E_bar_t) * C_bar_t
    l_bar_t[t + 1] = -numpy.linalg.inv(E_bar_t) * e_bar_t

    S_bar_t[t + 1] = D_bar_t + C_bar_t.T * L_bar_t[t + 1]
    s_bar_t[t + 1] = d_bar_t + C_bar_t.T * l_bar_t[t + 1]
    s_scalar_bar_t[t + 1] = -0.5 * e_bar_t.T * numpy.linalg.inv(E_bar_t) * e_bar_t + 0.5 * c_bar_t.T * (S_bar_t[t] + Q_t) * c_bar_t + c_bar_t.T * s_bar_t[t] + c_bar_t.T * q_t + s_scalar_bar_t[t] + q_scalar_t

    x_hat = -numpy.linalg.solve((S_t[t + 1] + S_bar_t[t + 1]), (s_t[t + 1] + s_bar_t[t + 1]))

  S_t[l] = cost_fn_obj.estimate_Q_final(x_hat)
  s_t[l] = cost_fn_obj.estimate_q_final(x_hat)
  x_hat = -numpy.linalg.inv(S_t[l] + S_bar_t[l]) * (s_t[l] + s_bar_t[l])

  for t in reversed(range(l)):
    print 't backward', t
    # TODO(austin): I don't think we can use L_t like this here.
    # I think we are off by 1 somewhere...
    u_t = L_bar_t[t + 1] * x_hat + l_bar_t[t + 1]

    x_hat_prev = inverse_discrete_dynamics(x_hat, u_t)
    print 'x_hat[%2d]: %s' % (t, x_hat.T)
    print 'x_hat_prev[%2d]: %s' % (t, x_hat_prev.T)
    print ('L_bar[%2d]: %s' % (t + 1, L_bar_t[t + 1])).replace('\n', '\n            ')
    print ('l_bar[%2d]: %s' % (t + 1, l_bar_t[t + 1])).replace('\n', '\n            ')
    print 'u[%2d]: %s' % (t, u_t.T)
    # Now compute the linearized A, B, and C
    # Start by doing it numerically, and then optimize.
    A_t = numerical_jacobian_x(discrete_dynamics, x_hat_prev, u_t)
    B_t = numerical_jacobian_u(discrete_dynamics, x_hat_prev, u_t)
    c_t = x_hat - A_t * x_hat_prev - B_t * u_t

    print ('A_t[%2d]: %s' % (t, A_t)).replace('\n', '\n         ')
    print ('B_t[%2d]: %s' % (t, B_t)).replace('\n', '\n         ')
    print ('c_t[%2d]: %s' % (t, c_t)).replace('\n', '\n         ')

    Q_t = numerical_jacobian_x_x(cost_fn_obj.cost, x_hat_prev, u_t)
    P_t = numerical_jacobian_x_u(cost_fn_obj.cost, x_hat_prev, u_t)
    P_T_t = numerical_jacobian_u_x(cost_fn_obj.cost, x_hat_prev, u_t)
    R_t = numerical_jacobian_u_u(cost_fn_obj.cost, x_hat_prev, u_t)

    q_t = numerical_jacobian_x(cost_fn_obj.cost, x_hat_prev, u_t).T - Q_t * x_hat_prev - P_T_t * u_t
    r_t = numerical_jacobian_u(cost_fn_obj.cost, x_hat_prev, u_t).T - P_t * x_hat_prev - R_t * u_t

    q_scalar_t = cost_fn_obj.cost(x_hat_prev, u_t) - 0.5 * (x_hat_prev.T * (Q_t * x_hat_prev + P_t.T * u_t) + u_t.T * (P_t * x_hat_prev + R_t * u_t)) - x_hat_prev.T * q_t - u_t.T * r_t

    C_t = P_t + B_t.T * S_t[t + 1] * A_t
    D_t = Q_t + A_t.T * S_t[t + 1] * A_t
    E_t = R_t + B_t.T * S_t[t + 1] * B_t
    d_t = q_t + A_t.T * s_t[t + 1] + A_t.T * S_t[t + 1] * c_t
    e_t = r_t + B_t.T * s_t[t + 1] + B_t.T * S_t[t + 1] * c_t
    L_t[t] = -numpy.linalg.inv(E_t) * C_t
    l_t[t] = -numpy.linalg.inv(E_t) * e_t
    s_t[t] = d_t + C_t.T * l_t[t]
    S_t[t] = D_t + C_t.T * L_t[t]
    s_scalar_t[t] = q_scalar_t - 0.5 * e_t.T * numpy.linalg.inv(E_t) * e_t + 0.5 * c_t.T * S_t[t + 1] * c_t + c_t.T * s_t[t + 1] + s_scalar_t[t + 1]

    x_hat = -numpy.linalg.solve((S_t[t] + S_bar_t[t]), (s_t[t] + s_bar_t[t]))
    if t == 0:
      last_x_hat_t[t] = x_hat_initial
    else:
      last_x_hat_t[t] = x_hat

  x_hat_t = [x_hat_initial]

  pylab.figure('states %d' % a)
  pylab.ion()
  for dim in range(num_states):
    pylab.plot(numpy.arange(len(last_x_hat_t)),
             [x_hat_loop[dim, 0] for x_hat_loop in last_x_hat_t], marker='o', label='Xhat[%d]'%dim)
  pylab.legend()
  pylab.draw()

  pylab.figure('xy %d' % a)
  pylab.ion()
  pylab.plot([x_hat_loop[0, 0] for x_hat_loop in last_x_hat_t],
             [x_hat_loop[1, 0] for x_hat_loop in last_x_hat_t], marker='o', label='trajectory')
  pylab.legend()
  pylab.draw()

final_u_t = [numpy.matrix(numpy.zeros((num_inputs, 1))) for _ in range(l + 1)]
cost_to_go = []
cost_to_come = []
cost = []
for t in range(l):
  cost_to_go.append((0.5 * last_x_hat_t[t].T * S_t[t] * last_x_hat_t[t] + last_x_hat_t[t].T * s_t[t] + s_scalar_t[t])[0, 0])
  cost_to_come.append((0.5 * last_x_hat_t[t].T * S_bar_t[t] * last_x_hat_t[t] + last_x_hat_t[t].T * s_bar_t[t] + s_scalar_bar_t[t])[0, 0])
  cost.append(cost_to_go[-1] + cost_to_come[-1])
  final_u_t[t] = L_t[t] * last_x_hat_t[t] + l_t[t]

for t in range(l):
  A_t = numerical_jacobian_x(discrete_dynamics, last_x_hat_t[t], final_u_t[t])
  B_t = numerical_jacobian_u(discrete_dynamics, last_x_hat_t[t], final_u_t[t])
  c_t = discrete_dynamics(last_x_hat_t[t], final_u_t[t]) - A_t * last_x_hat_t[t] - B_t * final_u_t[t]
  print("Infeasability at", t, "is", ((A_t * last_x_hat_t[t] + B_t * final_u_t[t] + c_t) - last_x_hat_t[t + 1]).T)

pylab.figure('u')
samples = numpy.arange(len(final_u_t))
for i in range(num_inputs):
  pylab.plot(samples, [u[i, 0] for u in final_u_t], label='u[%d]' % i, marker='o')
  pylab.legend()

pylab.figure('cost')
cost_samples = numpy.arange(len(cost))
pylab.plot(cost_samples, cost_to_go, label='cost to go', marker='o')
pylab.plot(cost_samples, cost_to_come, label='cost to come', marker='o')
pylab.plot(cost_samples, cost, label='cost', marker='o')
pylab.legend()

pylab.ioff()
pylab.show()

sys.exit(1)
