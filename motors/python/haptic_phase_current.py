#!/usr/bin/python3

import numpy
from matplotlib import pylab
import scipy.integrate
from frc971.control_loops.python import controls
import time
import operator

K1 = 1.81e04
K2 = 0.0

# Make the amplitude of the fundamental 1 for ease of playing with.
K2 /= K1
K1 = 1

vcc = 14.0  # volts
R_motor = 0.1073926073926074  # ohms for the motor
R = R_motor + 0.080 + 0.02  # motor + fets + wires ohms for system

L = 80.0 * 1e-6  # Henries
M = L / 10.0

Kv = 37.6  # rad/s/volt, where the voltage is measured from the neutral to the phase.
J = 0.0000007

R_shunt = 0.0003

# RC circuit for current sense filtering.
R_sense1 = 768.0
R_sense2 = 1470.0
C_sense = 10.0 * 1e-9

# So, we measured the inductance by switching between ~5 and ~20 amps through
# the motor.
# We then looked at the change in voltage that should give us (assuming duty
# cycle * vin), and divided it by the corresponding change in current.

# We then looked at the amount of time it took to decay the current to 1/e
# That gave us the inductance.

# Overrides for experiments
J = J * 10.0

# Firing phase A -> 0.0
# Firing phase B -> - numpy.pi * 2.0 / 3.0
# Firing phase C -> + numpy.pi * 2.0 / 3.0

hz = 20000.0

#switching_pattern = 'front'
switching_pattern = 'centered'
#switching_pattern = 'rear'
#switching_pattern = 'centered front shifted'
#switching_pattern = 'anticentered'

Vconv = numpy.matrix([[2.0, -1.0, -1.0],
                      [-1.0, 2.0, -1.0],
                      [-1.0, -1.0, 2.0]]) / 3.0

def f_single(theta):
  return K1 * numpy.sin(theta) + K2 * numpy.sin(theta * 5)

def g_single(theta):
  return K1 * numpy.sin(theta) - K2 * numpy.sin(theta * 5)

def gdot_single(theta):
  """Derivitive of the current.

  Must be multiplied by omega externally.
  """
  return K1 * numpy.cos(theta) - 5.0 * K2 * numpy.cos(theta * 5.0)

f = numpy.vectorize(f_single, otypes=(numpy.float,))
g = numpy.vectorize(g_single, otypes=(numpy.float,))
gdot = numpy.vectorize(gdot_single, otypes=(numpy.float,))

def torque(theta):
  return f(theta) * g(theta)

def phase_a(function, theta):
  return function(theta)

def phase_b(function, theta):
  return function(theta + 2 * numpy.pi / 3)

def phase_c(function, theta):
  return function(theta + 4 * numpy.pi / 3)

def phases(function, theta):
  return numpy.matrix([[phase_a(function, theta)],
                       [phase_b(function, theta)],
                       [phase_c(function, theta)]])

def all_phases(function, theta_range):
  return (phase_a(function, theta_range) +
          phase_b(function, theta_range) +
          phase_c(function, theta_range))

theta_range = numpy.linspace(start=0, stop=4 * numpy.pi, num=10000)
one_amp_driving_voltage = R * g(theta_range) + (L * gdot(theta_range) + M * gdot(theta_range + 2.0 / 3.0 * numpy.pi) + M * gdot(theta_range - 2.0 / 3.0 * numpy.pi)) * Kv * vcc / 2.0

max_one_amp_driving_voltage = max(one_amp_driving_voltage)

# The number to divide the product of the unit BEMF and the per phase current
# by to get motor current.
one_amp_scalar = (phases(f_single, 0.0).T * phases(g_single, 0.0))[0, 0]

print 'Max BEMF', max(f(theta_range))
print 'Max current', max(g(theta_range))
print 'Max drive voltage (one_amp_driving_voltage)', max(one_amp_driving_voltage)
print 'one_amp_scalar', one_amp_scalar

pylab.figure()
pylab.subplot(1, 1, 1)
pylab.plot(theta_range, f(theta_range), label='bemf')
pylab.plot(theta_range, g(theta_range), label='phase_current')
pylab.plot(theta_range, torque(theta_range), label='phase_torque')
pylab.plot(theta_range, all_phases(torque, theta_range), label='sum_torque/current')
pylab.legend()


def full_sample_times(Ton, Toff, dt, n, start_time):
  """Returns n + 4 samples for the provided switching times.

  We need the timesteps and Us to integrate.

  Args:
    Ton: On times for each phase.
    Toff: Off times for each phase.
    dt: The cycle time.
    n: Number of intermediate points to include in the result.
    start_time: Starting value for the t values in the result.

  Returns:
    array of [t, U matrix]
  """

  assert((Toff <= 1.0).all())
  assert((Ton <= 1.0).all())
  assert((Toff >= 0.0).all())
  assert((Ton >= 0.0).all())

  if (Ton <= Toff).all():
    on_before_off = True
  else:
    # Verify that they are all ordered correctly.
    assert(not (Ton <= Toff).any())
    on_before_off = False

  Toff = Toff.copy() * dt
  Toff[Toff < 100e-9] = -1.0
  Toff[Toff > dt] = dt

  Ton = Ton.copy() * dt
  Ton[Ton < 100e-9] = -1.0
  Ton[Ton > dt - 100e-9] = dt + 1.0

  result = []
  t = 0

  result_times = numpy.concatenate(
      (numpy.linspace(0, dt, num=n),
       numpy.reshape(numpy.asarray(Ton[numpy.logical_and(Ton < dt, Ton > 0.0)]), (-1,)),
       numpy.reshape(numpy.asarray(Toff[numpy.logical_and(Toff < dt, Toff > 0.0)]), (-1,))
       ))
  result_times.sort()
  assert((result_times >= 0).all())
  assert((result_times <= dt).all())

  for t in result_times:
    if on_before_off:
      U = numpy.matrix([[vcc], [vcc], [vcc]])
      U[t <= Ton] = 0.0
      U[Toff < t] = 0.0
    else:
      U = numpy.matrix([[0.0], [0.0], [0.0]])
      U[t > Ton] = vcc
      U[t <= Toff] = vcc
    result.append((float(t + start_time), U.copy()))

  return result

def sample_times(T, dt, n, start_time):
  if switching_pattern == 'rear':
    T = 1.0 - T
    ans = full_sample_times(T, numpy.matrix(numpy.ones((3, 1))) * 1.0, dt, n, start_time)
  elif switching_pattern == 'centered front shifted':
    # Centered, but shifted to the beginning of the cycle.
    Ton = 0.5 - T / 2.0
    Toff = 0.5 + T / 2.0

    tn = min(Ton)[0, 0]
    Ton -= tn
    Toff -= tn

    ans = full_sample_times(Ton, Toff, dt, n, start_time)
  elif switching_pattern == 'centered':
    # Centered, looks waaay better.
    Ton = 0.5 - T / 2.0
    Toff = 0.5 + T / 2.0

    ans = full_sample_times(Ton, Toff, dt, n, start_time)
  elif switching_pattern == 'anticentered':
    # Centered, looks waaay better.
    Toff = T / 2.0
    Ton = 1.0 - T / 2.0

    ans = full_sample_times(Ton, Toff, dt, n, start_time)
  elif switching_pattern == 'front':
    ans = full_sample_times(numpy.matrix(numpy.zeros((3, 1))), T, dt, n, start_time)
  else:
    assert(False)

  return ans

class DataLogger(object):
  def __init__(self, title=None):
    self.title = title
    self.ia = []
    self.ib = []
    self.ic = []
    self.ia_goal = []
    self.ib_goal = []
    self.ic_goal = []
    self.ia_controls = []
    self.ib_controls = []
    self.ic_controls = []
    self.isensea = []
    self.isenseb = []
    self.isensec = []

    self.va = []
    self.vb = []
    self.vc = []
    self.van = []
    self.vbn = []
    self.vcn = []

    self.ea = []
    self.eb = []
    self.ec = []

    self.theta = []
    self.omega = []

    self.i_goal = []

    self.time = []
    self.controls_time = []
    self.predicted_time = []

    self.ia_pred = []
    self.ib_pred = []
    self.ic_pred = []

    self.voltage_time = []
    self.estimated_velocity = []
    self.U_last = numpy.matrix(numpy.zeros((3, 1)))

  def log_predicted(self, current_time, p):
    self.predicted_time.append(current_time)
    self.ia_pred.append(p[0, 0])
    self.ib_pred.append(p[1, 0])
    self.ic_pred.append(p[2, 0])

  def log_controls(self, current_time, measured_current, In, E, estimated_velocity):
    self.controls_time.append(current_time)
    self.ia_controls.append(measured_current[0, 0])
    self.ib_controls.append(measured_current[1, 0])
    self.ic_controls.append(measured_current[2, 0])

    self.ea.append(E[0, 0])
    self.eb.append(E[1, 0])
    self.ec.append(E[2, 0])

    self.ia_goal.append(In[0, 0])
    self.ib_goal.append(In[1, 0])
    self.ic_goal.append(In[2, 0])
    self.estimated_velocity.append(estimated_velocity)

  def log_data(self, X, U, current_time, Vn, i_goal):
    self.ia.append(X[0, 0])
    self.ib.append(X[1, 0])
    self.ic.append(X[2, 0])

    self.i_goal.append(i_goal)

    self.isensea.append(X[5, 0])
    self.isenseb.append(X[6, 0])
    self.isensec.append(X[7, 0])

    self.theta.append(X[3, 0])
    self.omega.append(X[4, 0])

    self.time.append(current_time)

    self.van.append(Vn[0, 0])
    self.vbn.append(Vn[1, 0])
    self.vcn.append(Vn[2, 0])

    if (self.U_last != U).any():
      self.va.append(self.U_last[0, 0])
      self.vb.append(self.U_last[1, 0])
      self.vc.append(self.U_last[2, 0])
      self.voltage_time.append(current_time)

      self.va.append(U[0, 0])
      self.vb.append(U[1, 0])
      self.vc.append(U[2, 0])
      self.voltage_time.append(current_time)
      self.U_last = U.copy()

  def plot(self):
    fig = pylab.figure()
    pylab.subplot(3, 1, 1)
    pylab.plot(self.controls_time, self.ia_controls, 'ro', label='ia_controls')
    pylab.plot(self.controls_time, self.ib_controls, 'go', label='ib_controls')
    pylab.plot(self.controls_time, self.ic_controls, 'bo', label='ic_controls')
    pylab.plot(self.controls_time, self.ia_goal, 'r--', label='ia_goal')
    pylab.plot(self.controls_time, self.ib_goal, 'g--', label='ib_goal')
    pylab.plot(self.controls_time, self.ic_goal, 'b--', label='ic_goal')

    #pylab.plot(self.controls_time, self.ia_pred, 'r*', label='ia_pred')
    #pylab.plot(self.controls_time, self.ib_pred, 'g*', label='ib_pred')
    #pylab.plot(self.controls_time, self.ic_pred, 'b*', label='ic_pred')
    pylab.plot(self.time, self.isensea, 'r:', label='ia_sense')
    pylab.plot(self.time, self.isenseb, 'g:', label='ib_sense')
    pylab.plot(self.time, self.isensec, 'b:', label='ic_sense')
    pylab.plot(self.time, self.ia, 'r', label='ia')
    pylab.plot(self.time, self.ib, 'g', label='ib')
    pylab.plot(self.time, self.ic, 'b', label='ic')
    pylab.plot(self.time, self.i_goal, label='i_goal')
    if self.title is not None:
      fig.canvas.set_window_title(self.title)
    pylab.legend()

    pylab.subplot(3, 1, 2)
    pylab.plot(self.voltage_time, self.va, label='va')
    pylab.plot(self.voltage_time, self.vb, label='vb')
    pylab.plot(self.voltage_time, self.vc, label='vc')
    pylab.plot(self.time, self.van, label='van')
    pylab.plot(self.time, self.vbn, label='vbn')
    pylab.plot(self.time, self.vcn, label='vcn')
    pylab.plot(self.controls_time, self.ea, label='ea')
    pylab.plot(self.controls_time, self.eb, label='eb')
    pylab.plot(self.controls_time, self.ec, label='ec')
    pylab.legend()

    pylab.subplot(3, 1, 3)
    pylab.plot(self.time, self.theta, label='theta')
    pylab.plot(self.time, self.omega, label='omega')
    #pylab.plot(self.controls_time, self.estimated_velocity, label='estimated omega')

    pylab.legend()

    fig = pylab.figure()
    pylab.plot(self.controls_time,
               map(operator.sub, self.ia_goal, self.ia_controls), 'r', label='ia_error')
    pylab.plot(self.controls_time,
               map(operator.sub, self.ib_goal, self.ib_controls), 'g', label='ib_error')
    pylab.plot(self.controls_time,
               map(operator.sub, self.ic_goal, self.ic_controls), 'b', label='ic_error')
    if self.title is not None:
      fig.canvas.set_window_title(self.title)
    pylab.legend()
    pylab.show()


# So, from running a bunch of math, we know the following:
# Van + Vbn + Vcn = 0
# ia + ib + ic = 0
# ea + eb + ec = 0
# d ia/dt + d ib/dt + d ic/dt = 0
#
# We also have:
#  [ Van ]   [  2/3 -1/3 -1/3] [Va]
#  [ Vbn ] = [ -1/3  2/3 -1/3] [Vb]
#  [ Vcn ]   [ -1/3 -1/3  2/3] [Vc]
#
# or,
#
#  Vabcn = Vconv * V
#
# The base equation is:
#
# [ Van ]   [ R 0 0 ] [ ia ]   [ L M M ] [ dia/dt ]   [ ea ]
# [ Vbn ] = [ 0 R 0 ] [ ib ] + [ M L M ] [ dib/dt ] + [ eb ]
# [ Vbn ]   [ 0 0 R ] [ ic ]   [ M M L ] [ dic/dt ]   [ ec ]
#
# or
#
# Vabcn = R_matrix * I + L_matrix * I_dot + E
#
# We can re-arrange this as:
#
# inv(L_matrix) * (Vconv * V - E - R_matrix * I) = I_dot
# B * V - inv(L_matrix) * E - A * I = I_dot
class Simulation(object):
  def __init__(self):
    self.R_matrix = numpy.matrix(numpy.eye(3)) * R
    self.L_matrix = numpy.matrix([[L, M, M], [M, L, M], [M, M, L]])
    self.L_matrix_inv = numpy.linalg.inv(self.L_matrix)
    self.A = self.L_matrix_inv * self.R_matrix
    self.B = self.L_matrix_inv * Vconv
    self.A_discrete, self.B_discrete = controls.c2d(-self.A, self.B, 1.0 / hz)
    self.B_discrete_inverse = numpy.matrix(numpy.eye(3)) / (self.B_discrete[0, 0] - self.B_discrete[1, 0])

    self.R_model = R * 1.0
    self.L_model = L * 1.0
    self.M_model = M * 1.0
    self.R_matrix_model = numpy.matrix(numpy.eye(3)) * self.R_model
    self.L_matrix_model = numpy.matrix([[self.L_model, self.M_model, self.M_model],
                                        [self.M_model, self.L_model, self.M_model],
                                        [self.M_model, self.M_model, self.L_model]])
    self.L_matrix_inv_model = numpy.linalg.inv(self.L_matrix_model)
    self.A_model = self.L_matrix_inv_model * self.R_matrix_model
    self.B_model = self.L_matrix_inv_model * Vconv
    self.A_discrete_model, self.B_discrete_model = \
        controls.c2d(-self.A_model, self.B_model, 1.0 / hz)
    self.B_discrete_inverse_model = numpy.matrix(numpy.eye(3)) / (self.B_discrete_model[0, 0] - self.B_discrete_model[1, 0])

    print 'constexpr double kL = %g;' % self.L_model
    print 'constexpr double kM = %g;' % self.M_model
    print 'constexpr double kR = %g;' % self.R_model
    print 'constexpr float kAdiscrete_diagonal = %gf;' % self.A_discrete_model[0, 0]
    print 'constexpr float kAdiscrete_offdiagonal = %gf;' % self.A_discrete_model[1, 0]
    print 'constexpr float kBdiscrete_inv_diagonal = %gf;' % self.B_discrete_inverse_model[0, 0]
    print 'constexpr float kBdiscrete_inv_offdiagonal = %gf;' % self.B_discrete_inverse_model[1, 0]
    print 'constexpr double kOneAmpScalar = %g;' % one_amp_scalar
    print 'constexpr double kMaxOneAmpDrivingVoltage = %g;' % max_one_amp_driving_voltage
    print('A_discrete', self.A_discrete)
    print('B_discrete', self.B_discrete)
    print('B_discrete_sub', numpy.linalg.inv(self.B_discrete[0:2, 0:2]))
    print('B_discrete_inv', self.B_discrete_inverse)

    # Xdot[5:, :] = (R_sense2 + R_sense1) / R_sense2 * (
    #      (1.0 / (R_sense1 * C_sense)) * (-Isense * R_sense2 / (R_sense1 + R_sense2) * (R_sense1 / R_sense2 + 1.0) + I))
    self.mk1 = (R_sense2 + R_sense1) / R_sense2 * (1.0 / (R_sense1 * C_sense))
    self.mk2 = -self.mk1 * R_sense2 / (R_sense1 + R_sense2) * (R_sense1 / R_sense2 + 1.0)

    # ia, ib, ic, theta, omega, isensea, isenseb, isensec
    self.X = numpy.matrix([[0.0], [0.0], [0.0], [-2.0 * numpy.pi / 3.0], [0.0], [0.0], [0.0], [0.0]])

    self.K = 0.05 * Vconv
    print('A %s' % repr(self.A))
    print('B %s' % repr(self.B))
    print('K %s' % repr(self.K))

    print('System poles are %s' % repr(numpy.linalg.eig(self.A)[0]))
    print('Poles are %s' % repr(numpy.linalg.eig(self.A - self.B * self.K)[0]))

    controllability = controls.ctrb(self.A, self.B)
    print('Rank of augmented controlability matrix. %d' % numpy.linalg.matrix_rank(
          controllability))

    self.data_logger = DataLogger(switching_pattern)
    self.current_time = 0.0

    self.estimated_velocity = self.X[4, 0]

  def motor_diffeq(self, x, t, U):
    I = numpy.matrix(x[0:3]).T
    theta = x[3]
    omega = x[4]
    Isense = numpy.matrix(x[5:]).T

    dflux = phases(f_single, theta) / Kv

    Xdot = numpy.matrix(numpy.zeros((8, 1)))
    di_dt = -self.A_model * I + self.B_model * U - self.L_matrix_inv_model * dflux * omega
    torque = I.T * dflux
    Xdot[0:3, :] = di_dt
    Xdot[3, :] = omega
    Xdot[4, :] = torque / J

    Xdot[5:, :] = self.mk1 * I + self.mk2 * Isense
    return numpy.squeeze(numpy.asarray(Xdot))

  def DoControls(self, goal_current):
    theta = self.X[3, 0]
    # Use the actual angular velocity.
    omega = self.X[4, 0]

    measured_current = self.X[5:, :].copy()

    # Ok, lets now fake it.
    E_imag1 = numpy.exp(1j * theta) * K1 * numpy.matrix(
            [[-1j],
             [-1j * numpy.exp(1j * numpy.pi * 2.0 / 3.0)],
             [-1j * numpy.exp(-1j * numpy.pi * 2.0 / 3.0)]])
    E_imag2 =  numpy.exp(1j * 5.0 * theta) * K2 * numpy.matrix(
            [[-1j],
             [-1j * numpy.exp(-1j * numpy.pi * 2.0 / 3.0)],
             [-1j * numpy.exp(1j * numpy.pi * 2.0 / 3.0)]])

    overall_measured_current = ((E_imag1 + E_imag2).real.T * measured_current / one_amp_scalar)[0, 0]

    current_error = goal_current - overall_measured_current
    #print(current_error)
    self.estimated_velocity += current_error * 1.0
    omega = self.estimated_velocity

    # Now, apply the transfer function of the inductor.
    # Use that to difference the current across the cycle.
    Icurrent = self.Ilast
    # No history:
    #Icurrent = phases(g_single, theta) * goal_current
    Inext = phases(g_single, theta + omega * 1.0 / hz) * goal_current

    deltaI = Inext - Icurrent

    H1 = -numpy.linalg.inv(1j * omega * self.L_matrix + self.R_matrix) * omega / Kv
    H2 = -numpy.linalg.inv(1j * omega * 5.0 * self.L_matrix + self.R_matrix) * omega / Kv
    p_imag = H1 * E_imag1 + H2 * E_imag2
    p_next_imag = numpy.exp(1j * omega * 1.0 / hz) * H1 * E_imag1 + \
        numpy.exp(1j * omega * 5.0 * 1.0 / hz) * H2 * E_imag2
    p = p_imag.real

    # So, we now know how much the change in current is due to changes in BEMF.
    # Subtract that, and then run the stock statespace equation.
    Vn_ff = self.B_discrete_inverse * (Inext - self.A_discrete * (Icurrent - p) - p_next_imag.real)
    print 'Vn_ff', Vn_ff
    print 'Inext', Inext
    Vn = Vn_ff + self.K * (Icurrent - measured_current)

    E = phases(f_single, self.X[3, 0]) / Kv * self.X[4, 0]
    self.data_logger.log_controls(self.current_time, measured_current, Icurrent, E, self.estimated_velocity)

    self.Ilast = Inext

    return Vn

  def Simulate(self):
    start_wall_time = time.time()
    self.Ilast = numpy.matrix(numpy.zeros((3, 1)))
    for n in range(200):
      goal_current = 1.0
      max_current = (vcc - (self.X[4, 0] / Kv * 2.0)) / max_one_amp_driving_voltage
      min_current = (-vcc - (self.X[4, 0] / Kv * 2.0)) / max_one_amp_driving_voltage
      goal_current = max(min_current, min(max_current, goal_current))

      Vn = self.DoControls(goal_current)

      #Vn = numpy.matrix([[1.00], [0.0], [0.0]])
      Vn = numpy.matrix([[0.00], [1.00], [0.0]])
      #Vn = numpy.matrix([[0.00], [0.0], [1.00]])

      # T is the fractional rate.
      T = Vn / vcc
      tn = -numpy.min(T)
      T += tn
      if (T > 1.0).any():
        T = T / numpy.max(T)

      for t, U in sample_times(T = T,
                               dt = 1.0 / hz, n = 10,
                               start_time = self.current_time):
        # Analog amplifier mode!
        #U = Vn

        self.data_logger.log_data(self.X, (U - min(U)), self.current_time, Vn, goal_current)
        t_array = numpy.array([self.current_time, t])
        self.X = numpy.matrix(scipy.integrate.odeint(
            self.motor_diffeq,
            numpy.squeeze(numpy.asarray(self.X)),
            t_array, args=(U,)))[1, :].T

        self.current_time = t

    print 'Took %f to simulate' % (time.time() - start_wall_time)

    self.data_logger.plot()

simulation = Simulation()
simulation.Simulate()
