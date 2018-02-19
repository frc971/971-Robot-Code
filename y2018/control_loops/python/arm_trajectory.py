#!/usr/bin/python

import copy
import numpy
import path_points
from matplotlib import pylab
from frc971.control_loops.python import controls

# This code computes the optimal velocity profile for the arm to follow
# the path while adhering to constraints.


def RungeKutta(f, x, dt):
    """4th order RungeKutta integration of F starting at X."""
    a = f(x)
    b = f(x + dt / 2.0 * a)
    c = f(x + dt / 2.0 * b)
    d = f(x + dt * c)
    return x + dt * (a + 2.0 * b + 2.0 * c + d) / 6.0


class Dynamics(object):
    def __init__(self, dt):
        self.dt = dt

        # 1 is the proximal link of the arm
        # 2 is the distal link of the arm
        self.l1 = 46.25 * 0.0254
        self.l2 = 41.80 * 0.0254

        self.m1 = 9.34 / 2.2
        self.m2 = 9.77 / 2.2

        self.J1 = 2957.05 * 0.0002932545454545454
        self.J2 = 2824.70 * 0.0002932545454545454

        self.r1 = 21.64 * 0.0254
        self.r2 = 26.70 * 0.0254

        self.G1 = 140.0
        self.G2 = 90.0

        # These constants are for a minicim and the double jointed arm as
        # modeled.
        self.stall_torque = 1.41
        self.free_speed = (5840.0 / 60.0) * 2.0 * numpy.pi
        self.stall_current = 89.0
        self.R = 12.0 / self.stall_current

        self.Kv = self.free_speed / 12.0
        self.Kt = self.stall_torque / self.stall_current

        self.alpha = self.J1 + self.r1 * self.r1 * \
            self.m1 + self.l1 * self.l1 * self.m2
        self.beta = self.l1 * self.r2 * self.m2
        self.gamma = self.J2 + self.r2 * self.r2 * self.m2

        self.kNumDistalMotors = 2.0

        self.K3 = numpy.matrix(
            [[self.G1 * self.Kt / self.R, 0.0],
             [0.0, self.G2 * self.kNumDistalMotors * self.Kt / self.R]])
        self.K4 = numpy.matrix(
            [[self.G1 * self.G1 * self.Kt / (self.Kv * self.R), 0.0], [
                0.0, self.G2 * self.G2 * self.Kt * self.kNumDistalMotors /
                (self.Kv * self.R)
            ]])

        # These constants are for the Extended Kalman Filter
        # Q is the covariance of the X values.  Use the square of the standard
        # deviation of the error accumulated each time step.
        self.Q_x_covariance = numpy.matrix([[0.001**2,0.0,0.0,0.0,0.0,0.0],
                                       [0.0,0.001**2,0.0,0.0,0.0,0.0],
                                       [0.0,0.0,0.001**2,0.0,0.0,0.0],
                                       [0.0,0.0,0.0,0.001**2,0.0,0.0],
                                       [0.0,0.0,0.0,0.0,10.0**2,0.0],
                                       [0.0,0.0,0.0,0.0,0.0,10.0**2]])
        # R is the covariance of the Z values.  Increase the responsiveness to
        # changes by reducing coresponding term in the R matrix.
        self.R_y_covariance = numpy.matrix([[0.01**2, 0.0],[0.0, 0.01**2]])
        # H is the jacobian of the h(x) measurement prediction function
        self.H_h_jacobian = numpy.matrix([[1.0,0.0,0.0,0.0,0.0,0.0],
                                          [0.0,0.0,1.0,0.0,0.0,0.0]])
        self.Identity_matrix = numpy.matrix(numpy.identity(6))


    def discrete_dynamics_ekf_predict(self, X_hat, P_covariance_estimate, U,
                                      sim_dt):
        """Updates the Extended Kalman Filter state for one timestep.
        The Extended Kalman Filter is used for estimating the state.
        From https://en.wikipedia.org/wiki/Extended_Kalman_filter

        The predict step is called for each time advancement in the state.
        The update step is called when sensor data is available.  The update
        step does not have to be called each time the predict step is called.

        Args:
          X, numpy.matrix(4, 1), The state.  [theta1, omega1, theta2, omega2]
          X_hat, numpy.matrix(6, 1), The EKF state.  [theta1, omega1,
            theta2, omega2, disturbance_torque1, disturbance_torque2]
          P_covariance_estimate, numpy.matrix(6,6), Covariance Estimate
          U, numpy.matrix(2, 1), The input.  [torque1, torque2]
          sim_dt, The simulation time step.

        Returns:
          numpy.matrix(6, 1), The Extended Kalman Filter predicted state.
        """
        # Predict step
        #   Compute the state trasition matrix using the Jacobian of state
        #   estimate
        F_k = numerical_jacobian_x(self.unbounded_discrete_dynamics_ekf,
          X_hat, U)
        #   State estimate
        X_hat = self.unbounded_discrete_dynamics_ekf(X_hat, U, sim_dt)
        #   Covariance estimate
        P_covariance_estimate = F_k * P_covariance_estimate * F_k.T +\
          self.Q_x_covariance
        return X_hat, P_covariance_estimate

    def discrete_dynamics_ekf_update(self, X_hat, P_covariance_estimate,
                                     sim_dt, Y_reading):
        """Updates the Extended Kalman Filter state for one timestep.

        See discrete_dynamics_ekf_predict() definition for more information.

        Args (in addition to those in discrete_dynamics_ekf_predict():
          Y_reading, numpy.matrix(2, 1), Position sensor readings.
            [encoder_angle1_sensor, encoder_angle2_sensor]

        Returns:
          numpy.matrix(6, 1), The Extended Kalman Filter updated state.
          numpy.matrix(6, 1), The Extended Kalman Filter updated state.
        """
        # Update step
        #   Measurement residual error of proximal and distal
        #   joint angles.
        Y_hat = Y_reading - numpy.matrix([[X_hat[0,0]],[X_hat[2,0]]])
        #   Residual covariance
        S = self.H_h_jacobian * P_covariance_estimate * self.H_h_jacobian.T + \
          self.R_y_covariance
        #   K is the Near-optimal Kalman gain
        Kalman_gain = P_covariance_estimate * self.H_h_jacobian.T * \
          numpy.linalg.inv(S)
        #   Updated state estimate
        X_hat = X_hat + Kalman_gain * Y_hat
        #   Updated covariance estimate
        P_covariance_estimate = (self.Identity_matrix -
          Kalman_gain * self.H_h_jacobian) * P_covariance_estimate
        return X_hat, P_covariance_estimate

    def NormilizedMatriciesForState(self, X):
        """Generate K1-4 for the arm ODE.

        K1 * d^2 theta / dt^2 + K2 * d theta / dt = K3 * V - K4 * d theta/dt
        """

        s = numpy.sin(X[0, 0] - X[2, 0])
        c = numpy.cos(X[0, 0] - X[2, 0])

        # K1 * d^2 theta/dt^2 + K2 * d theta/dt = torque
        K1 = numpy.matrix(
            [[self.alpha, c * self.beta], [c * self.beta, self.gamma]])

        K2 = numpy.matrix([[0.0, s * self.beta], [-s * self.beta, 0.0]])

        return K1, K2, self.K3, self.K4

    def MatriciesForState(self, X):
        K1, K2, K3, K4 = self.NormilizedMatriciesForState(X)
        K2[1, 0] *= X[1, 0]
        K2[0, 1] *= X[3, 0]
        return K1, K2, K3, K4

    def ff_u(self, X, omega_t, alpha_t):
        """Computes the feed forwards U required to track the provided state.

        Args:
          X, (4, 1) matrix, the state matrix
          omega_t, (2, 1) matrix, The angular velocities of the two joints.
          alpha_t, (2, 1) matrix, The angular accelerations of the two joints.

        Returns the voltages required to track the path.
        """
        K1, K2, K3, K4 = self.MatriciesForState(X)

        return numpy.linalg.inv(K3) * (
            K1 * alpha_t + K2 * omega_t + K4 * omega_t)

    def ff_u_distance(self, trajectory, distance):
        """Computes the feed forward U at the distance on the trajectory.

        Args:
            trajectory, Trajectory, The trajectory to follow.
            distance, float, the distance along the trajectory to compute
              theta, omega and alpha.
        """
        theta_t = trajectory.theta(distance)
        omega_t = trajectory.omega_t(distance)
        alpha_t = trajectory.alpha_t(distance)

        X = numpy.matrix([[theta_t[0, 0]], [omega_t[0, 0]], [theta_t[1, 0]],
                          [omega_t[1, 0]]])

        return self.ff_u(X, omega_t, alpha_t)

    def dynamics(self, X, U):
        """Calculates the dynamics for a double jointed arm.

        Args:
          X, numpy.matrix(4, 1), The state.  [theta1, omega1, theta2, omega2]
          U, numpy.matrix(2, 1), The input.  [torque1, torque2]

        Returns:
          numpy.matrix(4, 1), The derivative of the dynamics.
        """
        K1, K2, K3, K4 = self.MatriciesForState(X)

        velocity = numpy.matrix([[X[1, 0]], [X[3, 0]]])
        torque = K3 * U - K4 * velocity

        accel = numpy.linalg.inv(K1) * (torque - K2 * velocity)

        return numpy.matrix([[X[1, 0]], [accel[0, 0]], [X[3, 0]],
                             [accel[1, 0]]])

    def dynamics_ekf(self, X, U):
        """Calculates the dynamics for a double jointed arm for EKF state.

        The Extended Kalman Filter (EKF) has two more state variables so
        a second version of the dynamics method is needed.

        Args:
          X, numpy.matrix(6, 1), The state.  [theta1, omega1, theta2, omega2,
            disturbance_torque1, disturbance_torque2]
          U, numpy.matrix(2, 1), The input.  [torque1, torque2]

        Returns:
          numpy.matrix(4, 1), The derivative of the dynamics.
        """
        K1, K2, K3, K4 = self.MatriciesForState(X)

        velocity = numpy.matrix([[X[1, 0]], [X[3, 0]]])
        # Include the disturbance torques for the Extended Kalman Filter
        torque = K3 * U - K4 * velocity + numpy.matrix([[X[4, 0]], [X[5, 0]]])
        # Uncoment the following line to add in disturbance torque and see
        # if the Kalman Filter takes it out.
        # torque += numpy.matrix([[-5],[-17]])

        accel = numpy.linalg.inv(K1) * (torque - K2 * velocity)

        return numpy.matrix([[X[1, 0]], [accel[0, 0]], [X[3, 0]],
                             [accel[1, 0]], [0.0], [0.0]])

    def unbounded_discrete_dynamics(self, X, U, dt=None):
        return RungeKutta(lambda startingX: self.dynamics(startingX, U), X,
                          dt or self.dt)

    def unbounded_discrete_dynamics_ekf(self, X, U, dt=None):
        return RungeKutta(lambda startingX: self.dynamics_ekf(startingX, U), X,
                          dt or self.dt)

    def discrete_dynamics(self, X, U, dt=None):
        assert((numpy.abs(U) <= (12.0 + 1e-6)).all())
        return self.unbounded_discrete_dynamics(X, U, dt)


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
        answer[:, i] = (fn(dX_plus, U) - fn(dX_minus, U)) / (epsilon * 2.0)
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
        answer[:, i] = (fn(X, dU_plus) - fn(X, dU_minus)) / (epsilon * 2.0)
    return answer


def K_at_state(dynamics, x, u):
    """Computes the controller gain K at the given X and U."""
    q_pos = 0.2
    q_vel = 4.0
    Q = numpy.matrix(
        numpy.diag([
            1.0 / (q_pos**2.0), 1.0 / (q_vel**2.0), 1.0 / (q_pos**2.0), 1.0 / (
                q_vel**2.0)
        ]))

    R = numpy.matrix(numpy.diag([1.0 / (12.0**2.0), 1.0 / (12.0**2.0)]))

    final_A = numerical_jacobian_x(dynamics.unbounded_discrete_dynamics, x, u)
    final_B = numerical_jacobian_u(dynamics.unbounded_discrete_dynamics, x, u)

    return controls.dlqr(final_A, final_B, Q, R)

def get_encoder_values(X):
  """Returns simulated encoder readings.

  This method returns the encoder readings.  For now simply use values from X
  with some noise added in to make the simulation more interesting.
  """
  introduced_random_measurement_noise = 0.005
  introduced_random_measurement_noise = 0.05
  theta1_measured = X[0,0] + introduced_random_measurement_noise * \
    2.0 * ( numpy.random.random() - 0.5 )
  theta2_measured = X[2,0] + introduced_random_measurement_noise * \
    2.0 * ( numpy.random.random() - 0.5 )
  return numpy.matrix([[theta1_measured ],[theta2_measured]])

class Trajectory:
    """This class represents a trajectory in theta space."""

    def __init__(self, path_step_size):
        self.path_points = path_points.path_with_accelerations
        self._thetas = [
            numpy.matrix([[numpy.pi / 2.0 - x[0]], [numpy.pi / 2.0 - x[1]]])
            for x in self.path_points
        ]
        self._omegas = [numpy.matrix([[-x[2]], [-x[3]]]) for x in self.path_points]
        self._alphas = [numpy.matrix([[-x[4]], [-x[5]]]) for x in self.path_points]
        self.path_step_size = path_step_size
        self.point_distances = [0.0]
        last_point = self._thetas[0]
        for point in self._thetas[1:]:
            self.point_distances.append(
                numpy.linalg.norm(point - last_point) +
                self.point_distances[-1])
            last_point = point
        self._length = self.point_distances[-1]

    def theta(self, distance):
        """Interpolates the angle as a function of distance.

        Note:
          points before or after the path will get truncated to the ends of the path.
        """
        return self._interpolate(self._thetas, distance)

    def _interpolate(self, points, distance):
        """Interpolates a set of points spaced at self.point_distances.

        Returns:
          The point linearly interpolated for the provided distance.  Points
          before or after the path will get truncated to the ends of the path.
        """
        if distance <= 0.0:
            return points[0]
        elif distance >= self._length:
            return points[-1]
        after_index = numpy.searchsorted(
            self.point_distances, distance, side='right')
        before_index = after_index - 1
        return (distance - self.point_distances[before_index]) / (
            self.point_distances[after_index] -
            self.point_distances[before_index]
        ) * (points[after_index] - points[before_index]) + points[before_index]

    def length(self):
        """Returns the path length in radians."""
        return self.point_distances[-1]

    def omega(self, distance):
        """Returns d theta / dd for our path at the specified distance."""
        return self._interpolate(self._omegas, distance)

    def alpha(self, distance):
        """Returns d^2 theta / dd^2 for our path at the specified distance."""
        return self._interpolate(self._alphas, distance)

    def curvature_trajectory_pass(self, dynamics, alpha_unitizer,
                                  distance_array, vmax):
        """Computes the steady state curvature pass for optimizing our trajectory."""
        max_dvelocity_unfiltered = []
        # Pass 0
        for index, distance in enumerate(distance_array):
            theta = self.theta(distance)
            omega = self.omega(distance)
            alpha = self.alpha(distance)
            X = numpy.matrix([[theta[0, 0]], [0.0], [theta[1, 0]], [0.0]])
            K1, K2, K3, K4 = dynamics.NormilizedMatriciesForState(X)
            omega_square = numpy.matrix(
                [[omega[0, 0], 0.0], [0.0, omega[1, 0]]])
            # Here, we can say that
            #   d^2/dt^2 theta = d^2/dd^2 theta(d) * (d d/dt)^2
            # Normalize so that the max accel is 1, and take magnitudes. This
            # gives us the max velocity we can be at each point due to
            # curvature.
            vk1 = numpy.linalg.inv(K3) * (
                K1 * alpha + K2 * omega_square * omega)
            vk2 = numpy.linalg.inv(K3) * K4 * omega
            ddots = []
            for c in [-vmax, vmax]:
                for a, b in [(vk1[0, 0], vk2[0, 0]), (vk1[1, 0], vk2[1, 0])]:
                    sqrt_number = b * b - 4.0 * a * c
                    if sqrt_number > 0:
                        ddots.append(
                            (-b + numpy.sqrt(sqrt_number)) / (2.0 * a))
                        ddots.append(
                            (-b - numpy.sqrt(sqrt_number)) / (2.0 * a))
            good_ddots = []
            for ddot in ddots:
                U = vk1 * ddot * ddot + vk2 * ddot
                if (numpy.abs(U) <= vmax + 1e-6).all() and ddot > 0.0:
                    good_ddots.append(ddot)

            max_dvelocity_unfiltered.append(
                min(
                    min(good_ddots),
                    numpy.sqrt(1.0 / max(
                        0.001, numpy.linalg.norm(alpha_unitizer * alpha)))))

        return max_dvelocity_unfiltered

    def compute_feasable_back_acceleration(self, dynamics, distance, velocity,
                                           vmax, alpha_unitizer):
        theta = self.theta(distance)
        omega = self.omega(distance)
        alpha = self.alpha(distance)

        X = numpy.matrix([[theta[0, 0]], [0.0], [theta[1, 0]], [0.0]])
        K1, K2, K3, K4 = dynamics.NormilizedMatriciesForState(X)
        omega_square = numpy.matrix([[omega[0, 0], 0.0], [0.0, omega[1, 0]]])

        k_constant = numpy.linalg.inv(K3) * (
            (K1 * alpha + K2 * omega_square * omega) * velocity * velocity +
            K4 * omega * velocity)
        k_scalar = numpy.linalg.inv(K3) * K1 * omega

        voltage_accel_list = []
        for c in [-vmax, vmax]:
            for a, b in [(k_constant[0, 0], k_scalar[0, 0]), (k_constant[1, 0],
                                                              k_scalar[1, 0])]:
                # This time, we are doing the other pass.  So, find all
                # the decelerations (and flip them) to find the prior
                # velocity.
                voltage_accel_list.append((c - a) / b)
        filtered_voltage_accel_list = []
        for a in voltage_accel_list:
            U = k_constant + k_scalar * a
            if a < 0.0 and (numpy.abs(U) <= vmax + 1e-6).all():
                filtered_voltage_accel_list.append(a)

        goal_acceleration = numpy.sqrt(
            max(0.0, 1.0 -
                (numpy.linalg.norm(alpha_unitizer * alpha) * velocity *
                 velocity)**2.0)) / numpy.linalg.norm(alpha_unitizer * omega)
        if filtered_voltage_accel_list:
            # TODO(austin): The max of the list seems right, but I'm
            # not seeing many lists with a size > 1, so it's hard to
            # tell.  Max is conservative, for sure.
            goal_acceleration = min(-max(filtered_voltage_accel_list), goal_acceleration)
        return goal_acceleration

    def back_trajectory_pass(self, previous_pass, dynamics, alpha_unitizer,
                             distance_array, vmax):
        """Computes the backwards pass for optimizing our trajectory."""
        max_dvelocity_back_pass = copy.copy(previous_pass)
        # Now, iterate over the list of velocities and constrain the
        # acceleration.
        for index, distance in reversed(list(enumerate(distance_array))):
            if index == len(distance_array) - 1:
                continue

            prev_velocity = max_dvelocity_back_pass[index + 1]
            prev_distance = distance_array[index + 1]

            int_d = 0.0
            int_vel = prev_velocity
            num_steps = 10
            for _ in xrange(num_steps):
                int_accel_t = self.compute_feasable_back_acceleration(
                    dynamics, prev_distance - int_d, int_vel, vmax,
                    alpha_unitizer)

                integration_step_size = self.path_step_size / float(num_steps)
                int_d += integration_step_size
                int_vel = numpy.sqrt(2.0 * int_accel_t * integration_step_size
                                     + int_vel * int_vel)
            max_dvelocity_back_pass[index] = min(
                int_vel, max_dvelocity_back_pass[index])
        return max_dvelocity_back_pass

    def compute_feasable_forwards_acceleration(self, dynamics, goal_distance,
                                               goal_velocity, vmax,
                                               alpha_unitizer):
        """Computes the maximum forwards feasable acceleration.

        This gives us the maximum acceleration (d^2d/dt^2) for the forwards
        pass.
        """
        theta = self.theta(goal_distance)
        omega = self.omega(goal_distance)
        alpha = self.alpha(goal_distance)

        X = numpy.matrix([[theta[0, 0]], [0.0], [theta[1, 0]], [0.0]])
        K1, K2, K3, K4 = dynamics.NormilizedMatriciesForState(X)
        omega_square = numpy.matrix([[omega[0, 0], 0.0], [0.0, omega[1, 0]]])

        k_constant = numpy.linalg.inv(K3) * (
            (K1 * alpha + K2 * omega_square * omega
             ) * goal_velocity * goal_velocity + K4 * omega * goal_velocity)
        k_scalar = numpy.linalg.inv(K3) * K1 * omega
        voltage_accel_list = []
        for c in [-vmax, vmax]:
            for a, b in [(k_constant[0, 0], k_scalar[0, 0]), (k_constant[1, 0],
                                                              k_scalar[1, 0])]:
                voltage_accel_list.append((c - a) / b)

        goal_acceleration = numpy.sqrt(
            max(0.0, 1.0 -
                (numpy.linalg.norm(alpha_unitizer * alpha) * goal_velocity *
                 goal_velocity)**2.0)) / numpy.linalg.norm(
                     alpha_unitizer * omega)

        filtered_voltage_accel_list = []
        for a in voltage_accel_list:
            U = k_constant + k_scalar * a
            if a > 0.0 and (numpy.abs(U) <= vmax + 1e-6).all():
                filtered_voltage_accel_list.append(a)

        if filtered_voltage_accel_list:
            # TODO(austin): The max of the list seems right, but I'm not
            # seeing many lists with a size > 1, so it's hard to tell.
            # Min is conservative, for sure.
            goal_acceleration = min(
                min(filtered_voltage_accel_list), goal_acceleration)

        return goal_acceleration

    def forward_trajectory_pass(self, previous_pass, dynamics, alpha_unitizer,
                                distance_array, vmax):
        """Computes the forward pass for optimizing our trajectory."""
        max_dvelocity_forward_pass = copy.copy(previous_pass)
        # Now, iterate over the list of velocities and constrain the
        # acceleration.
        for index, distance in enumerate(distance_array):
            if index == 0:
                continue

            prev_velocity = max_dvelocity_forward_pass[index - 1]
            prev_distance = distance_array[index - 1]

            int_d = 0.0
            int_vel = prev_velocity
            num_steps = 10
            for _ in xrange(num_steps):
                int_accel_t = self.compute_feasable_forwards_acceleration(
                    dynamics, prev_distance + int_d, int_vel, vmax,
                    alpha_unitizer)

                integration_step_size = self.path_step_size / float(num_steps)
                int_d += integration_step_size
                int_vel = numpy.sqrt(2.0 * int_accel_t * integration_step_size
                                      + int_vel * int_vel)

            max_dvelocity_forward_pass[index] = min(
                int_vel, max_dvelocity_forward_pass[index])

        return max_dvelocity_forward_pass

    def compute_trajectory(self, dynamics, alpha_unitizer, distance_array,
                           vmax):
        self.distance_array = copy.copy(distance_array)
        self.max_dvelocity_unfiltered = self.curvature_trajectory_pass(
            dynamics, alpha_unitizer, distance_array, vmax)
        print 'Finished curvature pass'

        self.max_dvelocity_unfiltered[0] = 0.0
        self.max_dvelocity_unfiltered[-1] = 0.0

        self.max_dvelocity_back_pass = self.back_trajectory_pass(
            self.max_dvelocity_unfiltered, dynamics, alpha_unitizer,
            distance_array, vmax)
        self.max_dvelocity = self.max_dvelocity_back_pass
        print 'Finished backwards pass'

        self.max_dvelocity_forward_pass = self.forward_trajectory_pass(
            self.max_dvelocity_back_pass, dynamics, alpha_unitizer,
            distance_array, vmax)
        print 'Finished forwards pass'

    def interpolate_velocity(self, d, d0, d1, v0, v1):
        if v0 + v1 > 0:
            return numpy.sqrt(v0 * v0 +
                              (v1 * v1 - v0 * v0) * (d - d0) / (d1 - d0))
        else:
            return -numpy.sqrt(v0 * v0 +
                               (v1 * v1 - v0 * v0) * (d - d0) / (d1 - d0))
    def get_dvelocity(self, d):
        """Computes the path distance velocity of the plan as a function of the distance."""
        after_index = numpy.argmax(self.distance_array > d)
        before_index = after_index - 1
        v0 = self.max_dvelocity[before_index]
        v1 = self.max_dvelocity[after_index]
        d0 = self.distance_array[before_index]
        d1 = self.distance_array[after_index]
        return self.interpolate_velocity(d, d0, d1, v0, v1)

    def interpolate_acceleration(self, d0, d1, v0, v1):
        return 0.5 * (v1**2.0 - v0**2.0) / (d1 - d0)

    def get_dacceleration(self, d):
        """Computes the path distance acceleration of the plan as a function of the distance."""
        after_index = numpy.argmax(self.distance_array > d)
        before_index = after_index - 1
        v0 = self.max_dvelocity[before_index]
        v1 = self.max_dvelocity[after_index]
        d0 = self.distance_array[before_index]
        d1 = self.distance_array[after_index]
        return self.interpolate_acceleration(d0, d1, v0, v1)

    def omega_t(self, d, velocity=None):
        """Returns d theta/dt at a specified distance."""
        if d > self._length:
            return numpy.matrix(numpy.zeros((2, 1)))
        return self.omega(d) * (velocity or self.get_dvelocity(d))

    def alpha_t(self, d, velocity=None, acceleration=None):
        """Returns d^2 theta/dt^2 at a specified distance."""
        if d > self._length:
            return numpy.matrix(numpy.zeros((2, 1)))
        return self.alpha(d) * (
            (velocity or self.get_dvelocity(d))**2.0) + self.omega(d) * (
                acceleration or self.get_dacceleration(d))

    def R(self, d, velocity=None):
        theta_t = self.theta(d)
        omega_t = self.omega_t(d, velocity=velocity)
        return numpy.matrix([[theta_t[0, 0]], [omega_t[0, 0]], [theta_t[1, 0]],
                             [omega_t[1, 0]]])


def U_saturation_search(goal_distance, last_goal_distance, goal_velocity,
                        last_goal_velocity, fraction_along_path, K, X,
                        trajectory, dynamics, vmax):
    saturation_goal_distance = (
        (goal_distance - last_goal_distance) * fraction_along_path +
        last_goal_distance)

    # TODO(austin): use computed forward dynamics velocity here.
    theta_t = trajectory.theta(saturation_goal_distance)
    saturation_goal_velocity = trajectory.interpolate_velocity(
        saturation_goal_distance, last_goal_distance,
        goal_distance, last_goal_velocity, goal_velocity)
    saturation_goal_acceleration = trajectory.interpolate_acceleration(
        last_goal_distance, goal_distance, last_goal_velocity,
        goal_velocity)
    omega_t = trajectory.omega_t(
        saturation_goal_distance,
        velocity=saturation_goal_velocity)
    alpha_t = trajectory.alpha_t(
        saturation_goal_distance,
        velocity=saturation_goal_velocity,
        acceleration=saturation_goal_acceleration)
    R = trajectory.R(
        saturation_goal_distance,
        velocity=saturation_goal_velocity)
    U_ff = numpy.clip(dynamics.ff_u(R, omega_t, alpha_t), -12.0, 12.0)
    return U_ff + K * (
        R - X), saturation_goal_velocity, saturation_goal_acceleration


def main():
    dt = 0.00505
    path_step_size = 0.01
    vmax = 11.5
    dynamics = Dynamics(dt)

    trajectory = Trajectory(path_step_size)
    print 'Initialized path'

    distance_array = numpy.linspace(
        0.0, trajectory.length(),
        numpy.ceil(trajectory.length() / path_step_size) + 1)
    theta0_array = []
    theta1_array = []
    omega0_array = []
    omega1_array = []
    alpha0_array = []
    alpha1_array = []

    it0 = trajectory.theta(0.0)[0, 0]
    it1 = trajectory.theta(0.0)[1, 0]
    io0 = trajectory.omega(0.0)[0, 0]
    io1 = trajectory.omega(0.0)[1, 0]

    integrated_distance = []
    integrated_omega0_array = []
    integrated_omega1_array = []
    integrated_theta0_array = []
    integrated_theta1_array = []

    for index, distance in enumerate(distance_array):
        theta = trajectory.theta(distance)
        omega = trajectory.omega(distance)
        alpha = trajectory.alpha(distance)

        theta0_array.append(theta[0, 0])
        theta1_array.append(theta[1, 0])
        omega0_array.append(omega[0, 0])
        omega1_array.append(omega[1, 0])
        alpha0_array.append(alpha[0, 0])
        alpha1_array.append(alpha[1, 0])

    # For a consistency check, integrate back up the acceleration and see how
    # close we got.
    dd = 0.005
    for distance in numpy.linspace(0.0, trajectory.length(),
                                   numpy.ceil(trajectory.length() / dd) + 1):
        integrated_distance.append(distance)
        integrated_omega0_array.append(io0)
        integrated_omega1_array.append(io1)
        integrated_theta0_array.append(it0)
        integrated_theta1_array.append(it1)
        alpha = trajectory.alpha(distance)
        it0 += io0 * dd
        it1 += io1 * dd
        io0 += alpha[0, 0] * dd
        io1 += alpha[1, 0] * dd

    print 'Iterated through path'

    # Bounds on the accelerations of the two DOFs.
    # We'll draw an oval to represent the actual bounds here.
    alpha0_max = 40.0
    alpha1_max = 60.0

    alpha_unitizer = numpy.matrix(
        [[1.0 / alpha0_max, 0.0], [0.0, 1.0 / alpha1_max]])

    # Compute the trajectory taking into account our velocity, acceleration
    # and voltage constraints.
    trajectory.compute_trajectory(dynamics, alpha_unitizer, distance_array, vmax=vmax)

    print 'Computed trajectory'

    # Now, we can get acceleration, velocity, and position as a function of distance.
    # Acceleration is best effort (derived from the velocities), but velocity
    # is pretty good.  So, use that to integrate up positions to track.

    t_array = []
    theta0_goal_t_array = []
    theta1_goal_t_array = []
    omega0_goal_t_array = []
    omega1_goal_t_array = []
    alpha0_goal_t_array = []
    alpha1_goal_t_array = []
    theta0_t_array = []
    theta1_t_array = []
    omega0_t_array = []
    omega1_t_array = []
    theta0_hat_array = []
    omega0_hat_array = []
    theta1_hat_array = []
    omega1_hat_array = []
    torque_disturbance_0_hat_array = []
    torque_disturbance_1_hat_array = []
    alpha0_t_array = []
    alpha1_t_array = []
    distance_t_array = []
    velocity_t_array = []
    acceleration_t_array = []

    uff0_array = []
    uff1_array = []
    u0_array = []
    u1_array = []
    u0_unsaturated_array = []
    u1_unsaturated_array = []

    last_goal_distance = 0.0
    last_goal_velocity = 0.0
    last_goal_acceleration = 0.0

    goal_distance = 0.0
    goal_velocity = 0.0

    theta_t = trajectory.theta(goal_distance)
    X = numpy.matrix([[theta_t[0, 0]], [0.0], [theta_t[1, 0]], [0.0]])
    # X_hat is for the Extended Kalman Filter state estimate
    X_hat = numpy.matrix([[theta_t[0, 0]], [0.0], [theta_t[1, 0]],
      [0.0], [0.0], [0.0]])
    # P is the Covariance Estimate for the Etended Kalman Filter
    P_covariance_estimate = dynamics.Q_x_covariance.copy()

    sim_dt = dt

    print 'Starting simulation'
    # Now, we can start following the trajectory!
    for t in numpy.arange(0.0, 1.0, sim_dt):
        if goal_distance == trajectory.length():
            next_distance = goal_distance
            next_velocity = 0.0
            goal_acceleration = 0.0
        else:
            next_acceleration = trajectory.compute_feasable_forwards_acceleration(
                dynamics, goal_distance, goal_velocity, vmax, alpha_unitizer)
            next_distance = (goal_distance + goal_velocity * sim_dt +
                             0.5 * sim_dt * sim_dt * next_acceleration)
            next_velocity = goal_velocity + sim_dt * next_acceleration

            next_trajectory_velocity = trajectory.get_dvelocity(next_distance)
            if next_trajectory_velocity < next_velocity:
                next_velocity = next_trajectory_velocity
                goal_acceleration = trajectory.interpolate_acceleration(
                    goal_distance, next_distance, goal_velocity, next_velocity)
                next_distance = (goal_distance + goal_velocity * sim_dt +
                                 0.5 * sim_dt * sim_dt * goal_acceleration)
                next_velocity = trajectory.get_dvelocity(next_distance)

            goal_acceleration = trajectory.interpolate_acceleration(
                goal_distance, next_distance, goal_velocity, next_velocity)

        t_array.append(t)
        theta_t = trajectory.theta(goal_distance)
        omega_t = trajectory.omega_t(goal_distance, velocity=goal_velocity)
        alpha_t = trajectory.alpha_t(
            goal_distance,
            velocity=goal_velocity,
            acceleration=goal_acceleration)

        theta0_goal_t_array.append(theta_t[0, 0])
        theta1_goal_t_array.append(theta_t[1, 0])
        omega0_goal_t_array.append(omega_t[0, 0])
        omega1_goal_t_array.append(omega_t[1, 0])
        alpha0_goal_t_array.append(alpha_t[0, 0])
        alpha1_goal_t_array.append(alpha_t[1, 0])
        theta0_t_array.append(X[0, 0])
        omega0_t_array.append(X[1, 0])
        theta1_t_array.append(X[2, 0])
        omega1_t_array.append(X[3, 0])
        # Extended Kalman Filter values
        theta0_hat_array.append(X_hat[0, 0])
        omega0_hat_array.append(X_hat[1, 0])
        theta1_hat_array.append(X_hat[2, 0])
        omega1_hat_array.append(X_hat[3, 0])
        torque_disturbance_0_hat_array.append(X_hat[4, 0])
        torque_disturbance_1_hat_array.append(X_hat[5, 0])

        distance_t_array.append(goal_distance)
        velocity_t_array.append(goal_velocity)
        acceleration_t_array.append(goal_acceleration)

        # Extended Kalman Filter update step - call each time sensor data is
        # available.  For now, simulate the sensor reading by using the X
        # position and adding some noise to it.
        X_hat, P_covariance_estimate = dynamics.discrete_dynamics_ekf_update(
          X_hat, P_covariance_estimate, sim_dt, get_encoder_values(X))

        R = trajectory.R(goal_distance, velocity=goal_velocity)
        U_ff = numpy.clip(dynamics.ff_u(R, omega_t, alpha_t), -12.0, 12.0)
        K = K_at_state(dynamics, X, U_ff)
        U = U_ff + K * (R - X)

        u0_unsaturated_array.append(U[0, 0])
        u1_unsaturated_array.append(U[1, 0])

        # Ok, now we know if we are staturated or not.  If we are, time to
        # search between here and our previous goal either until we find a
        # state where we aren't saturated, or we are really close to our
        # starting point.
        if (numpy.abs(U) > vmax).any():
            # Saturated.  Let's do a binary search.
            print "Saturated."
            if (goal_distance - last_goal_distance) < 1e-8:
                print "Not bothering to move"
                # Avoid the divide by 0 when interpolating.  Just don't move
                # since we are saturated.
                fraction_along_path = 0.0
                step_size = 0.0
            else:
                fraction_along_path = 0.5
                step_size = 0.5

            # First, see if slowing down to our current velocity solves it.
            while step_size > 0.01:
                U, saturation_goal_velocity, saturation_goal_acceleration = U_saturation_search(
                    goal_distance, last_goal_distance, goal_velocity,
                    last_goal_velocity, fraction_along_path, K, X, trajectory,
                    dynamics, 12.0)
                step_size = step_size * 0.5
                if (numpy.abs(U) > vmax).any():
                    fraction_along_path -= step_size
                else:
                    fraction_along_path += step_size
            print "Fraction", fraction_along_path, "at", goal_distance, "rad,", t, "sec", goal_velocity

            goal_distance = ((goal_distance - last_goal_distance) *
                             fraction_along_path + last_goal_distance)
            goal_velocity = saturation_goal_velocity
            goal_acceleration = saturation_goal_acceleration

            next_acceleration = trajectory.compute_feasable_forwards_acceleration(
                dynamics, goal_distance, goal_velocity, vmax, alpha_unitizer)
            next_distance = (goal_distance + goal_velocity * sim_dt +
                             0.5 * sim_dt * sim_dt * next_acceleration)
            next_velocity = goal_velocity + sim_dt * next_acceleration

            next_trajectory_velocity = trajectory.get_dvelocity(next_distance)
            next_velocity = min(next_velocity, next_trajectory_velocity)

        # Now, artifically clip the available voltage.
        U = numpy.clip(U, -12.0, 12.0)

        xdot = dynamics.dynamics(X, U)
        alpha0_t_array.append(xdot[1, 0])
        alpha1_t_array.append(xdot[3, 0])

        uff0_array.append(U_ff[0, 0])
        uff1_array.append(U_ff[1, 0])
        u0_array.append(U[0, 0])
        u1_array.append(U[1, 0])

        # Push our dynamics forwards.
        X = dynamics.discrete_dynamics(X, U, sim_dt)

        last_goal_distance = goal_distance
        last_goal_velocity = goal_velocity

        goal_distance = next_distance
        goal_velocity = next_velocity

        # Push Extended Kalman filter state forwards.
        # Predict step - call for each time step
        X_hat, P_covariance_estimate = dynamics.discrete_dynamics_ekf_predict(
          X_hat, P_covariance_estimate, U, sim_dt)


        if abs(goal_distance - trajectory.length()) < 1e-2:
            # If we go backwards along the path near the goal, snap us to the
            # end point or we'll never actualy finish.
            if goal_acceleration * sim_dt + goal_velocity < 0.0:
                goal_distance = trajectory.length()
                goal_velocity = 0.0

    print 'Finished simulation'
    pylab.figure()
    pylab.title("Trajecotry")
    pylab.plot(theta0_array, theta1_array, label="desired path")
    pylab.plot(theta0_t_array, theta1_t_array, label="actual path")
    pylab.legend(loc='upper left')

    pylab.figure()
    pylab.title("Path derivitives")
    pylab.plot(distance_array, theta0_array, label="theta0")
    pylab.plot(distance_array, theta1_array, label="theta1")
    pylab.plot(distance_array, omega0_array, label="omega0")
    pylab.plot(distance_array, omega1_array, label="omega1")
    pylab.plot(distance_array, alpha0_array, label="alpha0")
    pylab.plot(distance_array, alpha1_array, label="alpha1")

    pylab.plot(integrated_distance, integrated_omega0_array, label='iomega0')
    pylab.plot(integrated_distance, integrated_omega1_array, label='iomega1')
    pylab.plot(integrated_distance, integrated_theta0_array, label='itheta0')
    pylab.plot(integrated_distance, integrated_theta1_array, label='itheta1')
    pylab.legend(loc='upper left')

    pylab.figure()
    pylab.title("Path Velocity Plan")
    pylab.plot(
        distance_array, trajectory.max_dvelocity_unfiltered, label="pass0")
    pylab.plot(
        distance_array, trajectory.max_dvelocity_back_pass, label="passb")
    pylab.plot(
        distance_array, trajectory.max_dvelocity_forward_pass, label="passf")
    pylab.legend(loc='center')
    pylab.legend()

    pylab.figure()
    pylab.plot(t_array, alpha0_goal_t_array, label="alpha0_t_goal")
    pylab.plot(t_array, alpha0_t_array, label="alpha0_t")
    pylab.plot(t_array, alpha1_goal_t_array, label="alpha1_t_goal")
    pylab.plot(t_array, alpha1_t_array, label="alpha1_t")
    pylab.plot(t_array, distance_t_array, label="distance_t")
    pylab.plot(t_array, velocity_t_array, label="velocity_t")
    pylab.plot(t_array, acceleration_t_array, label="acceleration_t")
    pylab.legend()

    pylab.figure()
    pylab.title("Angular Velocities")
    pylab.plot(t_array, omega0_goal_t_array, label="omega0_t_goal")
    pylab.plot(t_array, omega0_t_array, label="omega0_t")
    pylab.plot(t_array, omega1_goal_t_array, label="omega1_t_goal")
    pylab.plot(t_array, omega1_t_array, label="omega1_t")
    pylab.legend()

    pylab.figure()
    pylab.title("Voltages")
    pylab.plot(t_array, u0_unsaturated_array, label="u0_full")
    pylab.plot(t_array, u0_array, label="u0")
    pylab.plot(t_array, uff0_array, label="uff0")
    pylab.plot(t_array, u1_unsaturated_array, label="u1_full")
    pylab.plot(t_array, u1_array, label="u1")
    pylab.plot(t_array, uff1_array, label="uff1")
    pylab.legend()

    pylab.figure()
    pylab.title("Angles")
    pylab.plot(t_array, theta0_goal_t_array, label="theta0_t_goal")
    pylab.plot(t_array, theta0_t_array, label="theta0_t")
    pylab.plot(t_array, theta1_goal_t_array, label="theta1_t_goal")
    pylab.plot(t_array, theta1_t_array, label="theta1_t")
    pylab.legend(loc='upper left')

    pylab.figure()
    pylab.title("Angles with Extended Kalman Filter State Values")
    pylab.plot(t_array, theta0_t_array, label="theta0_t")
    pylab.plot(t_array, theta0_hat_array, label="theta0_hat")
    pylab.plot(t_array, theta1_t_array, label="theta1_t")
    pylab.plot(t_array, theta1_hat_array, label="theta1_hat")
    pylab.legend(loc='upper left')

    pylab.figure()
    pylab.title("Angular Velocities with Extended Kalman Filter State Values")
    pylab.plot(t_array, omega0_t_array, label="omega0_t")
    pylab.plot(t_array, omega0_hat_array, label="omega0_hat")
    pylab.plot(t_array, omega1_t_array, label="omega1_t")
    pylab.plot(t_array, omega1_hat_array, label="omega1_hat")
    pylab.legend()

    pylab.figure()
    pylab.title("Disturbance Force from Extended Kalman Filter State Values")
    pylab.plot(t_array, torque_disturbance_0_hat_array, label="torque_disturbance_0_hat")
    pylab.plot(t_array, torque_disturbance_1_hat_array, label="torque_disturbance_1_hat")
    pylab.legend()


    pylab.show()


if __name__ == '__main__':
    main()
