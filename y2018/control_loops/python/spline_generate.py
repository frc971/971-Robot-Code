#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
from frc971.control_loops.python import drivetrain

# used to define properties of the drivetrain, changes depending on robot
# see yXXXX/control_loops/python/drivetrain.py for the current values

kDrivetrain = drivetrain.DrivetrainParams(
    J=6.0,
    mass=68.0,
    robot_radius=0.616 / 2.0,
    wheel_radius=0.127 / 2.0 * 120.0 / 118.0,
    G_low=46.0 / 60.0 * 20.0 / 48.0 * 14.0 / 62.0,
    G_high=62.0 / 44.0 * 20.0 / 48.0 * 14.0 / 62.0,
    q_pos_low=0.12,
    q_pos_high=0.14,
    q_vel_low=1.0,
    q_vel_high=0.95,
    efficiency=0.70,
    has_imu=True,
    force=True,
    kf_q_voltage=13.0,
    controller_poles=[0.82, 0.82],
)

drivetrain = drivetrain.Drivetrain(kDrivetrain)
# set up coefficients for Hemrite basis function evaluation
coeffs = np.array([[1, 0, 0, -10, 15, -6], [0, 1, 0, -6, 8, -3],
                   [0, 0, 0.5, -1.5, 1.5, -0.5], [0, 0, 0, 0.5, -1, 0.5],
                   [0, 0, 0, -4, 7, -3], [0, 0, 0, 10, -15, 6]])
coeffs_prime = np.empty_like(coeffs)
for ii in range(0, len(coeffs)):
    for jj in range(0, len(coeffs[ii]) - 1):
        coeffs_prime[ii][jj] = (jj + 1) * coeffs[ii][jj]


def RungeKutta(f, x, dt):
    """4th order RungeKutta integration of F starting at X."""
    a = f(x)
    b = f(x + dt / 2.0 * a)
    c = f(x + dt / 2.0 * b)
    d = f(x + dt * c)

    return x + dt * (a + 2.0 * b + 2.0 * c + d) / 6.0


def normalize(v):
    norm = np.linalg.norm(v)
    return v / norm


def theta(v):
    return np.arctan2(v[1], v[0])


# evaluate Nth hermite basis function at t
def nth_H(N, t):
    return coeffs[N][0] + coeffs[N][1] * t + coeffs[N][2] * t**2 + coeffs[N][
        3] * t**3 + coeffs[N][4] * t**4 + coeffs[N][5] * t**5


def nth_H_prime(N, t):
    return coeffs[N][0] + coeffs[N][1] * t + coeffs[N][2] * t**2 + coeffs[N][
        3] * t**3 + coeffs[N][4] * t**4


# class defining a quintic Hermite spline, with utilities for modification and plotting
class Hermite_Spline:
    # init method given known parameters, ie savefile loading(if necessary)
    def __init__(self, start, control1, control2, end, resolution=200):
        self.start = start
        self.end = end
        self.control1 = control1
        self.control2 = control2

        self.points = np.array([])
        self.velocities = []
        self.accelerations = []
        self.arc_lengths = []
        self.thetas = []
        self.omegas = []
        self.curvatures = []

        self.shifted_points = []

        self.Ks = []
        self.dKs = []

        # coefficients are po, v0, a0, a1, v1, p1
        self.coeffs = np.array([])
        self.compute_coefficients()
        self.resolution = resolution
        self.setup()

    # take paramters and compute coeffcicents for Hermite basis functions, to be called every time he change control points
    def compute_coefficients(self):
        self.coeffs = np.append(self.coeffs, np.array(self.start))
        self.coeffs = np.append(self.coeffs,
                                np.array(self.control1) - np.array(self.start))
        self.coeffs = np.append(self.coeffs, [0, 0])
        self.coeffs = np.append(self.coeffs, [0, 0])
        self.coeffs = np.append(self.coeffs,
                                np.array(self.end) - np.array(self.control2))
        self.coeffs = np.append(self.coeffs, np.array(self.end))

        self.coeffs = np.reshape(self.coeffs, newshape=(6, 2))

    # setters for control points, set coefficients
    def set_positions(self, p1=None, p2=None):
        if p1 != None:
            self.start = p1
        if p2 != None:
            self.end = p2

        self.compute_coefficients()

    def set_controls(self, c1=None, c2=None):
        if c1 != None:
            self.control1 = c1
        if c2 != None:
            self.control2 = c2

        self.compute_coefficients()

    def set_velocities(self, v1=None, v2=None):
        if v1 != None:
            self.control1 = self.start + v1
        if v2 != None:
            self.control2 = self.end + v2

        self.compute_coefficients()

    def get_smoothness(self):
        K = self.get_curvature()
        return np.sum(np.abs(np.gradient(K)))

    # given Basis functions and controls compute coordinate given t
    def spline_eval_hermite(self, t):
        return np.array(self.coeffs[0] * nth_H(0, t) +
                        self.coeffs[1] * nth_H(1, t) +
                        self.coeffs[2] * nth_H(2, t) +
                        self.coeffs[3] * nth_H(3, t) +
                        self.coeffs[4] * nth_H(4, t) +
                        self.coeffs[5] * nth_H(5, t))

    # given Basis functions and controls compute velocity given t
    def spline_eval_hermite_v(self, t):
        return normalize(
            np.array(self.coeffs[0] * nth_H_prime(0, t) +
                     self.coeffs[1] * nth_H_prime(1, t) +
                     self.coeffs[2] * nth_H_prime(2, t) +
                     self.coeffs[3] * nth_H_prime(3, t) +
                     self.coeffs[4] * nth_H_prime(4, t) +
                     self.coeffs[5] * nth_H_prime(5, t)))

    # take coefficients and compute spline points/properties
    def setup(self, resolution_multiplier=10, dt=.000001):
        points = []
        velocities = []
        accelerations = []
        s = []
        thetas = []
        omegas = []
        curvatures = []

        last_point = self.spline_eval_hermite(0)
        distance = 0

        # iterate through interim points and compute pos_vectors, and at predefined points arc length,
        # velocity, and acceleration vectors and store them at their associated index
        for i in range(0, self.resolution * resolution_multiplier):
            t = i / (1.0 * self.resolution * resolution_multiplier)

            current_point = self.spline_eval_hermite(t)
            current_point_dt = self.spline_eval_hermite(t + dt)
            current_s = np.linalg.norm(current_point - last_point)

            ds = np.linalg.norm(current_point_dt - current_point)

            distance = current_s + distance
            # at important points compute important values and store
            if i % resolution_multiplier == 0:
                s.append(distance)
                points.append(current_point)

                v = self.spline_eval_hermite_v(t)
                v_dt = self.spline_eval_hermite_v(t + dt)
                theta_t = theta(v)
                theta_dt = theta(v_dt)

                a = (v_dt - v) / ds
                omega = (theta_dt - theta_t) / ds
                if np.linalg.norm(v) == 0:
                    curvature = 0
                else:
                    curvature = np.linalg.det(
                        np.column_stack((v, a)) / (np.linalg.norm(v)**(3 / 2)))

                velocities.append(v)
                accelerations.append(a)
                thetas.append(theta_t)
                omegas.append(omega)
                if curvature == 0:
                    curvatures.append(0.0001)
                else:
                    curvatures.append(curvature)

            last_point = current_point

        self.arc_lengths = np.array(s)
        self.points = np.reshape(points, newshape=(-1, 2))
        self.velocities = np.reshape(velocities, newshape=(-1, 2))
        self.accelerations = np.reshape(accelerations, newshape=(-1, 2))
        self.thetas = np.array(thetas)
        self.omegas = np.array(omegas)
        self.curvatures = np.array(curvatures)

    def plot_diagnostics(self):
        plt.figure("Spline")
        plt.title('Spline')
        plt.plot(self.points[:, 0], self.points[:, 1])
        # plt.scatter(self.points[:, 0], self.points[:, 1])

        plt.figure("Diagnostics")

        plt.subplot(2, 2, 1)
        plt.title('theta')
        plt.xlabel('arc_length')
        plt.ylabel('theta')
        theta, = plt.plot(self.arc_lengths, self.thetas, label='theta')
        plt.legend(handles=[theta])

        plt.subplot(2, 2, 2)
        plt.title('omegas')
        plt.xlabel('arc_length')
        plt.ylabel('omega')
        omega, = plt.plot(self.arc_lengths, self.omegas, label='omega')
        plt.legend(handles=[omega])

        plt.subplot(2, 2, 3)
        plt.title('Velocities')
        plt.xlabel('arc_length')
        plt.ylabel('velocity')
        dxds, = plt.plot(self.arc_lengths,
                         self.velocities[:, 0],
                         label='dx/ds')
        dyds, = plt.plot(self.arc_lengths,
                         self.velocities[:, 1],
                         label='dy/ds')
        plt.legend(handles=[dxds, dyds])

        plt.subplot(2, 2, 4)
        plt.title('Accelerations')
        plt.xlabel('arc_length')
        plt.ylabel('acceleration')
        dx2ds2, = plt.plot(self.arc_lengths,
                           self.accelerations[:, 0],
                           label='d^2x/ds^2')
        dy2ds2, = plt.plot(self.arc_lengths,
                           self.accelerations[:, 1],
                           label='d^2x/ds^2')
        plt.legend(handles=[dx2ds2, dy2ds2])


# class defining a number of splines with convinience methods
class Path:

    def __init__(self):
        self.splines = []
        self.knot_accels = []

    def add_spline(self, spline):
        self.splines.append(spline)

    def get_K(self):
        curvatures = []
        for spline in self.splines:
            curvatures.append(spline.curvatures)
        return np.array(curvatures).flatten()

    def get_S(self):
        arc_lengths = []
        for spline in self.splines:
            arc_lengths.append(spline.arc_lengths)
        return np.array(arc_lengths).flatten()

    def get_points(self):
        points = []
        for spline in self.splines:
            points.append(spline.points)
        return points

    def get_velocities(self, i):
        velocities = []
        for spline in self.splines:
            velocities.append(spline.points)
        return velocities

    def remove_spine(self, i):
        if i < len(self.splines):
            self.splines.pop(i)
        else:
            print("index %f out of bounds, no spline of that index" % i)

    def join(self, first_priority=False):
        for i in range(0, len(self.splines)):
            if first_priority & i != len(self.splines):
                print("unfinished")


# class which takes a Path object along with constraints and reparamterizes it with respect to time
class Trajectory:

    def __init__(self,
                 path,
                 max_angular_accel=3,
                 max_voltage=11,
                 max_normal_accel=.2):
        self.path = path
        self.A = drivetrain.A_continuous
        self.B = drivetrain.B_continuous
        self.robot_radius = drivetrain.robot_radius
        self.Kv = 100
        self.robot_radius = 3
        self.max_angular_accel = max_angular_accel
        self.max_voltage = max_voltage
        self.max_normal_accel = max_normal_accel

        self.max_velocities_adhering_to_normal_accel = []
        self.max_velocities_adhering_to_voltage = []
        self.path.splines[0].setup(resolution_multiplier=100)

        self.set_max_v_adhering_to_normal_accel()
        self.max_voltageK_pass()

    def set_max_v_adhering_to_normal_accel(self):
        Ks = self.path.get_K()
        accels = np.full_like(Ks, fill_value=self.max_normal_accel)
        max_velocities = np.sqrt(np.abs(accels / Ks))
        self.max_velocities_adhering_to_normal_accel = max_velocities

    def max_voltageK_pass(self):
        max_ds_dt = []
        Ks = self.path.get_K()
        turning_radii = np.full_like(Ks, fill_value=1) / np.abs(Ks)

        # compute max steady-state velocity given voltage constraints
        for i in range(0, len(Ks)):
            v_ratio = (turning_radii[i] + self.robot_radius) / (
                turning_radii[i] - self.robot_radius)
            matrix = np.array([[self.A[1, 1], self.A[1, 3], self.B[1, 1]],
                               [self.A[3, 1] - 1, self.A[3, 3], self.B[3, 1]],
                               [-1, v_ratio, 0]])
            sols = np.array([
                -1 * self.max_voltage * self.B[1, 0],
                -1 * self.max_voltage * self.B[3, 0], 0
            ])
            Vs = np.dot(np.linalg.inv(matrix), sols)
            max_ds_dt.append((Vs[0] + Vs[1]) / 2)

        self.max_velocities_adhering_to_voltage = max_ds_dt

    # compute the maximum acceleration we can ask for given voltage and, ya know, staying on the path.
    '''
    These methods use the continuous form of our drivetrain state equation
    in order to compute the maximum acceleration which adheres to the path
    and voltage constraints, as well as any arbitary set of constraints
    on velocity as a function of arc_length
    '''

    def forward_accel_pass(self):
        points = self.path.get_points()
        velocities = self.path.get_velocities()
        curvatures = self.path.get_K()
        arc_lenghts = self.path.get_S()

        for i in range(0, len(points)):
            #Xn1 =
            pass

    def backward_accelaration_pass(self):

        print("max backward accel pass")

    def plot_diagnostics(self, i=0):

        plt.figure('max velocity')
        plt.title('max_v_normal_accel')
        plt.xlabel('arc_length')
        plt.ylabel('max V')
        max_v_normal = plt.plot(self.path.get_S(),
                                self.max_velocities_adhering_to_normal_accel,
                                label='ds/dt (normal)')  #   , label = 'ds/dt')
        curvature = plt.plot(self.path.get_S(),
                             1000 * np.abs(self.path.get_K()),
                             label='K')
        max_v_K_V = plt.plot(self.path.get_S(),
                             self.max_velocities_adhering_to_voltage,
                             label='ds/dt (voltage)')
        plt.legend(handles=[max_v_normal[0], curvature[0], max_v_K_V[0]])


def main():
    A = Hermite_Spline(np.array([0, 0]),
                       np.array([0, 400]),
                       np.array([200, 300]),
                       np.array([200, 200]),
                       resolution=200)
    A.plot_diagnostics()
    path = Path()
    path.add_spline(A)
    trajectory = Trajectory(path, 0)
    trajectory.plot_diagnostics()
    plt.show()


main()
