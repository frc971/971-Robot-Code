#!/usr/bin/python3

import casadi
from casadi import cos, sin
from typing import Callable, Optional, Union, TypeVar

from frc971.control_loops.swerve import dynamics
from frc971.control_loops.python.control_loop import KrakenX60FOC

# This code based off this paper: Minimum-time speed optimisation over a fixed path (https://web.stanford.edu/~boyd/papers/pdf/speed_opt.pdf)
# It can certainly be optimized some more, including using fatrop
# and making the problem construction itself much faster (either by parameterizing/caching
# the construction or just by constructing it in a language faster than python).


# Convert a (possibly nested) list of MX single expressions to one MX matrix
def create_MX_from_MX(
        matrix: Union[list[list[casadi.MX]], list[casadi.MX]]) -> casadi.MX:
    if type(matrix[0]) is list:
        return casadi.vertcat(*[casadi.horzcat(*i) for i in matrix])
    else:
        return casadi.vertcat(*matrix)


kraken = KrakenX60FOC()


class SwervePath():
    """
    This class implements a Path for the optimizer to follow.

    This corresponds with Equation 3 in the paper, where the member `s` takes in a `theta \in [0, 1]` along the path; `s_p` and `s_pp` are the first and second derivatives of `s` with respect to `theta`.

    This provides a variety of factories for creating and combining common path types.
    """
    s: casadi.Function
    s_p: casadi.Function
    s_pp: casadi.Function

    def __init__(self, path_func: casadi.MX, theta: casadi.MX):
        s = path_func
        s_p = casadi.densify(casadi.jacobian(s, theta))
        s_pp = casadi.densify(casadi.jacobian(s_p, theta))

        self.s = casadi.Function("path", [theta], [s], {"allow_free": True})
        self.s_p = casadi.Function("path", [theta], [s_p],
                                   {"allow_free": True})
        self.s_pp = casadi.Function("path", [theta], [s_pp],
                                    {"allow_free": True})

    def _lerp(a, b, t):
        return a * (1 - t) + b * t

    def _spline_to_equation(spline_points, t):
        # Convert points to casadi DMs
        pointDMs = [casadi.DM(p) for p in spline_points]

        point_eqs = pointDMs
        while len(point_eqs) > 1:
            new_point_eqs = []
            for i in range(len(point_eqs) - 1):
                new_point_eqs.append(
                    SwervePath._lerp(point_eqs[i], point_eqs[i + 1], t))
            point_eqs = new_point_eqs
        return point_eqs[0]

    def spline_path(points: list[list[float]], start_rot: float,
                    end_rot: float):
        theta = casadi.MX.sym("theta")
        s_pos = SwervePath._spline_to_equation(points, theta)

        eq = casadi.vertcat(s_pos, SwervePath._lerp(start_rot, end_rot, theta))
        return SwervePath(eq, theta)

    def fullspline_path(points: list[list[float]]):
        """In this path, rotation is defined in the now 3D spline"""
        theta = casadi.MX.sym("theta")
        s = SwervePath._spline_to_equation(points, theta)
        return SwervePath(s, theta)

    def spline_path_spline_rot(points: list[list[float]], rots: list[float]):
        theta = casadi.MX.sym("theta")
        s_pos = SwervePath._spline_to_equation(points, theta)
        heading = SwervePath._spline_to_equation(rots, theta)
        eq = casadi.vertcat(s_pos, heading)
        return SwervePath(eq, theta)

    def line_path(start_point: list[float], start_rot: float,
                  end_point: list[float], end_rot: float):
        start_pos_DM = casadi.DM([*start_point, start_rot])
        end_pos_DM = casadi.DM([*end_point, end_rot])
        theta = casadi.MX.sym("theta")
        return SwervePath(SwervePath._lerp(start_pos_DM, end_pos_DM, theta),
                          theta)

    def control_rot(self, rot_eq: casadi.MX, theta: casadi.MX):
        """
        Override the heading function of the path with a custom equation in terms of theta
        """
        s = casadi.vertcat(self.s(theta)[:2], rot_eq)
        s_p = casadi.densify(casadi.jacobian(s, theta))
        s_pp = casadi.densify(casadi.jacobian(s_p, theta))

        self.s = casadi.Function("path", [theta], [s], {"allow_free": True})
        self.s_p = casadi.Function("path", [theta], [s_p],
                                   {"allow_free": True})
        self.s_pp = casadi.Function("path", [theta], [s_pp],
                                    {"allow_free": True})

    def control_rot_positional(self, rot_eq: casadi.MX,
                               pos_x_y: tuple[casadi.MX, casadi.MX]):
        """
        Override the heading function of the path with a custom equation in terms of its x and y position
        """
        theta = casadi.MX.sym("theta")
        theta_pos = self.s(theta)
        self.control_rot(
            casadi.substitute([rot_eq], [*pos_x_y],
                              [theta_pos[0], theta_pos[1]])[0], theta)

    def rotate_in_place(point: list[float], start_rot: float, end_rot: float):
        theta = casadi.MX.sym("theta")
        return SwervePath(
            casadi.vertcat(casadi.DM(point),
                           SwervePath._lerp(start_rot, end_rot, theta)), theta)

    def combine_paths(*paths):
        for i in range(len(paths) - 1):
            path1 = paths[i]
            path2 = paths[i + 1]
            if (casadi.norm_2(path1.s(1) - path2.s(0)) > 1e-10):
                raise ValueError(
                    f"Paths {i}, {i + 1} are not G0 continuous\n\tpath1: {path1.s(1)}\n\tpath2: {path2.s(0)}"
                )
            if (casadi.norm_2(path1.s_p(1) - path2.s_p(0)) > 1e-10):
                raise ValueError(
                    f"Paths {i}, {i + 1} are not G1 continuous\n\tpath1: {path1.s_p(1)}\n\tpath2: {path2.s_p(0)}"
                )
        theta = casadi.MX.sym("theta")
        expr = 0
        for i, path in enumerate(paths):
            sel = casadi.if_else(theta * len(paths) <= i + 1, 1, 0)
            if i != 0:
                sel = casadi.if_else(theta * len(paths) > i, 1,
                                     0) * casadi.if_else(
                                         theta * len(paths) <= i + 1, 1, 0)
            expr += path.s(theta * len(paths) - i) * sel
        return SwervePath(expr, theta)


SwerveSolutionType = TypeVar('SwerveSolutionType', bound='SwerveSolution')


class SwerveSolution():
    """A solution for a swerve robot to follow a given path"""

    # The number of discretization points
    n: int
    # The changes in time/'dt' for each interval (s)
    timesteps: list[float]
    # The actual times at each step (s)
    times: list[float]
    # The x, y, heading positions at each step (m, rad)
    positions: list[list[float]]
    # The dx, dy, and dheading at each step (m/s, rad/s)
    velocities: list[list[float]]
    # The ddx, ddy, and ddheading at each step (m/s^2, rad/s^2)
    accelerations: list[list[float]]
    # All forces exerted by each module at each timestep (N)
    module_forces: list[list[list[float]]]
    # Currents to the driving motors at each timestep (A)
    driving_currents: list[list[float]]
    # Voltages to the driving motors at each timestep (V)
    voltages: list[list[float]]
    # Lateral (non-motor) forces exerted by each module at each timestep (N)
    lat_forces: list[list[float]]
    # Velocity of each module at each timestep (m/s)
    mod_vels: list[list[list[float]]]
    # Rotational velocity of each module at each timestep (rad/s)
    # Likely not accurate on the actual robot for tight turns due to slip angle not being accounted for by the optimizer
    ang_vels: list[list[list[float]]]
    # The total length of the path (m)
    path_length: float
    # The SwervePath the optimizer used
    path: SwervePath
    # The restricted starting speed (m/s) - None if none was specified
    start_speed: Optional[float]
    # The restricted ending speed (m/s) - None if none was specified
    end_speed: Optional[float]
    # Whether the optimizer succeeded in finalizing its optimization
    valid: bool
    # The casadi Opti optimizer used
    opti: casadi.Opti
    # The casadi OptiSol solution to the problem
    sol: casadi.OptiSol

    def __repr__(self):
        return ("SwerveSolution(\n\tn: " + str(self.n) + "\n\tvalid: " +
                str(self.valid) + "\n\ttime: " + str(self.times[-1]) +
                "\n\tpath_length: " + str(self.path_length) + "\n)")

    def __init__(self,
                 n: int,
                 path: SwervePath,
                 start_speed: float = 0.0,
                 end_speed: float = 0.0,
                 callback: Callable[[int, SwerveSolutionType], None] = None,
                 solver_print=False,
                 prev_value=None,
                 max_current: Optional[float] = 40,
                 max_voltage: Optional[float] = 12,
                 max_velocity: Optional[float] = None,
                 max_acceleration: Optional[float] = None,
                 max_current_sections: list[tuple[float, float,
                                                  Optional[float]]] = [],
                 max_voltage_sections: list[tuple[float, float,
                                                  Optional[float]]] = [],
                 max_velocity_sections: list[tuple[float, float,
                                                   Optional[float]]] = [],
                 max_acceleration_sections: list[tuple[float, float,
                                                       Optional[float]]] = []):
        """
        n is the number of discretization points for the solver.\n
        callback will be called each iteration, and accepts the iteration number and the current working solution of the solver.\n
        start_speed and end_speed can be set to None to remove any constraints on starting and ending speed
        prev_value can be used to start from the previous state of a solve
        max_[thing]_sections is a list of overrides to the default maximum specified in max_[thing]
        """

        #system dynamics:
        #there are four wheel which can exert force in the directions they are moving in via a gear ratio from the motor
        #(see swerve dynamics in frc971/control_loops/swerve/swerve_notes.tex)
        #because slip angle makes the model unhappy when solving
        #the wheels can instead exert up to 0.5 radians worth of deflection of lateral force
        #while not actually changing where they point at all

        opti = casadi.Opti()

        m = opti.parameter()
        opti.set_value(m, dynamics.ROBOT_MASS)

        j = opti.parameter()
        opti.set_value(j, dynamics.ROBOT_MOI)

        self.start_speed = start_speed
        self.end_speed = end_speed

        def M(q_i):
            vec = casadi.MX(3, 1)
            vec[0, 0] = m
            vec[1, 0] = m
            vec[2, 0] = j
            return casadi.diag(vec)

        def R(q_i):
            c = cos(q_i[2])
            s = sin(q_i[2])
            ml = dynamics.mounting_location_static
            R = [[c, -s, c, -s, c, -s, c, -s], [s, c, s, c, s, c, s, c],
                 [
                     -ml(0)[1],
                     ml(0)[0], -ml(1)[1],
                     ml(1)[0], -ml(2)[1],
                     ml(2)[0], -ml(3)[1],
                     ml(3)[0]
                 ]]
            R = create_MX_from_MX(R)
            return R

        def C(q_i, q_dot_i):
            return casadi.MX.zeros(3, 3)

        def d(q_i):
            return casadi.MX.zeros(3)

        def Rot(phi):
            return create_MX_from_MX([[casadi.cos(phi), -casadi.sin(phi)],
                                      [casadi.sin(phi),
                                       casadi.cos(phi)]])

        def Rot_n(phi, n):
            return Rot(phi + n * casadi.pi / 2)

        def get_module_info(q_dot_i, q_dot_dot_i, u_i, q_i, q_dot_sq_i=None):
            if q_dot_sq_i is None:
                q_dot_sq_i = q_dot_i * q_dot_i
            i_d = []
            lf = []
            ang_vels = []
            mod_vels = []
            z_dots = []
            # iterate through the modules and calculate things about them
            for i in range(4):
                loc = dynamics.mounting_location_static(i)
                #the position of the module
                z = Rot(q_i[2]) @ loc + q_i[:2]
                #the velocity of the module
                z_dot = q_dot_i[2] * Rot_n(q_i[2], 1) @ loc + q_dot_i[:2]
                z_dots.append(z_dot)
                z_dot_norm_2 = casadi.norm_2(z_dot)
                mod_vels.append(z_dot_norm_2)
                #the acceleration of the module
                z_dot_dot = (q_dot_dot_i[2] * (Rot_n(q_i[2], 1) @ loc) +
                             q_dot_sq_i[2] * (Rot_n(q_i[2], 2) @ loc) +
                             q_dot_dot_i[0:2])
                #the angular velocity of the module
                z_ang_vel = (z_dot[0] * z_dot_dot[1] - z_dot[1] * z_dot_dot[0]
                             ) / (z_dot[0]**2 + z_dot[1]**2) - q_dot_i[2]
                ang_vels.append(z_ang_vel)

                #Get the force exerted laterally (Fl) and longitudinally (Fwx)
                f = Rot(q_i[2]) @ u_i[i * 2:i * 2 + 2]
                Fwx = casadi.dot(f, z_dot / z_dot_norm_2)
                Fl = casadi.dot(f, (Rot(casadi.pi / 2) @ z_dot) / z_dot_norm_2)

                i_d.append(dynamics.driving_current_from_force(Fwx))
                lf.append(Fl)
            return i_d, lf, ang_vels, mod_vels, z_dots

        def constrain_sections(constraint_list, sections, fallback, i,
                               constraint_func):
            for section in sections:
                if i / n >= section[0] and i / n <= section[1]:
                    if not section[2] is None:
                        constraint_list.append(constraint_func(section[2]))
                    break
            else:
                if not fallback is None:
                    constraint_list.append(constraint_func(fallback))

        def constraints(q_dot_sq_i, q_dot_dot_i, u_i, q_i, i):
            q_dot_i = casadi.sign(path.s_p(theta(i))) * casadi.sqrt(q_dot_sq_i)
            i_d, lf, s_v, m_v, zds = get_module_info(q_dot_i,
                                                     q_dot_dot_i,
                                                     u_i,
                                                     q_i,
                                                     q_dot_sq_i=q_dot_sq_i)

            constraint_list = []
            for j in range(4):
                d_motor_vel = dynamics.drive_motor_vel(m_v[j], s_v[j])

                voltage = d_motor_vel / kraken.Kv + i_d[j] * kraken.resistance

                constrain_sections(
                    constraint_list, max_current_sections, max_current, i,
                    lambda limit: opti.bounded(-limit, i_d[j], limit))

                constrain_sections(
                    constraint_list, max_voltage_sections, max_voltage, i,
                    lambda limit: opti.bounded(-limit, voltage, limit))

                constraint_list.append(
                    opti.bounded(-0.5 * dynamics.CORNERING_TIRE_STIFFNESS,
                                 lf[j],
                                 0.5 * dynamics.CORNERING_TIRE_STIFFNESS))

            vel_sq = q_dot_sq_i[0] + q_dot_sq_i[1]
            constrain_sections(constraint_list, max_velocity_sections,
                               max_velocity, i,
                               lambda limit: vel_sq <= limit**2)
            accel_sq = q_dot_dot_i[0]**2 + q_dot_dot_i[1]**2
            constrain_sections(constraint_list, max_acceleration_sections,
                               max_acceleration, i,
                               lambda limit: accel_sq <= limit**2)

            return constraint_list

        def R_n(theta):
            return R(path.s(theta))

        def m_n(theta):
            return M(path.s(theta)) @ path.s_p(theta)

        def c_n(theta):
            return M(path.s(theta)) @ path.s_pp(theta) + C(
                path.s(theta),
                path.s_p(theta) *
                b_disc[round(theta * (n - 1))]) @ (path.s_p(theta)**2)

        def d_n(theta):
            return d(path.s(theta))

        a_disc = [None, *[opti.variable() for _ in range(1, n)]]
        b_disc = [opti.variable() for _ in range(n)]
        for b in b_disc:
            opti.set_initial(b, 0.001)
        u_disc = [None, *[opti.variable(8) for _ in range(1, n)]]
        for u in u_disc:
            if not u is None:
                opti.set_initial(u, [1e-2] * 8)

        dtheta = 1 / (n - 1)

        def theta(i):
            return i / (n - 1)

        def time(c=(n - 1)):
            sum = 0
            for i in range(c):
                sum = sum + dtheta / ((b_disc[i])**0.5 + (b_disc[i + 1])**0.5)
            return 2 * sum

        opti.minimize(time())

        def theta_bar(i):
            return (theta(i - 1) + theta(i)) / 2

        def q_dot(i):
            return path.s_p(theta(i)) * casadi.sqrt(b_disc[i])

        self._q_dot = q_dot

        def b_bar(i):
            return 2 * (theta_bar(i + 1) - theta(i)) * a_disc[i] + b_disc[i -
                                                                          1]

        def q_dot_dot(i):
            return path.s_p(theta(i)) * a_disc[i] + path.s_pp(
                theta(i)) * b_bar(i)

        def constraint_n(a_i, b_i, u_i, theta, i):
            return constraints(
                path.s_p(theta)**2 * b_i, q_dot_dot(i), u_i, path.s(theta), i)

        def constrain():
            # enforcing differential constraints at the midpoints
            for i in range(1, n):
                opti.subject_to(
                    R_n(theta_bar(i)) @ u_disc[i] == m_n(theta_bar(i)) *
                    a_disc[i] + c_n(theta_bar(i)) *
                    (b_disc[i - 1] + b_disc[i]) / 2 + d_n(theta_bar(i)))
                pass

            # enforcing constraints
            for i in range(1, n):
                opti.subject_to(
                    constraint_n(a_disc[i], b_disc[i], u_disc[i], theta(i), i))
                pass

            # enforcing derivatives
            for i in range(1, n):
                opti.subject_to(b_disc[i] - b_disc[i - 1] == 2 * a_disc[i] *
                                dtheta)

            # b > 0 (implied but helps solver)
            for b in b_disc:
                opti.subject_to(b > 0)

            start_speed = self.start_speed
            end_speed = self.end_speed

            if start_speed < 1e-3:
                start_speed = 1e-3
            if end_speed < 1e-3:
                end_speed = 1e-3

            if not (start_speed is None):
                s_vel = q_dot(0)
                opti.subject_to(casadi.norm_2(s_vel) ==
                                start_speed)  # set initial velocity

            if not (end_speed is None):
                e_vel = q_dot(n - 1)
                opti.subject_to(
                    casadi.norm_2(e_vel) == end_speed)  # set final velocity

        self._constrain = constrain
        constrain()

        p_opts = {"expand": True}
        s_opts = {"max_iter": 3000, "print_level": 5}
        if not solver_print:
            p_opts["print_time"] = 0
            s_opts["print_level"] = 0

        opti.solver("ipopt", p_opts, s_opts)

        if not prev_value is None:
            opti.set_initial(prev_value)

        def InitSwerveSolution(sol, obj: SwerveSolutionType, val):
            obj.n = n
            obj.valid = val
            obj.timesteps = [
                sol.value(2 * dtheta / ((b_disc[i])**0.5 +
                                        (b_disc[i + 1])**0.5))
                for i in range(n - 1)
            ]
            t = 0
            obj.times = [
                t,
                *[(t := t + obj.timesteps[i]) for i in range(n - 1)],
            ]
            obj.positions = [sol.value(path.s(theta(i))) for i in range(n)]
            obj.velocities = [sol.value(q_dot(i)) for i in range(n)]
            obj.accelerations = [sol.value(q_dot_dot(i)) for i in range(1, n)]
            obj.module_forces = [[
                *zip(sol.value(u_disc[i])[::2],
                     sol.value(u_disc[i])[1::2])
            ] for i in range(1, n)]
            obj.driving_currents = []
            obj.voltages = []
            obj.lat_forces = []
            obj.mod_vels = []
            obj.ang_vels = []
            for i in range(1, n):
                i_d, lat_force, s_v, m_v, zds = get_module_info(
                    q_dot(i), q_dot_dot(i), u_disc[i], path.s(theta(i)))
                voltage = [0] * 4
                for k in range(4):
                    d_motor_vel = dynamics.drive_motor_vel(m_v[k], s_v[k])
                    voltage[k] = sol.value(d_motor_vel / kraken.Kv +
                                           i_d[k] * kraken.resistance)
                obj.voltages.append(voltage)
                obj.driving_currents.append(
                    [sol.value(i_d[j]) for j in range(4)])

                obj.lat_forces.append(
                    [sol.value(lat_force[j]) for j in range(4)])
                obj.mod_vels.append([sol.value(zds[j]) for j in range(4)])
                obj.ang_vels.append([sol.value(s_v[j]) for j in range(4)])

            obj.path_length = sum(
                ((obj.positions[i][0] - obj.positions[i + 1][0])**2 +
                 (obj.positions[i][1] - obj.positions[i + 1][1])**2)**0.5
                for i in range(len(obj.positions) - 1))
            obj.path = path
            obj.opti = opti
            obj.sol = sol

            return obj

        if callback:
            opti.callback(lambda i: callback(
                i, InitSwerveSolution(opti.debug, {}, False)))
        try:
            sol: casadi.OptiSol = opti.solve()
        except RuntimeError:
            InitSwerveSolution(opti.debug, self, False)
            return
        InitSwerveSolution(sol, self, True)
        return
