import abc
import numpy as np
import sys
import traceback

# joint_center in x-y space.
IN_TO_M = 0.0254
joint_center = (-0.203, 0.787)

# Joint distances (l1 = "proximal", l2 = "distal")
l1 = 20.0 * IN_TO_M
l2 = 38.0 * IN_TO_M

max_dist = 0.01
max_dist_theta = np.pi / 64
xy_end_circle_size = 0.01
theta_end_circle_size = 0.07


# Shift the angle between the convention used for input/output and the convention we use for some computations here
def shift_angle(theta):
    return np.pi / 2 - theta


def shift_angles(thetas):
    return [shift_angle(theta) for theta in thetas]


# Convert from x-y coordinates to theta coordinates.
# orientation is a bool. This orientation is circular_index mod 2.
# where circular_index is the circular index, or the position in the
# "hyperextension" zones. "cross_point" allows shifting the place where
# it rounds the result so that it draws nicer (no other functional differences).
def to_theta(pt, circular_index, cross_point=-np.pi, die=True):
    orient = (circular_index % 2) == 0
    x = pt[0]
    y = pt[1]
    x -= joint_center[0] - 1e-9
    y -= joint_center[1]
    l3 = np.hypot(x, y)
    t3 = np.arctan2(y, x)
    theta1 = np.arccos((l1**2 + l3**2 - l2**2) / (2 * l1 * l3))
    if np.isnan(theta1):
        print(("Couldn't fit triangle to %f, %f, %f" % (l1, l2, l3)))
        if die:
            traceback.print_stack()
            sys.exit(1)
        return None

    if orient:
        theta1 = -theta1
    theta1 += t3
    theta1 = (theta1 - cross_point) % (2 * np.pi) + cross_point
    theta2 = np.arctan2(y - l1 * np.sin(theta1), x - l1 * np.cos(theta1))
    return np.array((theta1, theta2))


# Simple trig to go back from theta1, theta2 to x-y
def to_xy(theta1, theta2):
    x = np.cos(theta1) * l1 + np.cos(theta2) * l2 + joint_center[0]
    y = np.sin(theta1) * l1 + np.sin(theta2) * l2 + joint_center[1]
    orient = ((theta2 - theta1) % (2.0 * np.pi)) < np.pi
    return (x, y, orient)


END_EFFECTOR_X_LEN = (-1.0 * IN_TO_M, 10.425 * IN_TO_M)
END_EFFECTOR_Y_LEN = (-3.5 * IN_TO_M, 7.325 * IN_TO_M)
END_EFFECTOR_Z_LEN = (-11.0 * IN_TO_M, 11.0 * IN_TO_M)


def abs_sum(l):
    result = 0
    for e in l:
        result += abs(e)
    return result


def affine_3d(R, T):
    H = np.eye(4)
    H[:3, 3] = T
    H[:3, :3] = R
    return H


# Simple trig to go back from theta1, theta2, and theta3 to
# the 8 corners on the roll joint x-y-z
def to_end_effector_points(theta1, theta2, theta3):
    x, y, _ = to_xy(theta1, theta2)
    # Homogeneous end effector points relative to the end_effector
    # ee = end effector
    endpoints_ee = []
    for i in range(2):
        for j in range(2):
            for k in range(2):
                endpoints_ee.append(
                    np.array((END_EFFECTOR_X_LEN[i], END_EFFECTOR_Y_LEN[j],
                              END_EFFECTOR_Z_LEN[k], 1.0)))

    # Only roll.
    # rj = roll joint
    roll = theta3
    T_rj_ee = np.zeros(3)
    R_rj_ee = np.array([[1.0, 0.0, 0.0], [0.0,
                                          np.cos(roll), -np.sin(roll)],
                        [0.0, np.sin(roll), np.cos(roll)]])
    H_rj_ee = affine_3d(R_rj_ee, T_rj_ee)

    # Roll joint pose relative to the origin
    # o = origin
    T_o_rj = np.array((x, y, 0))
    # Only yaw
    yaw = theta1 + theta2
    R_o_rj = [[np.cos(yaw), -np.sin(yaw), 0.0],
              [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
    H_o_rj = affine_3d(R_o_rj, T_o_rj)

    # Now compute the pose of the end effector relative to the origin
    H_o_ee = H_o_rj @ H_rj_ee

    # Get the translation from these transforms
    endpoints_o = [(H_o_ee @ endpoint_ee)[:3] for endpoint_ee in endpoints_ee]

    diagonal_distance = np.linalg.norm(
        np.array(endpoints_o[0]) - np.array(endpoints_o[-1]))
    actual_diagonal_distance = np.linalg.norm(
        np.array((abs_sum(END_EFFECTOR_X_LEN), abs_sum(END_EFFECTOR_Y_LEN),
                  abs_sum(END_EFFECTOR_Z_LEN))))
    assert abs(diagonal_distance - actual_diagonal_distance) < 1e-5

    return np.array(endpoints_o)


# Returns all permutations of rectangle points given two opposite corners.
# x is the two x values, y is the two y values, z is the two z values
def rect_points(x, y, z):
    points = []
    for i in range(2):
        for j in range(2):
            for k in range(2):
                points.append((x[i], y[j], z[k]))
    return np.array(points)


DRIVER_CAM_Z_OFFSET = 3.225 * IN_TO_M
DRIVER_CAM_POINTS = rect_points(
    (-5.126 * IN_TO_M + joint_center[0], 0.393 * IN_TO_M + joint_center[0]),
    (5.125 * IN_TO_M + joint_center[1], 17.375 * IN_TO_M + joint_center[1]),
    (-8.475 * IN_TO_M - DRIVER_CAM_Z_OFFSET,
     -4.350 * IN_TO_M - DRIVER_CAM_Z_OFFSET))


def rect_collision_1d(min_1, max_1, min_2, max_2):
    return (min_1 <= min_2 <= max_1) or (min_1 <= max_2 <= max_1) or (
        min_2 < min_1 and max_2 > max_1)


def roll_joint_collision(theta1, theta2, theta3):
    theta1 = shift_angle(theta1)
    theta2 = shift_angle(theta2)
    theta3 = shift_angle(theta3)

    end_effector_points = to_end_effector_points(theta1, theta2, theta3)

    assert len(end_effector_points) == 8 and len(end_effector_points[0]) == 3
    assert len(DRIVER_CAM_POINTS) == 8 and len(DRIVER_CAM_POINTS[0]) == 3
    collided = True

    for i in range(len(end_effector_points[0])):
        min_ee = min(end_effector_points[:, i])
        max_ee = max(end_effector_points[:, i])

        min_dc = min(DRIVER_CAM_POINTS[:, i])
        max_dc = max(DRIVER_CAM_POINTS[:, i])

        collided &= rect_collision_1d(min_ee, max_ee, min_dc, max_dc)
    return collided


# Delta limit means theta2 - theta1.
# The limit for the proximal and distal is relative,
# so define constraints for this delta.
UPPER_DELTA_LIMIT = 0.0
LOWER_DELTA_LIMIT = -1.9 * np.pi

# TODO(milind): put actual proximal limits
UPPER_PROXIMAL_LIMIT = np.pi * 1.5
LOWER_PROXIMAL_LIMIT = -np.pi

UPPER_DISTAL_LIMIT = 0.75 * np.pi
LOWER_DISTAL_LIMIT = -0.75 * np.pi

UPPER_ROLL_JOINT_LIMIT = 0.75 * np.pi
LOWER_ROLL_JOINT_LIMIT = -0.75 * np.pi


def arm_past_limit(theta1, theta2, theta3):
    delta = theta2 - theta1
    return delta > UPPER_DELTA_LIMIT or delta < LOWER_DELTA_LIMIT or \
            theta1 > UPPER_PROXIMAL_LIMIT or theta1 < LOWER_PROXIMAL_LIMIT or \
            theta2 > UPPER_DISTAL_LIMIT or theta2 < LOWER_DISTAL_LIMIT or \
            theta3 > UPPER_ROLL_JOINT_LIMIT or theta3 < LOWER_ROLL_JOINT_LIMIT


def get_circular_index(theta):
    return int(
        np.floor((shift_angle(theta[1]) - shift_angle(theta[0])) / np.pi))


def get_xy(theta):
    theta1 = shift_angle(theta[0])
    theta2 = shift_angle(theta[1])
    x = np.cos(theta1) * l1 + np.cos(theta2) * l2 + joint_center[0]
    y = np.sin(theta1) * l1 + np.sin(theta2) * l2 + joint_center[1]
    return np.array((x, y))


# Subdivide in theta space.
def subdivide_theta(lines):
    out = []
    last_pt = lines[0]
    out.append(last_pt)
    for n_pt in lines[1:]:
        for pt in subdivide(last_pt, n_pt, max_dist_theta):
            out.append(pt)
        last_pt = n_pt

    return out


def to_theta_with_ci(pt, circular_index):
    return (to_theta_with_circular_index(pt[0], pt[1], circular_index))


# to_theta, but distinguishes between
def to_theta_with_circular_index(x, y, circular_index):
    theta1, theta2 = to_theta((x, y), circular_index)
    n_circular_index = int(np.floor((theta2 - theta1) / np.pi))
    theta2 = theta2 + ((circular_index - n_circular_index)) * np.pi
    return np.array((shift_angle(theta1), shift_angle(theta2)))


# to_theta, but distinguishes between
def to_theta_with_circular_index_and_roll(x, y, roll, circular_index):
    theta12 = to_theta_with_circular_index(x, y, circular_index)
    theta3 = roll
    return np.array((theta12[0], theta12[1], theta3))


# alpha is in [0, 1] and is the weight to merge a and b.
def alpha_blend(a, b, alpha):
    """Blends a and b.

    Args:
      alpha: double, Ratio.  Needs to be in [0, 1] and is the weight to blend a
          and b.
    """
    return b * alpha + (1.0 - alpha) * a


def normalize(v):
    """Normalize a vector while handling 0 length vectors."""
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


# Generic subdivision algorithm.
def subdivide(p1, p2, max_dist):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dist = np.sqrt(dx**2 + dy**2)
    n = int(np.ceil(dist / max_dist))
    return [(alpha_blend(p1[0], p2[0],
                         float(i) / n), alpha_blend(p1[1], p2[1],
                                                    float(i) / n))
            for i in range(1, n + 1)]


def spline_eval(start, control1, control2, end, alpha):
    a = alpha_blend(start, control1, alpha)
    b = alpha_blend(control1, control2, alpha)
    c = alpha_blend(control2, end, alpha)
    return alpha_blend(alpha_blend(a, b, alpha), alpha_blend(b, c, alpha),
                       alpha)


SPLINE_SUBDIVISIONS = 100


def subdivide_multistep():
    # TODO: pick N based on spline parameters? or otherwise change it to be more evenly spaced?
    for i in range(0, SPLINE_SUBDIVISIONS + 1):
        yield i / float(SPLINE_SUBDIVISIONS)


def get_proximal_distal_derivs(t_prev, t, t_next):
    d_prev = normalize(t - t_prev)
    d_next = normalize(t_next - t)
    accel = (d_next - d_prev) / np.linalg.norm(t - t_next)
    return (ThetaPoint(t[0], d_next[0],
                       accel[0]), ThetaPoint(t[1], d_next[1], accel[1]))


def get_roll_joint_theta(theta_i, theta_f, t):
    # Fit a theta(t) = (1 - cos(pi*t)) / 2,
    # so that theta(0) = theta_i, and theta(1) = theta_f
    offset = theta_i
    scalar = (theta_f - theta_i) / 2.0
    freq = np.pi
    theta_curve = lambda t: scalar * (1 - np.cos(freq * t)) + offset

    return theta_curve(t)


def get_roll_joint_theta_multistep(alpha_rolls, alpha):
    # Figure out which segment in the motion we're in
    theta_i = None
    theta_f = None
    t = None

    for i in range(len(alpha_rolls) - 1):
        # Find the alpha segment we're in
        if alpha_rolls[i][0] <= alpha <= alpha_rolls[i + 1][0]:
            theta_i = alpha_rolls[i][1]
            theta_f = alpha_rolls[i + 1][1]

            total_dalpha = alpha_rolls[-1][0] - alpha_rolls[0][0]
            assert total_dalpha == 1.0
            dalpha = alpha_rolls[i + 1][0] - alpha_rolls[i][0]
            t = (alpha - alpha_rolls[i][0]) * (total_dalpha / dalpha)
            break
    assert theta_i is not None
    assert theta_f is not None
    assert t is not None

    return get_roll_joint_theta(theta_i, theta_f, t)


# Draw a list of lines to a cairo context.
def draw_lines(cr, lines):
    cr.move_to(lines[0][0], lines[0][1])
    for pt in lines[1:]:
        cr.line_to(pt[0], pt[1])


class Path(abc.ABC):

    def __init__(self, name):
        self.name = name

    @abc.abstractmethod
    def DoToThetaPoints(self):
        pass

    @abc.abstractmethod
    def DoDrawTo(self):
        pass

    @abc.abstractmethod
    def joint_thetas(self):
        pass

    @abc.abstractmethod
    def intersection(self, event):
        pass

    def roll_joint_collision(self, points, verbose=False):
        for point in points:
            if roll_joint_collision(*point):
                if verbose:
                    print("Roll joint collision for path %s in point %s" %
                          (self.name, point))
                return True
        return False

    def arm_past_limit(self, points, verbose=True):
        for point in points:
            if arm_past_limit(*point):
                if verbose:
                    print("Arm past limit for path %s in point %s" %
                          (self.name, point))
                return True
        return False

    def DrawTo(self, cr, theta_version):
        points = self.DoToThetaPoints()
        if self.roll_joint_collision(points):
            # Draw the spline red
            cr.set_source_rgb(1.0, 0.0, 0.0)
        elif self.arm_past_limit(points):
            # Draw the spline orange
            cr.set_source_rgb(1.0, 0.5, 0.0)

        self.DoDrawTo(cr, theta_version)

    def VerifyPoints(self):
        points = self.DoToThetaPoints()
        if self.roll_joint_collision(points, verbose=True) or \
           self.arm_past_limit(points, verbose=True):
            sys.exit(1)


class SplineSegmentBase(Path):

    def __init__(self, name):
        super().__init__(name)

    @abc.abstractmethod
    # Returns (start, control1, control2, end), each in the form
    # (theta1, theta2, theta3)
    def get_controls_theta(self):
        pass

    def intersection(self, event):
        start, control1, control2, end = self.get_controls_theta()
        for alpha in subdivide_multistep():
            x, y = get_xy(spline_eval(start, control1, control2, end, alpha))
            spline_point = np.array([x, y])
            hovered_point = np.array([event.x, event.y])
            if np.linalg.norm(hovered_point - spline_point) < 0.03:
                return alpha
        return None


class ThetaSplineSegment(SplineSegmentBase):

    # start and end are [theta1, theta2, theta3].
    # controls are just [theta1, theta2].
    # control_alpha_rolls are a list of [alpha, roll]
    def __init__(self,
                 name,
                 start,
                 control1,
                 control2,
                 end,
                 control_alpha_rolls=[],
                 alpha_unitizer=None,
                 vmax=None):
        super().__init__(name)
        self.start = start[:2]
        self.control1 = control1
        self.control2 = control2
        self.end = end[:2]
        # There will always be roll at alpha = 0 and 1
        self.alpha_rolls = [[0.0, start[2]]
                            ] + control_alpha_rolls + [[1.0, end[2]]]
        self.alpha_unitizer = alpha_unitizer
        self.vmax = vmax

    def Print(self, points):
        # Find the name of the start end end points.
        start_name = None
        end_name = None
        for name in points:
            point = points[name]
            if (self.start == point[:2]).all():
                start_name = name
            elif (self.end == point[:2]).all():
                end_name = name

        alpha_points = '[' + ', '.join([
            f"({alpha}, np.pi * {theta / np.pi})"
            for alpha, theta in self.alpha_rolls[1:-1]
        ]) + ']'

        def FormatToTheta(point):
            x, y = get_xy(point)
            circular_index = get_circular_index(point)
            return "to_theta_with_circular_index(%.3f, %.3f, circular_index=%d)" % (
                x, y, circular_index)

        def FormatToThetaRoll(point, roll):
            x, y = get_xy(point)
            circular_index = get_circular_index(point)
            return "to_theta_with_circular_index_and_roll(%.3f, %.3f, np.pi * %.2f, circular_index=%d)" % (
                x, y, roll / np.pi, circular_index)

        print('named_segments.append(')
        print('    ThetaSplineSegment(')
        print(f'        name="{self.name}",')
        print(
            f'        start=points["{start_name}"], # {FormatToThetaRoll(self.start, self.alpha_rolls[0][1])}'
        )
        print(
            f'        control1=np.array([{self.control1[0]}, {self.control1[1]}]), # {FormatToTheta(self.control1)}'
        )
        print(
            f'        control2=np.array([{self.control2[0]}, {self.control2[1]}]), # {FormatToTheta(self.control2)}'
        )
        print(
            f'        end=points["{end_name}"], # {FormatToThetaRoll(self.end, self.alpha_rolls[-1][1])}'
        )
        print(f'        control_alpha_rolls={alpha_points},')
        print(f'))')

    def __repr__(self):
        return "ThetaSplineSegment(%s, %s, %s, %s)" % (repr(
            self.start), repr(self.control1), repr(
                self.control2), repr(self.end))

    def DoDrawTo(self, cr, theta_version):
        if (theta_version):
            draw_lines(cr, [
                shift_angles(
                    spline_eval(self.start, self.control1, self.control2,
                                self.end, alpha))
                for alpha in subdivide_multistep()
            ])
        else:
            start = get_xy(self.start)
            end = get_xy(self.end)

            draw_lines(cr, [
                get_xy(
                    spline_eval(self.start, self.control1, self.control2,
                                self.end, alpha))
                for alpha in subdivide_multistep()
            ])

            cr.move_to(start[0] + xy_end_circle_size, start[1])
            cr.arc(start[0], start[1], xy_end_circle_size, 0, 2.0 * np.pi)
            cr.move_to(end[0] + xy_end_circle_size, end[1])
            cr.arc(end[0], end[1], xy_end_circle_size, 0, 2.0 * np.pi)

    def DoToThetaPoints(self):
        points = []
        for alpha in subdivide_multistep():
            proximal, distal = spline_eval(self.start, self.control1,
                                           self.control2, self.end, alpha)
            roll_joint = get_roll_joint_theta_multistep(
                self.alpha_rolls, alpha)
            points.append((proximal, distal, roll_joint))

        return points

    def get_controls_theta(self):
        return (self.start, self.control1, self.control2, self.end)

    def joint_thetas(self):
        ts = []
        thetas = [[], [], []]
        for alpha in subdivide_multistep():
            proximal, distal = spline_eval(self.start, self.control1,
                                           self.control2, self.end, alpha)
            roll_joint = get_roll_joint_theta_multistep(
                self.alpha_rolls, alpha)
            thetas[0].append(proximal)
            thetas[1].append(distal)
            thetas[2].append(roll_joint)
            ts.append(alpha)
        return ts, thetas
