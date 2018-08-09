import numpy

# joint_center in x-y space.
joint_center = (-0.299, 0.299)

# Joint distances (l1 = "proximal", l2 = "distal")
l1 = 46.25 * 0.0254
l2 = 43.75 * 0.0254


# Convert from x-y coordinates to theta coordinates.
# orientation is a bool. This orientation is circular_index mod 2.
# where circular_index is the circular index, or the position in the
# "hyperextension" zones. "cross_point" allows shifting the place where
# it rounds the result so that it draws nicer (no other functional differences).
def to_theta(pt, circular_index, cross_point=-numpy.pi):
    orient = (circular_index % 2) == 0
    x = pt[0]
    y = pt[1]
    x -= joint_center[0]
    y -= joint_center[1]
    l3 = numpy.hypot(x, y)
    t3 = numpy.arctan2(y, x)
    theta1 = numpy.arccos((l1**2 + l3**2 - l2**2) / (2 * l1 * l3))

    if orient:
        theta1 = -theta1
    theta1 += t3
    theta1 = (theta1 - cross_point) % (2 * numpy.pi) + cross_point
    theta2 = numpy.arctan2(y - l1 * numpy.sin(theta1),
                           x - l1 * numpy.cos(theta1))
    return numpy.array((theta1, theta2))


# Simple trig to go back from theta1, theta2 to x-y
def to_xy(theta1, theta2):
    x = numpy.cos(theta1) * l1 + numpy.cos(theta2) * l2 + joint_center[0]
    y = numpy.sin(theta1) * l1 + numpy.sin(theta2) * l2 + joint_center[1]
    orient = ((theta2 - theta1) % (2.0 * numpy.pi)) < numpy.pi
    return (x, y, orient)


def get_circular_index(theta):
    return int(numpy.floor((theta[1] - theta[0]) / numpy.pi))


def get_xy(theta):
    theta1 = theta[0]
    theta2 = theta[1]
    x = numpy.cos(theta1) * l1 + numpy.cos(theta2) * l2 + joint_center[0]
    y = numpy.sin(theta1) * l1 + numpy.sin(theta2) * l2 + joint_center[1]
    return numpy.array((x, y))


# Draw a list of lines to a cairo context.
def draw_lines(cr, lines):
    cr.move_to(lines[0][0], lines[0][1])
    for pt in lines[1:]:
        cr.line_to(pt[0], pt[1])


max_dist = 0.01
max_dist_theta = numpy.pi / 64
xy_end_circle_size = 0.01
theta_end_circle_size = 0.07


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


# subdivide in xy space.
def subdivide_xy(lines, max_dist=max_dist):
    out = []
    last_pt = lines[0]
    out.append(last_pt)
    for n_pt in lines[1:]:
        for pt in subdivide(last_pt, n_pt, max_dist):
            out.append(pt)
        last_pt = n_pt

    return out


def to_theta_with_ci(pt, circular_index):
    return to_theta_with_circular_index(pt[0], pt[1], circular_index)


# to_theta, but distinguishes between
def to_theta_with_circular_index(x, y, circular_index):
    theta1, theta2 = to_theta((x, y), circular_index)
    n_circular_index = int(numpy.floor((theta2 - theta1) / numpy.pi))
    theta2 = theta2 + ((circular_index - n_circular_index)) * numpy.pi
    return numpy.array((theta1, theta2))


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
    norm = numpy.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


# CI is circular index and allows selecting between all the stats that map
# to the same x-y state (by giving them an integer index).
# This will compute approximate first and second derivatives with respect
# to path length.
def to_theta_with_circular_index_and_derivs(x, y, dx, dy,
                                            circular_index_select):
    a = to_theta_with_circular_index(x, y, circular_index_select)
    b = to_theta_with_circular_index(x + dx * 0.0001, y + dy * 0.0001,
                                     circular_index_select)
    c = to_theta_with_circular_index(x - dx * 0.0001, y - dy * 0.0001,
                                     circular_index_select)
    d1 = normalize(b - a)
    d2 = normalize(c - a)
    accel = (d1 + d2) / numpy.linalg.norm(a - b)
    return (a[0], a[1], d1[0], d1[1], accel[0], accel[1])


def to_theta_with_ci_and_derivs(p_prev, p, p_next, c_i_select):
    a = to_theta(p, c_i_select)
    b = to_theta(p_next, c_i_select)
    c = to_theta(p_prev, c_i_select)
    d1 = normalize(b - a)
    d2 = normalize(c - a)
    accel = (d1 + d2) / numpy.linalg.norm(a - b)
    return (a[0], a[1], d1[0], d1[1], accel[0], accel[1])


# Generic subdivision algorithm.
def subdivide(p1, p2, max_dist):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dist = numpy.sqrt(dx**2 + dy**2)
    n = int(numpy.ceil(dist / max_dist))
    return [(alpha_blend(p1[0], p2[0],
                         float(i) / n), alpha_blend(p1[1], p2[1],
                                                    float(i) / n))
            for i in range(1, n + 1)]


# convert from an xy space loop into a theta loop.
# All segements are expected go from one "hyper-extension" boundary
# to another, thus we must go backwards over the "loop" to get a loop in
# x-y space.
def to_theta_loop(lines, cross_point=-numpy.pi):
    out = []
    last_pt = lines[0]
    for n_pt in lines[1:]:
        for pt in subdivide(last_pt, n_pt, max_dist):
            out.append(to_theta(pt, 0, cross_point))
        last_pt = n_pt
    for n_pt in reversed(lines[:-1]):
        for pt in subdivide(last_pt, n_pt, max_dist):
            out.append(to_theta(pt, 1, cross_point))
        last_pt = n_pt
    return out


# Convert a loop (list of line segments) into
# The name incorrectly suggests that it is cyclic.
def back_to_xy_loop(lines):
    out = []
    last_pt = lines[0]
    out.append(to_xy(last_pt[0], last_pt[1]))
    for n_pt in lines[1:]:
        for pt in subdivide(last_pt, n_pt, max_dist_theta):
            out.append(to_xy(pt[0], pt[1]))
        last_pt = n_pt

    return out


# Segment in angle space.
class AngleSegment:
    def __init__(self, start, end, name=None, alpha_unitizer=None, vmax=None):
        """Creates an angle segment.

        Args:
          start: (double, double),  The start of the segment in theta1, theta2
              coordinates in radians
          end: (double, double),  The end of the segment in theta1, theta2
              coordinates in radians
        """
        self.start = start
        self.end = end
        self.name = name
        self.alpha_unitizer = alpha_unitizer
        self.vmax = vmax

    def __repr__(self):
        return "AngleSegment(%s, %s)" % (repr(self.start), repr(self.end))

    def DrawTo(self, cr, theta_version):
        if theta_version:
            cr.move_to(self.start[0], self.start[1] + theta_end_circle_size)
            cr.arc(self.start[0], self.start[1], theta_end_circle_size, 0,
                   2.0 * numpy.pi)
            cr.move_to(self.end[0], self.end[1] + theta_end_circle_size)
            cr.arc(self.end[0], self.end[1], theta_end_circle_size, 0,
                   2.0 * numpy.pi)
            cr.move_to(self.start[0], self.start[1])
            cr.line_to(self.end[0], self.end[1])
        else:
            start_xy = to_xy(self.start[0], self.start[1])
            end_xy = to_xy(self.end[0], self.end[1])
            draw_lines(cr, back_to_xy_loop([self.start, self.end]))
            cr.move_to(start_xy[0] + xy_end_circle_size, start_xy[1])
            cr.arc(start_xy[0], start_xy[1], xy_end_circle_size, 0,
                   2.0 * numpy.pi)
            cr.move_to(end_xy[0] + xy_end_circle_size, end_xy[1])
            cr.arc(end_xy[0], end_xy[1], xy_end_circle_size, 0, 2.0 * numpy.pi)

    def ToThetaPoints(self):
        dx = self.end[0] - self.start[0]
        dy = self.end[1] - self.start[1]
        mag = numpy.hypot(dx, dy)
        dx /= mag
        dy /= mag

        return [(self.start[0], self.start[1], dx, dy, 0.0, 0.0),
                (self.end[0], self.end[1], dx, dy, 0.0, 0.0)]


class XYSegment:
    """Straight line in XY space."""

    def __init__(self, start, end, name=None, alpha_unitizer=None, vmax=None):
        """Creates an XY segment.

        Args:
          start: (double, double),  The start of the segment in theta1, theta2
              coordinates in radians
          end: (double, double),  The end of the segment in theta1, theta2
              coordinates in radians
        """
        self.start = start
        self.end = end
        self.name = name
        self.alpha_unitizer = alpha_unitizer
        self.vmax = vmax

    def __repr__(self):
        return "XYSegment(%s, %s)" % (repr(self.start), repr(self.end))

    def DrawTo(self, cr, theta_version):
        if theta_version:
            theta1, theta2 = self.start
            circular_index_select = int(
                numpy.floor((self.start[1] - self.start[0]) / numpy.pi))
            start = get_xy(self.start)
            end = get_xy(self.end)

            ln = [(start[0], start[1]), (end[0], end[1])]
            draw_lines(cr, [
                to_theta_with_circular_index(x, y, circular_index_select)
                for x, y in subdivide_xy(ln)
            ])
            cr.move_to(self.start[0] + theta_end_circle_size, self.start[1])
            cr.arc(self.start[0], self.start[1], theta_end_circle_size, 0,
                   2.0 * numpy.pi)
            cr.move_to(self.end[0] + theta_end_circle_size, self.end[1])
            cr.arc(self.end[0], self.end[1], theta_end_circle_size, 0,
                   2.0 * numpy.pi)
        else:
            start = get_xy(self.start)
            end = get_xy(self.end)
            cr.move_to(start[0], start[1])
            cr.line_to(end[0], end[1])
            cr.move_to(start[0] + xy_end_circle_size, start[1])
            cr.arc(start[0], start[1], xy_end_circle_size, 0, 2.0 * numpy.pi)
            cr.move_to(end[0] + xy_end_circle_size, end[1])
            cr.arc(end[0], end[1], xy_end_circle_size, 0, 2.0 * numpy.pi)

    def ToThetaPoints(self):
        """ Converts to points in theta space via to_theta_with_circular_index_and_derivs"""
        theta1, theta2 = self.start
        circular_index_select = int(
            numpy.floor((self.start[1] - self.start[0]) / numpy.pi))
        start = get_xy(self.start)
        end = get_xy(self.end)

        ln = [(start[0], start[1]), (end[0], end[1])]

        dx = end[0] - start[0]
        dy = end[1] - start[1]
        mag = numpy.hypot(dx, dy)
        dx /= mag
        dy /= mag

        return [
            to_theta_with_circular_index_and_derivs(x, y, dx, dy,
                                                    circular_index_select)
            for x, y in subdivide_xy(ln, 0.01)
        ]


def spline_eval(start, control1, control2, end, alpha):
    a = alpha_blend(start, control1, alpha)
    b = alpha_blend(control1, control2, alpha)
    c = alpha_blend(control2, end, alpha)
    return alpha_blend(
        alpha_blend(a, b, alpha), alpha_blend(b, c, alpha), alpha)


def subdivide_spline(start, control1, control2, end):
    # TODO: pick N based on spline parameters? or otherwise change it to be more evenly spaced?
    n = 100
    for i in range(0, n + 1):
        yield i / float(n)


class SplineSegment:
    def __init__(self,
                 start,
                 control1,
                 control2,
                 end,
                 name=None,
                 alpha_unitizer=None,
                 vmax=None):
        self.start = start
        self.control1 = control1
        self.control2 = control2
        self.end = end
        self.name = name
        self.alpha_unitizer = alpha_unitizer
        self.vmax = vmax

    def __repr__(self):
        return "SplineSegment(%s, %s, %s, %s)" % (repr(self.start),
                                                  repr(self.control1),
                                                  repr(self.control2),
                                                  repr(self.end))

    def DrawTo(self, cr, theta_version):
        if theta_version:
            c_i_select = get_circular_index(self.start)
            start = get_xy(self.start)
            control1 = get_xy(self.control1)
            control2 = get_xy(self.control2)
            end = get_xy(self.end)

            draw_lines(cr, [
                to_theta(
                    spline_eval(start, control1, control2, end, alpha),
                    c_i_select)
                for alpha in subdivide_spline(start, control1, control2, end)
            ])
            cr.move_to(self.start[0] + theta_end_circle_size, self.start[1])
            cr.arc(self.start[0], self.start[1], theta_end_circle_size, 0,
                   2.0 * numpy.pi)
            cr.move_to(self.end[0] + theta_end_circle_size, self.end[1])
            cr.arc(self.end[0], self.end[1], theta_end_circle_size, 0,
                   2.0 * numpy.pi)
        else:
            start = get_xy(self.start)
            control1 = get_xy(self.control1)
            control2 = get_xy(self.control2)
            end = get_xy(self.end)

            draw_lines(cr, [
                spline_eval(start, control1, control2, end, alpha)
                for alpha in subdivide_spline(start, control1, control2, end)
            ])

            cr.move_to(start[0] + xy_end_circle_size, start[1])
            cr.arc(start[0], start[1], xy_end_circle_size, 0, 2.0 * numpy.pi)
            cr.move_to(end[0] + xy_end_circle_size, end[1])
            cr.arc(end[0], end[1], xy_end_circle_size, 0, 2.0 * numpy.pi)

    def ToThetaPoints(self):
        t1, t2 = self.start
        c_i_select = get_circular_index(self.start)
        start = get_xy(self.start)
        control1 = get_xy(self.control1)
        control2 = get_xy(self.control2)
        end = get_xy(self.end)

        return [
            to_theta_with_ci_and_derivs(
                spline_eval(start, control1, control2, end, alpha - 0.00001),
                spline_eval(start, control1, control2, end, alpha),
                spline_eval(start, control1, control2, end, alpha + 0.00001),
                c_i_select)
            for alpha in subdivide_spline(start, control1, control2, end)
        ]


def get_derivs(t_prev, t, t_next):
    c, a, b = t_prev, t, t_next
    d1 = normalize(b - a)
    d2 = normalize(c - a)
    accel = (d1 + d2) / numpy.linalg.norm(a - b)
    return (a[0], a[1], d1[0], d1[1], accel[0], accel[1])


class ThetaSplineSegment:
    def __init__(self,
                 start,
                 control1,
                 control2,
                 end,
                 name=None,
                 alpha_unitizer=None,
                 vmax=None):
        self.start = start
        self.control1 = control1
        self.control2 = control2
        self.end = end
        self.name = name
        self.alpha_unitizer = alpha_unitizer
        self.vmax = vmax

    def __repr__(self):
        return "ThetaSplineSegment(%s, %s, &s, %s)" % (repr(self.start),
                                                       repr(self.control1),
                                                       repr(self.control2),
                                                       repr(self.end))

    def DrawTo(self, cr, theta_version):
        if (theta_version):
            draw_lines(cr, [
                spline_eval(self.start, self.control1, self.control2, self.end,
                            alpha)
                for alpha in subdivide_spline(self.start, self.control1,
                                              self.control2, self.end)
            ])
        else:
            start = get_xy(self.start)
            end = get_xy(self.end)

            draw_lines(cr, [
                get_xy(
                    spline_eval(self.start, self.control1, self.control2,
                                self.end, alpha))
                for alpha in subdivide_spline(self.start, self.control1,
                                              self.control2, self.end)
            ])

            cr.move_to(start[0] + xy_end_circle_size, start[1])
            cr.arc(start[0], start[1], xy_end_circle_size, 0, 2.0 * numpy.pi)
            cr.move_to(end[0] + xy_end_circle_size, end[1])
            cr.arc(end[0], end[1], xy_end_circle_size, 0, 2.0 * numpy.pi)

    def ToThetaPoints(self):
        return [
            get_derivs(
                spline_eval(self.start, self.control1, self.control2, self.end,
                            alpha - 0.00001),
                spline_eval(self.start, self.control1, self.control2, self.end,
                            alpha),
                spline_eval(self.start, self.control1, self.control2, self.end,
                            alpha + 0.00001))
            for alpha in subdivide_spline(self.start, self.control1,
                                          self.control2, self.end)
        ]


tall_box_x = 0.411
tall_box_y = 0.125

short_box_x = 0.431
short_box_y = 0.082

ready_above_box = to_theta_with_circular_index(
    tall_box_x, tall_box_y + 0.08, circular_index=-1)
tall_box_grab = to_theta_with_circular_index(
    tall_box_x, tall_box_y, circular_index=-1)
short_box_grab = to_theta_with_circular_index(
    short_box_x, short_box_y, circular_index=-1)

# TODO(austin): Drive the front/back off the same numbers a bit better.
front_high_box = to_theta_with_circular_index(0.378, 2.46, circular_index=-1)
front_middle3_box = to_theta_with_circular_index(
    0.700, 2.125, circular_index=-1.000000)
front_middle2_box = to_theta_with_circular_index(
    0.700, 2.268, circular_index=-1)
front_middle1_box = to_theta_with_circular_index(
    0.800, 1.915, circular_index=-1)
front_low_box = to_theta_with_circular_index(0.87, 1.572, circular_index=-1)
back_high_box = to_theta_with_circular_index(-0.75, 2.48, circular_index=0)
back_middle2_box = to_theta_with_circular_index(
    -0.700, 2.27, circular_index=0)
back_middle1_box = to_theta_with_circular_index(
    -0.800, 1.93, circular_index=0)
back_low_box = to_theta_with_circular_index(-0.87, 1.64, circular_index=0)

back_extra_low_box = to_theta_with_circular_index(-0.87, 1.52, circular_index=0)

front_switch = to_theta_with_circular_index(0.88, 0.967, circular_index=-1)
back_switch = to_theta_with_circular_index(-0.88, 0.967, circular_index=-2)

neutral = to_theta_with_circular_index(0.0, 0.33, circular_index=-1)

up = to_theta_with_circular_index(0.0, 2.547, circular_index=-1)

front_switch_auto = to_theta_with_circular_index(
    0.750, 2.20, circular_index=-1.000000)

duck = numpy.array(
    [numpy.pi / 2.0 - 0.92, numpy.pi / 2.0 - 4.26])

starting = numpy.array(
    [numpy.pi / 2.0 - 0.593329, numpy.pi / 2.0 - 3.749631])
vertical_starting = numpy.array(
    [numpy.pi / 2.0, -numpy.pi / 2.0])

self_hang = numpy.array(
    [numpy.pi / 2.0 - 0.191611, numpy.pi / 2.0])
partner_hang = numpy.array(
    [numpy.pi / 2.0 - (-0.30), numpy.pi / 2.0])

above_hang = numpy.array(
    [numpy.pi / 2.0 - 0.14, numpy.pi / 2.0 - (-0.165)])
below_hang = numpy.array(
    [numpy.pi / 2.0 - 0.39, numpy.pi / 2.0 - (-0.517)])

up_c1 = to_theta((0.63, 1.17), circular_index=-1)
up_c2 = to_theta((0.65, 1.62), circular_index=-1)

front_high_box_c1 = to_theta((0.63, 1.04), circular_index=-1)
front_high_box_c2 = to_theta((0.50, 1.60), circular_index=-1)

front_middle2_box_c1 = to_theta((0.41, 0.83), circular_index=-1)
front_middle2_box_c2 = to_theta((0.52, 1.30), circular_index=-1)

front_middle1_box_c1 = to_theta((0.34, 0.82), circular_index=-1)
front_middle1_box_c2 = to_theta((0.48, 1.15), circular_index=-1)

#c1: (1.421433, -1.070254)
#c2: (1.434384, -1.057803
ready_above_box_c1 = numpy.array([1.480802, -1.081218])
ready_above_box_c2 = numpy.array([1.391449, -1.060331])

front_switch_c1 = numpy.array([1.903841, -0.622351])
front_switch_c2 = numpy.array([1.903841, -0.622351])


sparse_front_points = [
    (front_high_box, "FrontHighBox"),
    (front_middle2_box, "FrontMiddle2Box"),
    (front_middle3_box, "FrontMiddle3Box"),
    (front_middle1_box, "FrontMiddle1Box"),
    (front_low_box, "FrontLowBox"),
    (front_switch, "FrontSwitch"),
]  # yapf: disable

sparse_back_points = [
    (back_high_box, "BackHighBox"),
    (back_middle2_box, "BackMiddle2Box"),
    (back_middle1_box, "BackMiddle1Box"),
    (back_low_box, "BackLowBox"),
    (back_extra_low_box, "BackExtraLowBox"),
]  # yapf: disable

def expand_points(points, max_distance):
    """Expands a list of points to be at most max_distance apart

    Generates the paths to connect the new points to the closest input points,
    and the paths connecting the points.

    Args:
      points, list of tuple of point, name, The points to start with and fill
          in.
      max_distance, float, The max distance between two points when expanding
          the graph.

    Return:
      points, edges
    """
    result_points = [points[0]]
    result_paths = []
    for point, name in points[1:]:
        previous_point = result_points[-1][0]
        previous_point_xy = get_xy(previous_point)
        circular_index = get_circular_index(previous_point)

        point_xy = get_xy(point)
        norm = numpy.linalg.norm(point_xy - previous_point_xy)
        num_points = int(numpy.ceil(norm / max_distance))
        last_iteration_point = previous_point
        for subindex in range(1, num_points):
            subpoint = to_theta(
                alpha_blend(previous_point_xy, point_xy,
                            float(subindex) / num_points),
                circular_index=circular_index)
            result_points.append((subpoint, '%s%dof%d' % (name, subindex,
                                                          num_points)))
            result_paths.append(
                XYSegment(last_iteration_point, subpoint, vmax=6.0))
            if (last_iteration_point != previous_point).any():
                result_paths.append(XYSegment(previous_point, subpoint))
            if subindex == num_points - 1:
              result_paths.append(XYSegment(subpoint, point, vmax=6.0))
            else:
              result_paths.append(XYSegment(subpoint, point))
            last_iteration_point = subpoint
        result_points.append((point, name))

    return result_points, result_paths

front_points, front_paths = expand_points(sparse_front_points, 0.06)
back_points, back_paths = expand_points(sparse_back_points, 0.06)

points = [(ready_above_box, "ReadyAboveBox"),
          (tall_box_grab, "TallBoxGrab"),
          (short_box_grab, "ShortBoxGrab"),
          (back_switch, "BackSwitch"),
          (neutral, "Neutral"),
          (up, "Up"),
          (above_hang, "AboveHang"),
          (below_hang, "BelowHang"),
          (self_hang, "SelfHang"),
          (partner_hang, "PartnerHang"),
          (front_switch_auto, "FrontSwitchAuto"),
          (starting, "Starting"),
          (duck, "Duck"),
          (vertical_starting, "VerticalStarting"),
] + front_points + back_points  # yapf: disable

duck_c1 = numpy.array([1.337111, -1.721008])
duck_c2 = numpy.array([1.283701, -1.795519])

ready_to_up_c1 = numpy.array([1.792962, 0.198329])
ready_to_up_c2 = numpy.array([1.792962, 0.198329])

front_switch_auto_c1 = numpy.array([1.792857, -0.372768])
front_switch_auto_c2 = numpy.array([1.861885, -0.273664])


# We need to define critical points so we can create paths connecting them.
# TODO(austin): Attach velocities to the slow ones.
ready_to_back_low_c1 = numpy.array([2.524325, 0.046417])

neutral_to_back_low_c1 = numpy.array([2.381942, -0.070220])

tall_to_back_low_c1 = numpy.array([2.603918, 0.088298])
tall_to_back_low_c2 = numpy.array([1.605624, 1.003434])

tall_to_back_high_c2 = numpy.array([1.508610, 0.946147])

# If true, only plot the first named segment
isolate = False

long_alpha_unitizer = numpy.matrix([[1.0 / 17.0, 0.0], [0.0, 1.0 / 17.0]])

neutral_to_back_c1 = numpy.array([0.702527, -2.618276])
neutral_to_back_c2 = numpy.array([0.526914, -3.109691])


named_segments = [
    ThetaSplineSegment(neutral, neutral_to_back_c1, neutral_to_back_c2, back_switch, "BackSwitch"),

    ThetaSplineSegment(neutral, neutral_to_back_low_c1, tall_to_back_high_c2, back_high_box, "NeutralBoxToHigh", alpha_unitizer=long_alpha_unitizer),
    ThetaSplineSegment(neutral, neutral_to_back_low_c1, tall_to_back_high_c2, back_middle2_box, "NeutralBoxToMiddle2", long_alpha_unitizer),
    ThetaSplineSegment(neutral, neutral_to_back_low_c1, tall_to_back_low_c2, back_middle1_box, "NeutralBoxToMiddle1", long_alpha_unitizer),
    ThetaSplineSegment(neutral, neutral_to_back_low_c1, tall_to_back_low_c2, back_low_box, "NeutralBoxToLow", long_alpha_unitizer),

    ThetaSplineSegment(ready_above_box, ready_to_back_low_c1, tall_to_back_high_c2, back_high_box, "ReadyBoxToHigh", long_alpha_unitizer),
    ThetaSplineSegment(ready_above_box, ready_to_back_low_c1, tall_to_back_high_c2, back_middle2_box, "ReadyBoxToMiddle2", long_alpha_unitizer),
    ThetaSplineSegment(ready_above_box, ready_to_back_low_c1, tall_to_back_low_c2, back_middle1_box, "ReadyBoxToMiddle1", long_alpha_unitizer),
    ThetaSplineSegment(ready_above_box, ready_to_back_low_c1, tall_to_back_low_c2, back_low_box, "ReadyBoxToLow", long_alpha_unitizer),

    ThetaSplineSegment(short_box_grab, tall_to_back_low_c1, tall_to_back_high_c2, back_high_box, "ShortBoxToHigh", long_alpha_unitizer),
    ThetaSplineSegment(short_box_grab, tall_to_back_low_c1, tall_to_back_high_c2, back_middle2_box, "ShortBoxToMiddle2", long_alpha_unitizer),
    ThetaSplineSegment(short_box_grab, tall_to_back_low_c1, tall_to_back_low_c2, back_middle1_box, "ShortBoxToMiddle1", long_alpha_unitizer),
    ThetaSplineSegment(short_box_grab, tall_to_back_low_c1, tall_to_back_low_c2, back_low_box, "ShortBoxToLow", long_alpha_unitizer),

    ThetaSplineSegment(tall_box_grab, tall_to_back_low_c1, tall_to_back_high_c2, back_high_box, "TallBoxToHigh", long_alpha_unitizer),
    ThetaSplineSegment(tall_box_grab, tall_to_back_low_c1, tall_to_back_high_c2, back_middle2_box, "TallBoxToMiddle2", long_alpha_unitizer),
    ThetaSplineSegment(tall_box_grab, tall_to_back_low_c1, tall_to_back_low_c2, back_middle1_box, "TallBoxToMiddle1", long_alpha_unitizer),
    ThetaSplineSegment(tall_box_grab, tall_to_back_low_c1, tall_to_back_low_c2, back_low_box, "TallBoxToLow", long_alpha_unitizer),

    SplineSegment(neutral, ready_above_box_c1, ready_above_box_c2,
                  ready_above_box, "ReadyToNeutral"),
    XYSegment(ready_above_box, tall_box_grab, "ReadyToTallBox", vmax=6.0),
    XYSegment(ready_above_box, short_box_grab, "ReadyToShortBox", vmax=6.0),
    XYSegment(tall_box_grab, short_box_grab, "TallToShortBox", vmax=6.0),
    SplineSegment(neutral, ready_above_box_c1, ready_above_box_c2,
                  tall_box_grab, "TallToNeutral"),
    SplineSegment(neutral, ready_above_box_c1, ready_above_box_c2,
                  short_box_grab, "ShortToNeutral"),
    SplineSegment(neutral, up_c1, up_c2, up, "NeutralToUp"),
    SplineSegment(neutral, front_high_box_c1, front_high_box_c2,
                  front_high_box, "NeutralToFrontHigh"),
    SplineSegment(neutral, front_middle2_box_c1, front_middle2_box_c2,
                  front_middle2_box, "NeutralToFrontMiddle2"),
    SplineSegment(neutral, front_middle1_box_c1, front_middle1_box_c2,
                  front_middle1_box, "NeutralToFrontMiddle1"),
]

unnamed_segments = [
    SplineSegment(neutral, front_switch_auto_c1, front_switch_auto_c2, front_switch_auto),
    SplineSegment(tall_box_grab, ready_to_up_c1, ready_to_up_c2, up),
    SplineSegment(short_box_grab, ready_to_up_c1, ready_to_up_c2, up),
    SplineSegment(ready_above_box, ready_to_up_c1, ready_to_up_c2, up),
    ThetaSplineSegment(duck, duck_c1, duck_c2, neutral),
    SplineSegment(neutral, front_switch_c1, front_switch_c2, front_switch),

    XYSegment(ready_above_box, front_low_box),
    XYSegment(ready_above_box, front_switch),
    XYSegment(ready_above_box, front_middle1_box),
    XYSegment(ready_above_box, front_middle2_box),
    XYSegment(ready_above_box, front_middle3_box),
    SplineSegment(ready_above_box, ready_to_up_c1, ready_to_up_c2, front_high_box),

    AngleSegment(starting, vertical_starting),
    AngleSegment(vertical_starting, neutral),

    XYSegment(neutral, front_low_box),
    XYSegment(up, front_high_box),
    XYSegment(up, front_middle2_box),

    XYSegment(front_middle3_box, up),
    XYSegment(front_middle3_box, front_high_box),
    XYSegment(front_middle3_box, front_middle2_box),
    XYSegment(front_middle3_box, front_middle1_box),

    XYSegment(up, front_middle1_box),
    XYSegment(up, front_low_box),
    XYSegment(front_high_box, front_middle2_box),
    XYSegment(front_high_box, front_middle1_box),
    XYSegment(front_high_box, front_low_box),
    XYSegment(front_middle2_box, front_middle1_box),
    XYSegment(front_middle2_box, front_low_box),
    XYSegment(front_middle1_box, front_low_box),
    XYSegment(front_switch, front_low_box),
    XYSegment(front_switch, up),
    XYSegment(front_switch, front_high_box),
    AngleSegment(up, back_high_box),
    AngleSegment(up, back_middle2_box),
    AngleSegment(up, back_middle1_box),
    AngleSegment(up, back_low_box),
    XYSegment(back_high_box, back_middle2_box),
    XYSegment(back_high_box, back_middle1_box),
    XYSegment(back_high_box, back_low_box),
    XYSegment(back_middle2_box, back_middle1_box),
    XYSegment(back_middle2_box, back_low_box),
    XYSegment(back_middle1_box, back_low_box),

    AngleSegment(up, above_hang),
    AngleSegment(above_hang, below_hang),
    AngleSegment(up, below_hang),
    AngleSegment(up, self_hang),
    AngleSegment(up, partner_hang),
] + front_paths + back_paths

segments = []
if isolate:
    segments += named_segments[:isolate]
else:
    segments += named_segments + unnamed_segments
