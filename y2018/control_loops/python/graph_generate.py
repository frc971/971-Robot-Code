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
    def __init__(self, start, end, name=None):
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

    def __init__(self, start, end, name=None):
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
    def __init__(self, start, control1, control2, end, name=None):
        self.start = start
        self.control1 = control1
        self.control2 = control2
        self.end = end
        self.name = name

    def __repr__(self):
        return "XYSegment(%s, %s, &s, %s)" % (repr(self.start),
                                              repr(self.control1),
                                              repr(self.control2),
                                              repr(self.end))

    def DrawTo(self, cr, theta_version):
        if (theta_version):
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
        else:
            start = get_xy(self.start)
            control1 = get_xy(self.control1)
            control2 = get_xy(self.control2)
            end = get_xy(self.end)
            #cr.move_to(start[0], start[1])
            draw_lines(cr, [
                spline_eval(start, control1, control2, end, alpha)
                for alpha in subdivide_spline(start, control1, control2, end)
            ])
            # cr.spline_to(control1[0], control1[1], control2[0], control2[1], end[0], end[1])

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


tall_box_x = 0.401
tall_box_y = 0.14

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
front_middle2_box = to_theta_with_circular_index(
    0.732, 2.268, circular_index=-1)
front_middle1_box = to_theta_with_circular_index(
    0.878, 1.885, circular_index=-1)
front_low_box = to_theta_with_circular_index(0.926, 1.522, circular_index=-1)
back_high_box = to_theta_with_circular_index(-0.75, 2.48, circular_index=0)
back_middle2_box = to_theta_with_circular_index(
    -0.732, 2.268, circular_index=0)
back_middle1_box = to_theta_with_circular_index(
    -0.878, 1.885, circular_index=0)
back_low_box = to_theta_with_circular_index(-0.926, 1.522, circular_index=0)

front_switch = to_theta_with_circular_index(0.88, 0.967, circular_index=-1)
back_switch = to_theta_with_circular_index(-0.88, 0.967, circular_index=-2)

neutral = to_theta_with_circular_index(0.0, 0.33, circular_index=-1)

up = to_theta_with_circular_index(0.0, 2.547, circular_index=-1)

up_c1 = to_theta((0.63, 1.17), circular_index=-1)
up_c2 = to_theta((0.65, 1.62), circular_index=-1)

front_high_box_c1 = to_theta((0.63, 1.04), circular_index=-1)
front_high_box_c2 = to_theta((0.50, 1.60), circular_index=-1)

front_middle2_box_c1 = to_theta((0.41, 0.83), circular_index=-1)
front_middle2_box_c2 = to_theta((0.52, 1.30), circular_index=-1)

front_middle1_box_c1 = to_theta((0.34, 0.82), circular_index=-1)
front_middle1_box_c2 = to_theta((0.48, 1.15), circular_index=-1)

ready_above_box_c1 = to_theta((0.38, 0.33), circular_index=-1)
ready_above_box_c2 = to_theta((0.42, 0.51), circular_index=-1)

points = [(ready_above_box, "ReadyAboveBox"),
          (tall_box_grab, "TallBoxGrab"),
          (short_box_grab, "ShortBoxGrab"),
          (front_high_box, "FrontHighBox"),
          (front_middle2_box, "FrontMiddle2Box"),
          (front_middle1_box, "FrontMiddle1Box"),
          (front_low_box, "FrontLowBox"),
          (back_high_box, "BackHighBox"),
          (back_middle2_box, "BackMiddle2Box"),
          (back_middle1_box, "BackMiddle1Box"),
          (back_low_box, "BackLowBox"),
          (front_switch, "FrontSwitch"),
          (back_switch, "BackSwitch"),
          (neutral, "Neutral"),
          (up, "Up")]  # yapf: disable

# We need to define critical points so we can create paths connecting them.
# TODO(austin): Attach velocities to the slow ones.
named_segments = [
    XYSegment(ready_above_box, tall_box_grab, "ReadyToTallBox"),
    XYSegment(ready_above_box, short_box_grab, "ReadyToShortBox"),
    XYSegment(tall_box_grab, short_box_grab, "TallToShortBox"),
    SplineSegment(neutral, ready_above_box_c1, ready_above_box_c2,
                  ready_above_box, "ReadyToNeutral"),
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
    AngleSegment(neutral, back_switch),
    XYSegment(neutral, front_switch),
    XYSegment(neutral, front_low_box),

    XYSegment(up, front_high_box),
    XYSegment(up, front_middle2_box),
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
    XYSegment(back_high_box, back_middle2_box),
    XYSegment(back_high_box, back_middle1_box),
    XYSegment(back_high_box, back_low_box),
    XYSegment(back_middle2_box, back_middle1_box),
    XYSegment(back_middle2_box, back_low_box),
    XYSegment(back_middle1_box, back_low_box),
]

segments = named_segments + unnamed_segments
