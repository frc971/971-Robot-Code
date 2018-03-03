import numpy

# joint_center in x-y space.
joint_center = (-12.275, 11.775)

# Joint distances (l1 = "proximal", l2 = "distal")
l1 = 46.25
l2 = 43.75

# Convert from x-y coordinates to theta coordinates.
# orientation is a bool. This orientation is c_i mod 2.
# where c_i is the circular index, or the position in the
# "hyperextension" zones. "cross_point" allows shifting the place where
# it rounds the result so that it draws nicer (no other functional differences).
def to_theta(x, y, orient, cross_point = -numpy.pi):
  x -= joint_center[0]
  y -= joint_center[1]
  l3 = numpy.sqrt(x ** 2 + y ** 2)
  t3 = numpy.arctan2(y, x)
  t1 = numpy.arccos((l1 ** 2 + l3 ** 2 - l2 ** 2) / (2 * l1 * l3))

  if orient:
    t1 = -t1
  t1 += t3
  t1 = (t1 - cross_point) % (2 * numpy.pi) + cross_point
  t2 = numpy.arctan2(y - l1 * numpy.sin(t1), x - l1 * numpy.cos(t1))
  return (t1, t2)

# Simple trig to go back from theta1, theta2 to x-y
def to_xy(t1, t2):
  x = numpy.cos(t1) * l1 + numpy.cos(t2) * l2 + joint_center[0]
  y = numpy.sin(t1) * l1 + numpy.sin(t2) * l2 + joint_center[1]
  orient = ((t2 - t1) % (2 * numpy.pi)) < numpy.pi
  return (x, y, orient)

# Draw a list of lines to a cairo context.
def draw_lines(cr, lines):
  cr.move_to(lines[0][0], lines[0][1])
  for pt in lines[1:]:
    cr.line_to(pt[0], pt[1])

max_dist = 1.0
max_dist_theta = numpy.pi / 64

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
def subdivide_xy(lines, max_dist = max_dist):
  out = []
  last_pt = lines[0]
  out.append(last_pt)
  for n_pt in lines[1:]:
    for pt in subdivide(last_pt, n_pt, max_dist):
      out.append(pt)
    last_pt = n_pt

  return out

# to_theta, but distinguishes between
def to_theta_with_ci(x, y, ci):
  t1, t2 = to_theta(x, y, (ci % 2) == 0)
  n_ci = int(numpy.floor((t2 - t1) / numpy.pi))
  t2 = t2 + ((ci - n_ci)) * numpy.pi
  return numpy.array((t1, t2))

# alpha is in [0, 1] and is the weight to merge a and b.
def alpha_blend(a, b, alpha):
  return b * alpha + (1 - alpha) * a

# Pure vector normalization.
def normalize(v):
  norm = numpy.linalg.norm(v)
  if norm == 0:
    return v
  return v / norm

# CI is circular index and allows selecting between all the stats that map
# to the same x-y state (by giving them an integer index).
# This will compute approximate first and second derivatives with respect
# to path length.
def to_theta_with_ci_and_derivs(x, y, dx, dy, c_i_select):
  a = to_theta_with_ci(x, y, c_i_select)
  b = to_theta_with_ci(x + dx * 0.0001, y + dy * 0.0001, c_i_select)
  c = to_theta_with_ci(x - dx * 0.0001, y - dy * 0.0001, c_i_select)
  d1 = normalize(b - a)
  d2 = normalize(c - a)
  accel = (d1 + d2) / numpy.linalg.norm(a - b)
  return (a[0], a[1], d1[0], d1[1], accel[0], accel[1])

# Generic subdivision algorithm.
def subdivide(p1, p2, max_dist):
  dx = p2[0] - p1[0]
  dy = p2[1] - p1[1]
  dist = numpy.sqrt(dx ** 2 + dy ** 2)
  n = int(numpy.ceil(dist / max_dist))
  return [(alpha_blend(p1[0], p2[0], float(i) / n),
      alpha_blend(p1[1], p2[1], float(i) / n)) for i in range(1, n + 1)]

# subdivision thresholds.
max_dist = 1.0
max_dist_theta = numpy.pi / 64

# convert from an xy space loop into a theta loop.
# All segements are expected go from one "hyper-extension" boundary
# to another, thus we must go backwards over the "loop" to get a loop in
# x-y space.
def to_theta_loop(lines, cross_point = -numpy.pi):
  out = []
  last_pt = lines[0]
  for n_pt in lines[1:]:
    for pt in subdivide(last_pt, n_pt, max_dist):
      out.append(to_theta(pt[0], pt[1], True, cross_point))
    last_pt = n_pt
  for n_pt in reversed(lines[:-1]):
    for pt in subdivide(last_pt, n_pt, max_dist):
      out.append(to_theta(pt[0], pt[1], False, cross_point))
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

  items = [to_xy(t1, t2) for t1, t2 in lines]
  return [(item[0], item[1]) for item in items]

# Segment in angle space.
class AngleSegment:
  def __init__(self, st, ed):
    self.st = st
    self.ed = ed
  def __repr__(self):
    return "AngleSegment(%s, %s)" % (repr(self.st), repr(self.ed))

  def DrawTo(self, cr, theta_version):
    if (theta_version):
      cr.move_to(self.st[0], self.st[1])
      cr.line_to(self.ed[0], self.ed[1])
    else:
      draw_lines(cr, back_to_xy_loop([self.st, self.ed]))

  def ToThetaPoints(self):
    return [self.st, self.ed]

# Segment in X-Y space.
class XYSegment:
  def __init__(self, st, ed):
    self.st = st
    self.ed = ed
  def __repr__(self):
    return "XYSegment(%s, %s)" % (repr(self.st), repr(self.ed))
  def DrawTo(self, cr, theta_version):
    if (theta_version):
      t1, t2 = self.st
      c_i_select = int(numpy.floor((self.st[1] - self.st[0]) / numpy.pi))
      st = to_xy(*self.st)
      ed = to_xy(*self.ed)

      ln = [(st[0], st[1]), (ed[0], ed[1])]
      draw_lines(cr, [to_theta_with_ci(x, y, c_i_select) for x, y in subdivide_xy(ln)])
    else:
      st = to_xy(*self.st)
      ed = to_xy(*self.ed)
      cr.move_to(st[0], st[1])
      cr.line_to(ed[0], ed[1])

  # Converts to points in theta space via to_theta_with_ci_and_derivs
  def ToThetaPoints(self):
    t1, t2 = self.st
    c_i_select = int(numpy.floor((self.st[1] - self.st[0]) / numpy.pi))
    st = to_xy(*self.st)
    ed = to_xy(*self.ed)

    ln = [(st[0], st[1]), (ed[0], ed[1])]

    dx = ed[0] - st[0]
    dy = ed[1] - st[1]
    mag = numpy.sqrt((dx) ** 2 + (dy) ** 2)
    dx /= mag
    dy /= mag

    return [to_theta_with_ci_and_derivs(x, y, dx, dy, c_i_select) for x, y in subdivide_xy(ln, 1.0)]

segs = [XYSegment((1.3583511559969876, 0.99753029519739866), (0.97145546090878643, -1.4797428713062153))]
segs = [XYSegment((1.3583511559969876, 0.9975302951973987), (1.5666193247337956, 0.042054827580659759))]
