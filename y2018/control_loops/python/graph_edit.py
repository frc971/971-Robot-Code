import os
import basic_window
import gi
import numpy
gi.require_version('Gtk', '3.0')
from gi.repository import Gdk
import cairo
import graph_generate
from graph_generate import XYSegment, AngleSegment, to_theta, to_xy, alpha_blend
from graph_generate import back_to_xy_loop, subdivide_theta, to_theta_loop
from graph_generate import l1, l2, joint_center

from basic_window import OverrideMatrix, identity, quit_main_loop

import shapely
from shapely.geometry import Polygon

def px(cr):
  return OverrideMatrix(cr, identity)

# Draws a cross with fixed dimensions in pixel space.
def draw_px_cross(cr, length_px):
  with px(cr):
    x,y = cr.get_current_point()
    cr.move_to(x, y - length_px)
    cr.line_to(x, y + length_px)
    cr.stroke()

    cr.move_to(x - length_px, y)
    cr.line_to(x + length_px, y)
    cr.stroke()

# Distance between two points in angle space.
def angle_dist_sqr(a1, a2):
  return (a1[0] - a2[0]) ** 2 + (a1[1] - a2[1]) ** 2

# Find the highest y position that intersects the vertical line defined by x.
def inter_y(x):
  return numpy.sqrt((l2 + l1) ** 2 - (x - joint_center[0]) ** 2) + joint_center[1]

# This is the x position where the inner (hyperextension) circle intersects the horizontal line
derr = numpy.sqrt((l1 - l2) ** 2 - (joint_center[1] - 12.0) ** 2)

# Define min and max l1 angles based on vertical constraints.
def get_angle(boundary):
  h = numpy.sqrt((l1) ** 2 - (boundary - joint_center[0]) ** 2) + joint_center[1]
  return numpy.arctan2(h, boundary - joint_center[0])

# left hand side lines
lines1 = [
    (-32.525, inter_y(-32.525)),
    (-32.525, 5.5),
    (-23.025, 5.5),
    (-23.025, 12.0),
    (joint_center[0] - derr, 12.0),
]

# right hand side lines
lines2 = [
    (joint_center[0] + derr, 12.0),
    (16.625, 12.0),
    (16.625, 5.5),
    (32.525, 5.5),
    (32.525, inter_y(32.525))
]

t1_min = get_angle(32.525 - 4.0)
t2_min = -7 / 4.0 * numpy.pi

t1_max = get_angle(-32.525 + 4.0)
t2_max = numpy.pi * 3 / 4.0

# Draw lines to cr + stroke.
def draw_lines(cr, lines):
  cr.move_to(lines[0][0], lines[0][1])
  for pt in lines[1:]:
    cr.line_to(pt[0], pt[1])
  with px(cr): cr.stroke()

# Rotate a rasterized loop such that it aligns to when the parameters loop
def rotate_to_jump_point(points):
  last_pt = points[0]
  for pt_i in range(1, len(points)):
    pt = points[pt_i]
    delta = last_pt[1] - pt[1]
    if abs(delta) > numpy.pi:
      print(delta)
      return points[pt_i:] + points[:pt_i]
    last_pt = pt
  return points

# shift points vertically by dy.
def y_shift(points, dy):
  return [(x, y + dy) for x, y in points]

lines1_theta_part = rotate_to_jump_point(to_theta_loop(lines1, 0))
lines2_theta_part = rotate_to_jump_point(to_theta_loop(lines2))

# Some hacks here to make a single polygon by shifting to get an extra copy of the contraints.
lines1_theta = y_shift(lines1_theta_part, -numpy.pi * 2) + lines1_theta_part + \
    y_shift(lines1_theta_part, numpy.pi * 2)
lines2_theta = y_shift(lines2_theta_part, numpy.pi * 2) + lines2_theta_part + \
    y_shift(lines2_theta_part, -numpy.pi * 2)

lines_theta = lines1_theta + lines2_theta

p1 = Polygon(lines_theta)

p2 = Polygon([(t1_min, t2_min), (t1_max, t2_min),
  (t1_max, t2_max), (t1_min, t2_max)])

# Fully computed theta constrints.
lines_theta = list(p1.intersection(p2).exterior.coords)

print(", ".join("{%s, %s}" % (a,b) for a, b in lines_theta))

lines1_theta_back = back_to_xy_loop(lines1_theta)
lines2_theta_back = back_to_xy_loop(lines2_theta)

lines_theta_back = back_to_xy_loop(lines_theta)

# Get the closest point to a line from a test pt.
def get_closest(prev, cur, pt):
  dx_ang = (cur[0] - prev[0])
  dy_ang = (cur[1] - prev[1])

  d = numpy.sqrt(dx_ang ** 2 + dy_ang ** 2)
  if (d < 0.000001):
    return prev, numpy.sqrt((prev[0] - pt[0]) ** 2 + (prev[1] - pt[1]) ** 2)


  pdx = -dy_ang / d
  pdy = dx_ang / d

  dpx = pt[0] - prev[0]
  dpy = pt[1] - prev[1]

  alpha = (dx_ang * dpx + dy_ang * dpy) / d / d

  if (alpha < 0):
    return prev, numpy.sqrt((prev[0] - pt[0]) ** 2 + (prev[1] - pt[1]) ** 2)
  elif (alpha > 1):
    return cur, numpy.sqrt((cur[0] - pt[0]) ** 2 + (cur[1] - pt[1]) ** 2)
  else:
    return (alpha_blend(prev[0], cur[0], alpha), alpha_blend(prev[1], cur[1], alpha)), \
        abs(dpx * pdx + dpy * pdy)

# 
def closest_segment(lines, pt):
  c_pt, c_pt_dist = get_closest(lines[-1], lines[0], pt)
  for i in range(1, len(lines)):
    prev = lines[i - 1]
    cur = lines[i]
    c_pt_new, c_pt_new_dist = get_closest(prev, cur, pt)
    if c_pt_new_dist < c_pt_dist:
      c_pt = c_pt_new
      c_pt_dist = c_pt_new_dist
  return c_pt, c_pt_dist

# Create a GTK+ widget on which we will draw using Cairo
class Silly(basic_window.BaseWindow):
  def __init__(self):
    super().__init__()

    self.theta_version = True
    self.reinit_extents()

    self.last_pos = (20, 20)
    self.c_i_select = 0
    self.click_bool = False


    # Extra stuff for drawing lines.
    self.segs = []
    self.prev_seg_pt = None
    self.now_seg_pt = None

  def reinit_extents(self):
    if self.theta_version:
      self.extents_x_min = -numpy.pi * 2
      self.extents_x_max =  numpy.pi * 2
      self.extents_y_min = -numpy.pi * 2
      self.extents_y_max =  numpy.pi * 2
    else:
      self.extents_x_min = -40.0
      self.extents_x_max =  40.0
      self.extents_y_min = -4.0
      self.extents_y_max =  110.0

    self.init_extents((0.5*(self.extents_x_min+self.extents_x_max), 0.5*(self.extents_y_max+self.extents_y_min)),
                      (1.0*(self.extents_x_max-self.extents_x_min), 1.0*(self.extents_y_max-self.extents_y_min)))

  # Handle the expose-event by drawing
  def handle_draw(self, cr):
    # use "with px(cr): blah;" to transform to pixel coordinates.

    # Fill the background color of the window with grey
    cr.set_source_rgb(0.5, 0.5, 0.5)
    cr.paint()

    # Draw a extents rectangle
    cr.set_source_rgb(1.0, 1.0, 1.0)
    cr.rectangle(self.extents_x_min, self.extents_y_min,
                 (self.extents_x_max-self.extents_x_min), self.extents_y_max-self.extents_y_min)
    cr.fill()

    if not self.theta_version:

      # Draw a filled white rectangle.
      cr.set_source_rgb(1.0, 1.0, 1.0)
      cr.rectangle(-2.0, -2.0, 4.0, 4.0)
      cr.fill()

      cr.set_source_rgb(0.0, 0.0, 1.0)
      cr.arc(joint_center[0], joint_center[1], l2 + l1, 0, 2 * numpy.pi)
      with px(cr): cr.stroke()
      cr.arc(joint_center[0], joint_center[1], l1 - l2, 0, 2 * numpy.pi)
      with px(cr): cr.stroke()

    else:
      # Draw a filled white rectangle.
      cr.set_source_rgb(1.0, 1.0, 1.0)
      cr.rectangle(-numpy.pi, -numpy.pi, numpy.pi * 2, numpy.pi * 2)
      cr.fill()

    if self.theta_version:
      cr.set_source_rgb(0.0, 0.0, 1.0)
      for i in range(-6, 6):
        cr.move_to(-40, -40 + i * numpy.pi)
        cr.line_to(40, 40 + i * numpy.pi)
      with px(cr): cr.stroke()


    if not self.theta_version:
      cr.set_source_rgb(0.2, 1.0, 0.2)
      draw_lines(cr, lines2)

    if self.theta_version:
      cr.set_source_rgb(0.5, 0.5, 1.0)
      draw_lines(cr, lines_theta)

    else:
      cr.set_source_rgb(0.5, 1.0, 1.0)
      draw_lines(cr, lines1)
      draw_lines(cr, lines2)

      def set_color(cr, c_i):
        if c_i == -2:
          cr.set_source_rgb(0.0, 0.25, 1.0)
        elif c_i == -1:
          cr.set_source_rgb(0.5, 0.0, 1.0)
        elif c_i == 0:
          cr.set_source_rgb(0.5, 1.0, 1.0)
        elif c_i == 1:
          cr.set_source_rgb(0.0, 0.5, 1.0)
        elif c_i == 2:
          cr.set_source_rgb(0.5, 1.0, 0.5)
        else:
          cr.set_source_rgb(1.0, 0.0, 0.0)

      def get_ci(pt):
        t1, t2 = pt
        c_i = int(numpy.floor((t2 - t1) / numpy.pi))
        return c_i

      cr.set_source_rgb(0.0, 0.0, 1.0)
      lines = subdivide_theta(lines_theta)
      o_c_i = c_i = get_ci(lines[0])
      p_xy = to_xy(lines[0][0], lines[0][1])
      if c_i == self.c_i_select: cr.move_to(p_xy[0] + c_i * 0, p_xy[1])
      for pt in lines[1:]:
        p_xy = to_xy(pt[0], pt[1])
        c_i = get_ci(pt)
        if o_c_i == self.c_i_select: cr.line_to(p_xy[0] + o_c_i * 0, p_xy[1])
        if c_i != o_c_i:
          o_c_i = c_i
          with px(cr): cr.stroke()
          if c_i == self.c_i_select: cr.move_to(p_xy[0] + c_i * 0, p_xy[1])

      with px(cr): cr.stroke()

    if not self.theta_version:
      t1, t2 = to_theta(self.last_pos[0], self.last_pos[1], (self.c_i_select % 2) == 0)
      x, y = joint_center[0], joint_center[1]
      cr.move_to(x, y)

      x += numpy.cos(t1) * l1
      y += numpy.sin(t1) * l1
      cr.line_to(x, y)
      x += numpy.cos(t2) * l2
      y += numpy.sin(t2) * l2
      cr.line_to(x, y)
      with px(cr): cr.stroke()

      cr.move_to(self.last_pos[0], self.last_pos[1])
      cr.set_source_rgb(0.0, 1.0, 0.2)
      draw_px_cross(cr, 20)

    if self.theta_version:
      cr.set_source_rgb(0.0, 1.0, 0.2)

      cr.set_source_rgb(0.0, 1.0, 0.2)
      cr.move_to(self.last_pos[0], self.last_pos[1])
      draw_px_cross(cr, 5)

      c_pt, dist = closest_segment(lines_theta, self.last_pos)
      print("dist:", dist, c_pt, self.last_pos)
      cr.set_source_rgb(0.0, 1.0, 1.0)
      cr.move_to(c_pt[0], c_pt[1])
      draw_px_cross(cr, 5)

    cr.set_source_rgb(0.0, 0.5, 1.0)
    for seg in self.segs:
      seg.DrawTo(cr, self.theta_version)
      with px(cr): cr.stroke()

    cr.set_source_rgb(0.0, 1.0, 0.5)
    seg = self.current_seg()
    print(seg)
    if seg:
      seg.DrawTo(cr, self.theta_version)
      with px(cr): cr.stroke()

  def cur_pt_in_theta(self):
    if self.theta_version: return self.last_pos
    t1, t2 = to_theta(self.last_pos[0], self.last_pos[1], (self.c_i_select % 2) == 0)
    n_ci = int(numpy.floor((t2 - t1) / numpy.pi))
    t2 = t2 + ((self.c_i_select - n_ci)) * numpy.pi
    return (t1, t2)

  # Current seg based on which mode the drawing system is in.
  def current_seg(self):
    if self.prev_seg_pt and self.now_seg_pt:
      if self.theta_version:
        return AngleSegment(self.prev_seg_pt, self.now_seg_pt)
      else:
        return XYSegment(self.prev_seg_pt, self.now_seg_pt)

  def do_key_press(self, event):
    print("Gdk.KEY_" + Gdk.keyval_name(event.keyval))
    print("Gdk.KEY_" + Gdk.keyval_name(Gdk.keyval_to_lower(event.keyval)) + " is the lower case key for this button press.")
    if ( Gdk.keyval_to_lower(event.keyval) == Gdk.KEY_q ):
      print("Found q key and exiting.")
      quit_main_loop()
    elif ( Gdk.keyval_to_lower(event.keyval) == Gdk.KEY_c ):
      self.c_i_select += 1
    elif ( Gdk.keyval_to_lower(event.keyval) == Gdk.KEY_v ):
      self.c_i_select -= 1
    elif ( Gdk.keyval_to_lower(event.keyval) == Gdk.KEY_f ):
      self.click_bool = not self.click_bool

    elif ( Gdk.keyval_to_lower(event.keyval) == Gdk.KEY_w ):
      seg = self.current_seg();
      if seg: self.segs.append(seg)
      self.prev_seg_pt = self.now_seg_pt

    elif ( Gdk.keyval_to_lower(event.keyval) == Gdk.KEY_r ):
      self.prev_seg_pt = self.now_seg_pt

    elif ( Gdk.keyval_to_lower(event.keyval) == Gdk.KEY_p ):
      print(repr(self.segs))
    elif ( Gdk.keyval_to_lower(event.keyval) == Gdk.KEY_g ):
      if self.segs:
        print(repr(self.segs[0].ToThetaPoints()))
    elif ( Gdk.keyval_to_lower(event.keyval) == Gdk.KEY_e ):
      best_pt = self.now_seg_pt
      best_dist = 1e10
      for seg in self.segs:
        d = angle_dist_sqr(seg.st, self.now_seg_pt)
        if (d < best_dist):
          best_pt = seg.st
          best_dist = d;
        d = angle_dist_sqr(seg.ed, self.now_seg_pt)
        if (d < best_dist):
          best_pt = seg.ed
          best_dist = d
      self.now_seg_pt = best_pt

    elif ( Gdk.keyval_to_lower(event.keyval) == Gdk.KEY_t ):
      if self.theta_version:
        t1, t2 = self.last_pos
        data = to_xy(t1, t2)
        self.c_i_select = int(numpy.floor((t2 - t1) / numpy.pi))
        self.last_pos = (data[0], data[1])
      else:
        self.last_pos = self.cur_pt_in_theta()

      self.theta_version = not self.theta_version
      self.reinit_extents()
    self.redraw()

  def do_button_press(self, event):
    print(event)
    print(event.x, event.y, event.button)
    self.last_pos = (event.x, event.y)
    self.now_seg_pt = self.cur_pt_in_theta();

    self.redraw()

silly = Silly()
silly.segs = graph_generate.segs
basic_window.RunApp()
