#!/usr/bin/python3

from __future__ import print_function
import os
from frc971.control_loops.python import basic_window
from frc971.control_loops.python.color import Color, palette
import random
import gi
import numpy
gi.require_version('Gtk', '3.0')
from gi.repository import Gdk, Gtk
import cairo
import graph_generate
from graph_generate import XYSegment, AngleSegment, to_theta, to_xy, alpha_blend
from graph_generate import back_to_xy_loop, subdivide_theta, to_theta_loop
from graph_generate import l1, l2, joint_center

from frc971.control_loops.python.basic_window import OverrideMatrix, identity, quit_main_loop, set_color

import shapely
from shapely.geometry import Polygon


def px(cr):
    return OverrideMatrix(cr, identity)


def draw_px_cross(cr, length_px):
    """Draws a cross with fixed dimensions in pixel space."""
    with px(cr):
        x, y = cr.get_current_point()
        cr.move_to(x, y - length_px)
        cr.line_to(x, y + length_px)
        cr.stroke()

        cr.move_to(x - length_px, y)
        cr.line_to(x + length_px, y)
        cr.stroke()


def angle_dist_sqr(a1, a2):
    """Distance between two points in angle space."""
    return (a1[0] - a2[0])**2 + (a1[1] - a2[1])**2


# Find the highest y position that intersects the vertical line defined by x.
def inter_y(x):
    return numpy.sqrt((l2 + l1)**2 -
                      (x - joint_center[0])**2) + joint_center[1]


# This is the x position where the inner (hyperextension) circle intersects the horizontal line
derr = numpy.sqrt((l1 - l2)**2 - (joint_center[1] - 0.3048)**2)


# Define min and max l1 angles based on vertical constraints.
def get_angle(boundary):
    h = numpy.sqrt((l1)**2 - (boundary - joint_center[0])**2) + joint_center[1]
    return numpy.arctan2(h, boundary - joint_center[0])


# left hand side lines
lines1 = [
    (-0.826135, inter_y(-0.826135)),
    (-0.826135, 0.1397),
    (-23.025 * 0.0254, 0.1397),
    (-23.025 * 0.0254, 0.3048),
    (joint_center[0] - derr, 0.3048),
]

# right hand side lines
lines2 = [(joint_center[0] + derr, 0.3048), (0.422275, 0.3048),
          (0.422275, 0.1397), (0.826135, 0.1397), (0.826135,
                                                   inter_y(0.826135))]

t1_min = get_angle((32.525 - 4.0) * 0.0254)
t2_min = -7.0 / 4.0 * numpy.pi

t1_max = get_angle((-32.525 + 4.0) * 0.0254)
t2_max = numpy.pi * 3.0 / 4.0


# Draw lines to cr + stroke.
def draw_lines(cr, lines):
    cr.move_to(lines[0][0], lines[0][1])
    for pt in lines[1:]:
        cr.line_to(pt[0], pt[1])
    with px(cr):
        cr.stroke()


# Rotate a rasterized loop such that it aligns to when the parameters loop
def rotate_to_jump_point(points):
    last_pt = points[0]
    for pt_i in range(1, len(points)):
        pt = points[pt_i]
        delta = last_pt[1] - pt[1]
        if abs(delta) > numpy.pi:
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

p2 = Polygon([(t1_min, t2_min), (t1_max, t2_min), (t1_max, t2_max), (t1_min,
                                                                     t2_max)])

# Fully computed theta constrints.
lines_theta = list(p1.intersection(p2).exterior.coords)

lines1_theta_back = back_to_xy_loop(lines1_theta)
lines2_theta_back = back_to_xy_loop(lines2_theta)

lines_theta_back = back_to_xy_loop(lines_theta)


# Get the closest point to a line from a test pt.
def get_closest(prev, cur, pt):
    dx_ang = (cur[0] - prev[0])
    dy_ang = (cur[1] - prev[1])

    d = numpy.sqrt(dx_ang**2 + dy_ang**2)
    if (d < 0.000001):
        return prev, numpy.sqrt((prev[0] - pt[0])**2 + (prev[1] - pt[1])**2)

    pdx = -dy_ang / d
    pdy = dx_ang / d

    dpx = pt[0] - prev[0]
    dpy = pt[1] - prev[1]

    alpha = (dx_ang * dpx + dy_ang * dpy) / d / d

    if (alpha < 0):
        return prev, numpy.sqrt((prev[0] - pt[0])**2 + (prev[1] - pt[1])**2)
    elif (alpha > 1):
        return cur, numpy.sqrt((cur[0] - pt[0])**2 + (cur[1] - pt[1])**2)
    else:
        return (alpha_blend(prev[0], cur[0], alpha), alpha_blend(prev[1], cur[1], alpha)), \
            abs(dpx * pdx + dpy * pdy)



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
        super(Silly, self).__init__()

        self.window = Gtk.Window()
        self.window.set_title("DrawingArea")

        self.window.set_events(Gdk.EventMask.BUTTON_PRESS_MASK
                               | Gdk.EventMask.BUTTON_RELEASE_MASK
                               | Gdk.EventMask.POINTER_MOTION_MASK
                               | Gdk.EventMask.SCROLL_MASK
                               | Gdk.EventMask.KEY_PRESS_MASK)
        self.method_connect("key-press-event", self.do_key_press)
        self.method_connect("button-press-event",
                            self._do_button_press_internal)
        self.method_connect("configure-event", self._do_configure)
        self.window.add(self)
        self.window.show_all()

        self.theta_version = False
        self.reinit_extents()

        self.last_pos = (numpy.pi / 2.0, 1.0)
        self.circular_index_select = -1

        # Extra stuff for drawing lines.
        self.segments = []
        self.prev_segment_pt = None
        self.now_segment_pt = None
        self.spline_edit = 0
        self.edit_control1 = True

    def do_key_press(self, event):
        pass

    def _do_button_press_internal(self, event):
        o_x = event.x
        o_y = event.y
        x = event.x - self.window_shape[0] / 2
        y = self.window_shape[1] / 2 - event.y
        scale = self.get_current_scale()
        event.x = x / scale + self.center[0]
        event.y = y / scale + self.center[1]
        self.do_button_press(event)
        event.x = o_x
        event.y = o_y

    def do_button_press(self, event):
        pass

    def _do_configure(self, event):
        self.window_shape = (event.width, event.height)

    def redraw(self):
        if not self.needs_redraw:
            self.needs_redraw = True
            self.window.queue_draw()

    def method_connect(self, event, cb):
        def handler(obj, *args):
            cb(*args)

        self.window.connect(event, handler)

    def reinit_extents(self):
        if self.theta_version:
            self.extents_x_min = -numpy.pi * 2
            self.extents_x_max = numpy.pi * 2
            self.extents_y_min = -numpy.pi * 2
            self.extents_y_max = numpy.pi * 2
        else:
            self.extents_x_min = -40.0 * 0.0254
            self.extents_x_max = 40.0 * 0.0254
            self.extents_y_min = -4.0 * 0.0254
            self.extents_y_max = 110.0 * 0.0254

        self.init_extents(
            (0.5 * (self.extents_x_min + self.extents_x_max), 0.5 *
             (self.extents_y_max + self.extents_y_min)),
            (1.0 * (self.extents_x_max - self.extents_x_min), 1.0 *
             (self.extents_y_max - self.extents_y_min)))

    # Handle the expose-event by drawing
    def handle_draw(self, cr):
        # use "with px(cr): blah;" to transform to pixel coordinates.

        # Fill the background color of the window with grey
        set_color(cr, palette["GREY"])
        cr.paint()

        # Draw a extents rectangle
        set_color(cr, palette["WHITE"])
        cr.rectangle(self.extents_x_min, self.extents_y_min,
                     (self.extents_x_max - self.extents_x_min),
                     self.extents_y_max - self.extents_y_min)
        cr.fill()

        if not self.theta_version:
            # Draw a filled white rectangle.
            set_color(cr, palette["WHITE"])
            cr.rectangle(-2.0, -2.0, 4.0, 4.0)
            cr.fill()

            set_color(cr, palette["BLUE"])
            cr.arc(joint_center[0], joint_center[1], l2 + l1, 0,
                   2.0 * numpy.pi)
            with px(cr):
                cr.stroke()
            cr.arc(joint_center[0], joint_center[1], l1 - l2, 0,
                   2.0 * numpy.pi)
            with px(cr):
                cr.stroke()
        else:
            # Draw a filled white rectangle.
            set_color(cr, palette["WHITE"])
            cr.rectangle(-numpy.pi, -numpy.pi, numpy.pi * 2.0, numpy.pi * 2.0)
            cr.fill()

        if self.theta_version:
            set_color(cr, palette["BLUE"])
            for i in range(-6, 6):
                cr.move_to(-40, -40 + i * numpy.pi)
                cr.line_to(40, 40 + i * numpy.pi)
            with px(cr):
                cr.stroke()

        if self.theta_version:
            set_color(cr, Color(0.5, 0.5, 1.0))
            draw_lines(cr, lines_theta)
        else:
            set_color(cr, Color(0.5, 1.0, 1.0))
            draw_lines(cr, lines1)
            draw_lines(cr, lines2)

            def get_circular_index(pt):
                theta1, theta2 = pt
                circular_index = int(numpy.floor((theta2 - theta1) / numpy.pi))
                return circular_index

            set_color(cr, palette["BLUE"])
            lines = subdivide_theta(lines_theta)
            o_circular_index = circular_index = get_circular_index(lines[0])
            p_xy = to_xy(lines[0][0], lines[0][1])
            if circular_index == self.circular_index_select:
                cr.move_to(p_xy[0] + circular_index * 0, p_xy[1])
            for pt in lines[1:]:
                p_xy = to_xy(pt[0], pt[1])
                circular_index = get_circular_index(pt)
                if o_circular_index == self.circular_index_select:
                    cr.line_to(p_xy[0] + o_circular_index * 0, p_xy[1])
                if circular_index != o_circular_index:
                    o_circular_index = circular_index
                    with px(cr):
                        cr.stroke()
                    if circular_index == self.circular_index_select:
                        cr.move_to(p_xy[0] + circular_index * 0, p_xy[1])

            with px(cr):
                cr.stroke()

        if not self.theta_version:
            theta1, theta2 = to_theta(self.last_pos, self.circular_index_select)
            x, y = joint_center[0], joint_center[1]
            cr.move_to(x, y)

            x += numpy.cos(theta1) * l1
            y += numpy.sin(theta1) * l1
            cr.line_to(x, y)
            x += numpy.cos(theta2) * l2
            y += numpy.sin(theta2) * l2
            cr.line_to(x, y)
            with px(cr):
                cr.stroke()

            cr.move_to(self.last_pos[0], self.last_pos[1])
            set_color(cr, Color(0.0, 1.0, 0.2))
            draw_px_cross(cr, 20)

        if self.theta_version:
            set_color(cr, Color(0.0, 1.0, 0.2))
            cr.move_to(self.last_pos[0], self.last_pos[1])
            draw_px_cross(cr, 5)

            c_pt, dist = closest_segment(lines_theta, self.last_pos)
            print("dist:", dist, c_pt, self.last_pos)
            set_color(cr, palette["CYAN"])
            cr.move_to(c_pt[0], c_pt[1])
            draw_px_cross(cr, 5)

        set_color(cr, Color(0.0, 0.5, 1.0))
        for segment in self.segments:
            color = [0, random.random(), 1]
            random.shuffle(color)
            set_color(cr, Color(color[0], color[1], color[2]))
            segment.DrawTo(cr, self.theta_version)
            with px(cr):
                cr.stroke()

        set_color(cr, Color(0.0, 1.0, 0.5))
        segment = self.current_seg()
        if segment:
            print(segment)
            segment.DrawTo(cr, self.theta_version)
            with px(cr):
                cr.stroke()

    def cur_pt_in_theta(self):
        if self.theta_version: return self.last_pos
        return to_theta(self.last_pos, self.circular_index_select)

    # Current segment based on which mode the drawing system is in.
    def current_seg(self):
        if self.prev_segment_pt and self.now_segment_pt:
            if self.theta_version:
                return AngleSegment(self.prev_segment_pt, self.now_segment_pt)
            else:
                return XYSegment(self.prev_segment_pt, self.now_segment_pt)

    def do_key_press(self, event):
        keyval = Gdk.keyval_to_lower(event.keyval)
        print("Gdk.KEY_" + Gdk.keyval_name(keyval))
        if keyval == Gdk.KEY_q:
            print("Found q key and exiting.")
            quit_main_loop()
        elif keyval == Gdk.KEY_c:
            # Increment which arm solution we render
            self.circular_index_select += 1
            print(self.circular_index_select)
        elif keyval == Gdk.KEY_v:
            # Decrement which arm solution we render
            self.circular_index_select -= 1
            print(self.circular_index_select)
        elif keyval == Gdk.KEY_w:
            # Add this segment to the segment list.
            segment = self.current_seg()
            if segment: self.segments.append(segment)
            self.prev_segment_pt = self.now_segment_pt

        elif keyval == Gdk.KEY_r:
            self.prev_segment_pt = self.now_segment_pt

        elif keyval == Gdk.KEY_p:
            # Print out the segments.
            print(repr(self.segments))
        elif keyval == Gdk.KEY_g:
            # Generate theta points.
            if self.segments:
                print(repr(self.segments[0].ToThetaPoints()))
        elif keyval == Gdk.KEY_e:
            best_pt = self.now_segment_pt
            best_dist = 1e10
            for segment in self.segments:
                d = angle_dist_sqr(segment.start, self.now_segment_pt)
                if (d < best_dist):
                    best_pt = segment.start
                    best_dist = d
                d = angle_dist_sqr(segment.end, self.now_segment_pt)
                if (d < best_dist):
                    best_pt = segment.end
                    best_dist = d
            self.now_segment_pt = best_pt

        elif keyval == Gdk.KEY_t:
            # Toggle between theta and xy renderings
            if self.theta_version:
                theta1, theta2 = self.last_pos
                data = to_xy(theta1, theta2)
                self.circular_index_select = int(
                    numpy.floor((theta2 - theta1) / numpy.pi))
                self.last_pos = (data[0], data[1])
            else:
                self.last_pos = self.cur_pt_in_theta()

            self.theta_version = not self.theta_version
            self.reinit_extents()

        elif keyval == Gdk.KEY_z:
            self.edit_control1 = not self.edit_control1
            if self.edit_control1:
                self.now_segment_pt = self.segments[0].control1
            else:
                self.now_segment_pt = self.segments[0].control2
            if not self.theta_version:
                data = to_xy(self.now_segment_pt[0], self.now_segment_pt[1])
                self.last_pos = (data[0], data[1])
            else:
                self.last_pos = self.now_segment_pt

            print("self.last_pos: ", self.last_pos, " ci: ",
                  self.circular_index_select)

        self.redraw()

    def do_button_press(self, event):
        self.last_pos = (event.x, event.y)
        self.now_segment_pt = self.cur_pt_in_theta()

        if self.edit_control1:
            self.segments[0].control1 = self.now_segment_pt
        else:
            self.segments[0].control2 = self.now_segment_pt

        print('Clicked at theta: %s' % (repr(self.now_segment_pt,)))
        if not self.theta_version:
            print('Clicked at xy, circular index: (%f, %f, %f)' %
                  (self.last_pos[0], self.last_pos[1],
                   self.circular_index_select))

        print('c1: numpy.array([%f, %f])' % (self.segments[0].control1[0],
                                             self.segments[0].control1[1]))
        print('c2: numpy.array([%f, %f])' % (self.segments[0].control2[0],
                                             self.segments[0].control2[1]))

        self.redraw()


silly = Silly()
silly.segments = graph_generate.segments
basic_window.RunApp()
