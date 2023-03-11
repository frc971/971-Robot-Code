#!/usr/bin/python3

from __future__ import print_function
# matplotlib overrides fontconfig locations, so it needs to be imported before gtk.
import matplotlib.pyplot as plt
import os
from frc971.control_loops.python import basic_window
from frc971.control_loops.python.color import Color, palette
import random
import gi
import numpy as np

gi.require_version('Gtk', '3.0')
from gi.repository import Gdk, Gtk
import cairo
from y2023.control_loops.python.graph_tools import to_theta, to_xy, alpha_blend, shift_angles, get_xy
from y2023.control_loops.python.graph_tools import l1, l2, joint_center
from y2023.control_loops.python.graph_tools import DRIVER_CAM_POINTS
from y2023.control_loops.python import graph_paths

from frc971.control_loops.python.basic_window import quit_main_loop, set_color, OverrideMatrix, identity

import shapely
from shapely.geometry import Polygon


def px(cr):
    return OverrideMatrix(cr, identity)


# Draw lines to cr + stroke.
def draw_lines(cr, lines):
    cr.move_to(lines[0][0], lines[0][1])
    for pt in lines[1:]:
        cr.line_to(pt[0], pt[1])
    with px(cr):
        cr.stroke()


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
    return np.sqrt((l2 + l1)**2 - (x - joint_center[0])**2) + joint_center[1]


# Define min and max l1 angles based on vertical constraints.
def get_angle(boundary):
    h = np.sqrt((l1)**2 - (boundary - joint_center[0])**2) + joint_center[1]
    return np.arctan2(h, boundary - joint_center[0])


# Rotate a rasterized loop such that it aligns to when the parameters loop
def rotate_to_jump_point(points):
    last_pt = points[0]
    for pt_i in range(1, len(points)):
        pt = points[pt_i]
        delta = last_pt[1] - pt[1]
        if abs(delta) > np.pi:
            return points[pt_i:] + points[:pt_i]
        last_pt = pt
    return points


# shift points vertically by dy.
def y_shift(points, dy):
    return [(x, y + dy) for x, y in points]


# Get the closest point to a line from a test pt.
def get_closest(prev, cur, pt):
    dx_ang = (cur[0] - prev[0])
    dy_ang = (cur[1] - prev[1])

    d = np.sqrt(dx_ang**2 + dy_ang**2)
    if (d < 0.000001):
        return prev, np.sqrt((prev[0] - pt[0])**2 + (prev[1] - pt[1])**2)

    pdx = -dy_ang / d
    pdy = dx_ang / d

    dpx = pt[0] - prev[0]
    dpy = pt[1] - prev[1]

    alpha = (dx_ang * dpx + dy_ang * dpy) / d / d

    if (alpha < 0):
        return prev, np.sqrt((prev[0] - pt[0])**2 + (prev[1] - pt[1])**2)
    elif (alpha > 1):
        return cur, np.sqrt((cur[0] - pt[0])**2 + (cur[1] - pt[1])**2)
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


# Defining outline of robot
class RobotOutline():

    def __init__(self, drivetrain_pos, drivetrain_width, joint_center_radius,
                 joint_tower_pos, joint_tower_width, joint_tower_height,
                 driver_cam_pos, driver_cam_width, driver_cam_height):
        self.drivetrain_pos = drivetrain_pos
        self.drivetrain_width = drivetrain_width

        self.joint_center_radius = joint_center_radius
        self.joint_tower_pos = joint_tower_pos
        self.joint_tower_width = joint_tower_width
        self.joint_tower_height = joint_tower_height

        self.driver_cam_pos = driver_cam_pos
        self.driver_cam_width = driver_cam_width
        self.driver_cam_height = driver_cam_height

    def draw(self, cr):
        set_color(cr, palette["BLUE"])

        cr.move_to(self.drivetrain_pos[0], self.drivetrain_pos[1])
        cr.line_to(self.drivetrain_pos[0] + self.drivetrain_width,
                   self.drivetrain_pos[1])

        with px(cr):
            cr.stroke()

        # Draw joint center
        cr.arc(joint_center[0], joint_center[1], self.joint_center_radius, 0,
               2.0 * np.pi)
        with px(cr):
            cr.stroke()

        # Draw joint tower
        cr.rectangle(self.joint_tower_pos[0], self.joint_tower_pos[1],
                     self.joint_tower_width, self.joint_tower_height)

        with px(cr):
            cr.stroke()

        # Draw driver cam
        cr.set_source_rgba(1, 0, 0, 0.5)
        cr.rectangle(self.driver_cam_pos[0], self.driver_cam_pos[1],
                     self.driver_cam_width, self.driver_cam_height)

        with px(cr):
            cr.fill()

    def draw_theta(self, cr):
        # TOOD(Max): add theta mode drawing
        pass


DRIVETRAIN_X = -0.490
DRIVETRAIN_Y = 0.184
DRIVETRAIN_WIDTH = 0.980

JOINT_CENTER_RADIUS = 0.173 / 2

JOINT_TOWER_X = -0.252
JOINT_TOWER_Y = DRIVETRAIN_Y
JOINT_TOWER_WIDTH = 0.098
JOINT_TOWER_HEIGHT = 0.864

DRIVER_CAM_X = DRIVER_CAM_POINTS[0][0]
DRIVER_CAM_Y = DRIVER_CAM_POINTS[0][1]
DRIVER_CAM_WIDTH = DRIVER_CAM_POINTS[-1][0] - DRIVER_CAM_POINTS[0][0]
DRIVER_CAM_HEIGHT = DRIVER_CAM_POINTS[-1][1] - DRIVER_CAM_POINTS[0][1]


class SegmentSelector(basic_window.BaseWindow):

    def __init__(self, segments):
        super(SegmentSelector, self).__init__()

        self.window = Gtk.Window()
        self.window.set_title("Segment Selector")

        self.segments = segments

        self.segment_store = Gtk.ListStore(int, str)

        for i, segment in enumerate(segments):
            self.segment_store.append([i, segment.name])

        self.segment_box = Gtk.ComboBox.new_with_model_and_entry(
            self.segment_store)
        self.segment_box.connect("changed", self.on_combo_changed)
        self.segment_box.set_entry_text_column(1)

        self.current_path_index = None

        self.window.add(self.segment_box)
        self.window.show_all()

    def on_combo_changed(self, combo):
        iter = combo.get_active_iter()

        if iter is not None:
            model = combo.get_model()
            id, name = model[iter][:2]
            print("Selected: ID=%d, name=%s" % (id, name))
            self.current_path_index = id


# Create a GTK+ widget on which we will draw using Cairo
class ArmUi(basic_window.BaseWindow):

    def __init__(self, segments):
        super(ArmUi, self).__init__()

        self.window = Gtk.Window()
        self.window.set_title("DrawingArea")

        self.window.set_events(Gdk.EventMask.BUTTON_PRESS_MASK
                               | Gdk.EventMask.BUTTON_RELEASE_MASK
                               | Gdk.EventMask.POINTER_MOTION_MASK
                               | Gdk.EventMask.SCROLL_MASK
                               | Gdk.EventMask.KEY_PRESS_MASK)
        self.method_connect("key-press-event", self.do_key_press)
        self.method_connect("motion-notify-event", self.do_motion)
        self.method_connect("button-press-event",
                            self._do_button_press_internal)
        self.method_connect("configure-event", self._do_configure)
        self.window.add(self)
        self.window.show_all()

        self.theta_version = False
        self.reinit_extents()

        self.last_pos = to_xy(*graph_paths.points['Neutral'][:2])
        self.circular_index_select = 1

        # Extra stuff for drawing lines.
        self.segments = segments
        self.prev_segment_pt = None
        self.now_segment_pt = None
        self.spline_edit = 0
        self.edit_control1 = True

        self.joint_thetas = None
        self.joint_points = None
        self.fig = plt.figure()
        self.axes = [
            self.fig.add_subplot(3, 1, 1),
            self.fig.add_subplot(3, 1, 2),
            self.fig.add_subplot(3, 1, 3)
        ]
        self.fig.subplots_adjust(hspace=1.0)
        plt.show(block=False)

        self.index = 0

        self.outline = RobotOutline([DRIVETRAIN_X, DRIVETRAIN_Y],
                                    DRIVETRAIN_WIDTH, JOINT_CENTER_RADIUS,
                                    [JOINT_TOWER_X, JOINT_TOWER_Y],
                                    JOINT_TOWER_WIDTH, JOINT_TOWER_HEIGHT,
                                    [DRIVER_CAM_X, DRIVER_CAM_Y],
                                    DRIVER_CAM_WIDTH, DRIVER_CAM_HEIGHT)

        self.segment_selector = SegmentSelector(self.segments)
        self.segment_selector.show()

        self.show_indicators = True
        # Lets you only view selected path
        self.view_current = False

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
            self.extents_x_min = -np.pi * 2
            self.extents_x_max = np.pi * 2
            self.extents_y_min = -np.pi * 2
            self.extents_y_max = np.pi * 2
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
        if self.segment_selector.current_path_index is not None:
            self.index = self.segment_selector.current_path_index

        # Fill the background color of the window with grey
        set_color(cr, palette["GREY"])
        cr.paint()

        # Draw a extents rectangle
        set_color(cr, palette["WHITE"])
        cr.rectangle(self.extents_x_min, self.extents_y_min,
                     (self.extents_x_max - self.extents_x_min),
                     self.extents_y_max - self.extents_y_min)
        cr.fill()

        if self.theta_version:
            # Draw a filled white rectangle.
            set_color(cr, palette["WHITE"])
            cr.rectangle(-np.pi, -np.pi, np.pi * 2.0, np.pi * 2.0)
            cr.fill()

            set_color(cr, palette["BLUE"])
            for i in range(-6, 6):
                cr.move_to(-40, -40 + i * np.pi)
                cr.line_to(40, 40 + i * np.pi)
            with px(cr):
                cr.stroke()

            set_color(cr, Color(0.0, 1.0, 0.2))
            cr.move_to(self.last_pos[0], self.last_pos[1])
            draw_px_cross(cr, 5)

        else:
            # Draw a filled white rectangle.
            set_color(cr, palette["WHITE"])
            cr.rectangle(-2.0, -2.0, 4.0, 4.0)
            cr.fill()

            self.outline.draw(cr)

            # Draw max radius
            set_color(cr, palette["BLUE"])
            cr.arc(joint_center[0], joint_center[1], l2 + l1, 0, 2.0 * np.pi)
            with px(cr):
                cr.stroke()
            cr.arc(joint_center[0], joint_center[1], l1 - l2, 0, 2.0 * np.pi)
            with px(cr):
                cr.stroke()

            set_color(cr, Color(0.5, 1.0, 1))

        if not self.theta_version:
            theta1, theta2 = to_theta(self.last_pos,
                                      self.circular_index_select)
            x, y = joint_center[0], joint_center[1]
            cr.move_to(x, y)

            x += np.cos(theta1) * l1
            y += np.sin(theta1) * l1
            cr.line_to(x, y)
            x += np.cos(theta2) * l2
            y += np.sin(theta2) * l2
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

            self.outline.draw_theta(cr)

        set_color(cr, Color(0.0, 0.5, 1.0))
        if not self.view_current:
            for i in range(len(self.segments)):
                color = None
                if i == self.index:
                    continue
                color = [0, random.random(), 1]
                random.shuffle(color)
                set_color(cr, Color(color[0], color[1], color[2]))
                self.segments[i].DrawTo(cr, self.theta_version)

                with px(cr):
                    cr.stroke()

        # Draw current spline in black
        color = [0, 0, 0]
        set_color(cr, Color(color[0], color[1], color[2]))
        self.segments[self.index].DrawTo(cr, self.theta_version)

        with px(cr):
            cr.stroke()
        control1 = get_xy(self.segments[self.index].control1)
        control2 = get_xy(self.segments[self.index].control2)

        if self.theta_version:
            control1 = shift_angles(self.segments[self.index].control1)
            control2 = shift_angles(self.segments[self.index].control2)

        if self.show_indicators:
            set_color(cr, Color(1.0, 0.0, 1.0))
            cr.move_to(control1[0] + 0.02, control1[1])
            cr.arc(control1[0], control1[1], 0.02, 0, 2.0 * np.pi)
            with px(cr):
                cr.stroke()
            set_color(cr, Color(1.0, 0.7, 0.0))
            cr.move_to(control2[0] + 0.02, control2[1])
            cr.arc(control2[0], control2[1], 0.02, 0, 2.0 * np.pi)

        with px(cr):
            cr.stroke()

        set_color(cr, Color(0.0, 1.0, 0.5))

        # Create the plots
        if self.joint_thetas:
            if self.joint_points:
                titles = ["Proximal", "Distal", "Roll joint"]
                for i in range(len(self.joint_points)):
                    self.axes[i].clear()
                    self.axes[i].plot(self.joint_thetas[0],
                                      self.joint_thetas[1][i])
                    self.axes[i].scatter([self.joint_points[i][0]],
                                         [self.joint_points[i][1]],
                                         s=10,
                                         c="red")
                    self.axes[i].set_title(titles[i])
            plt.title("Joint Angle")
            plt.xlabel("t (0 to 1)")
            plt.ylabel("theta (rad)")

            self.fig.canvas.draw()

    def cur_pt_in_theta(self):
        if self.theta_version: return self.last_pos
        return to_theta(self.last_pos,
                        self.circular_index_select,
                        cross_point=-np.pi,
                        die=False)

    def do_motion(self, event):
        o_x = event.x
        o_y = event.y
        x = event.x - self.window_shape[0] / 2
        y = self.window_shape[1] / 2 - event.y
        scale = self.get_current_scale()
        event.x = x / scale + self.center[0]
        event.y = y / scale + self.center[1]

        segment = self.segments[self.index]
        self.joint_thetas = segment.joint_thetas()

        hovered_t = segment.intersection(event)
        if hovered_t:
            min_diff = np.inf
            closest_t = None
            closest_thetas = None
            for i in range(len(self.joint_thetas[0])):
                t = self.joint_thetas[0][i]
                diff = abs(t - hovered_t)
                if diff < min_diff:
                    min_diff = diff
                    closest_t = t
                    closest_thetas = [
                        self.joint_thetas[1][0][i], self.joint_thetas[1][1][i],
                        self.joint_thetas[1][2][i]
                    ]
            self.joint_points = [(closest_t, closest_theta)
                                 for closest_theta in closest_thetas]

        event.x = o_x
        event.y = o_y

        self.redraw()

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

        elif keyval == Gdk.KEY_r:
            self.prev_segment_pt = self.now_segment_pt

        elif keyval == Gdk.KEY_o:
            # Only prints current segment
            print(repr(self.segments[self.index]))
        elif keyval == Gdk.KEY_g:
            # Generate theta points.
            if self.segments:
                print(repr(self.segments[self.index].ToThetaPoints()))
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

        elif keyval == Gdk.KEY_p:
            if self.index > 0:
                self.index -= 1
            else:
                self.index = len(self.segments) - 1
            print("Switched to segment:", self.segments[self.index].name)
            self.segments[self.index].Print(graph_paths.points)

        elif keyval == Gdk.KEY_i:
            self.show_indicators = not self.show_indicators

        elif keyval == Gdk.KEY_n:
            self.index += 1
            self.index = self.index % len(self.segments)
            print("Switched to segment:", self.segments[self.index].name)
            self.segments[self.index].Print(graph_paths.points)

        elif keyval == Gdk.KEY_l:
            self.view_current = not self.view_current

        elif keyval == Gdk.KEY_t:
            # Toggle between theta and xy renderings
            if self.theta_version:
                theta1, theta2 = self.last_pos
                data = to_xy(theta1, theta2)
                self.circular_index_select = int(
                    np.floor((theta2 - theta1) / np.pi))
                self.last_pos = (data[0], data[1])
            else:
                self.last_pos = self.cur_pt_in_theta()

            self.theta_version = not self.theta_version
            self.reinit_extents()

        elif keyval == Gdk.KEY_z:
            self.edit_control1 = not self.edit_control1
            if self.edit_control1:
                self.now_segment_pt = self.segments[self.index].control1
            else:
                self.now_segment_pt = self.segments[self.index].control2
            if not self.theta_version:
                data = to_xy(self.now_segment_pt[0], self.now_segment_pt[1])
                self.last_pos = (data[0], data[1])
            else:
                self.last_pos = self.now_segment_pt

            print("self.last_pos: ", self.last_pos, " ci: ",
                  self.circular_index_select)

        self.redraw()

    def do_button_press(self, event):
        last_pos = self.last_pos
        self.last_pos = (event.x, event.y)
        pt_theta = self.cur_pt_in_theta()
        if pt_theta is None:
            self.last_pos = last_pos
            return

        self.now_segment_pt = np.array(shift_angles(pt_theta))

        if self.edit_control1:
            self.segments[self.index].control1 = self.now_segment_pt
        else:
            self.segments[self.index].control2 = self.now_segment_pt

        print('Clicked at theta: np.array([%s, %s])' %
              (self.now_segment_pt[0], self.now_segment_pt[1]))
        if not self.theta_version:
            print(
                'Clicked at to_theta_with_circular_index(%.3f, %.3f, circular_index=%d)'
                % (self.last_pos[0], self.last_pos[1],
                   self.circular_index_select))

        self.segments[self.index].Print(graph_paths.points)

        self.redraw()


arm_ui = ArmUi(graph_paths.segments)
print('Starting with segment: ', arm_ui.segments[arm_ui.index].name)
arm_ui.segments[arm_ui.index].Print(graph_paths.points)
basic_window.RunApp()
