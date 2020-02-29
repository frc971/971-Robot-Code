#!/usr/bin/python3
from __future__ import print_function
import os
import sys
import copy
from color import Color, palette
import random
import gi
import numpy as np
import scipy.spatial.distance
gi.require_version('Gtk', '3.0')
gi.require_version('Gdk', '3.0')
from gi.repository import Gdk, Gtk, GLib
import cairo
from libspline import Spline, DistanceSpline, Trajectory
import enum
import json
from basic_window import *
from constants import *
from drawing_constants import *
from points import Points
from graph import Graph


class Mode(enum.Enum):
    kViewing = 0
    kPlacing = 1
    kEditing = 2
    kExporting = 3
    kImporting = 4


class GTK_Widget(BaseWindow):
    """Create a GTK+ widget on which we will draw using Cairo"""
    def __init__(self):
        super(GTK_Widget, self).__init__()

        self.points = Points()

        # init field drawing
        # add default spline for testing purposes
        # init editing / viewing modes and pointer location
        self.mode = Mode.kPlacing
        self.x = 0
        self.y = 0
        module_path = os.path.dirname(os.path.realpath(sys.argv[0]))
        self.path_to_export = os.path.join(module_path,
                                           'points_for_pathedit.json')

        # update list of control points
        self.point_selected = False
        # self.adding_spline = False
        self.index_of_selected = -1
        self.new_point = []

        # For the editing mode
        self.index_of_edit = -1  # Can't be zero beause array starts at 0
        self.held_x = 0
        self.spline_edit = -1

        self.curves = []

        self.colors = []

        for c in palette:
            self.colors.append(palette[c])

        self.reinit_extents()

        self.inStart = None
        self.inEnd = None
        self.inValue = None
        self.startSet = False

    """set extents on images"""

    def reinit_extents(self):
        self.extents_x_min = -1.0 * SCREEN_SIZE
        self.extents_x_max = SCREEN_SIZE
        self.extents_y_min = -1.0 * SCREEN_SIZE
        self.extents_y_max = SCREEN_SIZE

    # this needs to be rewritten with numpy, i dont think this ought to have
    # SciPy as a dependecy
    def get_index_of_nearest_point(self):
        cur_p = [[self.x, self.y]]
        distances = scipy.spatial.distance.cdist(cur_p, self.all_controls)

        return np.argmin(distances)

    # return the closest point to the loc of the click event
    def get_nearest_point(self):
        return self.all_controls[self.get_index_of_nearest_point()]

    def draw_field_elements(self, cr):
        draw_HAB(cr)
        draw_rockets(cr)
        draw_cargo_ship(cr)

    def draw_robot_at_point(self, cr, i, p, spline):
        p1 = [mToPx(spline.Point(i)[0]), mToPx(spline.Point(i)[1])]
        p2 = [mToPx(spline.Point(i + p)[0]), mToPx(spline.Point(i + p)[1])]

        #Calculate Robot
        distance = np.sqrt((p2[1] - p1[1])**2 + (p2[0] - p1[0])**2)
        x_difference_o = p2[0] - p1[0]
        y_difference_o = p2[1] - p1[1]
        x_difference = x_difference_o * mToPx(LENGTH_OF_ROBOT / 2) / distance
        y_difference = y_difference_o * mToPx(LENGTH_OF_ROBOT / 2) / distance

        front_middle = []
        front_middle.append(p1[0] + x_difference)
        front_middle.append(p1[1] + y_difference)

        back_middle = []
        back_middle.append(p1[0] - x_difference)
        back_middle.append(p1[1] - y_difference)

        slope = [-(1 / x_difference_o) / (1 / y_difference_o)]
        angle = np.arctan(slope)

        x_difference = np.sin(angle[0]) * mToPx(WIDTH_OF_ROBOT / 2)
        y_difference = np.cos(angle[0]) * mToPx(WIDTH_OF_ROBOT / 2)

        front_1 = []
        front_1.append(front_middle[0] - x_difference)
        front_1.append(front_middle[1] - y_difference)

        front_2 = []
        front_2.append(front_middle[0] + x_difference)
        front_2.append(front_middle[1] + y_difference)

        back_1 = []
        back_1.append(back_middle[0] - x_difference)
        back_1.append(back_middle[1] - y_difference)

        back_2 = []
        back_2.append(back_middle[0] + x_difference)
        back_2.append(back_middle[1] + y_difference)

        x_difference = x_difference_o * mToPx(
            LENGTH_OF_ROBOT / 2 + ROBOT_SIDE_TO_BALL_CENTER) / distance
        y_difference = y_difference_o * mToPx(
            LENGTH_OF_ROBOT / 2 + ROBOT_SIDE_TO_BALL_CENTER) / distance

        #Calculate Ball
        ball_center = []
        ball_center.append(p1[0] + x_difference)
        ball_center.append(p1[1] + y_difference)

        x_difference = x_difference_o * mToPx(
            LENGTH_OF_ROBOT / 2 + ROBOT_SIDE_TO_HATCH_PANEL) / distance
        y_difference = y_difference_o * mToPx(
            LENGTH_OF_ROBOT / 2 + ROBOT_SIDE_TO_HATCH_PANEL) / distance

        #Calculate Panel
        panel_center = []
        panel_center.append(p1[0] + x_difference)
        panel_center.append(p1[1] + y_difference)

        x_difference = np.sin(angle[0]) * mToPx(HATCH_PANEL_WIDTH / 2)
        y_difference = np.cos(angle[0]) * mToPx(HATCH_PANEL_WIDTH / 2)

        panel_1 = []
        panel_1.append(panel_center[0] + x_difference)
        panel_1.append(panel_center[1] + y_difference)

        panel_2 = []
        panel_2.append(panel_center[0] - x_difference)
        panel_2.append(panel_center[1] - y_difference)

        #Draw Robot
        cr.move_to(front_1[0], front_1[1])
        cr.line_to(back_1[0], back_1[1])
        cr.line_to(back_2[0], back_2[1])
        cr.line_to(front_2[0], front_2[1])
        cr.line_to(front_1[0], front_1[1])

        cr.stroke()

        #Draw Ball
        set_color(cr, palette["ORANGE"], 0.5)
        cr.move_to(back_middle[0], back_middle[1])
        cr.line_to(ball_center[0], ball_center[1])
        cr.arc(ball_center[0], ball_center[1], mToPx(BALL_RADIUS), 0,
               2 * np.pi)
        cr.stroke()

        #Draw Panel
        set_color(cr, palette["YELLOW"], 0.5)
        cr.move_to(panel_1[0], panel_1[1])
        cr.line_to(panel_2[0], panel_2[1])

        cr.stroke()
        cr.set_source_rgba(0, 0, 0, 1)

    def handle_draw(self, cr):  # main
        # Fill the background color of the window with grey
        set_color(cr, palette["WHITE"])
        cr.paint()

        # Draw a extents rectangle
        set_color(cr, palette["WHITE"])
        cr.rectangle(self.extents_x_min, self.extents_y_min,
                     (self.extents_x_max - self.extents_x_min),
                     self.extents_y_max - self.extents_y_min)
        cr.fill()

        #Drawing the switch and scale in the field
        cr.move_to(0, 50)
        cr.show_text('Press "e" to export')
        cr.show_text('Press "i" to import')

        set_color(cr, palette["WHITE"])
        cr.rectangle(0, -mToPx(8.2296 / 2.0), SCREEN_SIZE, SCREEN_SIZE)
        cr.fill()
        set_color(cr, palette["BLACK"])
        cr.rectangle(0, -mToPx(8.2296 / 2.0), SCREEN_SIZE, SCREEN_SIZE)
        cr.set_line_join(cairo.LINE_JOIN_ROUND)
        cr.stroke()
        self.draw_field_elements(cr)
        y = 0

        # update everything

        if self.mode == Mode.kPlacing or self.mode == Mode.kViewing:
            set_color(cr, palette["BLACK"])
            cr.move_to(-SCREEN_SIZE, 170)
            plotPoints = self.points.getPoints()
            if plotPoints:
                for i, point in enumerate(plotPoints):
                    draw_px_x(cr, mToPx(point[0]), mToPx(point[1]), 10)
                    cr.move_to(mToPx(point[0]), mToPx(point[1]) - 15)
                    display_text(cr, str(i), 0.5, 0.5, 2, 2)
            set_color(cr, palette["WHITE"])

        elif self.mode == Mode.kEditing:
            set_color(cr, palette["BLACK"])
            cr.move_to(-SCREEN_SIZE, 170)
            display_text(cr, "EDITING", 1, 1, 1, 1)
            if self.points.getSplines():
                self.draw_splines(cr)
                for i, points in enumerate(self.points.getSplines()):

                    p0 = np.array([mToPx(points[0][0]), mToPx(points[0][1])])
                    p1 = np.array([mToPx(points[1][0]), mToPx(points[1][1])])
                    p2 = np.array([mToPx(points[2][0]), mToPx(points[2][1])])
                    p3 = np.array([mToPx(points[3][0]), mToPx(points[3][1])])
                    p4 = np.array([mToPx(points[4][0]), mToPx(points[4][1])])
                    p5 = np.array([mToPx(points[5][0]), mToPx(points[5][1])])

                    draw_control_points(cr, [p0, p1, p2, p3, p4, p5])
                    first_tangent = p0 + 2.0 * (p1 - p0)
                    second_tangent = p5 + 2.0 * (p4 - p5)
                    cr.set_source_rgb(0, 0.5, 0)
                    cr.move_to(p0[0], p0[1])
                    cr.set_line_width(1.0)
                    cr.line_to(first_tangent[0], first_tangent[1])
                    cr.move_to(first_tangent[0], first_tangent[1])
                    cr.line_to(p2[0], p2[1])

                    cr.move_to(p5[0], p5[1])
                    cr.line_to(second_tangent[0], second_tangent[1])

                    cr.move_to(second_tangent[0], second_tangent[1])
                    cr.line_to(p3[0], p3[1])

                    cr.stroke()
                    cr.set_line_width(2.0)
            self.points.update_lib_spline()
            set_color(cr, palette["WHITE"])

        cr.paint_with_alpha(0.2)

        mygraph = Graph(cr, self.points)
        draw_px_cross(cr, self.x, self.y, 10)

    def draw_splines(self, cr):
        holder_spline = []
        for i, points in enumerate(self.points.getSplines()):
            array = np.zeros(shape=(6, 2), dtype=float)
            for j, point in enumerate(points):
                array[j, 0] = point[0]
                array[j, 1] = point[1]
            spline = Spline(np.ascontiguousarray(np.transpose(array)))
            for k in np.linspace(0.01, 1, 100):
                cr.move_to(mToPx(spline.Point(k - 0.01)[0]),
                           mToPx(spline.Point(k - 0.01)[1]))
                cr.line_to(mToPx(spline.Point(k)[0]),
                           mToPx(spline.Point(k)[1]))
                cr.stroke()
                holding = [
                    spline.Point(k - 0.01)[0],
                    spline.Point(k - 0.01)[1]
                ]
                holder_spline.append(holding)
            if i == 0:
                self.draw_robot_at_point(cr, 0.00, 0.01, spline)
            self.draw_robot_at_point(cr, 1, 0.01, spline)
        self.curves.append(holder_spline)

    def mouse_move(self, event):
        old_x = self.x
        old_y = self.y
        self.x = event.x
        self.y = event.y
        dif_x = event.x - old_x
        dif_y = event.y - old_y
        difs = np.array([pxToM(dif_x), pxToM(dif_y)])

        if self.mode == Mode.kEditing:
            self.spline_edit = self.points.updates_for_mouse_move(
                self.index_of_edit, self.spline_edit, self.x, self.y, difs)

    def do_key_press(self, event, file_name):
        keyval = Gdk.keyval_to_lower(event.keyval)
        module_path = os.path.dirname(os.path.realpath(sys.argv[0]))
        self.path_to_export = os.path.join(module_path,
                                           "spline_jsons/" + file_name)
        if keyval == Gdk.KEY_q:
            print("Found q key and exiting.")
            quit_main_loop()
        file_name_end = file_name[-5:]
        if file_name_end != ".json":
            print("Error: Filename doesn't end in .json")
        else:
            if keyval == Gdk.KEY_e:
                # Will export to json file
                self.mode = Mode.kEditing
                print('out to: ', self.path_to_export)
                exportList = [l.tolist() for l in self.points.getSplines()]
                with open(self.path_to_export, mode='w') as points_file:
                    json.dump(exportList, points_file)

            if keyval == Gdk.KEY_i:
                # import from json file
                self.mode = Mode.kEditing
                self.points.resetPoints()
                self.points.resetSplines()
                with open(self.path_to_export) as points_file:
                    self.points.setUpSplines(json.load(points_file))

                self.points.update_lib_spline()

        if keyval == Gdk.KEY_p:
            self.mode = Mode.kPlacing
            # F0 = A1
            # B1 = 2F0 - E0
            # C1= d0 + 4F0 - 4E0
            spline_index = len(self.points.getSplines()) - 1
            self.points.resetPoints()
            self.points.extrapolate(
                self.points.getSplines()[len(self.points.getSplines()) - 1][5],
                self.points.getSplines()[len(self.points.getSplines()) - 1][4],
                self.points.getSplines()[len(self.points.getSplines()) - 1][3])

    def button_press_action(self):
        if self.mode == Mode.kPlacing:
            if self.points.add_point(self.x, self.y):
                self.mode = Mode.kEditing
        elif self.mode == Mode.kEditing:
            # Now after index_of_edit is not -1, the point is selected, so
            # user can click for new point
            if self.index_of_edit > -1 and self.held_x != self.x:
                self.points.setSplines(self.spline_edit, self.index_of_edit,
                                       pxToM(self.x), pxToM(self.y))

                self.spline_edit = self.points.splineExtrapolate(
                    self.spline_edit)

                self.index_of_edit = -1
                self.spline_edit = -1
            else:
                # Get clicked point
                # Find nearest
                # Move nearest to clicked
                cur_p = [pxToM(self.x), pxToM(self.y)]
                # Get the distance between each for x and y
                # Save the index of the point closest
                nearest = 1  # Max distance away a the selected point can be in meters
                index_of_closest = 0
                for index_splines, points in enumerate(self.points.getSplines()):
                    for index_points, val in enumerate(points):
                        distance = np.sqrt((cur_p[0] - val[0])**2 +
                                           (cur_p[1] - val[1])**2)
                        if distance < nearest:
                            nearest = distance
                            index_of_closest = index_points
                            print("Nearest: " + str(nearest))
                            print("Index: " + str(index_of_closest))
                            self.index_of_edit = index_of_closest
                            self.spline_edit = index_splines
                            self.held_x = self.x

    def do_button_press(self, event):
        # Be consistent with the scaling in the drawing_area
        self.x = event.x * 2
        self.y = event.y * 2
        self.button_press_action()
