#!/usr/bin/python3
from __future__ import print_function
import os
import sys
from color import palette
from graph import Graph
import gi
import numpy as np
gi.require_version('Gtk', '3.0')
gi.require_version('Gdk', '3.0')
from gi.repository import Gdk, Gtk, GLib
import cairo
from libspline import Spline
import enum
import json
from constants import *
from drawing_constants import set_color, draw_px_cross, draw_px_x, display_text, draw_control_points
from points import Points
import time


class Mode(enum.Enum):
    kViewing = 0
    kPlacing = 1
    kEditing = 2


class FieldWidget(Gtk.DrawingArea):
    """Create a GTK+ widget on which we will draw using Cairo"""

    def __init__(self):
        super(FieldWidget, self).__init__()
        self.set_size_request(mToPx(FIELD.width), mToPx(FIELD.length))

        self.points = Points()
        self.graph = Graph()
        self.set_vexpand(True)
        self.set_hexpand(True)

        # init field drawing
        # add default spline for testing purposes
        # init editing / viewing modes and pointer location
        self.mode = Mode.kPlacing
        self.mousex = 0
        self.mousey = 0
        self.module_path = os.path.dirname(os.path.realpath(sys.argv[0]))
        self.path_to_export = os.path.join(self.module_path,
                                           'points_for_pathedit.json')

        # For the editing mode
        self.index_of_edit = -1  # Can't be zero beause array starts at 0
        self.held_x = 0
        self.spline_edit = -1

        try:
            self.field_png = cairo.ImageSurface.create_from_png(
                "frc971/control_loops/python/field_images/" + FIELD.field_id +
                ".png")
        except cairo.Error:
            self.field_png = None

    def draw_robot_at_point(self, cr, i, p, spline):
        p1 = [mToPx(spline.Point(i)[0]), mToPx(spline.Point(i)[1])]
        p2 = [mToPx(spline.Point(i + p)[0]), mToPx(spline.Point(i + p)[1])]

        #Calculate Robot
        distance = np.sqrt((p2[1] - p1[1])**2 + (p2[0] - p1[0])**2)
        x_difference_o = p2[0] - p1[0]
        y_difference_o = p2[1] - p1[1]
        x_difference = x_difference_o * mToPx(
            FIELD.robot.length / 2) / distance
        y_difference = y_difference_o * mToPx(
            FIELD.robot.length / 2) / distance

        front_middle = []
        front_middle.append(p1[0] + x_difference)
        front_middle.append(p1[1] + y_difference)

        back_middle = []
        back_middle.append(p1[0] - x_difference)
        back_middle.append(p1[1] - y_difference)

        slope = [-(1 / x_difference_o) / (1 / y_difference_o)]
        angle = np.arctan(slope)

        x_difference = np.sin(angle[0]) * mToPx(FIELD.robot.width / 2)
        y_difference = np.cos(angle[0]) * mToPx(FIELD.robot.width / 2)

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
            FIELD.robot.length / 2 + ROBOT_SIDE_TO_BALL_CENTER) / distance
        y_difference = y_difference_o * mToPx(
            FIELD.robot.length / 2 + ROBOT_SIDE_TO_BALL_CENTER) / distance

        #Calculate Ball
        ball_center = []
        ball_center.append(p1[0] + x_difference)
        ball_center.append(p1[1] + y_difference)

        x_difference = x_difference_o * mToPx(
            FIELD.robot.length / 2 + ROBOT_SIDE_TO_HATCH_PANEL) / distance
        y_difference = y_difference_o * mToPx(
            FIELD.robot.length / 2 + ROBOT_SIDE_TO_HATCH_PANEL) / distance

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

    def do_draw(self, cr):  # main

        start_time = time.perf_counter()

        cr.save()
        set_color(cr, palette["BLACK"])

        cr.rectangle(0, 0, mToPx(FIELD.width), mToPx(FIELD.length))
        cr.set_line_join(cairo.LINE_JOIN_ROUND)
        cr.stroke()

        if self.field_png:
            cr.save()
            cr.scale(
                mToPx(FIELD.width) / self.field_png.get_width(),
                mToPx(FIELD.length) / self.field_png.get_height(),
            )
            cr.set_source_surface(self.field_png)
            cr.paint()
            cr.restore()

        # update everything

        if self.mode == Mode.kPlacing or self.mode == Mode.kViewing:
            set_color(cr, palette["BLACK"])
            for i, point in enumerate(self.points.getPoints()):
                draw_px_x(cr, mToPx(point[0]), mToPx(point[1]), 10)
            set_color(cr, palette["WHITE"])
        elif self.mode == Mode.kEditing:
            set_color(cr, palette["BLACK"])
            if self.points.getSplines():
                self.draw_splines(cr)
                for i, points in enumerate(self.points.getSplines()):

                    points = [
                        np.array([mToPx(x), mToPx(y)])
                        for (x, y) in points
                    ]
                    draw_control_points(cr, points)

                    p0, p1, p2, p3, p4, p5 = points
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

        draw_px_cross(cr, self.mousex, self.mousey, 10)
        cr.restore()

        print("spent {:.2f} ms drawing the field widget".format(1000 * (time.perf_counter() - start_time)))

    def draw_splines(self, cr):
        for i, points in enumerate(self.points.getSplines()):
            array = np.zeros(shape=(6, 2), dtype=float)
            for j, point in enumerate(points):
                array[j, 0] = point[0]
                array[j, 1] = point[1]
            spline = Spline(np.ascontiguousarray(np.transpose(array)))
            for k in np.linspace(0.01, 1, 100):
                cr.move_to(
                    mToPx(spline.Point(k - 0.01)[0]),
                    mToPx(spline.Point(k - 0.01)[1]))
                cr.line_to(
                    mToPx(spline.Point(k)[0]), mToPx(spline.Point(k)[1]))
                cr.stroke()
            if i == 0:
                self.draw_robot_at_point(cr, 0.00, 0.01, spline)
            self.draw_robot_at_point(cr, 1, 0.01, spline)

    def mouse_move(self, event):
        old_x = self.mousex
        old_y = self.mousey
        self.mousex, self.mousey = event.x, event.y
        dif_x = self.mousex - old_x
        dif_y = self.mousey - old_y
        difs = np.array([pxToM(dif_x), pxToM(dif_y)])

        if self.mode == Mode.kEditing and self.spline_edit != -1:
            self.points.updates_for_mouse_move(self.index_of_edit,
                                               self.spline_edit,
                                               pxToM(self.mousex),
                                               pxToM(self.mousey), difs)

            self.points.update_lib_spline()
            self.graph.schedule_recalculate(self.points)
        self.queue_draw()

    def export_json(self, file_name):
        self.path_to_export = os.path.join(
            self.module_path,  # position of the python
            "../../..",  # root of the repository
            get_json_folder(FIELD),  # path from the root
            file_name  # selected file
        )

        # Will export to json file
        multi_spline = self.points.toMultiSpline()
        print(multi_spline)
        with open(self.path_to_export, mode='w') as points_file:
            json.dump(multi_spline, points_file)

    def import_json(self, file_name):
        self.path_to_export = os.path.join(
            self.module_path,  # position of the python
            "../../..",  # root of the repository
            get_json_folder(FIELD),  # path from the root
            file_name  # selected file
        )

        # import from json file
        print("LOADING LOAD FROM " + file_name)  # Load takes a few seconds
        with open(self.path_to_export) as points_file:
            multi_spline = json.load(points_file)

        # if people messed with the spline json,
        # it might not be the right length
        # so give them a nice error message
        try:  # try to salvage as many segments of the spline as possible
            self.points.fromMultiSpline(multi_spline)
        except IndexError:
            # check if they're both 6+5*(k-1) long
            expected_length = 6 + 5 * (multi_spline["spline_count"] - 1)
            x_len = len(multi_spline["spline_x"])
            y_len = len(multi_spline["spline_x"])
            if x_len is not expected_length:
                print(
                    "Error: spline x values were not the expected length; expected {} got {}"
                    .format(expected_length, x_len))
            elif y_len is not expected_length:
                print(
                    "Error: spline y values were not the expected length; expected {} got {}"
                    .format(expected_length, y_len))

        print("SPLINES LOADED")
        self.mode = Mode.kEditing

    def key_press(self, event):
        keyval = Gdk.keyval_to_lower(event.keyval)

        # TODO: This should be a button
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
            self.queue_draw()

    def button_press(self, event):
        self.mousex, self.mousey = event.x, event.y

        if self.mode == Mode.kPlacing:
            if self.points.add_point(
                    pxToM(self.mousex), pxToM(self.mousey)):
                self.mode = Mode.kEditing
        elif self.mode == Mode.kEditing:
            # Now after index_of_edit is not -1, the point is selected, so
            # user can click for new point
            if self.index_of_edit == -1:
                # Get clicked point
                # Find nearest
                # Move nearest to clicked
                cur_p = [pxToM(self.mousex), pxToM(self.mousey)]
                # Get the distance between each for x and y
                # Save the index of the point closest
                nearest = 1  # Max distance away a the selected point can be in meters
                index_of_closest = 0
                for index_splines, points in enumerate(
                        self.points.getSplines()):
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
                            self.held_x = self.mousex
        self.queue_draw()

    def button_release(self, event):
        self.mousex, self.mousey = event.x, event.y
        if self.mode == Mode.kEditing:
            if self.index_of_edit > -1 and self.held_x != self.mousex:

                self.points.setSplines(self.spline_edit, self.index_of_edit,
                                       pxToM(self.mousex),
                                       pxToM(self.mousey))

                self.points.splineExtrapolate(self.spline_edit)

                self.points.update_lib_spline()
                self.graph.schedule_recalculate(self.points)

                self.index_of_edit = -1
                self.spline_edit = -1
