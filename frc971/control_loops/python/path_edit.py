#!/usr/bin/python3
from __future__ import print_function
import os
import sys
from color import palette
from graph import Graph
import gi
import numpy as np
gi.require_version('Gtk', '3.0')
from gi.repository import Gdk, Gtk, GLib
import cairo
from libspline import Spline, DistanceSpline
import enum
import json
from constants import FIELD
from constants import get_json_folder
from constants import ROBOT_SIDE_TO_BALL_CENTER, ROBOT_SIDE_TO_HATCH_PANEL, HATCH_PANEL_WIDTH, BALL_RADIUS
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
        self.set_field(FIELD)
        self.set_size_request(self.mToPx(self.field.width),
                              self.mToPx(self.field.length))

        self.points = Points()
        self.graph = Graph()
        self.graph.cursor_watcher = self
        self.set_vexpand(True)
        self.set_hexpand(True)
        # list of multisplines
        self.multispline_stack = []
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

        self.zoom_transform = cairo.Matrix()

        self.set_events(Gdk.EventMask.BUTTON_PRESS_MASK
                        | Gdk.EventMask.BUTTON_PRESS_MASK
                        | Gdk.EventMask.BUTTON_RELEASE_MASK
                        | Gdk.EventMask.POINTER_MOTION_MASK
                        | Gdk.EventMask.SCROLL_MASK)

    def set_field(self, field):
        self.field = field
        try:
            self.field_png = cairo.ImageSurface.create_from_png(
                "frc971/control_loops/python/field_images/" +
                self.field.field_id + ".png")
        except cairo.Error:
            self.field_png = None

        self.queue_draw()

    def invert(self, transform):
        xx, yx, xy, yy, x0, y0 = transform
        matrix = cairo.Matrix(xx, yx, xy, yy, x0, y0)
        matrix.invert()
        return matrix

    # returns the transform from widget space to field space
    @property
    def input_transform(self):
        # the transform for input needs to be the opposite of the transform for drawing
        return self.invert(self.field_transform.multiply(self.zoom_transform))

    @property
    def field_transform(self):
        field_transform = cairo.Matrix()
        field_transform.scale(1, -1)  # flipped y-axis
        field_transform.scale(1 / self.pxToM_scale(), 1 / self.pxToM_scale())
        field_transform.translate(self.field.width / 2,
                                  -1 * self.field.length / 2)
        return field_transform

    # returns the scale from pixels in field space to meters in field space
    def pxToM_scale(self):
        available_space = self.get_allocation()
        return np.maximum(self.field.width / available_space.width,
                          self.field.length / available_space.height)

    def pxToM(self, p):
        return p * self.pxToM_scale()

    def mToPx(self, m):
        return m / self.pxToM_scale()

    def draw_robot_at_point(self, cr, spline, t):
        """Draws the robot at a point along a Spline or DistanceSpline"""

        # we accept both Spline and DistanceSpline
        if type(spline) is Spline:
            point = spline.Point(t)
            theta = spline.Theta(t)
        elif type(spline) is DistanceSpline:
            point = spline.XY(t)
            theta = spline.Theta(t)
        else:
            raise TypeError(
                f"expected Spline or DistanceSpline (got {type(spline)})")

        # Transform so that +y is forward along the spline
        transform = cairo.Matrix()
        transform.translate(*point)
        transform.rotate(theta - np.pi / 2)

        cr.save()
        cr.set_matrix(transform.multiply(cr.get_matrix()))

        # Draw Robot
        set_color(cr, palette["BLACK"])
        cr.rectangle(-self.field.robot.width / 2, -self.field.robot.length / 2,
                     self.field.robot.width, self.field.robot.length)
        cr.stroke()

        #Draw Ball
        set_color(cr, palette["ORANGE"], 0.5)
        cr.arc(0, self.field.robot.length / 2 + BALL_RADIUS, BALL_RADIUS, 0,
               2 * np.pi)
        cr.stroke()

        # undo the transform
        cr.restore()

    def do_draw(self, cr):  # main
        cr.set_matrix(
            self.field_transform.multiply(self.zoom_transform).multiply(
                cr.get_matrix()))

        cr.save()

        set_color(cr, palette["BLACK"])

        cr.set_line_width(self.pxToM(1))
        cr.rectangle(-0.5 * self.field.width, -0.5 * self.field.length,
                     self.field.width, self.field.length)
        cr.set_line_join(cairo.LINE_JOIN_ROUND)
        cr.stroke()

        if self.field_png:
            cr.save()
            cr.translate(-0.5 * self.field.width, 0.5 * self.field.length)
            cr.scale(
                self.field.width / self.field_png.get_width(),
                -self.field.length / self.field_png.get_height(),
            )
            cr.set_source_surface(self.field_png)
            cr.paint()
            cr.restore()

        # update everything

        cr.set_line_width(self.pxToM(1))
        if self.mode == Mode.kPlacing or self.mode == Mode.kViewing:
            set_color(cr, palette["BLACK"])
            for i, point in enumerate(self.points.getPoints()):
                draw_px_x(cr, point[0], point[1], self.pxToM(2))
            set_color(cr, palette["WHITE"])
        elif self.mode == Mode.kEditing:
            set_color(cr, palette["BLACK"])
            if self.points.getSplines():
                self.draw_splines(cr)
                for i, points in enumerate(self.points.getSplines()):
                    points = [np.array([x, y]) for (x, y) in points]
                    draw_control_points(cr,
                                        points,
                                        width=self.pxToM(5),
                                        radius=self.pxToM(2))

                    p0, p1, p2, p3, p4, p5 = points
                    first_tangent = p0 + 2.0 * (p1 - p0)
                    second_tangent = p5 + 2.0 * (p4 - p5)
                    cr.set_source_rgb(0, 0.5, 0)
                    cr.move_to(*p0)
                    cr.set_line_width(self.pxToM(1.0))
                    cr.line_to(*first_tangent)
                    cr.move_to(*first_tangent)
                    cr.line_to(*p2)

                    cr.move_to(*p5)
                    cr.line_to(*second_tangent)

                    cr.move_to(*second_tangent)
                    cr.line_to(*p3)

                    cr.stroke()
                    cr.set_line_width(self.pxToM(2))
            set_color(cr, palette["WHITE"])

        cr.paint_with_alpha(0.2)

        draw_px_cross(cr, self.mousex, self.mousey, self.pxToM(2))
        cr.restore()

    def draw_splines(self, cr):
        for i, spline in enumerate(self.points.getLibsplines()):
            for k in np.linspace(0.02, 1, 200):
                cr.move_to(*spline.Point(k - 0.008))
                cr.line_to(*spline.Point(k))
                cr.stroke()
            if i == 0:
                self.draw_robot_at_point(cr, spline, 0)
            self.draw_robot_at_point(cr, spline, 1)

        mouse = np.array((self.mousex, self.mousey))

        # Find the distance along the spline that is closest to the mouse
        result, distance_spline = self.points.nearest_distance(mouse)

        # if the mouse is close enough, draw the robot to show its width
        if result and result.fun < 2:
            self.draw_robot_at_point(cr, distance_spline, result.x)
            self.graph.place_cursor(result.x[0])
        elif self.graph.cursor:
            x = self.graph.find_cursor()
            self.draw_robot_at_point(cr, distance_spline, x)

            # clear the cursor each draw so that it does not persist
            # after you move off the spline
            self.graph.cursor = None

    def export_json(self, file_name):
        self.path_to_export = os.path.join(
            self.module_path,  # position of the python
            "../../..",  # root of the repository
            get_json_folder(self.field),  # path from the root
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
            get_json_folder(self.field),  # path from the root
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
        self.queue_draw()
        self.graph.schedule_recalculate(self.points)

    def attempt_append_multispline(self):
        if (len(self.multispline_stack) == 0
                or self.points.toMultiSpline() != self.multispline_stack[-1]):
            self.multispline_stack.append(self.points.toMultiSpline())

    def clear_graph(self, should_attempt_append=True):
        if should_attempt_append:
            self.attempt_append_multispline()
        self.points = Points()
        #recalulate graph using new points
        self.graph.axis.clear()
        self.graph.queue_draw()
        #allow placing again
        self.mode = Mode.kPlacing
        #redraw entire graph
        self.queue_draw()

    def undo(self):
        try:
            self.multispline_stack.pop()
        except IndexError:
            return
        if len(self.multispline_stack) == 0:
            self.clear_graph(
                should_attempt_append=False)  #clear, don't do anything
            return
        multispline = self.multispline_stack[-1]
        if multispline['spline_count'] > 0:
            self.points.fromMultiSpline(multispline)
            self.mode = Mode.kEditing
        else:
            self.mode = Mode.kPlacing
            self.clear_graph(should_attempt_append=False)
        self.queue_draw()

    def do_key_press_event(self, event):
        keyval = Gdk.keyval_to_lower(event.keyval)
        if keyval == Gdk.KEY_z and event.state & Gdk.ModifierType.CONTROL_MASK:
            self.undo()
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

    def do_button_release_event(self, event):
        self.attempt_append_multispline()
        self.mousex, self.mousey = self.input_transform.transform_point(
            event.x, event.y)
        if self.mode == Mode.kEditing:
            if self.index_of_edit > -1:
                self.points.setSplines(self.spline_edit, self.index_of_edit,
                                       self.mousex, self.mousey)

                self.points.splineExtrapolate(self.spline_edit)

                self.points.update_lib_spline()
                self.graph.schedule_recalculate(self.points)

                self.index_of_edit = -1
                self.spline_edit = -1

    def do_button_press_event(self, event):
        self.mousex, self.mousey = self.input_transform.transform_point(
            event.x, event.y)

        if self.mode == Mode.kPlacing:
            if self.points.add_point(self.mousex, self.mousey):
                self.mode = Mode.kEditing
                self.graph.schedule_recalculate(self.points)
        elif self.mode == Mode.kEditing:
            # Now after index_of_edit is not -1, the point is selected, so
            # user can click for new point
            if self.index_of_edit == -1:
                # Get clicked point
                # Find nearest
                # Move nearest to clicked
                cur_p = [self.mousex, self.mousey]
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

    def do_motion_notify_event(self, event):
        old_x = self.mousex
        old_y = self.mousey
        self.mousex, self.mousey = self.input_transform.transform_point(
            event.x, event.y)
        dif_x = self.mousex - old_x
        dif_y = self.mousey - old_y
        difs = np.array([dif_x, dif_y])

        if self.mode == Mode.kEditing and self.spline_edit != -1:
            self.points.updates_for_mouse_move(self.index_of_edit,
                                               self.spline_edit, self.mousex,
                                               self.mousey, difs)

            self.points.update_lib_spline()
            self.graph.schedule_recalculate(self.points)
        self.queue_draw()

    def do_scroll_event(self, event):

        self.mousex, self.mousey = self.input_transform.transform_point(
            event.x, event.y)

        step_size = self.pxToM(20)  # px

        if event.direction == Gdk.ScrollDirection.UP:
            # zoom out
            scale_by = step_size
        elif event.direction == Gdk.ScrollDirection.DOWN:
            # zoom in
            scale_by = -step_size
        else:
            return

        scale = (self.field.width + scale_by) / self.field.width

        # This restricts the amount it can be scaled.
        if self.zoom_transform.xx <= 0.5:
            scale = max(scale, 1)
        elif self.zoom_transform.xx >= 16:
            scale = min(scale, 1)

        # undo the scaled translation that the old zoom transform did
        x, y = self.invert(self.zoom_transform).transform_point(
            event.x, event.y)

        # move the origin to point
        self.zoom_transform.translate(x, y)

        # scale from new origin
        self.zoom_transform.scale(scale, scale)

        # move back
        self.zoom_transform.translate(-x, -y)

        self.queue_draw()
