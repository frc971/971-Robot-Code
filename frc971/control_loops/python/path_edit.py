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
import copy
from constants import FIELD
from constants import get_json_folder
from constants import ROBOT_SIDE_TO_BALL_CENTER, ROBOT_SIDE_TO_HATCH_PANEL, HATCH_PANEL_WIDTH, BALL_RADIUS
from drawing_constants import set_color, draw_px_cross, draw_px_x, display_text, draw_control_points_cross
from multispline import Multispline, ControlPointIndex
import time
from pathlib import Path


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

        self.multisplines = []
        self.graph = Graph()
        self.graph.cursor_watcher = self
        self.set_vexpand(True)
        self.set_hexpand(True)
        self.undo_history = []
        # init field drawing
        # add default spline for testing purposes
        # init editing / viewing modes and pointer location
        self.mode = Mode.kPlacing
        self.previous_mode = Mode.kPlacing
        self.mousex = 0
        self.lastx = 0
        self.mousey = 0
        self.lasty = 0
        self.drag_start = None
        self.module_path = os.path.dirname(os.path.realpath(sys.argv[0]))
        self.repository_root = Path(self.module_path, "../../..").resolve()
        self.path_to_export = os.path.join(self.module_path,
                                           'points_for_pathedit.json')

        # For the editing mode
        self.control_point_index = None
        self.active_multispline_index = 0

        self.zoom_transform = cairo.Matrix()

        self.set_events(Gdk.EventMask.BUTTON_PRESS_MASK
                        | Gdk.EventMask.BUTTON_PRESS_MASK
                        | Gdk.EventMask.BUTTON_RELEASE_MASK
                        | Gdk.EventMask.POINTER_MOTION_MASK
                        | Gdk.EventMask.SCROLL_MASK)

    @property
    def active_multispline(self):
        """Get the current active multispline or create a new one"""
        if not self.multisplines:
            self.multisplines.append(Multispline())
            self.active_multispline_index = len(self.multisplines) - 1

        return self.multisplines[self.active_multispline_index]

    def set_field(self, field):
        self.field = field
        try:
            if self.field.field_id.startswith('//'):
                self.field_png = cairo.ImageSurface.create_from_png(
                    self.field.field_id[2:])
            else:
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
            for multispline in self.multisplines:
                for i, point in enumerate(multispline.staged_points):
                    draw_px_x(cr, point[0], point[1], self.pxToM(2))
            if len(self.multisplines) != 0 and self.multisplines[0].getSplines(
            ):  #still in testing
                self.draw_cursor(cr)
                self.draw_splines(cr)
        elif self.mode == Mode.kEditing:
            if len(self.multisplines) != 0 and self.multisplines[0].getSplines(
            ):
                self.draw_cursor(cr)
                self.draw_splines(cr)
                self.draw_control_points(cr)

        set_color(cr, palette["WHITE"])
        cr.paint_with_alpha(0.2)

        draw_px_cross(cr, self.mousex, self.mousey, self.pxToM(2))
        cr.restore()

    def draw_control_points(self, cr):
        for i, points in enumerate(self.active_multispline.getSplines()):
            points = [np.array([x, y]) for (x, y) in points]
            draw_control_points_cross(cr,
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

    def draw_cursor(self, cr):
        mouse = np.array((self.mousex, self.mousey))

        multispline, result = Multispline.nearest_distance(
            self.multisplines, mouse)

        if self.graph.cursor is not None:
            cursor = self.graph.find_cursor()
            if cursor is None:
                return
            multispline_index, x = cursor
            distance_spline = DistanceSpline(
                self.multisplines[multispline_index].getLibsplines())

            self.draw_robot_at_point(cr, distance_spline, x)

            # clear the cursor each draw so it doesn't persist
            # after you move off the spline
            self.graph.cursor = None
        elif result is not None and result.fun < 2:
            distance_spline = DistanceSpline(multispline.getLibsplines())
            x = result.x[0]

            # draw the robot to show its width
            self.draw_robot_at_point(cr, distance_spline, x)

            multispline_index = self.multisplines.index(multispline)
            self.graph.place_cursor(multispline_index, distance=result.x[0])

    def draw_splines(self, cr):
        for multispline in self.multisplines:
            for i, spline in enumerate(multispline.getLibsplines()):
                alpha = 1 if multispline == self.active_multispline else 0.2
                set_color(cr, palette["BLACK"], alpha)

                # draw lots of really small line segments to
                # approximate the shape of the spline
                for k in np.linspace(0.005, 1, 200):
                    cr.move_to(*spline.Point(k - 0.008))
                    cr.line_to(*spline.Point(k))
                    cr.stroke()

                if i == 0:
                    self.draw_robot_at_point(cr, spline, 0)

                is_last_spline = spline is multispline.getLibsplines()[-1]

                if multispline == self.active_multispline or is_last_spline:
                    self.draw_robot_at_point(cr, spline, 1)

    def export_json(self, file_name):
        export_folder = Path(
            self.repository_root,
            get_json_folder(self.field),  # path from the root
        )

        filename = Path(export_folder, file_name)

        # strip suffix
        filename = filename.with_suffix("")
        print(file_name, filename)

        print(f"Exporting {len(self.multisplines)} splines")
        # Export each multispline to its own json file
        for index, multispline in enumerate(self.multisplines):
            file = filename.with_suffix(f".{index}.json")
            print(f"  {file.relative_to(export_folder)}")
            with open(file, mode='w') as points_file:
                json.dump(multispline.toJsonObject(), points_file)

    def import_json(self, file_name):
        # Abort place mode
        if self.mode is Mode.kPlacing and len(self.multisplines) > 0 and len(
                self.multisplines[-1].getSplines()) == 0:
            self.multisplines.pop()
            self.mode = Mode.kEditing
            self.queue_draw()

        import_folder = Path(
            self.repository_root,
            get_json_folder(self.field),  # path from the root
        )

        file_candidates = []

        # try exact match first
        filename = Path(import_folder, file_name)
        if filename.exists():
            file_candidates.append(filename)
        else:
            # look for other files with the same stem but different numbers
            stripped_stem = Path(file_name).with_suffix('').stem
            file_candidates = list(
                import_folder.glob(f"{stripped_stem}.*.json"))
            print([file.stem for file in file_candidates])
            file_candidates.sort()

        print(f"Found {len(file_candidates)} files")
        for file in file_candidates:
            print(f"  {file.relative_to(import_folder)}")

            with open(file) as points_file:
                self.multisplines.append(
                    Multispline.fromJsonObject(json.load(points_file)))

        self.attempt_append_multisplines()

        print("SPLINES LOADED")
        self.mode = Mode.kEditing
        self.queue_draw()
        self.graph.schedule_recalculate(self.multisplines)

    def attempt_append_multisplines(self):
        if len(self.undo_history
               ) == 0 or self.multisplines != self.undo_history[-1]:
            self.undo_history.append(copy.deepcopy(self.multisplines))

    def clear(self, should_attempt_append=True):
        if should_attempt_append:
            self.attempt_append_multisplines()
        self.multisplines = []
        self.active_multispline_index = 0
        self.control_point_index = None
        #recalulate graph using new points
        self.graph.axis.clear()
        self.graph.canvas.draw_idle()
        #go back into viewing mode
        self.mode = Mode.kViewing
        #redraw entire graph
        self.queue_draw()

    def undo(self):
        try:
            self.undo_history.pop()
        except IndexError:
            return
        if len(self.undo_history) == 0:
            self.clear(should_attempt_append=False)  #clear, don't do anything
            return
        if len(self.multisplines) > 0 and not any(
                multispline.staged_points
                for multispline in self.multisplines):
            self.mode = Mode.kEditing
        else:
            self.mode = Mode.kPlacing
            self.clear(should_attempt_append=False)
        self.multisplines = copy.deepcopy(self.undo_history[-1])
        self.queue_draw()

    def do_key_press_event(self, event):
        keyval = Gdk.keyval_to_lower(event.keyval)
        if keyval == Gdk.KEY_z and event.state & Gdk.ModifierType.CONTROL_MASK:
            self.undo()

        if keyval == Gdk.KEY_p:
            self.new_spline()
        elif keyval == Gdk.KEY_m:
            self.new_multispline()

    def new_spline(self):
        self.mode = Mode.kPlacing
        # F0 = A1
        # B1 = 2F0 - E0
        # C1= d0 + 4F0 - 4E0
        multispline = self.active_multispline
        if len(multispline.getSplines()) != 0:
            multispline.extrapolate(multispline.getSplines()[-1])
        self.queue_draw()

    def new_multispline(self):
        if len(self.active_multispline.getSplines()) != 0:
            self.mode = Mode.kPlacing
            self.active_multispline_index += 1
            self.multisplines.insert(self.active_multispline_index,
                                     Multispline())

            prev_multispline = self.multisplines[self.active_multispline_index
                                                 - 1]
            if len(prev_multispline.getSplines()) != 0:
                self.active_multispline.extrapolate(
                    prev_multispline.getSplines()[-1])
            self.queue_draw()

    def on_graph_clicked(self):
        if self.graph.cursor is not None:
            cursor = self.graph.find_cursor()
            if cursor is None:
                return
            multispline_index, x = cursor

            self.active_multispline_index = multispline_index

    def do_button_release_event(self, event):
        self.drag_start = None

        self.attempt_append_multisplines()
        self.mousex, self.mousey = self.input_transform.transform_point(
            event.x, event.y)
        if self.mode == Mode.kEditing:
            if self.control_point_index != None:
                multispline = self.multisplines[
                    self.control_point_index.multispline_index]

                multispline.setControlPoint(self.control_point_index,
                                            self.mousex, self.mousey)

                Multispline.splineExtrapolate(self.multisplines,
                                              self.control_point_index)

                multispline.update_lib_spline()
                self.graph.schedule_recalculate(self.multisplines)
                self.queue_draw()

                self.control_point_index = None

    def do_button_press_event(self, event):
        self.mousex, self.mousey = self.input_transform.transform_point(
            event.x, event.y)

        self.lastx = event.x
        self.lasty = event.y

        if self.mode == Mode.kPlacing:
            if self.active_multispline.addPoint(self.mousex, self.mousey):
                self.mode = Mode.kEditing
                self.graph.schedule_recalculate(self.multisplines)
        elif self.mode == Mode.kEditing:
            # Now after we have no control point index,
            # the user can click for new point
            if self.control_point_index == None:
                # Get clicked point
                # Find nearest
                # Move nearest to clicked
                cur_p = [self.mousex, self.mousey]

                # Get the distance between each for x and y
                # Save the index of the point closest
                nearest = 0.4  # Max distance away a the selected point can be in meters
                index_of_closest = 0
                index_multisplines = self.active_multispline_index
                multispline = self.active_multispline

                for index_splines, points in enumerate(
                        multispline.getSplines()):
                    for index_points, val in enumerate(points):
                        distance = np.sqrt((cur_p[0] - val[0])**2 +
                                           (cur_p[1] - val[1])**2)
                        if distance < nearest:
                            nearest = distance
                            index_of_closest = index_points
                            self.control_point_index = ControlPointIndex(
                                index_multisplines, index_splines,
                                index_points)

                if self.control_point_index == None:
                    self.drag_start = (event.x, event.y)

            multispline, result = Multispline.nearest_distance(
                self.multisplines, cur_p)
            if self.control_point_index == None and result and result.fun < 0.1:
                self.active_multispline_index = self.multisplines.index(
                    multispline)

        elif self.mode == Mode.kViewing:

            if self.control_point_index == None:
                self.drag_start = (event.x, event.y)

        self.queue_draw()

    def do_motion_notify_event(self, event):
        self.mousex, self.mousey = self.input_transform.transform_point(
            event.x, event.y)
        mouse = np.array([self.mousex, self.mousey])

        if self.mode == Mode.kEditing and self.control_point_index != None:
            multispline = self.multisplines[
                self.control_point_index.multispline_index]

            multispline.updates_for_mouse_move(self.multisplines,
                                               self.control_point_index, mouse)

            multispline.update_lib_spline()
            self.graph.schedule_recalculate(self.multisplines)

        if self.drag_start != None and self.control_point_index == None:
            if self.mode == Mode.kEditing or self.mode == Mode.kViewing:

                self.zoom_transform.translate(event.x - self.lastx,
                                              event.y - self.lasty)
                self.lastx = event.x
                self.lasty = event.y

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
        if self.zoom_transform.xx <= 0.05:
            scale = max(scale, 1)
        elif self.zoom_transform.xx >= 32:
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
