#!/usr/bin/python3
from __future__ import print_function
import os
import copy
import basic_window
from color import Color, palette
import random
import gi
import numpy as np
from libspline import Spline
import scipy.spatial.distance
gi.require_version('Gtk', '3.0')
from gi.repository import Gdk, Gtk, GLib
import cairo
import enum
import csv  # For writing to csv files

from basic_window import OverrideMatrix, identity, quit_main_loop, set_color

LENGTH_OF_FIELD = 323.65
PIXELS_ON_SCREEN = 300


def pxToIn(p):
    return p * LENGTH_OF_FIELD / PIXELS_ON_SCREEN


def inToPx(i):
    return (i * PIXELS_ON_SCREEN / LENGTH_OF_FIELD)


def px(cr):
    return OverrideMatrix(cr, identity)


def draw_px_cross(cr, x, y, length_px, color=palette["RED"]):
    """Draws a cross with fixed dimensions in pixel space."""
    set_color(cr, color)
    cr.move_to(x, y - length_px)
    cr.line_to(x, y + length_px)
    cr.stroke()

    cr.move_to(x - length_px, y)
    cr.line_to(x + length_px, y)
    cr.stroke()
    set_color(cr, palette["LIGHT_GREY"])


def draw_px_x(cr, x, y, length_px1, color=palette["BLACK"]):
    """Draws a x with fixed dimensions in pixel space."""
    length_px = length_px1 / np.sqrt(2)
    set_color(cr, color)
    cr.move_to(x - length_px, y - length_px)
    cr.line_to(x + length_px, y + length_px)
    cr.stroke()

    cr.move_to(x - length_px, y + length_px)
    cr.line_to(x + length_px, y - length_px)
    cr.stroke()
    set_color(cr, palette["LIGHT_GREY"])


def draw_points(cr, p, size):
    for i in range(0, len(p)):
        draw_px_cross(cr, p[i][0], p[i][1], size, Color(
            0, np.sqrt(0.2 * i), 0))


class Mode(enum.Enum):
    kViewing = 0
    kPlacing = 1
    kEditing = 2
    kExporting = 3
    kImporting = 4
    kConstraint = 5


class ConstraintType(enum.Enum):
    kMaxSpeed = 0
    kMaxAcceleration = 1


def display_text(cr, text, widtha, heighta, widthb, heightb):
    cr.scale(widtha, -heighta)
    cr.show_text(text)
    cr.scale(widthb, -heightb)


def redraw(needs_redraw, window):
    print("Redrew")
    if not needs_redraw:
        window.queue_draw()


class Constraint():
    def __init__(self, start, end, constraint, value):
        self.start = start  #Array with index and distance from start of spline
        self.end = end  #Array with index and distance from start of spline
        self.constraint = constraint  #INT
        self.value = value  #INT
        if self.constraint == 0:
            self.conName = "kMaxSpeed"
        else:
            self.conName = "kMaxAcceleration"

    def toString(self):

        return "START: " + str(self.start[0]) + ",   " + str(
            self.start[1]) + " |  END: " + str(self.end[0]) + ",   " + str(
                self.end[1]) + " |  " + str(self.conName) + ":  " + str(
                    self.value)


class GTK_Widget(basic_window.BaseWindow):
    """Create a GTK+ widget on which we will draw using Cairo"""

    def __init__(self):
        super(GTK_Widget, self).__init__()

        # init field drawing
        # add default spline for testing purposes
        # init editing / viewing modes and pointer location
        self.mode = Mode.kPlacing
        self.x = 0
        self.y = 0

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

        self.selected_points = []
        self.splines = []
        self.reinit_extents()

        self.inStart = None
        self.inEnd = None
        self.inConstraint = None
        self.inValue = None
        self.startSet = False

    #John also wrote this
    def add_point(self, x, y):
        if (len(self.selected_points) < 6):
            self.selected_points.append([x, y])
            if (len(self.selected_points) == 6):
                self.mode = Mode.kEditing
                self.splines.append(np.array(self.selected_points))
                self.selected_points = []

    """set extents on images"""

    def reinit_extents(self):
        self.extents_x_min = -800
        self.extents_x_max = 800
        self.extents_y_min = -800
        self.extents_y_max = 800

    # this needs to be rewritten with numpy, i dont think this ought to have
    # SciPy as a dependecy
    def get_index_of_nearest_point(self):
        cur_p = [[self.x, self.y]]
        distances = scipy.spatial.distance.cdist(cur_p, self.all_controls)

        return np.argmin(distances)

    # return the closest point to the loc of the click event
    def get_nearest_point(self):
        return self.all_controls[self.get_index_of_nearest_point()]

    def set_index_to_nearest_spline_point(self):
        nearest = 50
        index_of_closest = 0
        self.spline_edit = 0
        cur_p = [self.x, self.y]

        for index_splines, points in enumerate(self.spline):
            for index_points, i in enumerate(points.curve):
                # pythagorean
                distance = np.sqrt((cur_p[0] - i[0])**2 + (cur_p[1] - i[1])**2)
                if distance < nearest:
                    nearest = distance
                    print("DISTANCE: ", distance, " | INDEX: ", index_points)
                    index_of_closest = index_points
                    self.index_of_edit = index_of_closest
                    self.spline_edit = index_splines
                    self.held_x = self.x
        if self.startSet == False:
            self.inStart = [self.index_of_edit, self.findDistance()]
            self.startSet = True
        else:
            self.inEnd = [self.index_of_edit, self.findDistance()]
            self.startSet = False
            self.mode = Mode.kEditing
            self.spline_edit = -1
            self.index_of_edit = -1

        print("Nearest: " + str(nearest))
        print("Spline: " + str(self.spline_edit))
        print("Index: " + str(index_of_closest))

    def findDistance(self):
        """ findDistance goes through each point on the spline finding distance to that point from the point before.
        It does this to find the the length of the spline to the point that is currently selected.
        """
        distance = 0
        points = self.curves[self.spline_edit]
        for index, point in enumerate(points):
            if index > 0 and index <= self.index_of_edit:
                distance += np.sqrt((points[index - 1][0] - point[0])**2 +
                                    (points[index - 1][1] - point[1])**2)
        return pxToIn(distance)

    # Handle the expose-event by updating the Window and drawing
    def handle_draw(self, cr):
        # print(self.new_point)
        # print("SELF.POINT_SELECTED: " + str(self.point_selected))

        # begin drawing
        # Fill the background color of the window with grey
        set_color(cr, palette["GREY"])
        cr.paint()
        #Scale the field to fit within drawing area
        cr.scale(0.5, 0.5)

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

        set_color(cr, Color(0.3, 0.3, 0.3))
        cr.rectangle(-450, -150, 300, 300)
        cr.fill()
        set_color(cr, palette["BLACK"])
        cr.rectangle(-450, -150, 300, 300)
        cr.set_line_join(cairo.LINE_JOIN_ROUND)
        cr.stroke()
        cr.rectangle((inToPx(140 - 161.825) - 300), inToPx(76.575), inToPx(56),
                     inToPx(-153.15))
        cr.set_line_join(cairo.LINE_JOIN_ROUND)
        cr.stroke()
        cr.rectangle((inToPx(161.825 - 24) - 300), inToPx(90), inToPx(24),
                     inToPx(-180))
        cr.set_line_join(cairo.LINE_JOIN_ROUND)
        cr.stroke()

        set_color(cr, Color(0.2, 0.2, 0.2))
        cr.rectangle(
            inToPx(140 - 161.825) - 300, inToPx(76.575), inToPx(56),
            inToPx(-153.15))
        cr.fill()
        cr.rectangle(
            inToPx(161.825 - 24) - 300, inToPx(90), inToPx(24), inToPx(-180))
        cr.fill()

        y = 0

        # update all the things

        if self.mode == Mode.kViewing:
            set_color(cr, palette["BLACK"])
            cr.move_to(-300, 170)
            cr.show_text("VIEWING")
            set_color(cr, palette["GREY"])

            if len(self.selected_points) > 0:
                print("SELECTED_POINTS: " + str(len(self.selected_points)))
                print("ITEMS:")
                # for item in self.selected_points:
                # print(str(item))
                for i, point in enumerate(self.selected_points):
                    # print("I: " + str(i))
                    draw_px_x(cr, point[0], point[1], 10)
                    cr.move_to(point[0], point[1] - 15)
                    display_text(cr, str(i), 0.5, 0.5, 2, 2)

        elif self.mode == Mode.kPlacing:
            set_color(cr, palette["BLACK"])
            cr.move_to(-300, 170)
            display_text(cr, "ADD", 1, 1, 1, 1)
            set_color(cr, palette["GREY"])

            if len(self.selected_points) > 0:
                print("SELECTED_POINTS: " + str(len(self.selected_points)))
                print("ITEMS:")
                for item in self.selected_points:
                    print(str(item))
                for i, point in enumerate(self.selected_points):
                    print("I: " + str(i))
                    draw_px_x(cr, point[0], point[1], 10)
                    cr.move_to(point[0], point[1] - 15)
                    display_text(cr, str(i), 0.5, 0.5, 2, 2)

        elif self.mode == Mode.kEditing:
            set_color(cr, palette["BLACK"])
            cr.move_to(-300, 170)
            display_text(cr, "EDITING", 1, 1, 1, 1)
            if len(self.splines) > 0:
                # print("Splines: " + str(len(self.splines)))
                # print("ITEMS:")
                holder_spline = []
                for i, points in enumerate(self.splines):
                    array = np.zeros(shape=(6, 2), dtype=float)
                    for j, point in enumerate(points):
                        array[j, 0] = point[0]
                        array[j, 1] = point[1]
                    spline = Spline(np.ascontiguousarray(np.transpose(array)))
                    for k in np.linspace(0.01, 1, 100):

                        cr.move_to(
                            spline.Point(k - 0.01)[0],
                            spline.Point(k - 0.01)[1])
                        cr.line_to(spline.Point(k)[0], spline.Point(k)[1])
                        cr.stroke()
                        holding = [
                            spline.Point(k - 0.01)[0],
                            spline.Point(k - 0.01)[1]
                        ]

                        holder_spline.append(holding)
                self.curves.append(holder_spline)

                for spline, points in enumerate(self.splines):
                    # for item in points:
                    #     print(str(item))
                    for i, point in enumerate(points):
                        # print("I: " + str(i))
                        if spline == self.spline_edit and i == self.index_of_edit:
                            draw_px_x(cr, point[0], point[1], 15,
                                      self.colors[spline])
                        elif (spline == 0 and not i == 5) or (not i == 0
                                                              and not i == 5):
                            draw_px_x(cr, point[0], point[1], 10,
                                      self.colors[spline])
                        cr.move_to(point[0], point[1] - 15)
                        display_text(cr, str(i), 0.5, 0.5, 2, 2)

        elif self.mode == Mode.kExporting:
            set_color(cr, palette["BLACK"])
            cr.move_to(-300, 170)
            display_text(cr, "VIEWING", 1, 1, 1, 1)
            set_color(cr, palette["GREY"])

            if len(self.selected_points) > 0:
                print("SELECTED_POINTS: " + str(len(self.selected_points)))
                print("ITEMS:")
                # for item in self.selected_points:
                #     print(str(item))
                for i, point in enumerate(self.selected_points):
                    # print("I: " + str(i))
                    draw_px_x(cr, point[0], point[1], 10)
                    cr.move_to(point[0], point[1] - 15)
                    display_text(cr, str(i), 0.5, 0.5, 2, 2)

        elif self.mode == Mode.kImporting:
            set_color(cr, palette["BLACK"])
            cr.move_to(-300, 170)
            display_text(cr, "VIEWING", 1, 1, 1, 1)
            set_color(cr, palette["GREY"])

            if len(self.selected_points) > 0:
                print("SELECTED_POINTS: " + str(len(self.selected_points)))
                print("ITEMS:")
                for item in self.selected_points:
                    print(str(item))
                for i, point in enumerate(self.selected_points):
                    print("I: " + str(i))
                    draw_px_x(cr, point[0], point[1], 10)
                    cr.move_to(point[0], point[1] - 15)
                    display_text(cr, str(i), 0.5, 0.5, 2, 2)

        elif self.mode == Mode.kConstraint:
            print("Drawn")
            set_color(cr, palette["BLACK"])
            cr.move_to(-300, 170)
            display_text(cr, "Adding Constraint", 1, 1, 1, 1)
            if len(self.splines) > 0:
                # print("Splines: " + str(len(self.splines)))
                # print("ITEMS:")
                for s, points in enumerate(self.splines):
                    # for item in points:
                    #     print(str(item))
                    for i, point in enumerate(points):
                        # print("I: " + str(i))
                        draw_px_x(cr, point[0], point[1], 10, self.colors[s])
                        cr.move_to(point[0], point[1] - 15)
                        display_text(cr, str(i), 0.5, 0.5, 2, 2)

        cr.paint_with_alpha(.65)

        draw_px_cross(cr, self.x, self.y, 10)

    def do_key_press(self, event):
        keyval = Gdk.keyval_to_lower(event.keyval)
        # print("Gdk.KEY_" + Gdk.keyval_name(keyval))
        if keyval == Gdk.KEY_q:
            print("Found q key and exiting.")
            quit_main_loop()
        if keyval == Gdk.KEY_e:
            self.mode = Mode.kExporting
            # Will export to csv file
            with open('points_for_pathedit.csv', mode='w') as points_file:
                writer = csv.writer(
                    points_file,
                    delimiter=',',
                    quotechar='"',
                    quoting=csv.QUOTE_MINIMAL)
                for item in self.selected_points:
                    writer.writerow([str(item[0]), str(item[1])])
                    print("Wrote: " + str(item[0]) + " " + str(item[1]))
        if keyval == Gdk.KEY_i:
            self.mode = Mode.kImporting
            # import from csv file
            self.selected_points = []
            with open('points_for_pathedit.csv') as points_file:
                reader = csv.reader(points_file, delimiter=',')
                for row in reader:
                    self.add_point(float(row[0]), float(row[1]))
                    print("Added: " + row[0] + " " + row[1])
        if keyval == Gdk.KEY_p:
            self.mode = Mode.kPlacing
            # F0 = A1
            # B1 = 2F0 - E0
            # C1= d0 + 4F0 - 4E0
            spline_index = len(self.splines) - 1
            self.selected_points = []
            f = self.splines[spline_index][5]
            e = self.splines[spline_index][4]
            d = self.splines[spline_index][3]
            self.selected_points.append(f)
            self.selected_points.append(f * 2 + e * -1)
            self.selected_points.append(d + f * 4 + e * -4)

        if keyval == Gdk.KEY_c:
            self.mode = Mode.kConstraint

    def button_press_action(self):
        if self.mode == Mode.kPlacing:
            #Check that the point clicked is on the field
            if (self.x < -150 and self.x > -450 and self.y < 150
                    and self.y > -150):
                self.add_point(self.x, self.y)
        elif self.mode == Mode.kEditing:
            # Now after index_of_edit is not -1, the point is selected, so
            # user can click for new point
            if self.index_of_edit > -1 and self.held_x != self.x:
                print("INDEX OF EDIT: " + str(self.index_of_edit))
                self.splines[self.spline_edit][self.index_of_edit] = [
                    self.x, self.y
                ]

                if not self.spline_edit == len(self.splines) - 1:
                    spline_edit = self.spline_edit + 1
                    f = self.splines[self.spline_edit][5]
                    e = self.splines[self.spline_edit][4]
                    d = self.splines[self.spline_edit][3]
                    self.splines[spline_edit][0] = f
                    self.splines[spline_edit][1] = f * 2 + e * -1
                    self.splines[spline_edit][2] = d + f * 4 + e * -4

                if not self.spline_edit == 0:
                    spline_edit = self.spline_edit - 1
                    a = self.splines[self.spline_edit][0]
                    b = self.splines[self.spline_edit][1]
                    c = self.splines[self.spline_edit][2]
                    self.splines[spline_edit][5] = a
                    self.splines[spline_edit][4] = a * 2 + b * -1
                    self.splines[spline_edit][3] = c + a * 4 + b * -4

                self.index_of_edit = -1
                self.spline_edit = -1
            else:
                print("mode == 2")
                # Get clicked point
                # Find nearest
                # Move nearest to clicked
                cur_p = [self.x, self.y]
                print("CUR_P: " + str(self.x) + " " + str(self.y))
                # Get the distance between each for x and y
                # Save the index of the point closest
                nearest = 50
                index_of_closest = 0
                for index_splines, points in enumerate(self.splines):
                    for index_points, val in enumerate(points):
                        # pythagorean
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
        elif self.mode == Mode.kConstraint:
            print("RAN")
            self.set_index_to_nearest_spline_point()
            print("FINISHED")

    def do_button_press(self, event):
        print("button press activated")
        #Be consistent with the scaling in the drawing_area
        self.x = event.x * 2
        self.y = event.y * 2
        self.button_press_action()


class GridWindow(Gtk.Window):
    def method_connect(self, event, cb):
        def handler(obj, *args):
            cb(*args)

        print("Method_connect ran")
        self.connect(event, handler)

    def button_press(self, event):
        print("button press activated")
        o_x = event.x
        o_y = event.y
        x = event.x - self.drawing_area.window_shape[0] / 2
        y = self.drawing_area.window_shape[1] / 2 - event.y
        scale = self.drawing_area.get_current_scale()
        event.x = x / scale + self.drawing_area.center[0]
        event.y = y / scale + self.drawing_area.center[1]
        self.drawing_area.do_button_press(event)
        event.x = o_x
        event.y = o_y

    def key_press(self, event):
        print("key press activated")
        self.drawing_area.do_key_press(event)
        self.queue_draw()

    def configure(self, event):
        print("configure activated")
        self.drawing_area.window_shape = (event.width, event.height)

    def on_submit_click(self, widget):
        self.drawing_area.inConstraint = int(self.constraint_box.get_text())
        self.drawing_area.inValue = int(self.value_box.get_text())

    def __init__(self):
        Gtk.Window.__init__(self)

        self.set_default_size(1366, 738)

        flowBox = Gtk.FlowBox()
        flowBox.set_valign(Gtk.Align.START)
        flowBox.set_selection_mode(Gtk.SelectionMode.NONE)

        flowBox.set_valign(Gtk.Align.START)

        self.add(flowBox)

        container = Gtk.Fixed()
        flowBox.add(container)

        self.eventBox = Gtk.EventBox()
        container.add(self.eventBox)

        self.eventBox.set_events(Gdk.EventMask.BUTTON_PRESS_MASK
                                 | Gdk.EventMask.BUTTON_RELEASE_MASK
                                 | Gdk.EventMask.POINTER_MOTION_MASK
                                 | Gdk.EventMask.SCROLL_MASK
                                 | Gdk.EventMask.KEY_PRESS_MASK)

        self.drawing_area = GTK_Widget()
        self.eventBox.add(self.drawing_area)

        self.method_connect("key-release-event", self.key_press)
        self.method_connect("button-release-event", self.button_press)
        self.method_connect("configure-event", self.configure)

        # Constraint Boxes

        self.start_box = Gtk.Entry()
        self.start_box.set_size_request(100, 20)

        self.constraint_box = Gtk.Entry()
        self.constraint_box.set_size_request(100, 20)

        self.constraint_box.set_text("Constraint")
        self.constraint_box.set_editable(True)

        container.put(self.constraint_box, 700, 0)

        self.value_box = Gtk.Entry()
        self.value_box.set_size_request(100, 20)

        self.value_box.set_text("Value")
        self.value_box.set_editable(True)

        container.put(self.value_box, 700, 40)

        self.submit_button = Gtk.Button("Submit")
        self.submit_button.connect('clicked', self.on_submit_click)

        container.put(self.submit_button, 880, 0)

        self.show_all()


window = GridWindow()
basic_window.RunApp()