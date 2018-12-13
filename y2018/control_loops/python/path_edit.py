#!/usr/bin/python3

from __future__ import print_function
import os
import basic_window
from color import Color, palette
import random
import gi
import numpy as np
import scipy.spatial.distance
gi.require_version('Gtk', '3.0')
from gi.repository import Gdk
import cairo

import enum
import csv # For writing to csv files

from basic_window import OverrideMatrix, identity, quit_main_loop, set_color

LENGTH_OF_FIELD = 323.65
PIXELS_ON_SCREEN = 300

def pxToIn(p):
    return p*LENGTH_OF_FIELD/PIXELS_ON_SCREEN

def inToPx(i):
    return i*PIXELS_ON_SCREEN/LENGTH_OF_FIELD

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
        draw_px_cross(cr, p[i][0], p[i][1], size, Color(0, np.sqrt(0.2 * i), 0))

class Mode(enum.Enum):
    kViewing = 0
    kPlacing = 1
    kEditing = 2
    kExporting = 3
    kImporting = 4

def display_text(cr, text, widtha, heighta, widthb, heightb):
    cr.scale(widtha, -heighta)
    cr.show_text(text)
    cr.scale(widthb, -heightb)

# Create a GTK+ widget on which we will draw using Cairo
class GTK_Widget(basic_window.BaseWindow):
    def __init__(self):
        super(GTK_Widget, self).__init__()

        # init field drawing
        # add default spline for testing purposes
        # init editing / viewing modes and pointer location
        self.mode = Mode.kPlacing
        self.x = 0
        self.y = 0

        self.switch = True

        # update list of control points
        self.point_selected = False
        # self.adding_spline = False
        self.index_of_selected = -1
        self.new_point = []

        # For the editing mode
        self.index_of_edit = -1 # Can't be zero beause array starts at 0
        self.held_x = 0

        # Theo take them from here?
        self.selected_points = []

        self.reinit_extents()

    #John also wrote this
    def add_point(self, x, y):
        if(len(self.selected_points)<4):
            self.selected_points.append([x,y])

    """set extents on images, this needs to be redone with proper distances"""
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

      # Handle the expose-event by updating the Window and drawing
    def handle_draw(self, cr):
        print(self.new_point)
        print("SELF.POINT_SELECTED: " + str(self.point_selected))

        # begin drawing
        # Fill the background color of the window with grey
        set_color(cr, palette["GREY"])
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

        set_color(cr, Color(0.3, 0.3, 0.3))
        cr.rectangle(-150,-150,300,300)
        cr.fill()
        set_color(cr, palette["BLACK"])
        cr.rectangle(-150,-150,300,300)
        cr.set_line_join(cairo.LINE_JOIN_ROUND)
        cr.stroke()
        cr.rectangle(inToPx(140-161.825),inToPx(76.575),inToPx(56),inToPx(-153.15))
        cr.set_line_join(cairo.LINE_JOIN_ROUND)
        cr.stroke()
        cr.rectangle(inToPx(161.825-24),inToPx(90),inToPx(24),inToPx(-180))
        cr.set_line_join(cairo.LINE_JOIN_ROUND)
        cr.stroke()

        set_color(cr, Color(0.2, 0.2, 0.2))
        cr.rectangle(inToPx(140-161.825),inToPx(76.575),inToPx(56),inToPx(-153.15))
        cr.fill()
        cr.rectangle(inToPx(161.825-24),inToPx(90),inToPx(24),inToPx(-180))
        cr.fill()

        # update all the things

        if self.mode == Mode.kViewing:
            set_color(cr, palette["BLACK"])
            cr.move_to(-300, 170)
            cr.show_text("VIEWING")
            set_color(cr, palette["GREY"])
            # its gonna check for points_added from button_press_action
            # The behavior of the click is such that it runs twice
            # This is consistant with graph_edit.py which someone smart wrote
            # So I'm just going to delete the last element in order to not get
            # repeating points
            if len(self.selected_points) > 0:
                print("SELECTED_POINTS: " + str(len(self.selected_points)))
                print("ITEMS:")
                for item in self.selected_points:
                    print(str(item))
                for i, point in enumerate(self.selected_points):
                    print("I: " + str(i))
                    draw_px_x(cr, point[0], point[1], 10)
                    cr.move_to(point[0], point[1]-15)
                    display_text(cr, str(i), 0.5, 0.5, 2, 2)

        if self.mode == Mode.kPlacing:
            set_color(cr, palette["BLACK"])
            cr.move_to(-300, 170)
            display_text(cr, "ADD", 1, 1, 1, 1)
            set_color(cr, palette["GREY"])
            # its gonna check for points_added from button_press_action
            # The behavior of the click is such that it runs twice
            # This is consistant with graph_edit.py which someone smart wrote
            # So I'm just going to delete the last element in order to not get
            # repeating points
            if len(self.selected_points) > 0:
                print("SELECTED_POINTS: " + str(len(self.selected_points)))
                print("ITEMS:")
                for item in self.selected_points:
                    print(str(item))
                for i, point in enumerate(self.selected_points):
                    print("I: " + str(i))
                    draw_px_x(cr, point[0], point[1], 10)
                    cr.move_to(point[0], point[1]-15)
                    display_text(cr, str(i), 0.5, 0.5, 2, 2)
                    if(i==3):
                        self.mode = Mode.kEditing

        elif self.mode == Mode.kEditing:
            set_color(cr, palette["BLACK"])
            cr.move_to(-300, 170)
            display_text(cr, "EDITING", 1, 1, 1, 1)
            set_color(cr, palette["GREY"])
            if len(self.selected_points) > 0:
                print("SELECTED_POINTS: " + str(len(self.selected_points)))
                print("ITEMS:")
                for item in self.selected_points:
                    print(str(item))
                for i, point in enumerate(self.selected_points):
                    print("I: " + str(i))
                    draw_px_x(cr, point[0], point[1], 10)
                    cr.move_to(point[0], point[1]-15)
                    display_text(cr, str(i), 0.5, 0.5, 2, 2)

        elif self.mode == Mode.kExporting:
            set_color(cr, palette["BLACK"])
            cr.move_to(-300, 170)
            display_text(cr, "VIEWING", 1, 1, 1, 1)
            set_color(cr, palette["GREY"])
            #its gonna check for points_added from button_press_action

            # The behavior of the click is such that it runs twice
            # This is consistant with graph_edit.py which someone smart wrote
            # So I'm just going to delete the last element in order to not get
            # repeating points
            if len(self.selected_points) > 0:
                print("SELECTED_POINTS: " + str(len(self.selected_points)))
                print("ITEMS:")
                for item in self.selected_points:
                    print(str(item))
                for i, point in enumerate(self.selected_points):
                    print("I: " + str(i))
                    draw_px_x(cr, point[0], point[1], 10)
                    cr.move_to(point[0], point[1]-15)
                    display_text(cr, str(i), 0.5, 0.5, 2, 2)
        elif self.mode == Mode.kImporting:
            set_color(cr, palette["BLACK"])
            cr.move_to(-300, 170)
            display_text(cr, "VIEWING", 1, 1, 1, 1)
            set_color(cr, palette["GREY"])
            # its gonna check for points_added from button_press_action

            # The behavior of the click is such that it runs twice
            # This is consistant with graph_edit.py which someone smart wrote
            # So I'm just going to delete the last element in order to not get
            # repeating points
            if len(self.selected_points) > 0:
                print("SELECTED_POINTS: " + str(len(self.selected_points)))
                print("ITEMS:")
                for item in self.selected_points:
                    print(str(item))
                for i, point in enumerate(self.selected_points):
                    print("I: " + str(i))
                    draw_px_x(cr, point[0], point[1], 10)
                    cr.move_to(point[0], point[1]-15)
                    display_text(cr, str(i), 0.5, 0.5, 2, 2)


        cr.paint_with_alpha(.65)

        draw_px_cross(cr, self.x, self.y, 10)

    def do_key_press(self, event):
        keyval = Gdk.keyval_to_lower(event.keyval)
        print("Gdk.KEY_" + Gdk.keyval_name(keyval))
        if keyval == Gdk.KEY_q:
            print("Found q key and exiting.")
            quit_main_loop()
        if keyval == Gdk.KEY_e:
            self.mode = Mode.kExporting
            # Will export to csv file
            with open('points_for_pathedit.csv', mode='w') as points_file:
                writer = csv.writer(points_file, delimiter=',', quotechar='"',
                        quoting=csv.QUOTE_MINIMAL)
                for item in self.selected_points:
                    writer.writerow([str(item[0]), str(item[1])])
                    print("Wrote: " + str(item[0]) + " " +  str(item[1]))
        if keyval == Gdk.KEY_i:
            self.mode = Mode.kImporting
            # import from csv file
            self.selected_points = []
            with open('points_for_pathedit.csv') as points_file:
                reader = csv.reader(points_file, delimiter=',')
                for row in reader:
                    self.add_point(float(row[0]), float(row[1]))
                    print("Added: " + row[0] + " " +  row[1])

        self.redraw()

    def button_press_action(self):
        if self.switch:
            self.switch = False
            if self.mode == Mode.kPlacing:
                #Check that the point clicked is on the field
                if(self.x<150 and self.x>-150 and self.y <150 and self.y >-150):
                    self.add_point(self.x, self.y)
            if self.mode == Mode.kEditing:
                # Now after index_of_edit is not -1, the point is selected, so
                # user can click for new point
                    print("INDEX OF EDIT: " + str(self.index_of_edit))

                    if self.index_of_edit > -1 and self.held_x != self.x:
                        print("INDEX OF EDIT: " + str(self.index_of_edit))
                        self.selected_points[self.index_of_edit] = [self.x, self.y]
                        self.index_of_edit = -1
                    else:
                        print("mode == 2")
                        # Get clicked point
                        # Find nearest
                        # Move nearest to clicked
                        cur_p = [self.x, self.y]
                        print("CUR_P: " + str(self.x) + " " + str(self.y))
                        # What I wanna do is get each point
                        # Get the distance between each for x and y
                        # Save the index of the point closest
                        nearest = 1000
                        index = 0
                        for ind, i in enumerate(self.selected_points):
                            # pythagorean
                            distance = np.sqrt((cur_p[0] - i[0])**2 + (cur_p[1] - i[1])**2)
                            if distance < nearest:
                                nearest = distance
                                index = ind
                                print("Nearest: " + str(nearest))
                                print("Index: " + str(index))
                                self.index_of_edit = index
                                self.held_x = self.x
        else:
            self.switch = True

        self.redraw()


    def do_button_press(self, event):
        print("button press activated")
        self.x = event.x
        self.y = event.y
        self.button_press_action()



silly = GTK_Widget()
basic_window.RunApp()
