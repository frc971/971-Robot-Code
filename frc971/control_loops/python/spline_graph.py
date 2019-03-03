#!/usr/bin/python3
import gi
from path_edit import *
import numpy as np
gi.require_version('Gtk', '3.0')
from gi.repository import Gdk, Gtk, GLib
import cairo

class GridWindow(Gtk.Window):
    def method_connect(self, event, cb):
        def handler(obj, *args):
            cb(*args)

        print("Method_connect ran")
        self.connect(event, handler)

    def mouse_move(self, event):
        #Changes event.x and event.y to be relative to the center.
        x = event.x - self.drawing_area.window_shape[0] / 2
        y = self.drawing_area.window_shape[1] / 2 - event.y
        scale = self.drawing_area.get_current_scale()
        event.x = x / scale + self.drawing_area.center[0]
        event.y = y / scale + self.drawing_area.center[1]
        self.drawing_area.mouse_move(event)
        self.queue_draw()

    def button_press(self, event):
        print("button press activated")
        o_x = event.x
        o_y = event.y
        x = event.x - self.drawing_area.window_shape[0] / 2
        y = self.drawing_area.window_shape[1] / 2 - event.y
        scale = 2 * self.drawing_area.get_current_scale()
        event.x = x / scale + self.drawing_area.center[0]
        event.y = y / scale + self.drawing_area.center[1]
        self.drawing_area.do_button_press(event)
        event.x = o_x
        event.y = o_y

    def key_press(self, event):
        print("key press activated")
        self.drawing_area.do_key_press(event, self.file_name_box.get_text())
        self.queue_draw()

    def configure(self, event):
        print("configure activated")
        self.drawing_area.window_shape = (event.width, event.height)

    # handle submitting a constraint
    def on_submit_click(self, widget):
        self.drawing_area.inConstraint = int(self.constraint_box.get_text())
        self.drawing_area.inValue = int(self.value_box.get_text())

    def __init__(self):
        Gtk.Window.__init__(self)

        self.set_default_size(1.5 * SCREEN_SIZE, SCREEN_SIZE)

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

        #add the graph box
        self.drawing_area = GTK_Widget()
        self.eventBox.add(self.drawing_area)

        self.method_connect("key-release-event", self.key_press)
        self.method_connect("button-release-event", self.button_press)
        self.method_connect("configure-event", self.configure)
        self.method_connect("motion_notify_event", self.mouse_move)

        self.file_name_box = Gtk.Entry()
        self.file_name_box.set_size_request(200, 40)

        self.file_name_box.set_text("File")
        self.file_name_box.set_editable(True)

        container.put(self.file_name_box, 0, 0)

        self.show_all()


window = GridWindow()
RunApp()
