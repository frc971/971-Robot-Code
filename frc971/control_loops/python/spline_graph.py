#!/usr/bin/python3
import gi
from path_edit import *
import numpy as np
gi.require_version('Gtk', '3.0')
from gi.repository import Gdk, Gtk, GLib
import cairo
import basic_window


class GridWindow(Gtk.Window):
    def method_connect(self, event, cb):
        def handler(obj, *args):
            cb(*args)

        self.connect(event, handler)

    def mouse_move(self, event):
        # Changes event.x and event.y to be relative to the center.
        x = event.x - self.drawing_area.window_shape[0] / 2
        y = self.drawing_area.window_shape[1] / 2 - event.y
        scale = self.drawing_area.get_current_scale()
        event.x = x / scale + self.drawing_area.center[0]
        event.y = y / scale + self.drawing_area.center[1]
        self.drawing_area.mouse_move(event)
        self.queue_draw()

    def button_press(self, event):
        original_x = event.x
        original_y = event.y
        x = event.x - self.drawing_area.window_shape[0] / 2
        y = self.drawing_area.window_shape[1] / 2 - event.y
        scale = 2 * self.drawing_area.get_current_scale()
        event.x = x / scale + self.drawing_area.center[0]
        event.y = y / scale + self.drawing_area.center[1]
        self.drawing_area.do_button_press(event)
        event.x = original_x
        event.y = original_y

    def key_press(self, event):
        self.drawing_area.do_key_press(event, self.file_name_box.get_text())
        self.queue_draw()

    def configure(self, event):
        self.drawing_area.window_shape = (event.width, event.height)

    def output_json_clicked(self, button):
        print("OUTPUT JSON CLICKED")
        self.drawing_area.export_json(self.file_name_box.get_text())

    def input_json_clicked(self, button):
        print("INPUT JSON CLICKED")
        self.drawing_area.import_json(self.file_name_box.get_text())
        self.long_input.set_value(
            self.drawing_area.points.getConstraint(
                "LONGITUDINAL_ACCELERATION"))
        self.lat_input.set_value(
            self.drawing_area.points.getConstraint("LATERAL_ACCELERATION"))
        self.vol_input.set_value(
            self.drawing_area.points.getConstraint("VOLTAGE"))

    def long_changed(self, button):
        value = self.long_input.get_value()
        self.drawing_area.points.setConstraint("LONGITUDINAL_ACCELERATION",
                                               value)

    def lat_changed(self, button):
        value = self.lat_input.get_value()
        self.drawing_area.points.setConstraint("LATERAL_ACCELERATION", value)

    def vel_changed(self, button):
        value = self.vel_input.get_value()

    def vol_changed(self, button):
        value = self.vol_input.get_value()
        self.drawing_area.points.setConstraint("VOLTAGE", value)

    def input_combobox_choice(self, combo):
        text = combo.get_active_text()
        if text is not None:
            print("Combo Clicked on: " + text)
            set_field(text)

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

        self.method_connect("delete-event", basic_window.quit_main_loop)
        self.method_connect("key-release-event", self.key_press)
        self.method_connect("button-release-event", self.button_press)
        self.method_connect("configure-event", self.configure)
        self.method_connect("motion_notify_event", self.mouse_move)

        self.file_name_box = Gtk.Entry()
        self.file_name_box.set_size_request(200, 40)

        self.file_name_box.set_text(FIELD.json_name)
        self.file_name_box.set_editable(True)

        container.put(self.file_name_box, 0, 0)

        self.long_input = Gtk.SpinButton()
        self.long_input.set_size_request(100, 20)
        self.long_input.set_numeric(True)
        self.long_input.set_range(0, 5)
        self.long_input.set_digits(3)
        self.long_input.set_increments(0.1, 100)
        self.long_input.connect("value-changed", self.long_changed)
        self.long_label = Gtk.Label()
        self.long_label.set_text("Longitudinal Acceleration Restriction")
        self.long_input.set_value(
            self.drawing_area.points.getConstraint(
                "LONGITUDINAL_ACCELERATION"))

        self.lat_input = Gtk.SpinButton()
        self.lat_input.set_size_request(100, 20)
        self.lat_input.set_numeric(True)
        self.lat_input.set_range(0, 5)
        self.lat_input.set_digits(3)
        self.lat_input.set_increments(0.1, 100)
        self.lat_input.connect("value-changed", self.lat_changed)
        self.lat_label = Gtk.Label()
        self.lat_label.set_text("Lateral Acceleration Restriction")
        self.lat_input.set_value(
            self.drawing_area.points.getConstraint("LATERAL_ACCELERATION"))

        self.vel_input = Gtk.SpinButton()
        self.vel_input.set_size_request(100, 20)
        self.vel_input.set_numeric(True)
        self.vel_input.set_range(0, 10)
        self.vel_input.set_digits(3)
        self.vel_input.set_increments(0.1, 100)
        self.vel_input.connect("value-changed", self.vel_changed)
        self.vel_label = Gtk.Label()
        self.vel_label.set_text(
            "Velocity Restriction"
        )  #note: the velocity restrictions are not yet working, they need to be hooked up later

        self.vol_input = Gtk.SpinButton()
        self.vol_input.set_size_request(100, 20)
        self.vol_input.set_numeric(True)
        self.vol_input.set_range(0, 12)
        self.vol_input.set_digits(3)
        self.vol_input.set_increments(0.1, 100)
        self.vol_input.connect("value-changed", self.vol_changed)
        self.vol_label = Gtk.Label()
        self.vol_label.set_text("Voltage Restriction")
        self.vol_input.set_value(
            self.drawing_area.points.getConstraint("VOLTAGE"))

        container.put(self.long_input, 0, 60)
        container.put(self.lat_input, 0, 110)
        container.put(self.vel_input, 0, 160)
        container.put(self.vol_input, 0, 210)
        container.put(self.long_label, 0, 40)
        container.put(self.lat_label, 0, 90)
        container.put(self.vel_label, 0, 140)
        container.put(self.vol_label, 0, 190)

        self.output_json = Gtk.Button.new_with_label("Output")
        self.output_json.set_size_request(100, 40)
        self.output_json.connect("clicked", self.output_json_clicked)

        self.input_json = Gtk.Button.new_with_label("Import")
        self.input_json.set_size_request(100, 40)
        self.input_json.connect("clicked", self.input_json_clicked)

        container.put(self.output_json, 210, 0)
        container.put(self.input_json, 320, 0)


        #Dropdown feature
        self.label = Gtk.Label("Change Map:")
        self.label.set_size_request(100,40)
        container.put(self.label,430,0)

        game_store = Gtk.ListStore(str)
        games = [
           "2020 Field",
           "2019 Field",
           "2021 Galactic Search ARed",
           "2021 Galactic Search ABlue",
           "2021 Galactic Search BRed",
           "2021 Galactic Search BBlue",
           "2021 AutoNav Barrel Racing",
           "2021 AutoNav Slalom",
           "2021 AutoNav Bounce",
           ]

        self.game_combo = Gtk.ComboBoxText()
        self.game_combo.set_entry_text_column(0)
        self.game_combo.connect("changed", self.input_combobox_choice)

        for game in games:
          self.game_combo.append_text(game)

        self.game_combo.set_active(0)
        self.game_combo.set_size_request(100,40)
        container.put(self.game_combo,440,30)

        self.show_all()

window = GridWindow()
RunApp()
