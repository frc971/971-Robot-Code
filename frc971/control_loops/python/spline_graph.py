#!/usr/bin/python3

# matplotlib overrides fontconfig locations, so has to be imported before gtk
import matplotlib
import gi
from path_edit import FieldWidget
from path_edit import Mode  # still being tested
from basic_window import RunApp
from constants import FIELDS, FIELD, SCREEN_SIZE

gi.require_version('Gtk', '3.0')
from gi.repository import Gdk, Gtk, GLib
import cairo
import basic_window
import os


class GridWindow(Gtk.Window):

    def method_connect(self, event, cb):

        def handler(obj, *args):
            cb(*args)

        self.connect(event, handler)

    def clear_clicked(self, button):
        self.field.clear()

    def toggle_view_clicked(self, button):
        temp_variable = self.field.mode

        if self.field.mode != Mode.kViewing:
            self.field.mode = Mode.kViewing
            self.field.queue_draw()
            Gtk.Button.set_label(self.toggle_view, "Switch to Editing Mode")

        else:
            self.field.mode = self.field.previous_mode
            self.field.queue_draw()
            if self.field.mode == Mode.kEditing:
                Gtk.Button.set_label(self.toggle_view,
                                     "Switch to Viewing Mode")

        self.field.previous_mode = temp_variable

    def output_json_clicked(self, button):
        self.field.export_json(self.file_name_box.get_text())

    def input_json_clicked(self, button):
        self.field.import_json(self.file_name_box.get_text())
        self.long_input.set_value(
            self.field.active_multispline.getConstraint(
                "LONGITUDINAL_ACCELERATION"))
        self.lat_input.set_value(
            self.field.active_multispline.getConstraint(
                "LATERAL_ACCELERATION"))
        self.vol_input.set_value(
            self.field.active_multispline.getConstraint("VOLTAGE"))

    def undo_func(self, *args):
        self.field.undo()

    def new_spline_clicked(self, *args):
        self.field.new_spline()

    def new_multispline_clicked(self, *args):
        self.field.new_multispline()

    def long_changed(self, button):
        value = self.long_input.get_value()
        self.field.active_multispline.setConstraint(
            "LONGITUDINAL_ACCELERATION", value)
        self.field.graph.schedule_recalculate(self.field.multisplines)

    def lat_changed(self, button):
        value = self.lat_input.get_value()
        self.field.active_multispline.setConstraint("LATERAL_ACCELERATION",
                                                    value)
        self.field.graph.schedule_recalculate(self.field.multisplines)

    def vel_changed(self, button):
        value = self.vel_input.get_value()

    def vol_changed(self, button):
        value = self.vol_input.get_value()
        self.field.active_multispline.setConstraint("VOLTAGE", value)
        self.field.graph.schedule_recalculate(self.field.multisplines)

    def input_combobox_choice(self, combo):
        text = combo.get_active_text()
        if text is not None:
            print("Combo Clicked on: " + text)
            self.field.set_field(FIELDS[text])

    def __init__(self):
        Gtk.Window.__init__(self)

        self.set_default_size(1.5 * SCREEN_SIZE, SCREEN_SIZE)

        container = Gtk.Grid()
        container.set_vexpand(True)
        self.add(container)

        self.field = FieldWidget()

        self.method_connect("delete-event", basic_window.quit_main_loop)
        self.method_connect("key-release-event", self.field.do_key_press_event)

        self.file_name_box = Gtk.Entry()
        self.file_name_box.set_size_request(50, 40)
        self.file_name_box.set_text("test.json")
        self.file_name_box.set_editable(True)

        self.long_input = Gtk.SpinButton()
        self.long_input.set_size_request(100, 20)
        self.long_input.set_numeric(True)
        self.long_input.set_range(0, 20)
        self.long_input.set_digits(3)
        self.long_input.set_increments(0.1, 100)
        self.long_input.connect("value-changed", self.long_changed)
        self.long_label = Gtk.Label()
        self.long_label.set_text("Longitudinal Acceleration Restriction")
        self.long_input.set_value(
            self.field.active_multispline.getConstraint(
                "LONGITUDINAL_ACCELERATION"))

        self.lat_input = Gtk.SpinButton()
        self.lat_input.set_size_request(100, 20)
        self.lat_input.set_numeric(True)
        self.lat_input.set_range(0, 20)
        self.lat_input.set_digits(3)
        self.lat_input.set_increments(0.1, 100)
        self.lat_input.connect("value-changed", self.lat_changed)
        self.lat_label = Gtk.Label()
        self.lat_label.set_text("Lateral Acceleration Restriction")
        self.lat_input.set_value(
            self.field.active_multispline.getConstraint(
                "LATERAL_ACCELERATION"))

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
            self.field.active_multispline.getConstraint("VOLTAGE"))

        self.output_json = Gtk.Button.new_with_label("Export")
        self.output_json.set_size_request(50, 40)
        self.output_json.connect("clicked", self.output_json_clicked)

        self.input_json = Gtk.Button.new_with_label("Import")
        self.input_json.set_size_request(50, 40)
        self.input_json.connect("clicked", self.input_json_clicked)

        self.clear = Gtk.Button.new_with_label("Clear")
        self.clear.set_size_request(50, 40)
        self.clear.connect("clicked", self.clear_clicked)
        #-----------currently being edited-----------#

        self.toggle_view = Gtk.Button.new_with_label("Switch to Viewing Mode")
        self.toggle_view.set_size_request(100, 40)
        self.toggle_view.connect("clicked", self.toggle_view_clicked)
        #--------------------------------------------#
        self.undo = Gtk.Button.new_with_label("Undo (Ctrl + Z)")
        self.undo.set_size_request(50, 40)
        self.undo.connect("clicked", self.undo_func)

        self.new_spline = Gtk.Button.new_with_label("Add Spline")
        self.new_spline.set_size_request(50, 40)
        self.new_spline.connect("clicked", self.new_spline_clicked)

        self.new_multispline = Gtk.Button.new_with_label("Add Multispline")
        self.new_multispline.set_size_request(50, 40)
        self.new_multispline.connect("clicked", self.new_multispline_clicked)

        #Dropdown feature
        self.label = Gtk.Label()
        self.label.set_text("Change Field:")
        self.label.set_size_request(100, 40)

        game_store = Gtk.ListStore(str)

        self.game_combo = Gtk.ComboBoxText()
        self.game_combo.set_entry_text_column(0)
        self.game_combo.connect("changed", self.input_combobox_choice)

        for game in FIELDS.keys():
            self.game_combo.append_text(game)

        if FIELD in FIELDS.values():
            self.game_combo.set_active(list(FIELDS.values()).index(FIELD))
        self.game_combo.set_size_request(100, 40)

        limitControls = Gtk.FlowBox()
        limitControls.set_min_children_per_line(1)
        limitControls.set_max_children_per_line(2)
        limitControls.add(self.long_label)
        limitControls.add(self.long_input)

        limitControls.add(self.lat_label)
        limitControls.add(self.lat_input)

        limitControls.add(self.vel_label)
        limitControls.add(self.vel_input)

        limitControls.add(self.vol_label)
        limitControls.add(self.vol_input)

        container.attach(limitControls, 5, 1, 1, 1)

        jsonControls = Gtk.FlowBox()
        jsonControls.set_min_children_per_line(8)
        jsonControls.add(self.file_name_box)
        jsonControls.add(self.output_json)
        jsonControls.add(self.input_json)
        jsonControls.add(self.clear)
        jsonControls.add(self.toggle_view)  #----------------in progress
        jsonControls.add(self.undo)
        jsonControls.add(self.new_spline)
        jsonControls.add(self.new_multispline)
        container.attach(jsonControls, 1, 0, 1, 1)

        container.attach(self.label, 4, 0, 1, 1)
        container.attach(self.game_combo, 5, 0, 1, 1)

        container.attach(self.field, 1, 1, 4, 4)

        container.attach(self.field.graph, 0, 10, 10, 1)

        self.show_all()


window = GridWindow()
RunApp()
