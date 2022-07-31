#!/usr/bin/python3

# Creates a UI for a user to select the regions in a camera image where the balls could be placed
# for each field layout.
# After the balls have been placed on the field and they submit the regions,
# galactic_search_path.py will take another picture and based on the yellow regions
# in that picture it will determine where the balls are.
# This tells us which path the current field is. It then sends the Alliance and Letter of the path
# with aos_send to the /camera channel for the robot to excecute the spline for that path.

from galactic_search_path import *

import getopt
import glog
import json
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
import os
import sys

_num_rects = 3  # can be 3 or 2, can be specified in commang line arg

_path = Path(Letter.kA, Alliance.kRed, [Rect(None, None, None, None)])

# current index in rects list
_rect_index = 0

_fig, _img_ax = plt.subplots()

_txt = _img_ax.text(0, 0, "", size=10, backgroundcolor="white")

_confirm = Button(plt.axes([0.7, 0.05, 0.1, 0.075]), "Confirm")
_cancel = Button(plt.axes([0.81, 0.05, 0.1, 0.075]), "Cancel")
_submit = Button(plt.axes([0.4, 0.05, 0.1, 0.1]), "Submit")


def draw_txt(txt):
    txt.set_text(
        "Click on top left point and bottom right point for rect #%u" %
        (_rect_index + 1))
    txt.set_color(_path.alliance.value)


def on_confirm(event):
    global _rect_index
    if _path.rects[_rect_index].x1 != None and _path.rects[
            _rect_index].x2 != None:
        _confirm.ax.set_visible(False)
        _cancel.ax.set_visible(False)
        _rect_index += 1
        clear_rect()
        if _rect_index == _num_rects:
            _submit.ax.set_visible(True)
        else:
            draw_txt(_txt)
            _path.rects.append(Rect(None, None, None, None))
        plt.show()


def on_cancel(event):
    global _rect_index
    if _rect_index < _num_rects:
        _confirm.ax.set_visible(False)
        _cancel.ax.set_visible(False)
        clear_rect()
        _path.rects[_rect_index].x1 = None
        _path.rects[_rect_index].y1 = None
        _path.rects[_rect_index].x2 = None
        _path.rects[_rect_index].y2 = None
        plt.show()


def on_submit(event):
    plt.close("all")
    dict = None
    dict = load_json()
    if _path.letter.name not in dict:
        dict[_path.letter.name] = {}
    if _path.alliance.name not in dict[_path.letter.name]:
        dict[_path.letter.name][_path.alliance.name] = []
    dict[_path.letter.name][_path.alliance.name] = [
        rect.to_list() for rect in _path.rects
    ]
    with open(RECTS_JSON_PATH, 'w') as rects_json:
        json.dump(dict, rects_json, indent=2)


# Clears rect on screen
def clear_rect():
    if len(_img_ax.patches) == 0:
        glog.error("There were no patches found in _img_ax")
    else:
        _img_ax.patches[-1].remove()


def on_click(event):
    # This gets called for each click of the rectangle corners,
    # but also gets called when the user clicks on the Submit button.
    # At that time _rect_index will equal the length of rects, and so we'll ignore that click.
    # If it checked the points of the rect at _rect_index, a list out of bounds exception would be thrown.
    # Additionally, the event xdata or ydata will be None if the user clicks out of
    # the bounds of the axis
    if _rect_index < _num_rects and event.xdata != None and event.ydata != None:
        if _path.rects[_rect_index].x1 == None:
            _path.rects[_rect_index].x1, _path.rects[_rect_index].y1 = int(
                event.xdata), int(event.ydata)
        elif _path.rects[_rect_index].x2 == None:
            _path.rects[_rect_index].x2, _path.rects[_rect_index].y2 = int(
                event.xdata), int(event.ydata)
            if _path.rects[_rect_index].x2 < _path.rects[_rect_index].x1:
                tmp = _path.rects[_rect_index].x1
                _path.rects[_rect_index].x1 = _path.rects[_rect_index].x2
                _path.rects[_rect_index].x2 = tmp
            if _path.rects[_rect_index].y2 < _path.rects[_rect_index].y1:
                tmp = _path.rects[_rect_index].y1
                _path.rects[_rect_index].y1 = _path.rects[_rect_index].y2
                _path.rects[_rect_index].y2 = tmp

            _img_ax.add_patch(
                patches.Rectangle(
                    (_path.rects[_rect_index].x1, _path.rects[_rect_index].y1),
                    _path.rects[_rect_index].x2 - _path.rects[_rect_index].x1,
                    _path.rects[_rect_index].y2 - _path.rects[_rect_index].y1,
                    edgecolor='r',
                    linewidth=1,
                    facecolor="none"))
            _confirm.ax.set_visible(True)
            _cancel.ax.set_visible(True)
            plt.show()
    else:
        glog.info(
            "Either submitted or user pressed out of the bounds of the axis")


def setup_button(button, on_clicked):
    button.on_clicked(on_clicked)
    button.ax.set_visible(False)


def setup_ui():
    _img_ax.imshow(capture_img())
    release_stream()

    _fig.canvas.mpl_connect("button_press_event", on_click)
    setup_button(_confirm, on_confirm)
    setup_button(_cancel, on_cancel)
    setup_button(_submit, on_submit)
    draw_txt(_txt)
    plt.show()


def view_rects():

    rects_dict = load_json()
    if (_path.letter.name in rects_dict
            and _path.alliance.name in rects_dict[_path.letter.name]):
        _confirm.ax.set_visible(False)
        _cancel.ax.set_visible(False)
        _submit.ax.set_visible(False)
        _img_ax.imshow(capture_img())

        for rect_list in rects_dict[_path.letter.name][_path.alliance.name]:
            rect = Rect.from_list(rect_list)
            _img_ax.add_patch(
                patches.Rectangle((rect.x1, rect.y1),
                                  rect.x2 - rect.x1,
                                  rect.y2 - rect.y1,
                                  edgecolor='r',
                                  linewidth=1,
                                  facecolor="none"))
        plt.show()
    else:
        glog.error("Could not find path %s %s in rects.json",
                   _path.alliance.value, _path.letter.value)


def main(argv):
    global _num_rects

    glog.setLevel("INFO")
    opts = getopt.getopt(
        argv[1:], "a:l:n:",
        ["alliance = ", "letter = ", "_num_rects = ", "view"])[0]
    view = False
    for opt, arg in opts:
        if opt in ["-a", "--alliance"]:
            _path.alliance = Alliance.from_value(arg)
        elif opt in ["-l", "--letter"]:
            _path.letter = Letter.from_value(arg.upper())
        elif opt in ["-n", "--_num_rects"] and arg.isdigit():
            _num_rects = int(arg)
        elif opt == "--view":
            view = True

    if view:
        view_rects()
    else:
        setup_ui()


if __name__ == "__main__":
    main(sys.argv)
