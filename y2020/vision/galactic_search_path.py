#!/usr/bin/python3

# Creates a UI for a user to select the regions in a camera image where the balls could be placed.
# After the balls have been placed on the field and they submit the regions,
# it will take another picture and based on the yellow regions in that picture it will determine where the
# balls are. This tells us which path the current field is. It then sends the Alliance and Letter of the path
# with aos_send to the /camera channel for the robot to excecute the spline for that path.

from rect import Rect
import ball_detection

import cv2 as cv
from enum import Enum
import glog
import json
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
import os

class Alliance(Enum):
    kRed = "red"
    kBlue = "blue"
    kUnknown = None

class Letter(Enum):
    kA = "A"
    kB = "B"


NUM_RECTS = 4
AOS_SEND_PATH = "bazel-bin/aos/aos_send"

if os.path.isdir("/home/pi/robot_code"):
    AOS_SEND_PATH = "/home/pi/robot_code/aos_send.stripped"
    os.system("./starter_cmd stop camera_reader")

# The minimum percentage of yellow for a region of a image to
# be considered to have a ball
BALL_PCT_THRESHOLD = 10

rects = [Rect(None, None, None, None)]

# current index in rects list
rect_index = 0

fig, img_ax = plt.subplots()

txt = img_ax.text(0, 0, "", size = 10, backgroundcolor = "white")

confirm = Button(plt.axes([0.7, 0.05, 0.1, 0.075]), "Confirm")
cancel = Button(plt.axes([0.81, 0.05, 0.1, 0.075]), "Cancel")
submit = Button(plt.axes([0.4, 0.4, 0.1, 0.1]), "Submit")

def draw_txt():
    alliance = (Alliance.kRed if rect_index % 2 == 0 else Alliance.kBlue)
    letter = (Letter.kA if rect_index < (NUM_RECTS / 2) else Letter.kB)
    txt.set_text("Click on top left point and bottom right point for " +
                 alliance.value + ", path " + letter.value)
    txt.set_color(alliance.value)


def on_confirm(event):
    global rect_index
    if rects[rect_index].x1 != None and rects[rect_index].x2 != None:
        confirm.ax.set_visible(False)
        cancel.ax.set_visible(False)
        rect_index += 1
        clear_rect()
        if rect_index == NUM_RECTS:
            submit.ax.set_visible(True)
        else:
            draw_txt()
            rects.append(Rect(None, None, None, None))
        plt.show()

def on_cancel(event):
    global rect_index
    if rect_index < NUM_RECTS:
        confirm.ax.set_visible(False)
        cancel.ax.set_visible(False)
        clear_rect()
        rects[rect_index].x1 = None
        rects[rect_index].y1 = None
        rects[rect_index].x2 = None
        rects[rect_index].y2 = None
        plt.show()

SLEEP = 100
img_fig = None

def on_submit(event):
    global img_fig
    plt.close("all")
    plt.ion()
    img_fig = plt.figure()
    running = True
    while running:
        detect_path()
        cv.waitKey(SLEEP)

def detect_path():
    img = ball_detection.capture_img()
    img_fig.figimage(img)
    plt.show()
    plt.pause(0.001)
    pcts = ball_detection.pct_yellow(img, rects)
    if len(pcts) == len(rects):
        paths = []
        for i in range(len(pcts)):
            alliance = (Alliance.kRed if i % 2 == 0 else Alliance.kBlue)
            letter = (Letter.kA if i < NUM_RECTS / 2 else Letter.kB)
            paths.append({"alliance" : alliance.name, "letter" : letter.name})
        max_index = np.argmax(pcts)
        path = paths[max_index]
        # Make sure that exactly one percentage is >= the threshold
        rects_with_balls = np.where(pcts >= BALL_PCT_THRESHOLD)[0].size
        glog.info("rects_with_balls: %s" % rects_with_balls)
        if rects_with_balls != 1:
            path["alliance"] = Alliance.kUnknown.name
            glog.warn("More than one ball found, path is unknown" if rects_with_balls > 1 else
                      "No balls found")
        glog.info("Path is %s" % path)
        os.system(AOS_SEND_PATH +
                  " /pi2/camera y2020.vision.GalacticSearchPath '" + json.dumps(path) + "'")

        for j in range(len(pcts)):
            glog.info("%s: %s%% yellow" % (rects[j], pcts[j]))
    else:
        glog.error("Error: len of pcts (%u) != len of rects: (%u)" % (len(pcts), len(rects)))

# Clears rect on screen
def clear_rect():
    if len(img_ax.patches) == 0:
        glog.error("There were no patches found in img_ax")
    else:
        img_ax.patches[-1].remove()

def on_click(event):
    # This will get called when user clicks on Submit button, don't want to override the points on
    # the last rect. Additionally, the event xdata or ydata will be None if the user clicks out of
    # the bounds of the axis
    if rect_index < NUM_RECTS and event.xdata != None and event.ydata != None:
        if rects[rect_index].x1 == None:
            rects[rect_index].x1, rects[rect_index].y1 = int(event.xdata), int(event.ydata)
        elif rects[rect_index].x2 == None:
            rects[rect_index].x2, rects[rect_index].y2 = int(event.xdata), int(event.ydata)
            if rects[rect_index].x2 < rects[rect_index].x1:
                rects[rect_index].x2 = rects[rect_index].x1 + (rects[rect_index].x1 - rects[rect_index].x2)
            if rects[rect_index].y2 < rects[rect_index].y1:
                    rects[rect_index].y2 = rects[rect_index].y1 + (rects[rect_index].y1 - rects[rect_index].y2)

            img_ax.add_patch(patches.Rectangle((rects[rect_index].x1, rects[rect_index].y1),
                rects[rect_index].x2 - rects[rect_index].x1, rects[rect_index].y2 - rects[rect_index].y1,
                edgecolor = 'r', linewidth = 1, facecolor="none"))
            confirm.ax.set_visible(True)
            cancel.ax.set_visible(True)
            plt.show()
    else:
        glog.info("Either submitted or user pressed out of the bounds of the axis")

def setup_button(button, on_clicked):
    button.on_clicked(on_clicked)
    button.ax.set_visible(False)

def main():
    glog.setLevel("INFO")

    img_ax.imshow(ball_detection.capture_img())

    fig.canvas.mpl_connect("button_press_event", on_click)
    setup_button(confirm, on_confirm)
    setup_button(cancel, on_cancel)
    setup_button(submit, on_submit)
    draw_txt()
    plt.show()

if __name__ == "__main__":
    main()
