#!/usr/bin/python3

import cv2 as cv
from enum import Enum
import glog
import json
import matplotlib.pyplot as plt
import numpy as np
import os

class Rect:

    # x1 and y1 are top left corner, x2 and y2 are bottom right
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def __str__(self):
        return "({}, {}), ({}, {})".format(self.x1, self.y1, self.x2, self.y2)

    def to_list(self):
        return [self.x1, self.y1, self.x2, self.y2]

    @classmethod
    def from_list(cls, list):
        rect = None
        if len(list) == 4:
            rect = cls(list[0], list[1], list[2], list[3])
        else:
            glog.error("Expected list len to be 4 but it was %u", len(list))
            rect = cls(None, None, None, None)
        return rect


class Alliance(Enum):
    kRed = "red"
    kBlue = "blue"
    kUnknown = None

    @staticmethod
    def from_value(value):
        return (Alliance.kRed if value == Alliance.kRed.value else Alliance.kBlue)

    @staticmethod
    def from_name(name):
        return (Alliance.kRed if name == Alliance.kRed.name else Alliance.kBlue)

class Letter(Enum):
    kA = 'A'
    kB = 'B'

    @staticmethod
    def from_value(value):
        return (Letter.kA if value == Letter.kA.value else Letter.kB)

    @staticmethod
    def from_name(name):
        return (Letter.kA if name == Letter.kA.name else Letter.kB)

class Path:

    def __init__(self, letter, alliance, rects):
        self.letter = letter
        self.alliance = alliance
        self.rects = rects

    def __str__(self):
        return "%s %s: " % (self.alliance.value, self.letter.value)

    def to_dict(self):
        return {"alliance": self.alliance.name, "letter": self.letter.name}

RECTS_JSON_PATH = "rects.json"

AOS_SEND_PATH = "bazel-bin/aos/aos_send"

def setup_if_pi():
    if os.path.isdir("/home/pi/bin"):
        AOS_SEND_PATH = "/home/pi/bin/aos_send.stripped"
        os.system("./starter_cmd stop camera_reader")

setup_if_pi()

# The minimum percentage of yellow for a region of a image to
# be considered to have a ball
BALL_PCT_THRESHOLD = 0.1

_paths = []

def load_json():
    rects_dict = None
    with open(RECTS_JSON_PATH, 'r') as rects_json:
        rects_dict = json.load(rects_json)
    return rects_dict

def _run_detection_loop():
    global img_fig, rects_dict

    rects_dict = load_json()
    for letter in rects_dict:
        for alliance in rects_dict[letter]:
            rects = []
            for rect_list in rects_dict[letter][alliance]:
                rects.append(Rect.from_list(rect_list))
            _paths.append(Path(Letter.from_name(letter), Alliance.from_name(alliance), rects))

    plt.ion()
    img_fig = plt.figure()

    running = True
    while running:
        _detect_path()

def _detect_path():
    img = capture_img()
    img_fig.figimage(img)
    plt.show()

    plt.pause(0.001)

    mask = _create_mask(img)

    current_path = None
    num_current_paths = 0
    for path in _paths:
        pcts = _pct_yellow(mask, path.rects)
        if len(pcts) == len(path.rects):
            glog.info(path)
            for i in range(len(pcts)):
                glog.info("Percent yellow of %s: %f", path.rects[i], pcts[i])
            glog.info("")

            # If all the balls in a path were detected then that path is present
            rects_with_balls = np.where(pcts >= BALL_PCT_THRESHOLD)[0].size
            if rects_with_balls == len(path.rects):
                current_path = path
                num_current_paths += 1
        else:
            glog.error("Error: len of pcts (%u) != len of rects: (%u)", len(pcts), len(rects))

    if num_current_paths != 1:
        if num_current_paths == 0:
            current_path = Path(Letter.kA, None, None)
        current_path.alliance = Alliance.kUnknown
        glog.warn("Expected 1 path but detected %u", num_current_paths)
        return


    path_dict = current_path.to_dict()
    glog.info("Path is %s", path_dict)
    os.system(AOS_SEND_PATH +
              " /pi2/camera y2020.vision.GalacticSearchPath '" + json.dumps(path_dict) + "'")

KERNEL = np.ones((5, 5), np.uint8)

def _create_mask(img):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lower_yellow = np.array([23, 100, 75], dtype = np.uint8)
    higher_yellow = np.array([40, 255, 255], dtype = np.uint8)
    mask = cv.inRange(hsv, lower_yellow, higher_yellow)
    mask = cv.erode(mask, KERNEL, iterations = 1)
    mask = cv.dilate(mask, KERNEL, iterations = 3)

    return mask

# This function finds the percentage of yellow pixels in the rectangles
# given that are regions of the given image. This allows us to determine
# whether there is a ball in those rectangles
def _pct_yellow(mask, rects):
    pcts = np.zeros(len(rects))
    for i in range(len(rects)):
        rect = rects[i]
        slice = mask[rect.y1 : rect.y2, rect.x1 : rect.x2]
        yellow_px = np.count_nonzero(slice)
        pcts[i] = yellow_px / (slice.shape[0] * slice.shape[1])

    return pcts

_video_stream = cv.VideoCapture(0)

def capture_img():
    global _video_stream
    return _video_stream.read()[1]

def release_stream():
    global _video_stream
    _video_stream.release()

def main():
    _run_detection_loop()
    release_stream()

if __name__ == "__main__":
    main()
