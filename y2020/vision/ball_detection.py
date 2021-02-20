#!/usr/bin/python3

from rect import Rect

import cv2 as cv
import numpy as np

# This function finds the percentage of yellow pixels in the rectangles
# given that are regions of the given image. This allows us to determine
# whether there is a ball in those rectangles
def pct_yellow(img, rects):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lower_yellow = np.array([23, 100, 75], dtype = np.uint8)
    higher_yellow = np.array([40, 255, 255], dtype = np.uint8)
    mask = cv.inRange(hsv, lower_yellow, higher_yellow)

    pcts = np.zeros(len(rects))
    for i in range(len(rects)):
        rect = rects[i]
        slice = mask[rect.y1 : rect.y2, rect.x1 : rect.x2]
        yellow_px = np.count_nonzero(slice)
        pcts[i] = 100 * (yellow_px / (slice.shape[0] * slice.shape[1]))

    return pcts

def capture_img():
    video_stream = cv.VideoCapture(0)
    frame = video_stream.read()[1]
    video_stream.release()
    frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
    return frame
