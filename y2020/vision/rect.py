#!/usr/bin/python3

class Rect:

    # x1 and y1 are top left corner, x2 and y2 are bottom right
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def __str__(self):
        return "({}, {}), ({}, {})".format(self.x1, self.y1, self.x2, self.y2)
