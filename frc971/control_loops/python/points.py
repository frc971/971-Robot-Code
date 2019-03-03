from constants import *
import numpy as np
from libspline import Spline, DistanceSpline, Trajectory


class Points():
    def __init__(self):
        self.points = []  # Holds all points not yet in spline
        self.libsplines = []  # Formatted for libspline library usage
        self.splines = []  # Formatted for drawing

    def getPoints(self):
        return self.points

    def resetPoints(self):
        self.points = []

    def getLibsplines(self):
        return self.libsplines

    def splineExtrapolate(self, o_spline_edit):
        spline_edit = o_spline_edit
        if not spline_edit == len(self.splines) - 1:
            spline_edit = spline_edit + 1
            f = self.splines[spline_edit][5]
            e = self.splines[spline_edit][4]
            d = self.splines[spline_edit][3]
            self.splines[spline_edit][0] = f
            self.splines[spline_edit][1] = f * 2 + e * -1
            self.splines[spline_edit][2] = d + f * 4 + e * -4

        if not spline_edit == 0:
            spline_edit = spline_edit - 1
            a = self.splines[spline_edit][0]
            b = self.splines[spline_edit][1]
            c = self.splines[spline_edit][2]
            self.splines[spline_edit][5] = a
            self.splines[spline_edit][4] = a * 2 + b * -1
            self.splines[spline_edit][3] = c + a * 4 + b * -4

        return spline_edit

    def updates_for_mouse_move(self, index_of_edit, spline_edit, x, y, difs):
        if index_of_edit > -1:
            self.splines[spline_edit][index_of_edit] = [pxToM(x), pxToM(y)]

            if index_of_edit == 5:
                self.splines[spline_edit][
                    index_of_edit -
                    2] = self.splines[spline_edit][index_of_edit - 2] + difs
                self.splines[spline_edit][
                    index_of_edit -
                    1] = self.splines[spline_edit][index_of_edit - 1] + difs

            if index_of_edit == 0:
                self.splines[spline_edit][
                    index_of_edit +
                    2] = self.splines[spline_edit][index_of_edit + 2] + difs
                self.splines[spline_edit][
                    index_of_edit +
                    1] = self.splines[spline_edit][index_of_edit + 1] + difs

            if index_of_edit == 4:
                self.splines[spline_edit][
                    index_of_edit -
                    1] = self.splines[spline_edit][index_of_edit - 1] + difs

            if index_of_edit == 1:
                self.splines[spline_edit][
                    index_of_edit +
                    1] = self.splines[spline_edit][index_of_edit + 1] + difs

            return self.splineExtrapolate(spline_edit)

    def update_lib_spline(self):
        self.libsplines = []
        array = np.zeros(shape=(6, 2), dtype=float)
        for points in self.splines:
            for j, point in enumerate(points):
                array[j, 0] = point[0]
                array[j, 1] = point[1]
            spline = Spline(np.ascontiguousarray(np.transpose(array)))
            self.libsplines.append(spline)

    def getSplines(self):
        return self.splines

    def resetSplines(self):
        self.splines = []

    def setUpSplines(self, newSplines):
        self.splines = newSplines

    def setSplines(self, spline_edit, index_of_edit, x, y):
        self.splines[spline_edit][index_of_edit] = [x, y]

    def add_point(self, x, y):
        if (len(self.points) < 6):
            self.points.append([pxToM(x), pxToM(y)])
        if (len(self.points) == 6):
            self.splines.append(np.array(self.points))
            self.points = []
            self.update_lib_spline()
            return True

    def extrapolate(self, point1, point2,
                    point3):  # where point3 is 3rd to last point
        self.points.append(point1)
        self.points.append(point1 * 2 - point2)
        self.points.append(point3 + point1 * 4 - point2 * 4)
