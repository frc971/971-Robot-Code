from constants import *
import numpy as np
import scipy.optimize
from libspline import Spline, DistanceSpline, Trajectory
import copy
from dataclasses import dataclass


@dataclass
class ControlPointIndex:
    """Class specifying the index of a control point"""

    # The index of the multispline in the list of multisplines
    multispline_index: int

    # The index of the spline in the multispline
    spline_index: int

    # The index of the control point in the spline [0-5]
    control_point_index: int


class Multispline():
    def __init__(self):
        self.staged_points = []  # Holds all points not yet in spline
        self.libsplines = []  # Formatted for libspline library usage
        self.splines = []  # Formatted for drawing
        self.constraints = [  # default constraints
            {
                "constraint_type": "LONGITUDINAL_ACCELERATION",
                "value": 3
            }, {
                "constraint_type": "LATERAL_ACCELERATION",
                "value": 2
            }, {
                "constraint_type": "VOLTAGE",
                "value": 10
            }
        ]

    def __deepcopy__(self, memo):
        new_copy = Multispline()
        new_copy.staged_points = copy.deepcopy(self.staged_points, memo)
        new_copy.splines = copy.deepcopy(self.splines, memo)
        new_copy.constraints = copy.deepcopy(self.constraints, memo)
        new_copy.update_lib_spline()
        return new_copy

    def getLibsplines(self):
        return self.libsplines

    def setConstraint(self, id, value):
        for constraint in self.constraints:
            if constraint["constraint_type"] == id:
                constraint["value"] = value
                break

    def getConstraint(self, id):
        for constraint in self.constraints:
            if constraint["constraint_type"] == id:
                return constraint["value"]

    def addConstraintsToTrajectory(self, trajectory):
        for constraint in self.constraints:
            if constraint["constraint_type"] == "VOLTAGE":
                trajectory.SetVoltageLimit(constraint["value"])
            elif constraint["constraint_type"] == "LATERAL_ACCELERATION":
                trajectory.SetLateralAcceleration(constraint["value"])
            elif constraint["constraint_type"] == "LONGITUDINAL_ACCELERATION":
                trajectory.SetLongitudinalAcceleration(constraint["value"])

    def splineExtrapolate(self, o_spline_edit):
        spline_edit = o_spline_edit
        if not spline_edit == len(self.splines) - 1:
            f = self.splines[spline_edit][5]
            e = self.splines[spline_edit][4]
            d = self.splines[spline_edit][3]
            self.splines[spline_edit + 1][0] = f
            self.splines[spline_edit + 1][1] = f * 2 + e * -1
            self.splines[spline_edit + 1][2] = d + f * 4 + e * -4

        if not spline_edit == 0:
            a = self.splines[spline_edit][0]
            b = self.splines[spline_edit][1]
            c = self.splines[spline_edit][2]
            self.splines[spline_edit - 1][5] = a
            self.splines[spline_edit - 1][4] = a * 2 + b * -1
            self.splines[spline_edit - 1][3] = c + a * 4 + b * -4

    def updates_for_mouse_move(self, index_of_edit, spline_edit, x, y, difs):
        if index_of_edit > -1:
            self.splines[spline_edit][index_of_edit] = [x, y]

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

            self.splineExtrapolate(spline_edit)

    def update_lib_spline(self):
        self.libsplines = []
        array = np.zeros(shape=(6, 2), dtype=float)
        for points in self.splines:
            for j, point in enumerate(points):
                array[j, 0] = point[0]
                array[j, 1] = point[1]
            spline = Spline(np.ascontiguousarray(np.transpose(array)))
            self.libsplines.append(spline)

    @staticmethod
    def nearest_distance(multisplines, point):
        """Finds the spot along the multisplines that is closest to a
        given point on the field

        Returns the closest multispline and the distance along that multispline
        """
        def distance(t, distance_spline, point):
            return np.sum((distance_spline.XY(t) - point)**2)

        # We know the derivative of the function,
        # so scipy doesn't need to compute it every time
        def ddistance(t, distance_spline, point):
            return np.sum(2 * (distance_spline.XY(t) - point) *
                          distance_spline.DXY(t))

        best_result = None
        best_multispline = None

        for multispline_index, multispline in enumerate(multisplines):
            distance_spline = DistanceSpline(multispline.getLibsplines())

            # The optimizer finds local minima that often aren't what we want,
            # so try from multiple locations to find a better minimum.
            guess_points = np.linspace(0, distance_spline.Length(), num=5)

            for guess in guess_points:
                result = scipy.optimize.minimize(
                    distance,
                    guess,
                    args=(distance_spline, point),
                    bounds=((0, distance_spline.Length()), ),
                    jac=ddistance,
                )

                if result.success and (best_result == None
                                       or result.fun < best_result.fun):
                    best_result = result
                    best_multispline = multispline

        return (best_multispline, best_result)

    def toJsonObject(self):
        multi_spline = {
            "spline_count": 0,
            "spline_x": [],
            "spline_y": [],
            "constraints": self.constraints,
        }
        for points in self.splines:
            multi_spline["spline_count"] += 1
            for j, point in enumerate(points):
                if j == 0 and multi_spline["spline_count"] > 1:
                    continue  # skip overlapping points
                multi_spline["spline_x"].append(point[0])
                multi_spline["spline_y"].append(point[1])
        return multi_spline

    @staticmethod
    def fromJsonObject(multi_spline):
        multispline = Multispline()
        multispline.constraints = multi_spline["constraints"]
        multispline.splines = []
        multispline.staged_points = []

        i = 0
        for j in range(multi_spline["spline_count"]):
            # get the last point of the last spline
            # and read in another 6 points
            for i in range(i, i + 6):
                multispline.staged_points.append(
                    [multi_spline["spline_x"][i], multi_spline["spline_y"][i]])
            multispline.splines.append(np.array(multispline.staged_points))
            multispline.staged_points = []
        multispline.update_lib_spline()

        return multispline

    def getSplines(self):
        return self.splines

    def setControlPoint(self, index, x, y):
        self.splines[index.spline_index][index.control_point_index] = [x, y]

    def addPoint(self, x, y):
        if (len(self.staged_points) < 6):
            self.staged_points.append([x, y])
        if (len(self.staged_points) == 6):
            self.splines.append(np.array(self.staged_points))
            self.staged_points = []
            self.update_lib_spline()
            return True

    def extrapolate(self):
        """Stages 3 points extrapolated from the end of the multispline"""
        if len(self.getSplines()) < 1: return

        self.staged_points = []

        spline = self.getSplines()[-1]
        point1 = spline[5]
        point2 = spline[4]
        point3 = spline[3]

        self.staged_points.append(point1)
        self.staged_points.append(point1 * 2 - point2)
        self.staged_points.append(point3 + point1 * 4 - point2 * 4)
