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

    @staticmethod
    def splineExtrapolate(multisplines, index, *, snap=True):
        multispline = multisplines[index.multispline_index]
        spline = multispline.splines[index.spline_index]

        # find the adjacent splines that will be affected by this spline

        prev_multispline = None
        prev_spline = None
        if index.spline_index != 0:
            prev_spline = multispline.splines[index.spline_index - 1]
        elif index.multispline_index != 0:
            prev_multispline = multisplines[index.multispline_index - 1]
            prev_spline = prev_multispline.splines[-1]

        next_multispline = None
        next_spline = None
        if spline is not multispline.splines[-1]:
            next_spline = multispline.splines[index.spline_index + 1]
        elif multispline is not multisplines[-1]:
            next_multispline = multisplines[index.multispline_index + 1]
            if next_multispline.splines:
                next_spline = next_multispline.splines[0]

        # adjust the adjacent splines according to our smoothness constraints
        # the three points on either side are entirely constrained by this spline

        if next_spline is not None:
            f = spline[5]  # the end of the spline
            e = spline[4]  # determines the heading
            d = spline[3]
            if next_multispline is None:
                next_spline[0] = f
                next_spline[1] = f * 2 + e * -1
                next_spline[2] = d + f * 4 + e * -4
            else:
                if snap:
                    Multispline.snapSplines(spline,
                                            next_spline,
                                            match_first_to_second=False)

                next_spline[0] = f
                next_multispline.update_lib_spline()

        if prev_spline is not None:
            a = spline[0]
            b = spline[1]
            c = spline[2]
            if prev_multispline is None:
                prev_spline[5] = a
                prev_spline[4] = a * 2 + b * -1
                prev_spline[3] = c + a * 4 + b * -4
            else:
                if snap:
                    Multispline.snapSplines(prev_spline,
                                            spline,
                                            match_first_to_second=True)

                prev_spline[5] = a
                prev_multispline.update_lib_spline()

    def snapSplines(first_spline, second_spline, *, match_first_to_second):
        """Snaps two adjacent splines together, preserving the heading from one to another.

        The end of `first_spline` connects to the beginning of `second_spline`
        The user will have their mouse one one of these splines, so we
        only want to snap the spline that they're not holding.

        They can have the same heading, or they can have opposite headings
        which represents the robot driving that spline backwards.
        """

        # Represent the position of the second control point (controls the heading)
        # as a vector from the end of the spline to the control point
        first_vector = first_spline[-2] - first_spline[-1]
        second_vector = second_spline[1] - second_spline[0]

        # we want to preserve the distance
        first_magnitude = np.linalg.norm(first_vector, ord=2)
        second_magnitude = np.linalg.norm(second_vector, ord=2)

        normalized_first = first_vector / first_magnitude
        normalized_second = second_vector / second_magnitude

        # the proposed new vector if we were to make them point the same direction
        swapped_second = normalized_first * second_magnitude
        swapped_first = normalized_second * first_magnitude

        # they were pointing in opposite directions
        if np.dot(first_vector, second_vector) < 0:

            # rotate the new vectors 180 degrees
            # to keep them pointing in opposite directions
            swapped_first = -swapped_first
            swapped_second = -swapped_second

        # Calculate how far we ended up moving the second control point
        # so we can move the third control point with it
        change_in_first = swapped_first - first_vector
        change_in_second = swapped_second - second_vector

        # apply the changes and discard the other proposed snap
        if match_first_to_second:
            first_spline[-2] = swapped_first + first_spline[-1]
            first_spline[-3] += change_in_first
            # swapped_second doesn't get used
        else:
            second_spline[1] = swapped_second + second_spline[0]
            second_spline[2] += change_in_second
            # swapped_first doesn't get used

    def updates_for_mouse_move(self, multisplines, index, mouse):
        """Moves the control point and adjacent points to follow the mouse"""
        if index == None: return
        spline_edit = index.spline_index
        index_of_edit = index.control_point_index

        spline = self.splines[spline_edit]

        # we want to move it to be on the mouse
        diffs = mouse - spline[index_of_edit]

        spline[index_of_edit] = mouse

        # all three points move together with the endpoint
        if index_of_edit == 5:
            spline[3] += diffs
            spline[4] += diffs

            # check if the next multispline exists and has a spline
            if index.spline_index == len(
                    self.splines) - 1 and not index.multispline_index == len(
                        multisplines) - 1 and len(
                            multisplines[index.multispline_index +
                                         1].splines) > 0:
                # move the points that lay across the multispline boundary
                other_spline = multisplines[index.multispline_index +
                                            1].splines[0]

                other_spline[1] += diffs
                other_spline[2] += diffs

        if index_of_edit == 0:
            spline[2] += diffs
            spline[1] += diffs

            # check if previous multispline exists
            if index.spline_index == 0 and not index.multispline_index == 0:
                other_spline = multisplines[index.multispline_index -
                                            1].splines[-1]

                other_spline[3] += diffs
                other_spline[4] += diffs

        # the third point moves with the second point
        if index_of_edit == 4:
            spline[3] += diffs

        if index_of_edit == 1:
            spline[2] += diffs

        Multispline.splineExtrapolate(multisplines, index, snap=False)

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
            if not multispline.getLibsplines():
                continue
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

    def extrapolate(self, spline):
        """Stages 3 points extrapolated from the end of the multispline"""

        self.staged_points = []

        point1 = spline[5]
        point2 = spline[4]
        point3 = spline[3]

        self.staged_points.append(point1)
        self.staged_points.append(point1 * 2 - point2)
        self.staged_points.append(point3 + point1 * 4 - point2 * 4)
