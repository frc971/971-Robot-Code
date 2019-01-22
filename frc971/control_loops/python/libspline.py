#!/usr/bin/python
"""Wrapper around spline.h/cc through spline_array.cc."""

__author__ = 'Alex Perry (alex.perry96@gmail.com)'

import ctypes as ct
import numpy as np
import os

libSpline = None
for path in os.environ.get('PYTHONPATH').split(':'):
    try:
        libSpline = ct.cdll.LoadLibrary(
            os.path.join(path, 'frc971/control_loops/drivetrain/spline.so'))
    except OSError, e:
        pass

# Define required output types.
libSpline.SplineTheta.restype = ct.c_double
libSpline.SplineDTheta.restype = ct.c_double
libSpline.SplineDDTheta.restype = ct.c_double
libSpline.DistanceSplineTheta.restype = ct.c_double
libSpline.DistanceSplineDTheta.restype = ct.c_double
libSpline.DistanceSplineDThetaDt.restype = ct.c_double
libSpline.DistanceSplineDDTheta.restype = ct.c_double
libSpline.DistanceSplineLength.restype = ct.c_double


class Spline:
    """A wrapper around spline.h/cc through libspline.cc."""

    def __init__(self, points):
        assert points.shape == (2, 6)
        self.__points = points
        self.__spline = libSpline.NewSpline(
            np.ctypeslib.as_ctypes(self.__points[0]),
            np.ctypeslib.as_ctypes(self.__points[1]))

    def __del__(self):
        libSpline.deleteSpline(self.__spline)

    def setPoint(self, index, x, y):
        self.__points[0, index] = x
        self.__points[1, index] = y
        libSpline.deleteSpline(self.__spline)
        self.__spline = libSpline.newSpline(
            np.ctypeslib.as_ctypes(self.__points[0]),
            np.ctypeslib.as_ctypes(self.__points[1]))

    def Point(self, alpha):
        result = np.zeros(2)
        libSpline.SplinePoint(self.__spline, ct.c_double(alpha),
                              np.ctypeslib.as_ctypes(result))
        return result

    def DPoint(self, alpha):
        result = np.zeros(2)
        libSpline.SplineDPoint(self.__spline, ct.c_double(alpha),
                               np.ctypeslib.as_ctypes(result))
        return result

    def DDPoint(self, alpha):
        result = np.zeros(2)
        libSpline.SplineDDPoint(self.__spline, ct.c_double(alpha),
                                np.ctypeslib.as_ctypes(result))
        return result

    def DDDPoint(self, alpha):
        result = np.zeros(2)
        libSpline.SplineDDDPoint(self.__spline, ct.c_double(alpha),
                                 np.ctypeslib.as_ctypes(result))
        return result

    def Theta(self, alpha):
        return libSpline.SplineTheta(self.__spline, ct.c_double(alpha))

    def DTheta(self, alpha):
        return libSpline.SplineDTheta(self.__spline, ct.c_double(alpha))

    def DDTheta(self, alpha):
        return libSpline.SplineDDTheta(self.__spline, ct.c_double(alpha))

    def ControlPoints(self):
        return self.__points

    def Spline(self):
        return self.__spline


class DistanceSpline:
    """A wrapper around distance_spline.h/cc through libdistancespline.cc."""

    def __init__(self, splines):
        self.__spline = None
        spline_ptrs = []
        for spline in splines:
            spline_ptrs.append(spline.Spline())
        spline_ptrs = np.array(spline_ptrs)

        spline_array = np.ctypeslib.as_ctypes(spline_ptrs)
        self.__spline = libSpline.NewDistanceSpline(
            ct.byref(spline_array), len(splines))

    def __del__(self):
        libSpline.deleteDistanceSpline(self.__spline)

    def XY(self, distance):
        result = np.zeros(2)
        libSpline.DistanceSplineXY(self.__spline, ct.c_double(distance),
                                   np.ctypeslib.as_ctypes(result))
        return result

    def DXY(self, distance):
        result = np.zeros(2)
        libSpline.DistanceSplineDXY(self.__spline, ct.c_double(distance),
                                    np.ctypeslib.as_ctypes(result))
        return result

    def DDXY(self, distance):
        result = np.zeros(2)
        libSpline.DistanceSplineDDXY(self.__spline, ct.c_double(distance),
                                     np.ctypeslib.as_ctypes(result))
        return result

    def Theta(self, distance):
        return libSpline.DistanceSplineTheta(self.__spline,
                                             ct.c_double(distance))

    def DTheta(self, distance):
        return libSpline.DistanceSplineDTheta(self.__spline,
                                              ct.c_double(distance))

    def DThetaDt(self, distance, velocity):
        return libSpline.DistanceSplineDThetaDt(self.__spline,
                                                ct.c_double(distance),
                                                ct.c_double(velocity))

    def DDTheta(self, distance):
        return libSpline.DistanceSplineDDTheta(self.__spline,
                                               ct.c_double(distance))

    def Length(self):
        return libSpline.DistanceSplineLength(self.__spline)
