#!/usr/bin/python

"""Wrapper around spline.h/cc through spline_array.cc."""

__author__ = 'Alex Perry (alex.perry96@gmail.com)'

import ctypes as ct
import numpy as np
import os

libSpline = None
for path in os.environ.get('PYTHONPATH').split(':'):
    try:
        libSpline = ct.cdll.LoadLibrary(os.path.join(path, 'frc971/control_loops/drivetrain/spline.so'))
    except OSError, e:
        pass

# Define required output types.
libSpline.Theta.restype = ct.c_double
libSpline.DTheta.restype = ct.c_double
libSpline.DDTheta.restype = ct.c_double

class Spline:
    """A wrapper around spline.h/cc through spline_array.cc."""

    def __init__(self):
        self.__points = np.zeros((2,6))
        self.__spline = libSpline.newSpline(np.ctypeslib.as_ctypes(self.__points[0]),
                                            np.ctypeslib.as_ctypes(self.__points[1]))

    def __del__(self):
        libSpline.deleteSpline(self.__spline)

    def setPoint(self, index, x, y):
        self.__points[0, index] = x
        self.__points[1, index] = y
        libSpline.deleteSpline(self.__spline)
        self.__spline = libSpline.newSpline(np.ctypeslib.as_ctypes(self.__points[0]),
                                            np.ctypeslib.as_ctypes(self.__points[1]))

    def Point(self, alpha):
        result = np.zeros(2)
        libSpline.Point(self.__spline, ct.c_double(alpha), np.ctypeslib.as_ctypes(result))
        return result

    def DPoint(self, alpha):
        result = np.zeros(2)
        libSpline.DPoint(self.__spline, ct.c_double(alpha), np.ctypeslib.as_ctypes(result))
        return result

    def DDPoint(self, alpha):
        result = np.zeros(2)
        libSpline.DDPoint(self.__spline, ct.c_double(alpha), np.ctypeslib.as_ctypes(result))
        return result

    def DDDPoint(self, alpha):
        result = np.zeros(2)
        libSpline.DDDPoint(self.__spline, ct.c_double(alpha), np.ctypeslib.as_ctypes(result))
        return result

    def Theta(self, alpha):
        return libSpline.Theta(self.__spline, ct.c_double(alpha))

    def DTheta(self, alpha):
        return libSpline.DTheta(self.__spline, ct.c_double(alpha))

    def DDTheta(self, alpha):
        return libSpline.DDTheta(self.__spline, ct.c_double(alpha))

    def ControlPoints(self):
        return self.__points;
