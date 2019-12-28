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
    except (OSError):
        pass

# Define required output types.
libSpline.NewSpline.restype = ct.c_void_p
libSpline.SplineTheta.restype = ct.c_double
libSpline.SplineDTheta.restype = ct.c_double
libSpline.SplineDDTheta.restype = ct.c_double
libSpline.NewDistanceSpline.restype = ct.c_void_p
libSpline.DistanceSplineTheta.restype = ct.c_double
libSpline.DistanceSplineDTheta.restype = ct.c_double
libSpline.DistanceSplineDThetaDt.restype = ct.c_double
libSpline.DistanceSplineDDTheta.restype = ct.c_double
libSpline.DistanceSplineLength.restype = ct.c_double
libSpline.NewTrajectory.restype = ct.c_void_p
libSpline.TrajectoryLength.restype = ct.c_double
libSpline.TrajectoryDistance.restype = ct.c_double
libSpline.TrajectoryGetPlanXVAPtr.restype = ct.c_void_p

# Required for trajectory
libSpline.SetUpLogging()

class Spline:
    """
    A wrapper around spline.h/cc through libspline.cc.
    The functions return values parameterized by alpha, a number that varies
    between 0 and 1 along the length of the spline.
    """

    def __init__(self, points):
        assert points.shape == (2, 6)
        self.__points = points
        self.__spline = ct.c_void_p(libSpline.NewSpline(
            np.ctypeslib.as_ctypes(self.__points[0]),
            np.ctypeslib.as_ctypes(self.__points[1])))

    def __del__(self):
        libSpline.deleteSpline(self.__spline)

    def setPoint(self, index, x, y):
        self.__points[0, index] = x
        self.__points[1, index] = y
        libSpline.deleteSpline(self.__spline)
        self.__spline = ct.c_void_p(libSpline.newSpline(
            np.ctypeslib.as_ctypes(self.__points[0]),
            np.ctypeslib.as_ctypes(self.__points[1])))

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

    def GetSplinePtr(self):
        return self.__spline


class DistanceSpline:
    """
    A wrapper around distance_spline.h/cc through libspline.cc.
    The functions return values parameterized by the distance along the spline,
    starting at 0 and and ending at the value returned by Length().
    """

    def __init__(self, splines):
        self.__spline = None
        spline_ptrs = []
        for spline in splines:
            spline_ptrs.append(spline.GetSplinePtr().value)
        spline_ptrs = np.array(spline_ptrs)

        spline_array = np.ctypeslib.as_ctypes(spline_ptrs)
        self.__spline = ct.c_void_p(libSpline.NewDistanceSpline(
            ct.byref(spline_array), len(splines)))

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

    def GetSplinePtr(self):
        return self.__spline


class Trajectory:
    """A wrapper around trajectory.h/cc through libspline.cc."""

    def __init__(self, distance_spline, vmax=10, num_distance=0):
        self.__trajectory = ct.c_void_p(libSpline.NewTrajectory(
            distance_spline.GetSplinePtr(), ct.c_double(vmax), num_distance))

    def __del__(self):
        libSpline.deleteTrajectory(self.__trajectory)

    def SetLongitudalAcceleration(self, accel):
        libSpline.TrajectorySetLongitualAcceleration(self.__trajectory,
                                                     ct.c_double(accel))

    def SetLateralAcceleration(self, accel):
        libSpline.TrajectorySetLateralAcceleration(self.__trajectory,
                                                   ct.c_double(accel))

    def SetVoltageLimit(self, limit):
        libSpline.TrajectorySetVoltageLimit(self.__trajectory,
                                            ct.c_double(limit))

    def LimitVelocity(self, start_distance, end_distance, max_vel):
        libSpline.TrajectoryLimitVelocity(self.__trajectory,
                                          ct.c_double(start_distance),
                                          ct.c_double(end_distance),
                                          ct.c_double(max_vel))

    def Plan(self):
        """
        Call this to compute the plan, if any of the limits change, a new plan
        must be generated.
        """
        libSpline.TrajectoryPlan(self.__trajectory)

    def Voltage(self, distance):
        """
        Returns a pair of voltages for a given distance.
        Order is left-right.
        """
        result = np.zeros(2)
        libSpline.TrajectoryVoltage(self.__trajectory, ct.c_double(distance),
                                    np.ctypeslib.as_ctypes(result))
        return result

    def Length(self):
        return libSpline.TrajectoryLength(self.__trajectory)

    def Distance(self, index):
        return libSpline.TrajectoryDistance(self.__trajectory, index)

    def Distances(self):
        """
        Returns an array of distances used to compute the plan. The linear
        distance between each distance is equal.
        """
        path_length = libSpline.TrajectoryGetPathLength(self.__trajectory)
        distances = numpy.zeros(path_length)
        libSpline.TrajectoryDistances(self.__trajectory,
                                      np.ctypeslib.as_ctypes(distances))
        return distances

    def GetPlan(self):
        """
        Returns the plan as an array of velocities matched to each distance
        from Distances.
        """
        path_length = libSpline.TrajectoryGetPathLength(self.__trajectory)
        velocities = numpy.zeros(path_length)
        libSpline.TrajectoryGetPlan(self.__trajectory,
                                    np.ctypeslib.as_ctypes(velocities))
        return velocities

    def GetPlanXVA(self, dt):
        """
        dt is in seconds
        Returns the position, velocity, and acceleration as a function of time.
        This is returned as a 3xN numpy array where N is proportional to how
        long it takes to run the path.
        This is slow so don't call more than once with the same data.
        """
        XVAPtr = ct.c_void_p(libSpline.TrajectoryGetPlanXVAPtr(self.__trajectory, int(dt*1e9)))
        XVALength = libSpline.TrajectoryGetVectorLength(XVAPtr)
        X = np.zeros(XVALength)
        V = np.zeros(XVALength)
        A = np.zeros(XVALength)
        libSpline.TrajectoryGetPlanXVA(XVAPtr, np.ctypeslib.as_ctypes(X),
                                       np.ctypeslib.as_ctypes(V),
                                       np.ctypeslib.as_ctypes(A))
        libSpline.TrajectoryDeleteVector(XVAPtr)
        return np.vstack([X, V, A])
