#!/usr/bin/python3

import math
import numpy as np
from numpy.testing import *
import unittest

from libspline import Spline, DistanceSpline, Trajectory


class TestSpline(unittest.TestCase):

    def testSimpleSpline(self):
        points = np.array([[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                           [2.0, 3.0, 4.0, 5.0, 6.0, 7.0]])
        spline = Spline(points)
        assert_allclose(spline.Point(.5), [3.5, 4.5])
        assert_almost_equal(spline.Theta(.5), math.atan2(1, 1))


class TestDistanceSpline(unittest.TestCase):

    def testDistanceSpline(self):
        points = np.array([[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                           [2.0, 3.0, 4.0, 5.0, 6.0, 7.0]])
        spline = Spline(points)
        dSpline = DistanceSpline([spline])
        assert_almost_equal(dSpline.Length(), 5 * math.sqrt(2), decimal=5)


class TestTrajectory(unittest.TestCase):

    def testTrajectory(self):
        points = np.array([[1.0, 2.0, 3.0, 4.0, 5.0, 5.0],
                           [2.0, 3.0, 4.0, 5.0, 6.0, 7.0]])
        spline = Spline(points)
        dSpline = DistanceSpline([spline])
        trajectory = Trajectory(dSpline)
        trajectory.Plan()
        plan = trajectory.GetPlanXVA(5.05 * 1e-3)
        self.assertEqual(plan.shape, (3, 745))

    def testLimits(self):
        """ A plan with a lower limit should take longer. """
        points = np.array([[1.0, 2.0, 3.0, 4.0, 5.0, 5.0],
                           [2.0, 3.0, 4.0, 5.0, 6.0, 7.0]])
        spline = Spline(points)
        dSpline = DistanceSpline([spline])
        trajectory = Trajectory(dSpline)
        trajectory.LimitVelocity(0, trajectory.Length(), 3)
        trajectory.Plan()
        plan = trajectory.GetPlanXVA(5.05 * 1e-3)
        self.assertEqual(plan.shape, (3, 753))


if __name__ == '__main__':
    unittest.main()
