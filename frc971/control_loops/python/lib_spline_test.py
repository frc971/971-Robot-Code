#!/usr/bin/python

import math
import numpy as np
from numpy.testing import *
import unittest

from libspline import Spline, DistanceSpline

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
        assert_almost_equal(dSpline.Length(), 5 * math.sqrt(2))

if __name__ == '__main__':
    unittest.main()
