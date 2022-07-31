#!/usr/bin/python3

import polydrivetrain
import numpy
from numpy.testing import *
import polytope
import unittest

__author__ = 'Austin Schuh (austin.linux@gmail.com)'


class TestVelocityDrivetrain(unittest.TestCase):

    def MakeBox(self, x1_min, x1_max, x2_min, x2_max):
        H = numpy.matrix([[1, 0], [-1, 0], [0, 1], [0, -1]])
        K = numpy.matrix([[x1_max], [-x1_min], [x2_max], [-x2_min]])
        return polytope.HPolytope(H, K)

    def test_coerce_inside(self):
        """Tests coercion when the point is inside the box."""
        box = self.MakeBox(1, 2, 1, 2)

        # x1 = x2
        K = numpy.matrix([[1, -1]])
        w = 0

        assert_array_equal(
            polydrivetrain.CoerceGoal(box, K, w, numpy.matrix([[1.5], [1.5]])),
            numpy.matrix([[1.5], [1.5]]))

    def test_coerce_outside_intersect(self):
        """Tests coercion when the line intersects the box."""
        box = self.MakeBox(1, 2, 1, 2)

        # x1 = x2
        K = numpy.matrix([[1, -1]])
        w = 0

        assert_array_equal(
            polydrivetrain.CoerceGoal(box, K, w, numpy.matrix([[5], [5]])),
            numpy.matrix([[2.0], [2.0]]))

    def test_coerce_outside_no_intersect(self):
        """Tests coercion when the line does not intersect the box."""
        box = self.MakeBox(3, 4, 1, 2)

        # x1 = x2
        K = numpy.matrix([[1, -1]])
        w = 0

        assert_array_equal(
            polydrivetrain.CoerceGoal(box, K, w, numpy.matrix([[5], [5]])),
            numpy.matrix([[3.0], [2.0]]))

    def test_coerce_middle_of_edge(self):
        """Tests coercion when the line intersects the middle of an edge."""
        box = self.MakeBox(0, 4, 1, 2)

        # x1 = x2
        K = numpy.matrix([[-1, 1]])
        w = 0

        assert_array_equal(
            polydrivetrain.CoerceGoal(box, K, w, numpy.matrix([[5], [5]])),
            numpy.matrix([[2.0], [2.0]]))

    def test_coerce_perpendicular_line(self):
        """Tests coercion when the line does not intersect and is in quadrant 2."""
        box = self.MakeBox(1, 2, 1, 2)

        # x1 = -x2
        K = numpy.matrix([[1, 1]])
        w = 0

        assert_array_equal(
            polydrivetrain.CoerceGoal(box, K, w, numpy.matrix([[5], [5]])),
            numpy.matrix([[1.0], [1.0]]))


if __name__ == '__main__':
    unittest.main()
