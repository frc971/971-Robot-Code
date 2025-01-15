from swerve_trajectory_optimizer import *

from math import pi
import unittest


class TestLinePath(unittest.TestCase):

    def testLinePath(self):
        path: SwervePath = SwervePath.line_path([0, 0], 0, [4, 0], 0)
        solution = SwerveSolution(50, path)
        self.assertTrue(solution.valid, msg="Solution did not converge")
        self.assertLess(solution.sol.stats()["iter_count"],
                        500,
                        msg="Solver took over 500 iterations to solve")
        self.assertAlmostEqual(solution.path_length, 4, msg="Pathing error")
        self.assertAlmostEqual(
            solution.driving_currents[0][0],
            40,
            places=1,
            msg=
            "Robot is not trying its hardest - starting current is less than 40 amps"
        )
        self.assertLess(solution.times[-1],
                        30,
                        msg="Robot took over 30 seconds")


class TestRotate(unittest.TestCase):

    def testLinePath(self):
        path: SwervePath = SwervePath.rotate_in_place([0, 0], 0.0, 2 * pi)
        solution = SwerveSolution(50, path)
        self.assertTrue(solution.valid, msg="Solution did not converge")
        self.assertLess(solution.sol.stats()["iter_count"],
                        500,
                        msg="Solver took over 500 iterations to solve")
        self.assertAlmostEqual(solution.path_length, 0, msg="Pathing error")
        self.assertAlmostEqual(solution.driving_currents[0][0],
                               40,
                               places=1,
                               msg="Robot is not trying its hardest")
        self.assertLess(solution.times[-1],
                        30,
                        msg="Robot took over 30 seconds")


class TestFourPointSpline(unittest.TestCase):

    def testFourPointSpline(self):
        path: SwervePath = SwervePath.spline_path(
            [[0, 0], [1, 0], [0, 1], [1, 1]], 0.0, pi)
        solution = SwerveSolution(50, path)
        self.assertTrue(solution.valid, msg="Solution did not converge")
        self.assertLess(solution.sol.stats()["iter_count"],
                        500,
                        msg="Solver took over 500 iterations to solve")
        self.assertLess(solution.times[-1],
                        30,
                        msg="Robot took over 30 seconds")


class TestSixPointSpline(unittest.TestCase):

    def testSixPointSpline(self):
        path: SwervePath = SwervePath.spline_path(
            [[0, 0], [1, 0], [2, 1], [-1, 0], [0, 1], [1, 1]], 0, -2 * pi)
        solution = SwerveSolution(200, path)
        self.assertTrue(solution.valid, msg="Solution did not converge")
        self.assertLess(solution.sol.stats()["iter_count"],
                        500,
                        msg="Solver took over 500 iterations to solve")
        self.assertLess(solution.times[-1],
                        30,
                        msg="Robot took over 30 seconds")


class TestPathCombine(unittest.TestCase):

    def testPathCombine(self):
        path1: SwervePath = SwervePath.spline_path(
            [[0, 0], [1, 0], [2, 1], [-1, 0], [0, 1], [1, 1]], 0, 1)
        path2: SwervePath = SwervePath.spline_path(
            [[1, 1], [2, 1], [3, 0], [0, 1], [1, 2], [2, 2]], 1, 2)
        path3: SwervePath = SwervePath.spline_path(
            [[2, 2], [3, 2], [4, 1], [4, 1], [4, 0], [5, 0]], 2, 3)
        path = SwervePath.combine_paths(path1, path2, path3)
        solution = SwerveSolution(100, path)
        self.assertTrue(solution.valid, msg="Solution did not converge")
        self.assertLess(solution.sol.stats()["iter_count"],
                        500,
                        msg="Solver took over 500 iterations to solve")
        self.assertLess(solution.times[-1],
                        30,
                        msg="Robot took over 30 seconds")


#TODO: Add test that compares the simulated path against full physics

if __name__ == '__main__':
    unittest.main()
