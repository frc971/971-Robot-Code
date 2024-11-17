import unittest
import numpy as np
from scipy.linalg import solve_discrete_are

from frc971.control_loops.cpp_dare import dare as cpp_dare


class TestSwervePhysics(unittest.TestCase):

    def test_cpp_scipy_match(self):
        """Tests that the cpp dare solver matches the scipy dare solver"""
        # test a barely stabilizable system
        d = 2
        A = np.zeros((8, 8))
        A[0, 0] = 0.9
        A[1, 1] = 0.9
        A[1, 2] = 1
        A[2, 2] = 0.9
        A[2, 3] = 1
        A[3, 3] = 0.9
        A[3, 4] = 1
        A[4, 4] = 0.9
        A[5, 5] = 1 - 10**-d
        A[5, 6] = 1
        A[6, 6] = 1 - 10**-d
        A[6, 7] = 1
        A[7, 7] = 1 - 10**-d
        B = np.zeros((8, 5))
        B[0, 0] = 1
        B[1, 1] = 1
        B[2, 2] = 10**-d
        B[5, 3] = 1
        B[6, 4] = 10**-d
        Q = np.diag(np.arange(1, 9)).astype(float)
        R = np.diag(np.array([0.1, 0.3, 0.4, 0.5, 0.2]))
        py_sol = solve_discrete_are(A, B, Q, R)
        cpp_sol = np.array(
            cpp_dare(A.tolist(), B.tolist(), Q.tolist(), R.tolist()))
        self.assertTrue(np.allclose(py_sol, cpp_sol))


if __name__ == "__main__":
    unittest.main()
