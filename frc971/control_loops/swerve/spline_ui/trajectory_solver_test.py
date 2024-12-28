from trajectory_solver import solve

import unittest


class TestTrajectorySolver(unittest.TestCase):

    def testPath(self):
        test_path = [{
            "splines": [[[0, 0], [0, 2], [1, 2], [2, 2], [3, 2], [3, 4]]],
            "rot_breakpoints": [[0, 0], [1, 3]],
            "constraints": [
                {
                    "selection": [0, 0.125],
                    "max_voltage": None,
                    "max_current": None,
                    "max_acceleration": None,
                    "max_velocity": 2,
                },
                {
                    "selection": [0.125, 0.25],
                    "max_voltage": None,
                    "max_current": 20,
                    "max_acceleration": None,
                    "max_velocity": None,
                },
                {
                    "selection": [0.25, 0.375],
                    "max_voltage": None,
                    "max_current": None,
                    "max_acceleration": 3,
                    "max_velocity": None,
                },
                {
                    "selection": [0.375, 0.5],
                    "max_voltage": 6,
                    "max_current": None,
                    "max_acceleration": None,
                    "max_velocity": None,
                },
            ]
        }]
        test_global_constraints = {
            "max_voltage": 12,
            "max_current": 40,
            "max_acceleration": 20,
            "max_velocity": 4.5,
        }

        solution = solve(test_path, test_global_constraints)[0]
        # Test the solution to ensure it has the correct positions and rotations
        start_pos = solution["positions"][0]
        end_pos = solution["positions"][-1]
        half_pos = solution["positions"][len(solution["positions"]) // 2]

        start_pos_msg = f"Starting position {start_pos} does not match [0, 0, 0]"
        self.assertAlmostEqual(start_pos[0], 0, msg=start_pos_msg)
        self.assertAlmostEqual(start_pos[1], 0, msg=start_pos_msg)
        self.assertAlmostEqual(start_pos[2], 0, msg=start_pos_msg)

        end_pos_msg = f"Ending position {end_pos} does not match [3, 4, 3]"
        self.assertAlmostEqual(end_pos[0], 3, msg=end_pos_msg)
        self.assertAlmostEqual(end_pos[1], 4, msg=end_pos_msg)
        self.assertAlmostEqual(end_pos[2], 3, msg=end_pos_msg)

        half_pos_msg = f"Halfway position {half_pos} does not match [1.5, 2, 1.5]"
        self.assertAlmostEqual(half_pos[0], 1.5, msg=half_pos_msg, places=1)
        self.assertAlmostEqual(half_pos[1], 2, msg=half_pos_msg, places=1)
        self.assertAlmostEqual(half_pos[2], 1.5, msg=half_pos_msg, places=1)

        # Ensure the solution obeys the local constraints
        # Ensure the velocity magnitude is always <= 2 from 0 to 1/8 of the trajectory
        self.assertFalse(any([
            v[0]**2 + v[1]**2 > 2**2 for v in solution["velocities"]
            [:int(len(solution["velocities"]) * 0.125)]
        ]),
                         msg="Local velocity constraint not obeyed")
        # Ensure the currents are all <= 20 from 1/8 to 1/4 of the trajectory
        self.assertFalse(any([
            any([c > 20 for c in currents])
            for currents in solution["driving_currents"]
            [int(len(solution["driving_currents"]) * 0.125 +
                 1):int(len(solution["driving_currents"]) * 0.25)]
        ]),
                         msg="Local current constraint not obeyed")
        # Ensure the acceleration magnitude is <= 3 from 1/4 to 3/8 of the trajectory
        self.assertFalse(any([
            a[0]**2 + a[1]**2 > 3**2 for a in solution["accelerations"]
            [int(len(solution["accelerations"]) * 0.25) +
             1:int(len(solution["accelerations"]) * 0.375)]
        ]),
                         msg="Local acceleration constraint not obeyed")
        # Ensure the voltages are all <= 6 from 3/8 to 1/2 of the trajectory
        self.assertFalse(any([
            any([v > 6 for v in voltages]) for voltages in solution["voltages"]
            [int(len(solution["voltages"]) * 0.375) +
             1:int(len(solution["voltages"]) * 0.5)]
        ]),
                         msg="Local voltage constraint not obeyed")

        # Ensure the solution obeys all the global constraints from 1/2 to the end
        self.assertFalse(any([
            v[0]**2 + v[1]**2 > 4.5**2
            for v in solution["velocities"][len(solution["velocities"]) // 2 +
                                            1:]
        ]),
                         msg="Global velocity constraint not obeyed")
        self.assertFalse(any([
            any([c > 40 for c in currents])
            for currents in solution["driving_currents"]
            [len(solution["driving_currents"]) // 2 + 1:]
        ]),
                         msg="Global current constraint not obeyed")
        self.assertFalse(any([
            a[0]**2 + a[1]**2 > 20**2 for a in solution["accelerations"]
            [len(solution["accelerations"]) // 2 + 1:]
        ]),
                         msg="Global acceleration constraint not obeyed")
        self.assertFalse(any([
            any([v > 12 for v in voltages])
            for voltages in solution["voltages"][len(solution["voltages"]) //
                                                 2 + 1:]
        ]),
                         msg="Global voltage constraint not obeyed")

        # Ensure the solution is a reasonable amount of time
        self.assertLess(solution["times"][-1],
                        30,
                        msg="Solution took longer than 30 seconds.")

    #TODO: Add test for solve_and_discretize()


if __name__ == '__main__':
    unittest.main()
