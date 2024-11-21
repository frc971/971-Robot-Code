#!/usr/bin/env python3
# This script runs the multi camera calibration code through its paces using log data
# It uses the AprilTag output logs, rather than directly from images
import argparse
import glob
import json
import os
import subprocess
import sys
import time
import unittest
from typing import Sequence, Text

# Compare two json files that may have a different timestamp


class calibrate_multi_cameras_test(unittest.TestCase):
    args = None

    def argsPassed(self):
        self.assertTrue(self.args != None)

    def testCorrectPose(self):
        self.check_calib_match(5e-5)

    def compare_files(self,
                      gt_file: str,
                      calc_file: str,
                      tolerance: float = 1.1e-1,
                      compare_line: bool = False):

        if not compare_line:
            with open(gt_file) as f:
                gt = json.load(f)

            with open(calc_file) as f:
                calc = json.load(f)

            print("gt:\n", gt, "\ncalc:\n", calc)

            self.assertEqual(gt["node_name"], calc["node_name"])

            self.assertEqual(gt["team_number"], calc["team_number"])

            self.assertEqual(gt["camera_id"], calc["camera_id"])

            self.assertEqual(gt["camera_number"], calc["camera_number"])

            self.assertEqual(len(gt["intrinsics"]), len(calc["intrinsics"]))

            for gt_intrinsics, calc_intrinsics in zip(gt["intrinsics"],
                                                      calc["intrinsics"]):
                self.assertTrue(
                    abs(gt_intrinsics - calc_intrinsics) < tolerance)

            self.assertEqual(len(gt["dist_coeffs"]), len(calc["dist_coeffs"]))

            for gt_dist_coeffs, calc_dist_coeffs in zip(
                    gt["dist_coeffs"], calc["dist_coeffs"]):
                self.assertTrue(
                    abs(gt_dist_coeffs - calc_dist_coeffs) < tolerance)

            self.assertEqual(
                len(gt["fixed_extrinsics"]["data"]),
                len(calc["fixed_extrinsics"]["data"]),
            )

            for gt_fixed_extrinsics, calc_fixed_extrinsics in zip(
                    gt["fixed_extrinsics"]["data"],
                    calc["fixed_extrinsics"]["data"]):
                print(gt_fixed_extrinsics, calc_fixed_extrinsics)
                self.assertTrue(
                    abs(gt_fixed_extrinsics -
                        calc_fixed_extrinsics) < tolerance)
        else:
            print(gt_file, calc_file)
            with open(gt_file, "r") as f_gt:
                with open(calc_file, "r") as f_calc:
                    while True:
                        line_gt = f_gt.readline()
                        line_calc = f_calc.readline()

                        if not line_gt and not line_calc:
                            break

                        self.assertTrue(line_gt)
                        self.assertTrue(line_calc)

                        # timestamp field will be different, so ignore this line
                        if "timestamp" in line_gt and "timestamp" in line_calc:
                            # there is timestamp in both so we skip
                            continue

                        # no timestamp check
                        self.assertTrue("timestamp" not in line_gt)
                        self.assertTrue("timestamp" not in line_calc)

                        # Compare line and raise assert error if different
                        self.assertTrue(line_gt == line_calc)

    # Run through the calibration routine and file checking with max_pose_error arg
    def check_calib_match(self, max_pose_error: float):
        calibrate_result = subprocess.run(
            [
                self.args.calibrate_binary,
                "--robot",
                self.args.logfile,
                "--target_type",
                "apriltag",
                "--team_number",
                "971",
                "--max_pose_error",
                str(max_pose_error),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            encoding="utf-8",
        )

        calibrate_result.check_returncode()

        # Check for the 3 pi's that get calibrated
        for orin in [("imu", "0"), ("imu", "1"), ("orin1", "1"),
                     ("orin1", "0")]:
            # multi camera calibration is is keeping the original position of IMU 0 instead of calculating its position, so no calc file exists for it
            if orin == ("imu", "0"):
                continue

            orin_filename = (
                f"calibration_{orin[0]}-{self.args.team_number}-{orin[1]}_*.json"
            )

            # Get the calculated calibration file
            calc_file = glob.glob(
                os.path.join(os.getenv("TEST_TMPDIR", "/tmp/"),
                             orin_filename))[0]  # newest

            # Get the ground truth calibration file (from original calibration)
            gt_file = glob.glob(
                os.path.join('y2024/constants/calib_files/',
                             orin_filename))[0]  # newest

            if not self.compare_files(calc_file=calc_file, gt_file=gt_file):
                print("Failed comparison for ", orin)
                return False

        return True


def main(argv: Sequence[Text]):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--calibrate_binary",
        required=False,
        default="y2024/vision/calibrate_multi_cameras",
        help="Path to calibrate_multi_cameras binary",
    )
    parser.add_argument("--logfile", required=True, help="Path to logfile.")
    parser.add_argument("--team_number", required=False, default="971")
    args = parser.parse_args(argv)

    while len(sys.argv) > 1:
        sys.argv.pop()

    calibrate_multi_cameras_test.args = args
    unittest.main()

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
