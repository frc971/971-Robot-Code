#!/usr/bin/env python3
# This script runs the multi camera calibration code through its paces using log data
# It uses the AprilTag output logs, rather than directly from images
import argparse
import os
import subprocess
import sys
import time
from typing import Sequence, Text


# Compare two json files that may have a different timestamp
def compare_files(gt_file: str, calc_file: str):
    with open(gt_file, "r") as f_gt:
        with open(calc_file, "r") as f_calc:
            while True:
                line_gt = f_gt.readline()
                line_calc = f_calc.readline()
                if not line_gt:
                    if not line_calc:
                        return True
                    else:
                        return False

                # timestamp field will be different, so ignore this line
                if "timestamp" in line_gt:
                    if "timestamp" in line_calc:
                        continue
                    else:
                        return False

                # Compare line and return False if different
                if line_gt != line_calc:
                    print("Lines don't match!")
                    print("\tGround truth file:", line_gt, end='')
                    print("\tCalculated file:", line_calc, end='')
                    return False
            return True

    return False


# Run through the calibration routine and file checking with max_pose_error arg
def check_calib_match(args, max_pose_error: float):
    calibrate_result = subprocess.run(
        [
            args.calibrate_binary,
            args.logfile,
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
    for pi in [2, 3, 4]:
        pi_name = "pi-971-" + str(pi)
        # Look for calculated calibration file in /tmp dir with pi_name
        calc_calib_dir = "/tmp/"
        files = os.listdir(calc_calib_dir)
        calc_file = ""
        # Read the calculated files in reverse order, so that we pick
        # up the most newly created file each time
        for file in files[::-1]:
            # check if file contains substring pi_name
            if pi_name in file:
                calc_file = calc_calib_dir + file

        # Next find the "ground truth" file with this pi_name
        external_dir = 'external/calibrate_multi_cameras_data/'
        files = os.listdir(external_dir)
        gt_file = ""
        for file in files[::-1]:
            if pi_name in file:
                gt_file = external_dir + file

        if calc_file != "" and gt_file != "":
            if not compare_files(gt_file, calc_file):
                return False

    return True


def main(argv: Sequence[Text]):
    parser = argparse.ArgumentParser()
    parser.add_argument("--logfile",
                        required=True,
                        default="calib1",
                        help="Path to logfile.")
    parser.add_argument(
        "--calibrate_binary",
        required=False,
        default=
        "/home/jimostrowski/code/FRC/971-Robot-Code/bazel-bin/y2023/vision/calibrate_multi_cameras",
        help="Path to calibrate_multi_cameras binary",
    )
    args = parser.parse_args(argv)

    # Run once with correct max_pose_error
    # These were the flags used to create the test file
    # max_pose_error = 5e-5
    # max_pose_error_ratio = 0.4
    if not check_calib_match(args, 5e-5):
        return -1

    # And once with the incorrect value for max_pose_error to see that it fails
    if check_calib_match(args, 1e-5):
        return -1

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
