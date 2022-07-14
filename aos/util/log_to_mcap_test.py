#!/usr/bin/env python3
# This script is meant to act as a test to confirm that our log_to_mcap converter produces
# a valid MCAP file. To do so, it first generates an AOS log, then converts it to MCAP, and
# then runs the "mcap doctor" tool on it to confirm compliance with the standard.
import argparse
import subprocess
import sys
import tempfile
import time
from typing import Sequence, Text


def main(argv: Sequence[Text]):
    parser = argparse.ArgumentParser()
    parser.add_argument("--log_to_mcap", required=True, help="Path to log_to_mcap binary.")
    parser.add_argument("--mcap", required=True, help="Path to mcap binary.")
    parser.add_argument("--generate_log", required=True, help="Path to logfile generator.")
    args = parser.parse_args(argv)
    with tempfile.TemporaryDirectory() as tmpdir:
        log_name = tmpdir + "/test_log/"
        mcap_name = tmpdir + "/log.mcap"
        subprocess.run([args.generate_log, "--output_folder", log_name]).check_returncode()
        # Run with a really small chunk size, to force a multi-chunk file.
        subprocess.run(
            [args.log_to_mcap, "--output_path", mcap_name, "--mcap_chunk_size", "1000", "--mode", "json",
             log_name]).check_returncode()
        # MCAP attempts to find $HOME/.mcap.yaml, and dies on $HOME not existing. So
        # give it an arbitrary config location (it seems to be fine with a non-existent config).
        doctor_result = subprocess.run(
            [args.mcap, "doctor", mcap_name, "--config", tmpdir + "/.mcap.yaml"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            encoding='utf-8')
        print(doctor_result.stdout)
        print(doctor_result.stderr)
        # mcap doctor doesn't actually return a non-zero exit code on certain failures...
        # See https://github.com/foxglove/mcap/issues/356
        if len(doctor_result.stderr) != 0:
            print("Didn't expect any stderr output.")
            return 1
        if doctor_result.stdout != f"Examining {mcap_name}\n":
            print("Only expected one line of stdout.")
            return 1
        doctor_result.check_returncode()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
