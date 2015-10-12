#!/bin/bash

# This script runs doc/gyp_to_bazel for most of our code base.
# We're skipping bot3 for now because it's under active development.

set -e
set -u

run_for_folder() {
  PYTHONPATH=${HOME}/pyyaml-3.11-prefix/lib/python3.4/site-packages/ find $1 \
    -type d \
    -exec bash -c '[[ -r {}/$(basename {}).gyp ]] && doc/gyp_to_bazel.py {}' \;
  find $1 -type f \( -name '*.gyp' -or -name '*.gypi' \) \
    -exec bash -c '[[ $(basename $(dirname {})).gyp = $(basename {}) ]] || \
    echo Need to manually convert {} >&2' \;
}

# Manual work on the other folders has started, so we don't want this script to
# automatically overwrite that work.
exit 0

run_for_folder bot3

run_for_folder aos
run_for_folder frc971
run_for_folder y2014
run_for_folder y2015
run_for_folder vision
