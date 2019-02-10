#!/bin/sh
set -e
set -x

TARGETS='//... @com_github_google_glog//... @com_google_ceres_solver//...'

# Include --config=eigen to enable Eigen assertions so that we catch potential
# bugs with Eigen.
bazel test -c opt --config=eigen --curses=no --color=no ${TARGETS}
bazel build -c opt --curses=no --color=no ${TARGETS} --cpu=roborio
bazel build --curses=no --color=no ${TARGETS} --cpu=armhf-debian
bazel build -c opt --curses=no --color=no //motors/... --cpu=cortex-m4f
