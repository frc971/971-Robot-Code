#!/bin/sh
set -e
set -x

TARGETS='//... @com_github_google_glog//... @com_google_ceres_solver//...'

# Include --config=eigen to enable Eigen assertions so that we catch potential
# bugs with Eigen.
bazel test -c opt --stamp=no --config=eigen --curses=no --color=no ${TARGETS}
bazel build -c opt --stamp=no --curses=no --color=no ${TARGETS} --cpu=roborio
bazel build --stamp=no --curses=no --color=no ${TARGETS} --cpu=armhf-debian
bazel build -c opt --stamp=no --curses=no --color=no \
    //motors/... //y2019/jevois/... --cpu=cortex-m4f
