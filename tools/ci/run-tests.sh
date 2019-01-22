#!/bin/sh
set -e
set -x

TARGETS='//... @com_github_google_glog//... @com_google_ceres_solver//...'

bazel test -c opt --curses=no --color=no ${TARGETS}
bazel build -c opt --curses=no --color=no ${TARGETS} --cpu=roborio
bazel build --curses=no --color=no ${TARGETS} --cpu=armhf-debian
bazel build -c opt --curses=no --color=no //motors/... --cpu=cortex-m4f
