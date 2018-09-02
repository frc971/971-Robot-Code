#!/bin/sh
set -e
set -x

bazel test -c opt --curses=no --color=no //...
bazel build -c opt --curses=no --color=no //... --cpu=roborio
bazel build -c opt --curses=no --color=no //motors/... --cpu=cortex-m4f
bazel build --curses=no --color=no //... --cpu=armhf-debian
