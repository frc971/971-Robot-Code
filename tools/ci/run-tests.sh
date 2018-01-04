#!/bin/sh
set -e

bazel --batch test -c opt --curses=no --color=no //...
bazel --batch build -c opt --curses=no --color=no //... --cpu=roborio
bazel --batch build -c opt --curses=no --color=no //motors/... --cpu=cortex-m4f
