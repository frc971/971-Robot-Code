#!/bin/sh
set -e

bazel --batch test -c opt --curses=no --color=no --jobs=1 //... -- $(cat NO_BUILD_AMD64)
bazel --batch build -c opt --curses=no --color=no --jobs=1 //... --cpu=roborio -- $(cat NO_BUILD_ROBORIO)
