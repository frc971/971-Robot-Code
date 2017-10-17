#!/bin/sh
set -e

bazel --batch test -c opt --curses=no --color=no --jobs=1 //...
bazel --batch build -c opt --curses=no --color=no --jobs=1 //... --cpu=roborio
