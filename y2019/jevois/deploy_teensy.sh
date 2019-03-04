#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"

bazel build --cpu=cortex-m4f -c opt //y2019/jevois:teensy.hex -s && bazel run //motors/teensy_loader_cli -- --mcu=mk64fx512 -s $(readlink -f ../../bazel-bin/y2019/jevois/teensy.hex)
