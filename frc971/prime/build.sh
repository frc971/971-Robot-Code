#!/bin/bash

cd $(dirname $0)

../../aos/build/build.py --processor prime --main_gyp prime.gyp "$@"
