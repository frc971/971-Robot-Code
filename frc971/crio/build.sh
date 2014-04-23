#!/bin/bash

cd $(dirname $0)

../../aos/build/build.py --processor crio --main_gyp crio.gyp "$@"
