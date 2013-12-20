#!/bin/bash

cd $(dirname $0)

../../aos/build/build.sh crio crio.gyp no "$@"
