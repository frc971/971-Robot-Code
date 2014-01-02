#!/bin/bash

cd $(dirname $0)

../../aos/build/build.sh atom atom_code.gyp no "$@"
