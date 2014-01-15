#!/bin/bash

cd $(dirname $0)

../../aos/build/build.sh linux prime.gyp no prime "$@"
