#!/bin/bash

cd $(dirname $0)

../../aos/build/build.sh linux-amd64 prime.gyp no prime-amd64 "$@" || exit 1
../../aos/build/build.sh linux prime.gyp no prime "$@"
