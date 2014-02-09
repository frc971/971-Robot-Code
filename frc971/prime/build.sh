#!/bin/bash

cd $(dirname $0)

if [[ "$1" == "amd64" ]]; then
  NO_TARGET=1
  shift 1
elif [[ "$1" == "arm" ]]; then
  NO_AMD64=1
  shift 1
fi

[[ "$1" == "tests" ]] && NO_TARGET=1

[[ ${NO_TARGET} ]] || ../../aos/build/build.sh linux prime.gyp no prime "$@"
[[ ${NO_AMD64} ]] || \
    ../../aos/build/build.sh linux-amd64 prime.gyp no prime-amd64 "$@"
