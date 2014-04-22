#!/bin/bash
#set -x
# TODO(brians): This needs to be a python script that still deals with
# stdout/stderr correctly but also has real argument parsing.
# That should probably fold most of the other shell scripts in too...

if [[ "$1" == "amd64" ]]; then
  NO_TARGET=1
  shift 1
elif [[ "$1" == "arm" ]]; then
  NO_AMD64=1
  shift 1
fi

[[ "$1" == "tests" ]] && NO_TARGET=1
[[ "$1" == "deploy" ]] && NO_AMD64=1

OTHER_ARGS="$@"

build_platform() {
  PLATFORM=$1
  OUT_NAME=$2
  HUMAN_NAME=$3

  echo "Building code with clang for ${HUMAN_NAME}..." 1>&2
  ../../aos/build/build.sh ${PLATFORM}-clang prime.gyp no ${OUT_NAME}-clang "${OTHER_ARGS}"
  if [[ $? -ne 0 ]]; then
    echo "Building code with clang for ${HUMAN_NAME} failed!" 1>&2
    exit 1
  fi
  echo "Building code with clang for ${HUMAN_NAME} succeeded." 1>&2

  echo "Building code for ${HUMAN_NAME}..." 1>&2
  ../../aos/build/build.sh ${PLATFORM} prime.gyp no ${OUT_NAME} "${OTHER_ARGS}"
  if [[ $? -ne 0 ]]; then
    echo "Building code for ${HUMAN_NAME} failed!" 1>&2
    exit 1
  fi
  echo "Building code for ${HUMAN_NAME} succeeded." 1>&2
}

if [[ ! ${NO_TARGET} ]]; then
  build_platform linux prime target
fi
if [[ ! ${NO_AMD64} ]]; then
  build_platform linux-amd64 prime-amd64 amd64
fi
