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
[[ "$1" == "deploy" ]] && NO_AMD64=1

if [[ ! ${NO_TARGET} ]]; then
	echo 'Building code for target...' 1>&2
	../../aos/build/build.sh linux prime.gyp no prime "$@"
    if [[ $? -ne 0 ]]; then
		echo 'Building code for target failed!' 1>&2
		exit 1
	fi
	echo 'Building code for target succeeded.' 1>&2
fi
if [[ ! ${NO_AMD64} ]]; then
	echo 'Building code for amd64...' 1>&2
	../../aos/build/build.sh linux-amd64 prime.gyp yes prime-amd64 "$@"
    if [[ $? -ne 0 ]]; then
		echo 'Building code for amd64 failed!' 1>&2
		exit 1
	fi
	echo 'Building code for amd64 succeeded.' 1>&2
fi
