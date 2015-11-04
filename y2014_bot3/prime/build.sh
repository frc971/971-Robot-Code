#!/bin/bash

cd $(dirname $0)

exec ../../aos/build/build.py $0 bot3_prime prime.gyp "$@"
