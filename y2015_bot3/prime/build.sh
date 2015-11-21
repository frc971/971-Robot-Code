#!/bin/bash

cd $(dirname $0)

exec ../../aos/build/build.py $0 prime bot3 prime.gyp "$@"
