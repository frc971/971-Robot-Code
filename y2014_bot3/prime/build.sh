#!/bin/bash

cd $(dirname $0)

exec ../../aos/build/build.py $0 prime y2014_bot3 prime.gyp "$@"
