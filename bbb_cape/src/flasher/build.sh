#!/bin/bash

cd $(dirname $0)

../../../aos/build/build.sh linux flasher.gyp no flasher "$@"
