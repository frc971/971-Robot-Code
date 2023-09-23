#!/bin/bash

OUTPUT=${BUILD_WORKSPACE_DIRECTORY}/aos/flatbuffers/test_dir/sample_test_static.h
cp $1 ${OUTPUT}
chmod 644 ${OUTPUT}
