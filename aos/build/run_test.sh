#!/bin/bash

# This gets called by build.sh to run a test.

EXECUTABLE=$1

echo "Running $(basename ${EXECUTABLE})."
${EXECUTABLE}
exit $?
