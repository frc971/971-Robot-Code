#!/bin/bash

# This program hides all the output on stderr from the called command, unless it
# fails, in which case all the output is printed at the end.

set -e
set -u

readonly STDERR_FILE="$(mktemp)"

if ! "$@" 2>"${STDERR_FILE}" ; then
  cat "${STDERR_FILE}"
  exit 1
fi
