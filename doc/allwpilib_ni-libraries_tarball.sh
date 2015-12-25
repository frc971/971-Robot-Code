#!/bin/bash

# A script to generate a allwpilib_ni-libraries_bla.tar.gz file from a given
# revision of allwpilib.

# Example: ./doc/allwpilib_ni-libraries_tarball.sh \
#   https://usfirst.collab.net/gerrit/allwpilib master

set -e
set -u
set -o pipefail

if [ $# -ne 2 ]; then
  echo "Usage: $0 remote ref" >&2
  exit 1
fi

REMOTE="$1"
REF="$2"

git fetch "${REMOTE}" "${REF}"

git archive \
  -o allwpilib_ni-libraries_$(git rev-parse --short FETCH_HEAD).tar.gz \
  FETCH_HEAD ni-libraries
