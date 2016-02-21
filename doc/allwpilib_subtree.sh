#!/bin/bash

# A wrapper around git-subtree which first filters to remove the stuff from
# allwpilib we don't want.
# Also simplifies the interface a bunch to just what we need.

# The implementation running `git filter-branch` over allwpilib's entire history
# isn't the fastest thing ever, but it's not all that bad.

# Example: ./doc/allwpilib_subtree.sh add third_party/allwpilib_2016 \
#   https://usfirst.collab.net/gerrit/allwpilib master

set -e
set -u
set -o pipefail

if [ $# -ne 4 ]; then
  echo "Usage: $0 add|merge|filter prefix remote ref" >&2
  exit 1
fi

COMMAND="$1"
PREFIX="$2"
REMOTE="$3"
REF="$4"

git fetch "${REMOTE}" "${REF}"

readonly REMOVE_DIRECTORIES="ni-libraries wpilibj wpilibjIntegrationTests gradle"
readonly TREE_FILTER="$(for d in ${REMOVE_DIRECTORIES}; do
  echo "if [ -d $d ]; then git rm -rf $d; fi && "
done)"
git filter-branch --tree-filter "${TREE_FILTER}true" FETCH_HEAD

FILTERED_REF="$(git rev-parse FETCH_HEAD)"

if [[ "${COMMAND}" = "filter" ]]; then
  echo "Filtered ref is ${FILTERED_REF}"
  exit 0
fi

exec git subtree "${COMMAND}" --squash --prefix="${PREFIX}" "${FILTERED_REF}"
