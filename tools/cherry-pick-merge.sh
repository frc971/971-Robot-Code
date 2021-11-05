#!/bin/bash
#
# The script replays a merge based on its first parent.
# This will create a new merge with the same message and second parent, but with
# its first parent as the current commit.
#
# It is advised for now to backup your repo before running this command and
# make sure the git history is what you want until we use it more.

set -e
set -o pipefail

COMMIT=$1

MESSAGE=$(git show -s --format=%B ${COMMIT})

git cherry-pick -m1 ${COMMIT}
git reset --soft HEAD~
echo ${COMMIT}^2 > .git/MERGE_HEAD
git commit --message="${MESSAGE}"
