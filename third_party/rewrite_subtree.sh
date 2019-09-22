#!/bin/bash
#
# Coppied from:
#  https://stackoverflow.com/questions/40297499/pushing-squashed-subtree-change-to-gerrit
#
# This command re-writes the history after doing a git subtree {pull|add}
# to add Gerrit Change-Id lines to the squash commit message.
#
# It assumes that HEAD is the merge commit merging the subtree into HEAD.
# The original HEAD commit will be backed up under refs/original, which
# is helpful if something goes wrong.

set -e
set -o pipefail

GIT_DIR=$(readlink -f "$(git rev-parse --git-dir)")
TMP_MSG="${GIT_DIR}/COMMIT_MSG_REWRITE"

git filter-branch --msg-filter \
  "cat > ${TMP_MSG} && \"${GIT_DIR}/hooks/commit-msg\" ${TMP_MSG} && \
  cat \"${TMP_MSG}\"" HEAD...HEAD~1

rm -rf "${TMP_MSG}"
