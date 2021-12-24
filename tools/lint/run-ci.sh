#!/bin/bash

# Perform runfile initialization here so that child processes have a proper
# RUNFILES_DIR variable set.

# --- begin runfiles.bash initialization v2 ---
# Copy-pasted from the Bazel Bash runfiles library v2.
set -uo pipefail; f=bazel_tools/tools/bash/runfiles/runfiles.bash
source "${RUNFILES_DIR:-/dev/null}/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "${RUNFILES_MANIFEST_FILE:-/dev/null}" | cut -f2- -d' ')" 2>/dev/null || \
  source "$0.runfiles/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.exe.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  { echo>&2 "ERROR: cannot find $f"; exit 1; }; f=; set -e
# --- end runfiles.bash initialization v2 ---

gofmt() {
    ./tools/lint/gofmt
}

git_status_is_clean() {
    cd "${BUILD_WORKSPACE_DIRECTORY}"
    if ! git diff --quiet; then
        echo "One or more linters appear to have made changes to your code!" >&2
        return 1
    fi
}

# All the linters that we are going to run.
readonly -a LINTERS=(
    gofmt
    git_status_is_clean  # This must the last linter.
)

failure=0
for linter in "${LINTERS[@]}"; do
    if ! (eval "${linter}"); then
        failure=1
    fi
done

if ((failure != 0)); then
    echo "One or more linters failed." >&2
    cd "${BUILD_WORKSPACE_DIRECTORY}"
    git --no-pager diff || :
    exit "${failure}"
fi

exit 0
