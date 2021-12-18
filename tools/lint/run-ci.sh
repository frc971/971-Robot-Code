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

set -o nounset

# Redirect the Go cache on buildkite. Otherwise we run into errors like:
# "failed to initialize build cache at /var/lib/buildkite-agent/.cache/go-build"
# due to permission errors.
if ((RUNNING_IN_CI == 1)); then
    export GOCACHE=/tmp/lint_go_cache
fi

gofmt() {
    ./tools/lint/gofmt
}

gomod() {
    local -r go="$(readlink -f external/go_sdk/bin/go)"
    cd "${BUILD_WORKSPACE_DIRECTORY}"
    "${go}" mod tidy -e
}

update_repos() {
    ./gazelle-runner.bash update-repos \
        -from_file=go.mod \
        -to_macro=go_deps.bzl%go_dependencies \
        -prune
}

gazelle() {
    ./gazelle-runner.bash
}

tweak_gazelle_go_deps() {
    local -r tweaker="$(readlink -f tools/go/tweak_gazelle_go_deps)"
    cd "${BUILD_WORKSPACE_DIRECTORY}"
    "${tweaker}" ./go_deps.bzl
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
    gomod
    update_repos
    gazelle
    tweak_gazelle_go_deps
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
