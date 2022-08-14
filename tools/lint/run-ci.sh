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

update_go_repos() {
    # Clear out the go_deps.bzl file so that gazelle won't hesitate to update
    # it. Without this step gazelle would never try to remove a dependency.
    cat > "${BUILD_WORKSPACE_DIRECTORY}"/go_deps.bzl <<EOF
def go_dependencies():
    pass
EOF
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

clean_up_go_mirrors() {
    ./tools/go/mirror_go_repos --prune
}

rustfmt() {
    ./tools/lint/rustfmt
}

cargo_raze() {
    local -r cargo_raze="$(readlink -f external/cargo_raze/impl/cargo_raze_bin)"
    export CARGO="$(readlink -f external/rust__x86_64-unknown-linux-gnu_tools/bin/cargo)"
    export RUSTC="$(readlink -f external/rust__x86_64-unknown-linux-gnu_tools/bin/rustc)"
    cd "${BUILD_WORKSPACE_DIRECTORY}"
    # Note we don't run with --generate-lockfile here. If there's a new
    # dependency, we don't want to download it, just failing with an error
    # is sufficient.
    "${cargo_raze}" --manifest-path=Cargo.toml
}

tweak_cargo_raze() {
    local -r tweaker="$(readlink -f tools/rust/tweak_cargo_raze_output)"
    cd "${BUILD_WORKSPACE_DIRECTORY}"
    "${tweaker}" .
}

buildifier() {
    ./tools/lint/buildifier
}

prettier() {
    ./tools/lint/prettier
}

yapf() {
    ./tools/lint/yapf
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
    update_go_repos
    gazelle
    tweak_gazelle_go_deps
    clean_up_go_mirrors
    rustfmt
    cargo_raze
    tweak_cargo_raze
    buildifier
    prettier
    yapf
    git_status_is_clean  # This must the last linter.
)

failure=0
for linter in "${LINTERS[@]}"; do
    echo "Running ${linter}..." >&2
    if ! (eval "${linter}"); then
        echo "LINTER FAILURE: ${linter}" >&2
        failure=1
    else
        echo "${linter} succeeded" >&2
    fi
done

if ((failure != 0)); then
    echo "One or more linters failed." >&2
    cd "${BUILD_WORKSPACE_DIRECTORY}"
    git --no-pager diff || :
    exit "${failure}"
fi

exit 0
