#!/bin/bash

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

readonly BUILDIFIER="$(rlocation com_github_bazelbuild_buildtools/buildifier/buildifier_/buildifier)"

# Run everything from the root of the tree.
cd "${BUILD_WORKSPACE_DIRECTORY}"

# Find all the Starlark files in the repo.
starlark_files=($(git ls-tree --name-only --full-tree -r @ \
    | grep -v '^third_party/' \
    | (grep \
        -e '\.bzl$' \
        -e '\.BUILD$' \
        -e '^BUILD$' \
        -e '/BUILD$' \
        -e '/BUILD.bazel$' \
        -e '^WORKSPACE$' \
        || :)))

# If we have any Starlark files, format them.
if ((${#starlark_files[@]} > 0)); then
    "${BUILDIFIER}" --lint=fix "${starlark_files[@]}"
    "${BUILDIFIER}" --lint=warn \
        --warnings=-module-docstring,-function-docstring,-function-docstring-args,-rule-impl-return,-no-effect,-provider-params,-unnamed-macro,-positional-args \
        "${starlark_files[@]}"
fi
