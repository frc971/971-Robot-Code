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

readonly CLANG_FORMAT="$(rlocation llvm_k8/bin/clang-format)"

# Run everything from the root of the tree.
cd "${BUILD_WORKSPACE_DIRECTORY}"

# Find all the C/C++ files in the repo.
# Exclude third-party code. Both in //third_party and the third-party code
# checked in to the main repo directly.
cc_files=($(git ls-tree --name-only --full-tree -r @ \
    | grep -v -e '^third_party/' \
        -e '^motors/core/kinetis.h$' \
        -e '^y2023/vision/rkisp1-config.h$' \
    | (grep -e '\.c$' -e '\.cc$' -e '\.h' || :)))

# If we have any C/C++ files, format them.
if ((${#cc_files[@]} > 0)); then
    exec "${CLANG_FORMAT}" -i "${cc_files[@]}"
fi
