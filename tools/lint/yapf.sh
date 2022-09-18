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

YAPF="$(rlocation pip_deps_yapf/rules_python_wheel_entry_point_yapf)"
readonly YAPF

# Run everything from the root of the tree.
cd "${BUILD_WORKSPACE_DIRECTORY}"

# Find all the Python files in the repo.
python_files=($(git ls-tree --name-only --full-tree -r @ \
    | grep -v '^third_party/' \
    | (grep '\.py$' || :)))

# If we have any Python files, format them.
if ((${#python_files[@]} > 0)); then
    exec "${YAPF}" --in-place --parallel --verbose "${python_files[@]}"
fi
