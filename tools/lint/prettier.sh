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

# The rules_nodejs runfiles discovery needs some help.
runfiles_export_envvars
export RUNFILES="${RUNFILES_DIR}"

PRETTIER="$(rlocation org_frc971/tools/lint/prettier_binary.sh)"
PRETTIER="$(readlink -f "${PRETTIER}")"
readonly PRETTIER

# Find the files to format at the root of the repo. For some reason, the
# prettier binary (probably because of the rules_nodejs wrapper), breaks when
# invoked from a directory outside of the runfiles tree.
pushd "${BUILD_WORKSPACE_DIRECTORY}"

# Find web-related files in the repo.
# TODO(phil): Support more than just //scouting.
web_files=($(git ls-tree --name-only --full-tree -r @ \
    | grep '^scouting/' \
    | (grep \
        -e '\.ts$' \
        -e '\.js$' \
        -e '\.css$' \
        -e '\.html$' \
        || :)))

web_files_abs_paths=($(for file in "${web_files[@]}"; do
  readlink -f "${file}"
done))

CONFIG_FILE="$(readlink -f .prettierrc.json)"
readonly CONFIG_FILE

# Go back to the runfiles directory so that prettier doesn't break.
popd

# If we have any web-related files, format them.
if ((${#web_files_abs_paths[@]} > 0)); then
    "${PRETTIER}" \
        --config "${CONFIG_FILE}" \
        --write \
        "${web_files_abs_paths[@]}"
fi
