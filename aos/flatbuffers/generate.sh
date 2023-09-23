#!/bin/bash

# Wrapper script to handle codegen for the static flatbuffer API. The actual
# work is done in calling the generate C++ script, but we also clang-format
# the output headers so that they are not completely unintelligible.

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

BFBS_FILE=$1
OUT_FILE=$2

$(rlocation org_frc971/aos/flatbuffers/generate) --reflection_bfbs "${BFBS_FILE}" --output_file "${OUT_FILE}"
$(rlocation llvm_k8/bin/clang-format) --style=file:"$(rlocation org_frc971/.clang-format)" -i "${OUT_FILE}"
