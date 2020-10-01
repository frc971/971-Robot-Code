#!/bin/bash

# Tests DetachedBufferWriter's behavior when running out of disk space. We do
# this by creating a small tmpfs in a new mount namespace, and then running a
# test binary in that namespace. There's no easy way for a process with user
# code to run itself in a new mount+user namespace, so we do that outside the
# test process itself.

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

set -euo pipefail

TMPFS="${TEST_TMPDIR}/small_tmpfs"
rm -rf "${TMPFS}"
mkdir "${TMPFS}"

function test {
  SIZE="$1"
  echo "Running test with ${SIZE}..." >&2
  unshare --mount --map-root-user bash <<END
set -euo pipefail

mount -t tmpfs tmpfs -o size=${SIZE} "${TMPFS}"

exec "$(rlocation org_frc971/aos/events/logging/logfile_utils_out_of_space_test_runner)" -tmpfs "${TMPFS}"
END
  echo "Finished test with ${SIZE}" >&2
}

# Run out of space exactly at the beginning of a block.
test 81920

# Run out of space 1 byte into a block.
test 81921

# Run out of space in the middle of a block.
test 87040
