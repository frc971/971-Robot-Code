#!/bin/bash

set -o errexit
set -o nounset
set -o pipefail

# --- begin runfiles.bash initialization v2 ---
# Copy-pasted from the Bazel Bash runfiles library v2.
set -uo pipefail; set +e; f=bazel_tools/tools/bash/runfiles/runfiles.bash
source "${RUNFILES_DIR:-/dev/null}/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "${RUNFILES_MANIFEST_FILE:-/dev/null}" | cut -f2- -d' ')" 2>/dev/null || \
  source "$0.runfiles/$f" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  source "$(grep -sm1 "^$f " "$0.exe.runfiles_manifest" | cut -f2- -d' ')" 2>/dev/null || \
  { echo>&2 "ERROR: cannot find $f"; exit 1; }; f=; set -e
# --- end runfiles.bash initialization v2 ---

DOCKERFILE="$(rlocation org_frc971/tools/python/update_helper_files/Dockerfile)"
CONTEXT_DIR="$(dirname "${DOCKERFILE}")"
CONTAINER_TAG="pip-lock:${USER}"

# Build the container that has the bare minimum to run the various setup.py
# scripts from our dependencies.
docker build \
  --file="${DOCKERFILE}" \
  --tag="${CONTAINER_TAG}" \
  "${CONTEXT_DIR}"

# Run the actual update. The assumption here is that mounting the user's home
# directory is sufficient to allow the tool to run inside the container without
# any issues. I.e. the cache and the source tree are available in the
# container.
docker run \
  --rm \
  --tty \
  --env BUILD_WORKSPACE_DIRECTORY="${BUILD_WORKSPACE_DIRECTORY}" \
  --workdir "${PWD}" \
  --volume "${HOME}:${HOME}" \
  "${CONTAINER_TAG}" \
  "$@"
