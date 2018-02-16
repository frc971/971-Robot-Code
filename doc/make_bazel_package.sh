#!/bin/bash

# This script builds a Debian package from a Bazel source tree with the
# correct version.
# The only argument is the path to a Bazel source tree.

set -e
set -u

BAZEL_SOURCE="$1"

VERSION="$(date +%Y%m%d%H%M)+$(GIT_DIR="${BAZEL_SOURCE}/.git" git rev-parse --short HEAD)"
DEB="bazel_${VERSION}_amd64.deb"

(
cd "${BAZEL_SOURCE}"
bazel build -c opt //scripts/packages/debian:bazel-debian --embed_label="${VERSION}" --stamp=yes --experimental_sandbox_base=/dev/shm
)

cp "${BAZEL_SOURCE}/bazel-bin/scripts/packages/debian/bazel-debian.deb" "${DEB}"

echo "Output is at ${DEB}"
