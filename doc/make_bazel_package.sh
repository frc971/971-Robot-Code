#!/bin/bash

# This script builds a Debian package from a Bazel source tree with the
# correct version.
# The only argument is the path to a Bazel source tree.

set -e
set -u

BAZEL_SOURCE="$1"

VERSION="0.19.0rc4-$(date +%Y%m%d%H%M)+$(GIT_DIR="${BAZEL_SOURCE}/.git" git rev-parse --short HEAD)"
OUTPUT="bazel_${VERSION}"

(
cd "${BAZEL_SOURCE}"
bazel build -c opt //scripts/packages:with-jdk/bazel-real --embed_label="${VERSION}" --stamp=yes --experimental_sandbox_base=/dev/shm
)

cp "${BAZEL_SOURCE}/bazel-genfiles/scripts/packages/with-jdk/bazel-real" "${OUTPUT}"
xz "${OUTPUT}"

echo "Output is at ${OUTPUT}.xz"
