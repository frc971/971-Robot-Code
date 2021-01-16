#!/bin/sh
set -e
set -x

# No need to pass this through.  It hurts caching.
unset SSH_AUTH_SOCK

if [ -n ${BUILDKITE+x} ]; then
  buildkite-agent pipeline upload tools/ci/buildkite.yaml
  exit 0
fi

echo "This script should only be run from buildkite."
exit 1
