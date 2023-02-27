#!/bin/bash

set -o errexit
set -o nounset
set -o pipefail

bazel test \
  --test_env=DISPLAY="${DISPLAY}" \
  --strategy=TestRunner=processwrapper-sandbox \
  --test_output=streamed \
  --test_arg=--headed \
  --test_timeout=9999 \
  "$@"
