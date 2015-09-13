#!/bin/bash

# Checks the output from ruby.rb.

set -e
set -u

OUTPUT="$("./build_tests/ruby_binary" "${PWD}")"

if [[ "${OUTPUT}" != "Hi from ruby" ]]; then
  echo "Output is actually:" >&2
  echo "${OUTPUT}" >&2
  exit 1
fi
