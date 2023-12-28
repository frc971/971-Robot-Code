#!/bin/bash
tools/bazel run //tools/rehosting:rehost -- "$(buildkite-agent meta-data get dependency-url)"
