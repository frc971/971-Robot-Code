#!/bin/sh
set -e
set -x

# No need to pass this through.  It hurts caching.
unset SSH_AUTH_SOCK

if [ -n ${BUILDKITE+x} ]; then
  buildkite-agent pipeline upload tools/ci/buildkite.yaml
  exit 0
fi

readonly TARGETS='//... @com_github_google_glog//... @com_google_ceres_solver//...'
readonly M4F_TARGETS='//...'
# Sanity check that we are able to build the y2020 roborio code, which confirms
# that we have the platform compatibility for the roborio set up correctly.
readonly ROBORIO_TARGETS="${TARGETS} //y2020:download_stripped"
# Stay up forever and use inotify to find changes quickly
readonly STARTUP='--max_idle_secs=0 --watchfs'
readonly COMMON='-c opt --stamp=no --curses=no --color=no --symlink_prefix=/ --disk_cache=~/.cache/bazel/disk_cache/'

# Put everything in different output bases so we can get 5 bazel servers
# running and keep them all warm.

# Include --config=eigen to enable Eigen assertions so that we catch potential
# bugs with Eigen.
tools/bazel ${STARTUP} --output_base=../k8_output_base test \
    ${COMMON} \
    --config=k8 \
    --config=eigen \
    ${TARGETS}

tools/bazel ${STARTUP} --output_base=../roborio_output_base build \
    ${COMMON} \
    --config=roborio \
    ${ROBORIO_TARGETS}

tools/bazel ${STARTUP} --output_base=../armhf-debian_output_base build \
    ${COMMON} \
    --config=armhf-debian \
    ${TARGETS}

tools/bazel ${STARTUP} --output_base=../cortex-m4f_output_base build \
    ${COMMON} \
    --config=cortex-m4f \
    ${M4F_TARGETS}
