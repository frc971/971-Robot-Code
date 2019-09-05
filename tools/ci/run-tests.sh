#!/bin/sh
set -e
set -x

readonly TARGETS='//... @com_github_google_glog//... @com_google_ceres_solver//...'
readonly M4F_TARGETS='//...'
readonly COMMON='-c opt --stamp=no --curses=no --color=no --symlink_prefix=/'

# Put everything in different output bases so we can get 5 bazel servers
# running and keep them all warm.

# Include --config=eigen to enable Eigen assertions so that we catch potential
# bugs with Eigen.
bazel --output_base=../k8_output_base test \
    ${COMMON} \
    --cpu=k8 \
    --config=eigen \
    ${TARGETS}

bazel --output_base=../roborio_output_base build \
    ${COMMON} \
    --cpu=roborio \
    ${TARGETS}

bazel --output_base=../armhf-debian_output_base build \
    ${COMMON} \
    --cpu=armhf-debian \
    ${TARGETS}

bazel --output_base=../cortex-m4f_output_base build \
    ${COMMON} \
    --cpu=cortex-m4f \
    ${M4F_TARGETS}

bazel --output_base=../web_output_base build \
    ${COMMON} \
    --cpu=web \
    ${TARGETS}
