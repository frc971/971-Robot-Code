#!/bin/bash

# Set default for disk utilisation
readonly DMAX="${1:-80%}"
# Cache folder
readonly CACHE_FOLDER="${HOME}/.cache/bazel/"
readonly BUILDS_FOLDER="${HOME}/builds/${BUILDKITE_AGENT_NAME}/spartan-robotics/"
# Retrieve disk usages in percentage
readonly DSIZE="$(df -hlP "${CACHE_FOLDER}" | sed 1d | awk '{print $5}')"

if [[ ${DSIZE} > ${DMAX} ]]; then
  echo "$(hostname): Disk over ${DMAX} Clean up needed on node."
  # Ensure that everything has permissions such that it can be deleted.
  chmod --recursive 777 ${CACHE_FOLDER}
  chmod --recursive 777 "${BUILDS_FOLDER}"/*_output_base
  rm -rf "${CACHE_FOLDER}"
  # Don't delete the git checkout.
  rm -rf "${BUILDS_FOLDER}"/*_output_base
else
  echo "$(hostname): No clean up needed. Disk usage is at: ${DSIZE}"
fi
