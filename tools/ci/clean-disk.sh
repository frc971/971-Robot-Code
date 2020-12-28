#!/bin/bash

# Set default for disk utilisation
readonly DMAX="${1:-80%}"
# Cache folder
readonly CACHE_FOLDER="${HOME}/.cache/bazel/disk_cache/"
# Retrieve disk usages in percentage
readonly DSIZE="$(df -hlP "${CACHE_FOLDER}" | sed 1d | awk '{print $5}')"

if [[ ${DSIZE} > ${DMAX} ]]; then
  echo "$(hostname): Disk over ${DMAX} Clean up needed on node."
  rm -rf "${CACHE_FOLDER}"
else
  echo "$(hostname): No clean up needed. Disk usage is at: ${DSIZE}"
fi
