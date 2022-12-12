#!/bin/bash

set -eux
set -o pipefail

PARTITION="$(systemctl show --property What --value -- -.mount | sed 's/\/dev\///')"
PARTITION_NUMBER="$(echo ${PARTITION} | sed 's/^[^p]*p\([0-9]\)/\1/')"
DEVICE="$(echo ${PARTITION} | sed 's/p[0-9]*$//')"

START=$(cat /sys/block/${DEVICE}/${PARTITION}/start)
END=$((${START}+$(cat /sys/block/${DEVICE}/${PARTITION}/size)))
NEWEND=$(($(cat /sys/block/${DEVICE}/size)-8))

if [[ "${NEWEND}" -gt "${END}" ]]; then
  sfdisk --delete "/dev/${DEVICE}" "${PARTITION_NUMBER}"
  sfdisk --force "/dev/${DEVICE}" <<-__EOF__
16M,64M,L,*
80M,,L,*
__EOF__
  partx -u "/dev/${DEVICE}"
fi
