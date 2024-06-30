#!/bin/bash

# Helper script to set orins clock based on our local timestamp

set -e

ROBOT_PREFIX="9" #71  (Should be one of 79, 89, 99, or 9)

ORIN_LIST="1 2"

echo "Setting hwclock on Orins"

for orin in $ORIN_LIST; do
    echo "========================================================"
    echo "Setting clock for 10.${ROBOT_PREFIX}.71.10${orin}"
    echo "========================================================"
    current_time=`sudo hwclock`
    IFS="."
    read -ra split_time <<< "$current_time"
    echo "Setting time to ${split_time[0]}"
    ssh pi@10.${ROBOT_PREFIX}.71.10${orin} "sudo timedatectl set-timezone US/Pacific"
    ssh pi@10.${ROBOT_PREFIX}.71.10${orin} "sudo hwclock --set --date \"${split_time[0]}\""
    ssh pi@10.${ROBOT_PREFIX}.71.10${orin} "sudo hwclock -s"
done
