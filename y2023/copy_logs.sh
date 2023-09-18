#!/bin/bash

# Helper script to copy most recent logs off of the pis

set -e

ROBOT_PREFIX="79" # ..71  (Should be one of 79, 89, 99, or 9)
PI_LIST="2 3"     # Should be some set of {1,2,3,4,5,6}

LOG_FILE_PATH=/media/sda1/fbs_log-current
if [[ -z $1 || ! -d $1 ]]; then
    echo "Please specify the base directory to store the logs ('$1' not found)"
    exit -1
fi

# Create output directory based on given directory + a timestamp
OUTPUT_DIR=$1"/"`date +"%Y-%m-%dT%H-%M-%S"`
mkdir ${OUTPUT_DIR}

echo "Copying logs from the robot ${ROBOT_PREFIX}71 and pis ${PI_LIST}"
echo "Storing logs in folder ${OUTPUT_DIR}"

for pi in $PI_LIST; do
    echo "========================================================"
    echo "Copying logs from pi-${ROBOT_PREFIX}71-$pi"
    echo "========================================================"
    scp -r pi@10.${ROBOT_PREFIX}.71.10${pi}:${LOG_FILE_PATH} ${OUTPUT_DIR}/fbs_log-pi${pi}
done

