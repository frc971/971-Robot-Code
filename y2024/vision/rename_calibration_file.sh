#!/bin/bash

# Helper script to rename the camera calibration file when moving to new robot

# grep isn't happy with set
# set -e


usage_and_exit () {
    echo
    echo "Usage:"
    echo "$0 ORIG_FILENAME NEW_TEAM_NUMBER NEW_ORIN_NUMBER NEW_CAMERA_NUMBER"
    echo
    exit 2
}

if [[ $# -ne 4 ]]; then
    echo "ERROR: Requires 4 parameters"
    usage_and_exit
fi

ORIG_FILENAME=$1
NEW_TEAM_NUMBER=$2
NEW_ORIN_NUMBER=$3
NEW_CAMERA_NUMBER=$4

if [[ ! -x ${ORIG_FILENAME} ]]; then
    echo "${ORIG_FILENAME} does not exist"
    usage_and_exit
fi

check_971=`echo "${NEW_TEAM_NUMBER}" | grep "971"`
if [[ ${check_971} == "" ]]; then
    echo "NEW_TEAM_NUMBER (${NEW_TEAM_NUMBER}) does not contain '971'"
    usage_and_exit
fi

if [[ ${NEW_ORIN_NUMBER} != 1 && ${NEW_ORIN_NUMBER} != 2 ]]; then
    echo "NEW_ORIN_NUMBER (${NEW_ORIN_NUMBER}) must be either 1 or 2"
    usage_and_exit
fi

if [[ ${NEW_CAMERA_NUMBER} != 0 && ${NEW_CAMERA_NUMBER} != 1 ]]; then
    echo "NEW_CAMERA_NUMBER (${NEW_CAMERA_NUMBER}) must be either 0 or 1"
    usage_and_exit
fi

# Extract parts of the filename, based on just the basename
# This assumes filenames of the form:
# calibration_orin-971-1-0_cam-24-01_2024-02-07_20-11-35.566609408.json
IFS='_' read -r -a name_parts <<< `basename "${ORIG_FILENAME}"`

echo "For ${ORIG_FILENAME}:"
for element in "${name_parts[@]}"
do
    echo "$element"
done

# Rename file based on this new info (be sure to handle paths properly)
NEW_FILENAME=`dirname ${ORIG_FILENAME}`"/${name_parts[0]}_orin-${NEW_TEAM_NUMBER}-${NEW_ORIN_NUMBER}-${NEW_CAMERA_NUMBER}_${name_parts[2]}_${name_parts[3]}_${name_parts[4]}"

echo
echo "For camera id: ${name_parts[2]}"
echo "Renaming from:"
echo "${ORIG_FILENAME} to: "
echo "${NEW_FILENAME}"
echo
echo "and changing from "
echo "${name_parts[1]} to: "
echo "orin-${NEW_TEAM_NUMBER}-${NEW_ORIN_NUMBER}-${NEW_CAMERA_NUMBER}"
echo 

mv ${ORIG_FILENAME} ${NEW_FILENAME}


echo "REPLACING ORIN_NUMBER"
sed -i s/orin./orin${NEW_ORIN_NUMBER}/ ${NEW_FILENAME}

echo "Replacing TEAM NUMBER"
sed -i s/\"team_number\"\:\ [1-9]*\,/\"team_number\"\:\ ${NEW_TEAM_NUMBER},/ ${NEW_FILENAME}

echo "REPLACING CAMERA_NUMBER"
sed -i s/\"camera_number\"\:\ [0-9]/\"camera_number\"\:\ ${NEW_CAMERA_NUMBER}/ ${NEW_FILENAME}



