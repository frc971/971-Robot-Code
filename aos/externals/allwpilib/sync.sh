#!/bin/bash -e

# This script copies the correct directories over from an allwpilib git
# repository (which needs to have a working tree).
# THIS WILL ERASE ANY LOCAL CHANGES TO THE WPILIB SOURCES!

# Takes 1 argument: the folder to copy from.

cd $(dirname $0)

SOURCE=$1
DIRS="wpilibc hal ni-libraries"

[ -n "${SOURCE}" ] || ( echo "Need a directory to sync from!" ; exit 1 )
[ -d "${SOURCE}" ] || ( echo "${SOURCE} is not a directory. Aborting!" ; exit 1 )

for dir in ${DIRS}; do
	[ -d ${dir} ] && rm -r ${dir}
	cp -r ${SOURCE}/${dir} ./
done
