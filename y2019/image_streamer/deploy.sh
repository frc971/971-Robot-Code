#!/bin/bash

set -e

echo ""
echo "USAGE: $0 ODROID_ip_address"
echo "Example: $0 10.9.71.179"
echo "Example: $0 10.99.71.179"
echo ""

if [ $# != 1 ]
  then
    echo "Illegal number of parameters"
    exit
fi

if [[ $1 == -*[hH]* ]]
  then
    exit
fi

# Get the script directory (from https://devhints.io/bash)
DIR="${0%/*}"

# Move into the script directory
cd "${DIR}"
echo "# Working in `pwd`"

ODROID_IP_ADDRESS=$1
ODROID="root@${ODROID_IP_ADDRESS}"
# Get the IP address of the roboRIO from the ODROID IP address
# This is needed to properly configure supervisorctl on the ODROID
# for image_streamer to communicate with the roboRIO.
ROBORIO=`echo ${ODROID_IP_ADDRESS} | sed 's/\.[0-9]*$/.2/'`

echo "# Using ODORID  ${ODROID}"
echo "# Using roboRIO ${ROBORIO}"

# This builds the ODROID image_streamer code.
echo -e "\n# Building image_streamer"
(
set -x
bazel build -c opt //y2019/image_streamer:image_streamer --cpu=armhf-debian
)

echo -e "\n# Copy files to ODROID"
(
set -x
rsync -av --progress ../../bazel-bin/y2019/image_streamer/image_streamer "${ODROID}":.
rsync -av --progress README_ODROID_setup.txt "${ODROID}":.
rsync -av --progress vision.conf "${ODROID}":/etc/supervisor/conf.d/
)

echo "# Make sure supervisorctl has the correct IP address."
(
set -x
ssh "${ODROID}" sed -i -e "'/image_streamer/ s/10.9.71.2/${ROBORIO}/'" /etc/supervisor/conf.d/vision.conf

ssh "${ODROID}" sync
)

echo -e "\nCan't restart image_streamer with supervisorctl because the USB devices don't come up reliably..." >&2
echo "Restart the ODROID now" >&2
