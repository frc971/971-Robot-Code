#!/bin/bash
#set -x

# Downloads code to the prime in a way that avoids clashing too badly with
# starter.
# Requires 1 argument (the directory to download).

GET_IP=$(dirname $0)/get_ip
SUM=md5sum
FROM_DIR=$1
TO_DIR=/home/driver/robot_code/bin
TMPDIR=/tmp/aos_downloader
TARGET=driver@$(${GET_IP} prime)

SUMS=$(cd ${FROM_DIR} && ${SUM} *)

TO_DOWNLOAD=$(ssh ${TARGET} "rm -rf ${TMPDIR} && mkdir ${TMPDIR} && cd ${TO_DIR} && echo '${SUMS}' | ${SUM} --check --quiet |& grep -F FAILED | sed 's/^\\(.*\\): FAILED"'$'"/\\1/g'")
if [[ $? != 0 ]]; then
	echo 'Connecting to target failed.'
	exit 1
fi

if [[ -z "${TO_DOWNLOAD}" ]]; then
	echo "Nothing to download"
	exit 0
fi

# Compression seems to make it go faster even when the network isn't the
# bottleneck with a BBB. Maybe it's because ethernet on the BBB uses so much
# CPU?
( cd ${FROM_DIR} && scp -o "Compression yes" ${TO_DOWNLOAD} ${TARGET}:${TMPDIR} )
if [[ $? != 0 ]]; then
	echo 'Copying files into /tmp on target failed.'
	exit 1
fi
ssh ${TARGET} "mv ${TMPDIR}/* ${TO_DIR} && echo 'Done moving new executables into place' && ionice -c 3 bash -c 'sync && sync && sync'"
exit $?
