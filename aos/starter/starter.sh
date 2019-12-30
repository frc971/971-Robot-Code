#!/bin/bash

# NI already has a core pattern, so we probably shouldn't change it.
#echo '/home/driver/tmp/robot_logs/%e-%s-%p-%t.coredump' > /proc/sys/kernel/core_pattern

ln -s /var/local/natinst/log/FRC_UserProgram.log /tmp/FRC_UserProgram.log
ROBOT_CODE="/home/admin/robot_code"
cd "${ROBOT_CODE}"
export PATH="${PATH}:${ROBOT_CODE}"
while true; do
	starter_exe $ROBOT_CODE/start_list.txt 2>&1
done
