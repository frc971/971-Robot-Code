#!/bin/bash

# NI already has a core pattern, so we probably shouldn't change it.
#echo '/home/driver/tmp/robot_logs/%e-%s-%p-%t.coredump' > /proc/sys/kernel/core_pattern

ROBOT_CODE=/home/admin/robot_code
cd $ROBOT_CODE
while true; do
	export PATH=$PATH:$ROBOT_CODE
	starter_exe $ROBOT_CODE/start_list.txt
done
