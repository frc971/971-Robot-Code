#!/bin/bash

# NI already has a core pattern, so we probably shouldn't change it.
#echo '/home/driver/tmp/robot_logs/%e-%s-%p-%t.coredump' > /proc/sys/kernel/core_pattern

while true; do
	export PATH=$PATH:/home/admin/robot_code
	starter_exe /home/admin/robot_code/start_list.txt
done
