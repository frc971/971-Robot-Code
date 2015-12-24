#!/bin/bash

# NI already has a core pattern, so we probably shouldn't change it.
#echo '/home/driver/tmp/robot_logs/%e-%s-%p-%t.coredump' > /proc/sys/kernel/core_pattern

# Just allow overcommit memory like usual. Various processes map memory they
# will never use, and the roboRIO doesn't have enough RAM to handle it.
echo 0 > /proc/sys/vm/overcommit_memory

while true; do
	export PATH=$PATH:/home/admin/robot_code
	starter_exe /home/admin/robot_code/start_list.txt
done
