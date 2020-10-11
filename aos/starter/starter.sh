#!/bin/bash


if [[ "$(hostname)" == "roboRIO"* ]]; then
  ROBOT_CODE="/home/admin/robot_code"

  # Configure throttling so we reserve 5% of the CPU for non-rt work.
  # This makes things significantly more stable.
  echo 950000 > /proc/sys/kernel/sched_rt_runtime_us
  echo 1000000 > /proc/sys/kernel/sched_rt_period_us

  ln -s /var/local/natinst/log/FRC_UserProgram.log /tmp/FRC_UserProgram.log
  ln -s /var/local/natinst/log/FRC_UserProgram.log "${ROBOT_CODE}/FRC_UserProgram.log"
elif [[ "$(hostname)" == "pi-"* ]]; then
  ROBOT_CODE="/home/pi/robot_code"

else
  ROBOT_CODE="${HOME}/robot_code"
fi

cd "${ROBOT_CODE}"
export PATH="${PATH}:${ROBOT_CODE}"
while true; do
	starter_exe $ROBOT_CODE/start_list.txt 2>&1
done
