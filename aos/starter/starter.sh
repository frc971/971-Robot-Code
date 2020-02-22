#!/bin/bash


if [[ "$(hostname)" == "roboRIO"* ]]; then
  ROBOT_CODE="/home/admin/robot_code"

  ln -s /var/local/natinst/log/FRC_UserProgram.log /tmp/FRC_UserProgram.log
  ln -s /var/local/natinst/log/FRC_UserProgram.log "${ROBOT_CODE}/FRC_UserProgram.log"
else
  ROBOT_CODE="${HOME}/robot_code"
fi

cd "${ROBOT_CODE}"
export PATH="${PATH}:${ROBOT_CODE}"
while true; do
	starter_exe $ROBOT_CODE/start_list.txt 2>&1
done
