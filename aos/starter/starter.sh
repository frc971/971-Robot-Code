#!/bin/bash

set -x

if [[ "$(hostname)" == "roboRIO"* ]]; then
  /usr/local/natinst/etc/init.d/systemWebServer stop

  ROBOT_CODE="/home/admin/bin"
  cd "${ROBOT_CODE}"

  ln -s /var/local/natinst/log/FRC_UserProgram.log /tmp/FRC_UserProgram.log
  ln -s /var/local/natinst/log/FRC_UserProgram.log "${ROBOT_CODE}/FRC_UserProgram.log"
elif [[ "$(hostname)" == "pi-"* ]]; then
  # We have systemd configured to handle restarting, so just exec.
  export PATH="${PATH}:/home/pi/bin"
  rm -rf /dev/shm/aos
  exec starterd
else
  ROBOT_CODE="${HOME}/bin"
fi

cd "${ROBOT_CODE}"
export PATH="${PATH}:${ROBOT_CODE}"
while true; do
  rm -rf /dev/shm/aos
  starterd 2>&1
done
