#!/bin/bash
set -Eeuo pipefail

if [ $# != 1 ];
then
  echo "Usage: setup_robot.sh 971"
  exit 1
fi

readonly ROBOT_HOSTNAME="$1"

echo "Looking to see if l is aliased right."

readonly HAS_ALIAS=$(ssh "admin@${ROBOT_HOSTNAME}" "cat /etc/profile" | grep -Fq "alias l")

if [[ $? -ne 0 ]]; then
  echo "ssh command failed remotely"
  exit 1
elif $HAS_ALIAS
  echo "Already has l alias"
else
  echo "Adding l alias"
  ssh "admin@${ROBOT_HOSTNAME}" 'echo "alias l=\"ls -la\"" >> /etc/profile'
fi

ssh "admin@${ROBOT_HOSTNAME}" 'PATH="${PATH}":/usr/local/natinst/bin/ /usr/local/frc/bin/frcKillRobot.sh -r -t'

echo "Deploying robotCommand startup script"
scp aos/config/robotCommand "admin@${ROBOT_HOSTNAME}:/home/lvuser/"

scp external/arm_frc_linux_gnueabi_repo/usr/arm-frc-linux-gnueabi/lib/libstdc++.so.6.0.21 "admin@${ROBOT_HOSTNAME}:/usr/lib/"
