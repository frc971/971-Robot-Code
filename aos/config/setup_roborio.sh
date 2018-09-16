#!/bin/bash
#
# Note: this should be run from within bazel

set -Eeuo pipefail

if [ $# != 1 ];
then
  echo "# setup_robot.sh is used to configure a newly flashed roboRIO"
  echo ""
  echo "Usage: setup_roborio.sh 10.9.71.2"
  echo ""
  echo "# or if that does not work, try"
  echo ""
  echo "Usage: setup_roborio.sh roboRIO-971-frc.local"
  exit 1
fi

readonly ROBOT_HOSTNAME="$1"

echo "Looking to see if l is aliased right."

if ! HAS_ALIAS=$(ssh "admin@${ROBOT_HOSTNAME}" "cat /etc/profile"); then
  echo "ssh command failed remotely"
  exit 1
elif echo "${HAS_ALIAS}" | grep -Fq "alias l"; then
  echo "Already has l alias"
else
  echo "Adding l alias"
  ssh "admin@${ROBOT_HOSTNAME}" 'echo "alias l=\"ls -la\"" >> /etc/profile'
  echo "Adding symbolic link to loging directory"
  ssh "admin@${ROBOT_HOSTNAME}" ln -s /media/sda1 logs
fi

ssh "admin@${ROBOT_HOSTNAME}" 'PATH="${PATH}":/usr/local/natinst/bin/ /usr/local/frc/bin/frcKillRobot.sh -r -t'

echo "Deploying robotCommand startup script"
scp aos/config/robotCommand "admin@${ROBOT_HOSTNAME}:/home/lvuser/"

echo "Copying libstdc++.so.6.0.21 library to /usr/lib"
scp external/arm_frc_linux_gnueabi_repo/usr/arm-frc-linux-gnueabi/lib/libstdc++.so.6.0.21 "admin@${ROBOT_HOSTNAME}:/usr/lib/"
