#!/bin/bash
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

readonly HAS_ALIAS=$(ssh "admin@${ROBOT_HOSTNAME}" "cat /etc/profile" | grep -Fq "alias l")

if [[ $? -ne 0 ]]; then
  echo "ssh command failed remotely"
  exit 1
elif $HAS_ALIAS
  echo "Already has l alias"
else
  echo "Adding l alias"
  ssh "admin@${ROBOT_HOSTNAME}" 'echo "alias l=\"ls -la\"" >> /etc/profile'
  echo "Adding symbolic link to loging directory"
  ssh "admin@${ROBOT_HOSTNAME}" ln -s /media/sda1 logs
fi

# Make sure starter.sh has the correct permissions to run the robot code.
# If missing o+rx, the robot code will not start.  No error messages on
# some driver stations.
ssh "admin@${ROBOT_HOSTNAME}" 'chmod go+rx robot_code/starter.sh'

ssh "admin@${ROBOT_HOSTNAME}" 'PATH="${PATH}":/usr/local/natinst/bin/ /usr/local/frc/bin/frcKillRobot.sh -r -t'

echo "Deploying robotCommand startup script"
scp aos/config/robotCommand "admin@${ROBOT_HOSTNAME}:/home/lvuser/"

echo "Copying libstdc++.so.6.0.21 library to /usr/lib"
scp external/arm_frc_linux_gnueabi_repo/usr/arm-frc-linux-gnueabi/lib/libstdc++.so.6.0.21 "admin@${ROBOT_HOSTNAME}:/usr/lib/"
