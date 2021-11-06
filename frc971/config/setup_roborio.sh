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
  ssh "admin@${ROBOT_HOSTNAME}" mkdir robot_code
  ssh "admin@${ROBOT_HOSTNAME}" ln -s /media/sda1/aos_log-current robot_code/aos_log-current
  echo "Adding aos_dump autocomplete to profile"
  ssh "admin@${ROBOT_HOSTNAME}" 'echo "if [ -f /home/admin/robot_code/aos_dump_autocomplete.sh ]; then source /home/admin/robot_code/aos_dump_autocomplete.sh; fi;" >> /etc/profile'
  ssh "admin@${ROBOT_HOSTNAME}" 'echo "export PATH=\"\${PATH}:/home/admin/robot_code:/home/admin/bin\"" >> /etc/profile'
fi

ssh "admin@${ROBOT_HOSTNAME}" "sed -i 's/vm\.overcommit_memory=2/vm\.overcommit_memory=0/' /etc/sysctl.conf"

ssh "admin@${ROBOT_HOSTNAME}" 'echo "vm.min_free_kbytes=4000" >> /etc/sysctl.conf'

ssh "admin@${ROBOT_HOSTNAME}" 'sed -i -e "s/^StartupDLLs/;StartupDLLs/" /etc/natinst/share/ni-rt.ini'

# This fails if the code isn't running.
ssh "admin@${ROBOT_HOSTNAME}" 'PATH="${PATH}":/usr/local/natinst/bin/ /usr/local/frc/bin/frcKillRobot.sh -r -t' || true

echo "Deploying robotCommand startup script"
scp frc971/config/robotCommand "admin@${ROBOT_HOSTNAME}:/home/lvuser/"
