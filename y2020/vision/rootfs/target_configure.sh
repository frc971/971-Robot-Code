#!/bin/bash

set -ex

mkdir -p /root/bin

# Give it a static IP
cp /tmp/dhcpcd.conf /etc/

# And provide a script to change it.
cp /tmp/change_hostname.sh /root/bin/
chmod a+x /root/bin/change_hostname.sh

chown -R pi.pi /home/pi/.ssh

apt-get update

apt-get install -y vim-nox \
  git \
  python3-pip \
  cpufrequtils \
  libopencv-calib3d3.2 \
  libopencv-contrib3.2 \
  libopencv-core3.2 \
  libopencv-features2d3.2 \
  libopencv-flann3.2 \
  libopencv-highgui3.2 \
  libopencv-imgcodecs3.2 \
  libopencv-imgproc3.2 \
  libopencv-ml3.2 \
  libopencv-objdetect3.2 \
  libopencv-photo3.2 \
  libopencv-shape3.2 \
  libopencv-stitching3.2 \
  libopencv-superres3.2 \
  libopencv-video3.2 \
  libopencv-videoio3.2 \
  libopencv-videostab3.2 \
  libopencv-viz3.2 \
  python3-opencv \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-x \
  gstreamer1.0-nice \
  gstreamer1.0-gl \
  libgstreamer-plugins-bad1.0-0 \
  libgstreamer-plugins-base1.0-0 \
  libgstreamer1.0-0 \
  libgstreamer-gl1.0-0 \
  libnice10 \
  libnice-dev

echo 'GOVERNOR="performance"' > /etc/default/cpufrequtils

# Add a .bashrc and friends for root.
if [[ ! -e "/root/.dotfiles" ]]; then
  cd /root/
  git clone --separate-git-dir=/root/.dotfiles https://github.com/AustinSchuh/.dotfiles.git tmpdotfiles
  rsync --recursive --verbose --exclude '.git' tmpdotfiles/ /root/
  rm -r tmpdotfiles
  git --git-dir=/root/.dotfiles/ --work-tree=/root/ config --local status.showUntrackedFiles no
  # Run the vundle installer which installs plugins on the first run of vim.
  vim -c ":qa!"
fi

# Add a .bashrc and friends for pi.
if [[ ! -e "/home/pi/.dotfiles" ]]; then
  cd /home/pi/
  su -c "git clone --separate-git-dir=/home/pi/.dotfiles https://github.com/AustinSchuh/.dotfiles.git tmpdotfiles" pi
  su -c "rsync --recursive --verbose --exclude '.git' tmpdotfiles/ /home/pi/" pi
  su -c "rm -r tmpdotfiles" pi
  su -c "git --git-dir=/home/pi/.dotfiles/ --work-tree=/home/pi/ config --local status.showUntrackedFiles no" pi
  # Run the vundle installer which installs plugins on the first run of vim.
  su -c "vim -c ':qa!'" pi
fi

# Make SSH work and not complain about pi being the username and pw still.
rm -f /etc/profile.d/sshpwd.sh
rm -f /etc/profile.d/wifi-check.sh

systemctl enable ssh.service
systemctl enable frc971.service

# Default us to pi-971-1
/root/bin/change_hostname.sh pi-971-1
