#!/usr/bin/env bash

set -e
set -u

export DEBIAN_FRONTEND=noninteractive

readonly PKGS=(
  iceweasel
  lightdm
  mousepad
  xfce4
  xfce4-terminal
)

# Install all the packages that we need/want.
for pkg in "${PKGS[@]}"; do
  apt-get install -y -f "$pkg"
done
