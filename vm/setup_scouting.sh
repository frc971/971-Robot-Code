#!/usr/bin/env bash

set -e
set -u

export DEBIAN_FRONTEND=noninteractive

readonly PKGS=(
  python3
  python3-flask
)

# Install all the packages that we need/want.
apt-get update
for pkg in "${PKGS[@]}"; do
  apt-get install -y -f --force-yes "$pkg"
done
