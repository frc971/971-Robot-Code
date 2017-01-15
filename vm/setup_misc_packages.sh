#!/usr/bin/env bash

set -e
set -u

export DEBIAN_FRONTEND=noninteractive

readonly PKGS=(
  colordiff
  tmux
  vim
)

# Install all the packages that we want.
for pkg in "${PKGS[@]}"; do
  apt-get install -y -f "$pkg"
done
