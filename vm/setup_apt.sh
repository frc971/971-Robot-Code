#!/usr/bin/env bash

set -e
set -u

export DEBIAN_FRONTEND=noninteractive

# Set up contrib and non-free so we can install some more interesting programs.
cat > /etc/apt/sources.list.d/contrib.list <<EOT
deb  http://ftp.us.debian.org/debian/ jessie contrib non-free
deb-src  http://ftp.us.debian.org/debian/ jessie contrib non-free
EOT

# Get a list of the latest packages.
apt-get update
