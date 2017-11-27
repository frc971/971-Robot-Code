#!/usr/bin/env bash

set -e
set -u

export DEBIAN_FRONTEND=noninteractive

readonly PKGS=(
  clang-3.9
  clang-format-3.9
  curl
  gfortran
  git
  g++
  libblas-dev
  liblapack-dev
  libpython3-dev
  libpython-dev
  openjdk-8-jdk
  python3
  python3-matplotlib
  python3-numpy
  python3-scipy
  python-matplotlib
  python-scipy
  resolvconf
  ruby
  zlib1g-dev
)

# Set up the backports repo.
cat > /etc/apt/sources.list.d/backports.list <<EOT
deb http://http.debian.net/debian stretch-backports main contrib
EOT

# Set up the LLVM repo.
cat > /etc/apt/sources.list.d/llvm-apt.list <<EOT
deb http://apt.llvm.org/jessie/ llvm-toolchain-jessie main
deb-src http://apt.llvm.org/jessie/ llvm-toolchain-jessie main
EOT

# Enable user namespace for sandboxing.
cat > /etc/sysctl.d/99-enable-user-namespaces.conf <<EOT
kernel.unprivileged_userns_clone = 1
EOT

# Accept the LLVM GPG key so we can install their packages.
wget -O - http://llvm.org/apt/llvm-snapshot.gpg.key | apt-key add -

# Install all the packages that we need/want.
apt-get update
for pkg in "${PKGS[@]}"; do
  apt-get install -y -f --allow-change-held-packages "$pkg"
done

# Install bazel if necessary.
if ! dpkg -l bazel > /dev/null; then
  pushd /tmp
  curl -OL 'https://github.com/bazelbuild/bazel/releases/download/0.8.1/bazel_0.8.1-linux-x86_64.deb'
  dpkg -i bazel_0.8.1-linux-x86_64.deb
  popd
fi
