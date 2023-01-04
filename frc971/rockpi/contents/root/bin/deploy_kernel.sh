#!/bin/bash

set -ex

KERNEL_VERSION=6.0.8-rt14-rockpi4b
TAR=${1-./linux-kernel-${KERNEL_VERSION}.tar.xz}

KERNEL_VERSION=$(echo "${TAR}" | sed 's/.*linux-kernel-\(.*\).tar.xz/\1/')

rm -rf /boot/dtbs/${KERNEL_VERSION}/
rm -rf /lib/modules/${KERNEL_VERSION}/
tar --owner=0 --group=0 --no-same-owner --no-same-permissions -xvf ${TAR} -C /boot/ --strip-components=2 ./boot/
tar --strip-components=3 -xvf ${TAR} -C /lib/modules/ ./lib/modules
