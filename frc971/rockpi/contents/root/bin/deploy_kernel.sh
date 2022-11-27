#!/bin/bash

set -ex

KERNEL_VERSION=6.0.8-rt14-rockpi4b

rm -rf /boot/dtbs/${KERNEL_VERSION}/
rm -rf /lib/modules/${KERNEL_VERSION}/
tar --owner=0 --group=0 --no-same-owner --no-same-permissions -xvf ./linux-kernel-${KERNEL_VERSION}.tar.xz -C /boot/ --strip-components=2 ./boot/
tar --strip-components=3 -xvf linux-kernel-${KERNEL_VERSION}.tar.xz -C /lib/modules/ ./lib/modules
