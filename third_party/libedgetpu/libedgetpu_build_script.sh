#!/bin/bash
#This script creates a compressed tarball file named libedgetpu-${GIT_VERSION}.tar.gz, 
# which contains the header files, libraries, and binaries needed to use Edge TPU on both arm and x86 architectures.
# This script assumes you have Docker installed.
#
# Clone the correct version of libedgetpu
git clone https://github.com/google-coral/libedgetpu.git
cd libedgetpu
GIT_VERSION=ddfa7bde33c23afd8c2892182faa3e5b4e6ad94e
git checkout ${GIT_VERSION}
# Build libedgetpu.so.1.0 for both arm and x86
DOCKER_CPUS="k8" DOCKER_IMAGE="ubuntu:18.04" DOCKER_TARGETS=libedgetpu make docker-build
DOCKER_CPUS="aarch64" DOCKER_IMAGE="debian:stretch" DOCKER_TARGETS=libedgetpu make docker-build
# Create the directory for the tarball and move the resulting files into it
rm -rf  libedgetpu-bazel
mkdir libedgetpu-bazel
mkdir libedgetpu-bazel/arm
mkdir libedgetpu-bazel/k8
cp out/direct/aarch64/libedgetpu.so.1.0 libedgetpu-bazel/arm
cp out/direct/k8/libedgetpu.so.1.0 libedgetpu-bazel/k8

# Copy header files to the include directory
mkdir -p libedgetpu-bazel/include/tflite/
rsync -zarv --include="*/" --include='*.h' --exclude='*' tflite/ libedgetpu-bazel/include/tflite/
tar zcvf libedgetpu-${GIT_VERSION}.tar.gz libedgetpu-bazel

