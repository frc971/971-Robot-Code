#!/bin/bash
# This script creates a compressed tarball file named tensorflow-${GIT_VERSION}.tar.gz, 
# which contains the header files, libraries, and binaries needed to use Tensorflow Lite on both arm and x86 architectures.
# This script assumes you have bazelisk and necessary permissions.
#
# Clone and checkout the correct version of Tensorflow
git clone https://github.com/tensorflow/tensorflow.git tensorflow_src
cd tensorflow_src
GIT_VERSION=a4dfb8d1a71385bd6d122e4f27f86dcebb96712d
git checkout $GIT_VERSION
# Build libtensorflowlite.so for both arm and x86
bazelisk build --config=elinux_aarch64 -c opt //tensorflow/lite:libtensorflowlite.so
bazelisk build --config=native_arch_linux -c opt //tensorflow/lite:libtensorflowlite.so
# Create the directory for the tarball and move the resulting files into it
rm -rf tensorflow-bazel
mkdir tensorflow-bazel
mkdir tensorflow-bazel/arm
mkdir tensorflow-bazel/k8
cp bazel-out/aarch64-opt/bin/tensorflow/lite/libtensorflowlite.so tensorflow-bazel/arm
cp bazel-out/k8-opt/bin/tensorflow/lite/libtensorflowlite.so tensorflow-bazel/k8

# Copy header files to the include directory
mkdir -p tensorflow-bazel/include/tensorflow/
mkdir -p tensorflow-bazel/include/flatbuffers/
rsync -zarv --include="*/" --include='*.h' --exclude='*' tensorflow/ tensorflow-bazel/include/tensorflow/
rsync -zarv --include="*/" --include='*.h' --exclude='*' bazel-out/../../../external/flatbuffers/include/flatbuffers/ tensorflow-bazel/include/flatbuffers/
tar zcvf tensorflow-${GIT_VERSION}.tar.gz tensorflow-bazel
