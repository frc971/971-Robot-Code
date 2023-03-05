# Clone the correct version of libedgetpu
git clone https://github.com/google-coral/libedgetpu.git
cd libedgetpu
# Build libedgetpu.so.1.0 for both arm and x86
DOCKER_CPUS="k8" DOCKER_IMAGE="ubuntu:18.04" DOCKER_TARGETS=libedgetpu make docker-build
DOCKER_CPUS="aarch64" DOCKER_IMAGE="debian:stretch" DOCKER_TARGETS=libedgetpu make docker-build
# Create the directory for the tarball and move the resulting files into it 
mkdir libedgetpu-bazel
mkdir libedgetpu-bazel/arm
mkdir libedgetpu-bazel/k8
cp out/direct/aarch64/libedgetpu.so.1.0 libedgetpu-bazel/arm
cp out/direct/k8/libedgetpu.so.1.0 libedgetpu-bazel/k8

# Copy header files to the include directory
mkdir libedgetpu-bazel/include
cp -r include/* libedgetpu-bazel/include/
