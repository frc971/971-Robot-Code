# Clone and checkout the correct version of Tensorflow
git clone https://github.com/tensorflow/tensorflow.git tensorflow_src
cd tensorflow_src
git checkout v2.8.0
# Build libtensorflowlite.so for both arm and x86
bazel build --config=elinux_aarch64 -c opt //tensorflow/lite:libtensorflowlite.so
bazel build --config=native_arch_linux -c opt //tensorflow/lite:libtensorflowlite.so
# Create the directory for the tarball and move the resulting files into it 
mkdir tensorflow-bazel
mkdir tensorflow-bazel/arm
mkdir tensorflow-bazel/k8
cp bazel-out/aarch64-opt/bin/tensorflow/lite/libtensorflowlite.so tensorflow-bazel/arm
cp bazel-out/k8-opt/bin/tensorflow/lite/libtensorflowlite.so tensorflow-bazel/k8

# Copy header files to the include directory
 mkdir -p tensorflow-bazel/tensorflow/core/util
 rsync -zarv --include='*/'  --include='*.h' --exclude='*' tensorflow/core/util tensorflow-bazel/tensorflow/core/util