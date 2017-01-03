package(default_visibility = ['//visibility:public'])

filegroup(
  name = 'gcc',
  srcs = [
    'bin/arm-linux-gnueabihf-gcc',
  ],
)

filegroup(
  name = 'ar',
  srcs = [
    'bin/arm-linux-gnueabihf-ar',
  ],
)

filegroup(
  name = 'ld',
  srcs = [
    'bin/arm-linux-gnueabihf-ld',
  ],
)

filegroup(
  name = 'nm',
  srcs = [
    'bin/arm-linux-gnueabihf-nm',
  ],
)

filegroup(
  name = 'objcopy',
  srcs = [
    'bin/arm-linux-gnueabihf-objcopy',
  ],
)

filegroup(
  name = 'objdump',
  srcs = [
    'bin/arm-linux-gnueabihf-objdump',
  ],
)

filegroup(
  name = 'strip',
  srcs = [
    'bin/arm-linux-gnueabihf-strip',
  ],
)

filegroup(
  name = 'as',
  srcs = [
    'bin/arm-linux-gnueabihf-as',
  ],
)

cc_library(
  name = 'librt',
  srcs = [
    'arm-linux-gnueabihf/libc/usr/lib/librt.so',
  ],
)

cc_library(
  name = 'libdl',
  srcs = [
    'arm-linux-gnueabihf/libc/usr/lib/libdl.so',
  ],
)

cc_library(
  name = 'libm',
  srcs = [
    'arm-linux-gnueabihf/libc/usr/lib/libm.so',
  ],
)

cc_library(
  name = 'libpthread',
  deps = [
    '@//tools/cpp/linaro_linux_gcc:libpthread',
  ],
)

filegroup(
  name = 'compiler_pieces',
  srcs = glob([
    'arm-linux-gnueabihf/**',
    'libexec/**',
    'lib/gcc/arm-linux-gnueabihf/**',
    'include/**',
  ],
  exclude=[
    # Exclude empty files so Bazel's caching works.
    # TODO(Brian): remove this once the Bazel bug is fixed.
    '**/.install',
  ]),
)

filegroup(
  name = 'compiler_components',
  srcs = [
    ':gcc',
    ':ar',
    ':ld',
    ':nm',
    ':objcopy',
    ':objdump',
    ':strip',
    ':as',
  ],
)
