package(default_visibility = ["//visibility:public"])

filegroup(
  name = 'gcc',
  srcs = [
    'usr/bin/arm-frc-linux-gnueabi-gcc',
  ],
)

filegroup(
  name = 'ar',
  srcs = [
    'usr/bin/arm-frc-linux-gnueabi-ar',
  ],
)

filegroup(
  name = 'as',
  srcs = [
    'usr/bin/arm-frc-linux-gnueabi-as',
  ],
)

filegroup(
  name = 'ld',
  srcs = [
    'usr/bin/arm-frc-linux-gnueabi-ld',
  ],
)

filegroup(
  name = 'nm',
  srcs = [
    'usr/bin/arm-frc-linux-gnueabi-nm',
  ],
)

filegroup(
  name = 'objcopy',
  srcs = [
    'usr/bin/arm-frc-linux-gnueabi-objcopy',
  ],
)

filegroup(
  name = 'objdump',
  srcs = [
    'usr/bin/arm-frc-linux-gnueabi-objdump',
  ],
)

filegroup(
  name = 'strip',
  srcs = [
    'usr/bin/arm-frc-linux-gnueabi-strip',
  ],
)

filegroup(
  name = 'compiler_pieces',
  srcs = glob([
    'usr/bin/**/*',
    'usr/include/**/*',
    'usr/lib/**/*',
    'usr/arm-frc-linux-gnueabi/**/*',
    'usr/lib/x86_64-linux-gnu/gcc/**/*',
  ]),
)

filegroup(
  name = 'compiler_components',
  srcs = [
    ':gcc',
    ':ar',
    ':as',
    ':ld',
    ':nm',
    ':objcopy',
    ':objdump',
    ':strip',
  ],
)

cc_library(
  name = 'librt',
  srcs = [
    'usr/arm-frc-linux-gnueabi/usr/lib/librt.so'
  ],
)

cc_library(
  name = 'libdl',
  srcs = [
    'usr/arm-frc-linux-gnueabi/usr/lib/libdl.so'
  ],
)

cc_library(
  name = 'libm',
  srcs = [
    'usr/arm-frc-linux-gnueabi/usr/lib/libm.so'
  ],
)

cc_library(
  name = 'libpthread',
)

