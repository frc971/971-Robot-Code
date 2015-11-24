package(default_visibility = ['//tools/cpp:__pkg__'])

filegroup(
  name = 'gcc',
  srcs = [
    '@arm-frc-linux-gnueabi-repo//:gcc',
    'arm-frc-linux-gnueabi-gcc',
  ],
)

filegroup(
  name = 'ar',
  srcs = [
    '@arm-frc-linux-gnueabi-repo//:ar',
    'arm-frc-linux-gnueabi-ar',
  ],
)

filegroup(
  name = 'as',
  srcs = [
    '@arm-frc-linux-gnueabi-repo//:as',
    'arm-frc-linux-gnueabi-as',
  ],
)

filegroup(
  name = 'ld',
  srcs = [
    '@arm-frc-linux-gnueabi-repo//:ld',
    'arm-frc-linux-gnueabi-ld',
  ],
)

filegroup(
  name = 'nm',
  srcs = [
    '@arm-frc-linux-gnueabi-repo//:nm',
    'arm-frc-linux-gnueabi-nm',
  ],
)

filegroup(
  name = 'objcopy',
  srcs = [
    '@arm-frc-linux-gnueabi-repo//:objcopy',
    'arm-frc-linux-gnueabi-objcopy',
  ],
)

filegroup(
  name = 'objdump',
  srcs = [
    '@arm-frc-linux-gnueabi-repo//:objdump',
    'arm-frc-linux-gnueabi-objdump',
  ],
)

filegroup(
  name = 'strip',
  srcs = [
    '@arm-frc-linux-gnueabi-repo//:strip',
    'arm-frc-linux-gnueabi-strip',
  ],
)

filegroup(
  name = 'tool-wrappers',
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
