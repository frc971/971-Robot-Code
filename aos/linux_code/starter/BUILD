# This target is everything which should get deployed to the robot.
filegroup(
  name = 'starter',
  visibility = ['//visibility:public'],
  srcs = [
    'starter_exe',
    'starter.sh',
  ],
)

cc_binary(
  name = 'starter_exe',
  srcs = [
    'starter.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//third_party/libevent',
    '//aos/common/logging',
    '//aos/common/logging:implementations',
    '//aos/common:once',
    '//aos/common:time',
    '//aos/common/libc:aos_strsignal',
    '//aos/common/util:run_command',
    '//aos/common:unique_malloc_ptr',
  ],
)
