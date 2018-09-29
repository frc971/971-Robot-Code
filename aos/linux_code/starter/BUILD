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
    '//aos/logging',
    '//aos/logging:implementations',
    '//aos:once',
    '//aos/time:time',
    '//aos/libc:aos_strsignal',
    '//aos/util:run_command',
    '//aos:unique_malloc_ptr',
  ],
)
