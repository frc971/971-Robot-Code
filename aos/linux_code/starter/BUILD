package(default_visibility = ['//visibility:public'])

cc_binary(
  name = 'starter_exe',
  srcs = [
    'starter.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//third_party/libevent',
    '//aos/common/logging',
    '//aos/common:once',
    '//aos/common:time',
    '//aos/common/libc:aos_strsignal',
    '//aos/common/util:run_command',
  ],
)
