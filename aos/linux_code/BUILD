package(default_visibility = ['//visibility:public'])

cc_binary(
  name = 'dump_rtprio',
  srcs = [
    'dump_rtprio.cc',
  ],
  deps = [
    '//aos/common/logging:logging',
    '//aos/common:time',
  ],
)

cc_library(
  name = 'complex_thread_local',
  srcs = [
    'complex_thread_local.cc'
  ],
  deps = [
    '//aos/common:once',
    '//aos/common:die',
  ],
)

cc_test(
  name = 'complex_thread_local_test',
  srcs = [
    'complex_thread_local_test.cc',
  ],
  deps = [
    ':complex_thread_local',
    '//third_party/gtest',
    '//aos/common/util:thread',
    '//aos/common/logging:logging',
  ],
)

cc_library(
  name = 'init',
  srcs = [
    'init.cc',
  ],
  deps = [
    '//aos/linux_code/ipc_lib:shared_mem',
    '//aos/common:die',
    '//aos/common/logging:logging',
  ],
)

cc_library(
  name = 'configuration',
  src = [
    'configuration.cc',
  ],
  deps = [
    '//aos/common:once',
    '//aos/common/logging:logging',
  ],
)

cc_binary(
  name = 'core',
  srcs = [
    'core.cc',
  ],
  deps = [
    ':init',
    '//aos/common/util:run_command',
  ],
)
