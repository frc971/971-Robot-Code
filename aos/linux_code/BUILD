package(default_visibility = ['//visibility:public'])

cc_binary(
  name = 'dump_rtprio',
  srcs = [
    'dump_rtprio.cc',
  ],
  deps = [
    '//aos/common/logging',
    '//aos/common/logging:implementations',
    '//aos/common:time',
  ],
)

cc_library(
  name = 'queue',
  visibility = ['//aos/common:__pkg__'],
  hdrs = [
    'queue-tmpl.h',
  ],
)

cc_library(
  name = 'complex_thread_local',
  srcs = [
    'complex_thread_local.cc',
  ],
  hdrs = [
    'complex_thread_local.h',
  ],
  deps = [
    '//aos/common:once',
    '//aos/common:die',
    '//debian:libpthread',
  ],
)

cc_test(
  name = 'complex_thread_local_test',
  srcs = [
    'complex_thread_local_test.cc',
  ],
  deps = [
    ':complex_thread_local',
    '//aos/testing:googletest',
    '//aos/common/util:thread',
    '//aos/common/logging',
  ],
)

cc_library(
  name = 'init',
  srcs = [
    'init.cc',
  ],
  hdrs = [
    'init.h',
  ],
  deps = [
    '//aos/linux_code/ipc_lib:shared_mem',
    '//aos/common:die',
    '//aos/common/logging:implementations',
  ],
)

cc_library(
  name = 'configuration',
  srcs = [
    'configuration.cc',
  ],
  hdrs = [
    'configuration.h',
  ],
  deps = [
    '//aos/common:once',
    '//aos/common/logging',
    '//aos/common:unique_malloc_ptr',
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
