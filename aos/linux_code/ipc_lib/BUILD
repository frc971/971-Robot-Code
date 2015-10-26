package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'aos_sync',
  srcs = [
    'aos_sync.cc',
  ],
  hdrs = [
    'aos_sync.h',
  ],
  deps = [
    '//aos/common/logging:logging_interface',
    '//aos/common:once',
    '//aos/common:macros',
  ],
)

cc_library(
  name = 'core_lib',
  srcs = [
    'core_lib.c',
  ],
  hdrs = [
    'core_lib.h',
  ],
  deps = [
    ':aos_sync',
    ':shared_mem_types',
  ],
)

cc_library(
  name = 'shared_mem',
  srcs = [
    'shared_mem.c',
  ],
  hdrs = [
    'shared_mem.h',
  ],
  deps = [
    ':aos_sync',
    ':core_lib',
    ':shared_mem_types',
    '//aos/common/logging:logging_interface',
    '//debian:librt',
  ],
)

cc_library(
  # TODO(Brian): This should be shared_mem{,.h}, and the other one should be
  # shared_mem_init{,.cc,.h}.
  name = 'shared_mem_types',
  hdrs = [
    'shared_mem_types.h',
  ],
  deps = [
    ':aos_sync',
  ],
)

cc_library(
  name = 'queue',
  srcs = [
    'queue.cc',
  ],
  hdrs = [
    'queue.h',
  ],
  deps = [
    '//aos/linux_code/ipc_lib:condition',
    '//aos/linux_code/ipc_lib:mutex',
    ':core_lib',
    ':shared_mem',
    '//aos/common/logging:logging_interface',
    '//debian:librt',
    '//aos/common/util:options',
  ],
)

cc_test(
  name = 'raw_queue_test',
  srcs = [
    'raw_queue_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    ':queue',
    '//aos/common/logging',
    ':core_lib',
    '//aos/common:queue_testutils',
    '//aos/common:time',
    '//aos/common:die',
    '//aos/common/util:thread',
    '//aos/common/util:death_test_log_implementation',
  ],
)

cc_test(
  name = 'ipc_stress_test',
  srcs = [
    'ipc_stress_test.cc',
  ],
  tags = [
    'manual',
  ],
  deps = [
    '//aos/testing:googletest',
    '//aos/common:time',
    '//aos/common:queue_testutils',
    '//aos/linux_code/ipc_lib:mutex',
    ':core_lib',
    '//aos/common:die',
    '//aos/common/libc:dirname',
    '//aos/common/libc:aos_strsignal',
    '//aos/common/logging',
  ],
)

cc_library(
  name = 'scoped_message_ptr',
  deps = [
    ':queue',
  ],
)

cc_library(
  name = 'condition',
  visibility = ['//aos/common:__pkg__'],
  srcs = [
    'condition.cc',
  ],
  deps = [
    ':mutex',
    '//aos/common:real_condition',
    ':aos_sync',
    '//aos/common/logging:logging_interface',
  ],
)

cc_library(
  name = 'mutex',
  srcs = [
    'mutex.cc',
  ],
  deps = [
    ':aos_sync',
    '//aos/common/logging:logging_interface',
    '//aos/common:type_traits',
    '//aos/common:mutex',
  ],
)

cc_library(
  name = 'event',
  visibility = ['//aos/common:__pkg__'],
  srcs = [
    'event.cc',
  ],
  deps = [
    ':aos_sync',
    '//aos/common/logging:logging_interface',
    '//aos/common:real_event',
  ],
)
