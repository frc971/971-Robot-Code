package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'test_queue',
  srcs = [
    'test_queue.q',
  ],
)

cc_library(
  name = 'queue_testutils',
  srcs = [
    'queue_testutils.cc',
  ],
  hdrs = [
    'queue_testutils.h',
  ],
  deps = [
    '//aos/common/logging',
    ':once',
    '//aos/linux_code/ipc_lib:shared_mem',
    ':mutex',
  ],
)

cc_library(
  name = 'time',
  srcs = [
    'time.cc',
  ],
  hdrs = [
    'time.h',
  ],
  deps = [
    '//aos/common/logging:logging_interface',
    ':mutex',
  ],
)

cc_library(
  name = 'queue_types',
  srcs = [
    'queue_types.cc',
  ],
  hdrs = [
    'queue_types.h',
  ],
  deps = [
    '//aos/common/logging:logging_interface',
    '//aos/linux_code/ipc_lib:shared_mem',
    '//aos/linux_code/ipc_lib:core_lib',
    ':mutex',
    ':time',
  ],
)

cc_test(
  name = 'queue_types_test',
  srcs = [
    'queue_types_test.cc',
  ],
  deps = [
    ':queue_types',
    '//third_party/gtest',
    ':test_queue',
    '//aos/common/logging',
    ':queue_testutils',
  ],
)

cc_library(
  name = 'queues',
  srcs = [
    'queue.cc',
  ],
  hdrs = [
    'queue.h',
  ],
  deps = [
    '//aos/linux_code/ipc_lib:queue',
    ':time',
  ],
)

cc_library(
  name = 'scoped_fd',
  deps = [
    '//aos/common/logging',
  ],
)

cc_test(
  name = 'queue_test',
  srcs = [
    'queue_test.cc',
  ],
  deps = [
    '//third_party/gtest',
    ':queue_testutils',
    ':test_queue',
    '//aos/common/util:thread',
    ':die',
  ],
)

cc_test(
  name = 'type_traits_test',
  srcs = [
    'type_traits_test.cpp',
  ],
  deps = [
    '//third_party/gtest',
  ],
)

cc_library(
  name = 'gtest_prod',
)

cc_library(
  name = 'once',
  deps = [
    ':gtest_prod',
  ],
)

cc_test(
  name = 'once_test',
  srcs = [
    'once_test.cc',
  ],
  deps = [
    '//third_party/gtest',
    ':once',
  ],
)

cc_test(
  name = 'time_test',
  srcs = [
    'time_test.cc',
  ],
  deps = [
    '//third_party/gtest',
    ':time',
    '//aos/common/logging',
    '//aos/common/util:death_test_log_implementation',
  ],
)

cc_library(
  name = 'die',
  srcs = [
    'die.cc',
  ],
  hdrs = [
    'die.h',
  ],
  deps = [
    '//aos/common/libc:aos_strerror',
  ],
)

cc_test(
  name = 'queue_testutils_test',
  srcs = [
    'queue_testutils_test.cc',
  ],
  deps = [
    '//third_party/gtest',
    ':queue_testutils',
    '//aos/common/logging',
  ],
)

cc_test(
  name = 'mutex_test',
  srcs = [
    'mutex_test.cc',
  ],
  deps = [
    '//third_party/gtest',
    ':mutex',
    ':die',
    '//aos/common/logging',
    '//aos/common/util:death_test_log_implementation',
    '//aos/common/util:thread',
    '//aos/common:time',
    ':queue_testutils',
  ],
)

cc_test(
  name = 'event_test',
  srcs = [
    'event_test.cc',
  ],
  deps = [
    '//third_party/gtest',
    ':event',
    ':queue_testutils',
    ':time',
  ],
)

cc_test(
  name = 'condition_test',
  srcs = [
    'condition_test.cc',
  ],
  deps = [
    '//third_party/gtest',
    ':condition',
    '//aos/common/util:thread',
    ':time',
    ':mutex',
    '//aos/common/logging',
    ':queue_testutils',
    '//aos/linux_code/ipc_lib:core_lib',
    '//aos/linux_code/ipc_lib:aos_sync',
    ':die',
    '//aos/common/util:thread',
  ],
)

cc_test(
  name = 'die_test',
  srcs = [
    'die_test.cc',
  ],
  deps = [
    '//third_party/gtest',
    ':die',
  ],
)

cc_library(
  name = 'stl_mutex',
  deps = [
    '//aos/linux_code/ipc_lib:aos_sync',
    '//aos/common/logging',
  ],
)

cc_test(
  name = 'stl_mutex_test',
  srcs = [
    'stl_mutex_test.cc',
  ],
  deps = [
    ':stl_mutex',
    '//third_party/gtest',
    ':queue_testutils',
    '//aos/common/util:thread',
    ':die',
  ],
)
