package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'test_queue',
  srcs = [
    'test_queue.q',
  ],
)

cc_library(
  name = 'math',
  hdrs = [
    'commonmath.h',
  ],
)

cc_library(
  name = 'macros',
  hdrs = [
    'macros.h',
  ],
)

cc_library(
  name = 'type_traits',
  hdrs = [
    'type_traits.h',
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
    '//aos/common/logging:logging',
    ':mutex',
    ':macros',
    '//aos/linux_code/ipc_lib:shared_mem',
  ],
)

genrule(
  name = 'gen_queue_primitives',
  visibility = ['//visibility:private'],
  tools = ['//aos/build/queues:queue_primitives'],
  outs = ['queue_primitives.h'],
  cmd = '$(location //aos/build/queues:queue_primitives) $@',
)

genrule(
  name = 'gen_print_field',
  visibility = ['//visibility:private'],
  tools = ['//aos/build/queues:print_field'],
  outs = ['print_field.cc'],
  cmd = '$(location //aos/build/queues:print_field) $@',
)

cc_library(
  name = 'generated_queue_headers',
  visibility = ['//aos/common/logging:__pkg__'],
  hdrs = [
    ':gen_queue_primitives',
  ],
)

cc_library(
  name = 'event',
  hdrs = [
    'event.h',
  ],
  srcs = [
    'event.cc',
  ],
  deps = [
    '//aos/linux_code/ipc_lib:aos_sync',
    ':time',
    '//aos/common/logging:logging',
  ],
)

cc_library(
  name = 'unique_malloc_ptr',
  hdrs = [
    'unique_malloc_ptr.h',
  ],
)

cc_library(
  name = 'queue_types',
  srcs = [
    'queue_types.cc',
    ':gen_print_field',
    'print_field_helpers.h',
  ],
  hdrs = [
    'queue_types.h',
  ],
  deps = [
    ':generated_queue_headers',
    '//aos/linux_code/ipc_lib:shared_mem',
    '//aos/linux_code/ipc_lib:core_lib',
    ':mutex',
    '//aos/common/logging:printf_formats',
    ':time',
    ':byteorder'
  ],
)

cc_test(
  name = 'queue_types_test',
  srcs = [
    'queue_types_test.cc',
  ],
  deps = [
    ':queue_types',
    '//aos/testing:googletest',
    ':test_queue',
    '//aos/common/logging',
    '//aos/testing:test_logging',
  ],
)

cc_library(
  name = 'network_port',
  hdrs = [
    'network_port.h',
  ],
)

cc_library(
  name = 'byteorder',
  hdrs = [
    'byteorder.h',
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
    ':macros',
    '//aos/linux_code:queue',
    ':byteorder',
  ],
)

cc_library(
  name = 'scoped_fd',
  hdrs = [
    'scoped_fd.h',
  ],
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
    '//aos/testing:googletest',
    '//aos/testing:test_shm',
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
    '//aos/testing:googletest',
    ':type_traits',
  ],
)

cc_library(
  name = 'gtest_prod',
  hdrs = [
    'gtest_prod.h',
  ],
)

cc_library(
  name = 'once',
  hdrs = [
    'once.h',
  ],
  srcs = [
    'once-tmpl.h',
  ],
  deps = [
    ':gtest_prod',
    ':type_traits',
  ],
)

cc_test(
  name = 'once_test',
  srcs = [
    'once_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    ':once',
  ],
)

cc_test(
  name = 'time_test',
  srcs = [
    'time_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
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
    ':macros',
    '//aos/common/libc:aos_strerror',
  ],
)

cc_test(
  name = 'mutex_test',
  srcs = [
    'mutex_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    ':mutex',
    ':die',
    '//aos/common/logging',
    '//aos/common/util:death_test_log_implementation',
    '//aos/common/util:thread',
    '//aos/common:time',
    '//aos/testing:test_logging',
    '//aos/testing:test_shm',
  ],
)

cc_test(
  name = 'event_test',
  srcs = [
    'event_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    ':event',
    '//aos/testing:test_logging',
    ':time',
  ],
)

cc_library(
  name = 'condition',
  hdrs = [
    'condition.h',
  ],
  srcs = [
    'condition.cc',
  ],
  deps = [
    ':mutex',
    '//aos/linux_code/ipc_lib:aos_sync',
    '//aos/common/logging:logging',
  ],
)

cc_test(
  name = 'condition_test',
  srcs = [
    'condition_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    '//aos/testing:prevent_exit',
    ':condition',
    '//aos/common/util:thread',
    ':time',
    ':mutex',
    '//aos/common/logging',
    '//aos/testing:test_shm',
    '//aos/linux_code/ipc_lib:core_lib',
    '//aos/linux_code/ipc_lib:aos_sync',
    ':die',
  ],
)

cc_test(
  name = 'die_test',
  srcs = [
    'die_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    ':die',
  ],
)

cc_library(
  name = 'stl_mutex',
  hdrs = [
    'stl_mutex.h',
  ],
  deps = [
    '//aos/linux_code/ipc_lib:aos_sync',
    '//aos/common/logging',
  ],
)

cc_library(
  name = 'mutex',
  hdrs = [
    'mutex.h',
  ],
  srcs = [
    'mutex.cc',
  ],
  deps = [
    '//aos/linux_code/ipc_lib:aos_sync',
    ':die',
    '//aos/common/logging:logging',
    ':type_traits',
  ],
)

cc_test(
  name = 'stl_mutex_test',
  srcs = [
    'stl_mutex_test.cc',
  ],
  deps = [
    ':stl_mutex',
    '//aos/testing:googletest',
    '//aos/testing:test_logging',
    '//aos/common/util:thread',
    ':die',
  ],
)

cc_library(
  name = 'transaction',
  hdrs = [
    'transaction.h',
  ],
  deps = [
    '//aos/common/logging:logging',
    '//aos/common/util:compiler_memory_barrier',
  ],
)

cc_test(
  name = 'transaction_test',
  srcs = [
    'transaction_test.cc',
  ],
  deps = [
    ':transaction',
    '//aos/testing:googletest',
    '//aos/common/logging',
    '//aos/common/util:death_test_log_implementation',
  ],
)

cc_library(
  name = 'ring_buffer',
  hdrs = [
    'ring_buffer.h',
  ],
)

cc_test(
  name = 'ring_buffer_test',
  srcs = [
    'ring_buffer_test.cc',
  ],
  deps = [
    ':ring_buffer',
    '//aos/testing:googletest',
  ],
)
