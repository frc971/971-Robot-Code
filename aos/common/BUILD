"""
aos_queue(
  name = 'test_queue',
  srcs = [
    'test_queue.q',
  ],
)
"""

cc_library(
  name = 'queue_testutils',
  srcs = [
    'queue_testutils.cc',
  ],
  deps = [
    ':once',
    ':mutex',
    '//aos/build:logging',
    '//aos/linux_code/ipc_lib:shared_mem',
  ],
)

cc_library(
  name = 'time',
  hdrs = [
    'time.h',
  ],
  srcs = [
    'time.cc',
  ],
  deps = [
    ':mutex',
    '//aos/build:logging_interface',
  ],
)

cc_library(
  name = 'queue_types',
  srcs = [
    'queue_types.cc',
  ],
)
