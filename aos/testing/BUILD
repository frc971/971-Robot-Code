cc_library(
  name = 'googletest',
  visibility = ['//visibility:public'],
  srcs = [
    'gtest_main.cc',
  ],
  deps = [
    '//third_party/googletest',
  ],
  testonly = True,
)

cc_library(
  name = 'test_logging',
  visibility = ['//visibility:public'],
  srcs = [
    'test_logging.cc',
  ],
  hdrs = [
    'test_logging.h',
  ],
  deps = [
    ':googletest',
    '//aos/common/logging:implementations',
    '//aos/common:once',
    '//aos/common:mutex',
  ],
  testonly = True,
)

cc_test(
  name = 'test_logging_test',
  srcs = [
    'test_logging_test.cc',
  ],
  deps = [
    ':googletest',
    ':test_logging',
    '//aos/common/logging',
  ],
)

cc_library(
  name = 'prevent_exit',
  visibility = ['//visibility:public'],
  srcs = [
    'prevent_exit.cc',
  ],
  hdrs = [
    'prevent_exit.h',
  ],
  deps = [
    '//aos/common/logging',
  ],
  testonly = True,
)

cc_library(
  name = 'test_shm',
  visibility = ['//visibility:public'],
  srcs = [
    'test_shm.cc',
  ],
  hdrs = [
    'test_shm.h',
  ],
  deps = [
    '//aos/common:queues',
    '//aos/linux_code/ipc_lib:shared_mem',
    '//aos/common/logging',
    ':test_logging',
  ],
  testonly = True,
)
