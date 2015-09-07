package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'linux_interface',
  srcs = [
    'linux_interface.cc',
  ],
  deps = [
    '//aos/linux_code:complex_thread_local',
    '//aos/common:die',
    '//aos/common/libc:aos_strerror',
  ],
)

cc_library(
  name = 'linux_logging',
  srcs = [
    'linux_logging.cc',
  ],
  deps = [
    '//aos/linux_code/ipc_lib:queue',
    '//aos/common:time',
  ],
)

cc_test(
  name = 'logging_impl_test',
  srcs = [
    'logging_impl_test.cc',
  ],
  deps = [
    '//third_party/gtest',
    '//aos/common/logging:logging',
  ],
)

cc_library(
  name = 'queue_logging',
  srcs = [
    'queue_logging.cc',
  ],
  deps = [
    '//aos/common/logging:logging',
    '//aos/common:die',
    '//aos/common:queue_types',
  ],
)

cc_library(
  name = 'matrix_logging',
  srcs = [
    'matrix_logging.cc',
  ],
  deps = [
    '//aos/common/logging:logging',
    '//aos/common:die',
    '//aos/common:queue_types',
    '//third_party/eigen',
  ],
)
