package(default_visibility = ['//visibility:public'])

cc_test(
  name = 'logging_impl_test',
  srcs = [
    'logging_impl_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    '//aos/common/logging',
  ],
)

cc_library(
  name = 'queue_logging',
  srcs = [
    'queue_logging.cc',
  ],
  hdrs = [
    'queue_logging.h',
  ],
  deps = [
    '//aos/common/logging',
    '//aos/common:die',
    '//aos/common:queue_types',
  ],
)

cc_library(
  name = 'matrix_logging',
  srcs = [
    'matrix_logging.cc',
  ],
  hdrs = [
    'matrix_logging.h',
  ],
  deps = [
    '//aos/common:generated_queue_headers',
    '//aos/common/logging',
    '//aos/common:die',
    '//aos/common:queue_types',
    '//third_party/eigen',
  ],
)

cc_library(
  name = 'logging_interface',
  srcs = [
    'logging_interface.cc',
  ],
  deps = [
    '//aos/linux_code/logging:linux_interface',
    '//aos/common:die',
    '//aos/common/libc:aos_strerror',
  ],
)

cc_library(
  name = 'logging',
  srcs = [
    'logging_impl.cc',
  ],
  deps = [
    '//aos/linux_code/logging:linux_logging',
    '//aos/common:time',
    '//aos/common:once',
    ':logging_interface',
    '//aos/common:queue_types',
  ],
)
