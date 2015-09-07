package(default_visibility = ['//visibility:public'])

cc_test(
  name = 'logging_impl_test',
  srcs = [
    'logging_impl_test.cc',
  ],
  deps = [
    '//third_party/gtest',
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
    '//aos/common/logging',
    '//aos/common:die',
    '//aos/common:queue_types',
    '//third_party/eigen',
  ],
)
