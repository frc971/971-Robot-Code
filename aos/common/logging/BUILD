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
    'queue_logging-tmpl.h',
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
  name = 'sizes',
  hdrs = [
    'sizes.h',
  ],
)

cc_library(
  name = 'matrix_logging',
  srcs = [
    'matrix_logging.cc',
    'matrix_logging-tmpl.h',
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
  name = 'logging_printf_formats',
  hdrs = [
    'logging_printf_formats.h',
  ],
  deps = [
    '//aos/common:macros',
  ],
)

cc_library(
  name = 'logging_interface',
  hdrs = [
    'logging.h',
    'logging_interface.h',
  ],
  srcs = [
    'logging_interface.cc',
  ],
  deps = [
    '//aos/common:die',
    '//aos/common/libc:aos_strerror',
    '//aos/linux_code/logging:linux_interface',
    '//aos/common:macros',
  ],
)

cc_library(
  name = 'context',
  hdrs = [
    'context.h',
  ],
  srcs = [
    'context.cc',
  ],
  deps = [
    ':sizes',
  ],
)

cc_library(
  name = 'logging',
  srcs = [
    'logging_impl.cc',
  ],
  hdrs = [
    'logging_impl.h',
  ],
  deps = [
    '//aos/common:time',
    '//aos/common:once',
    '//aos/common:queue_types',
  ],
)
