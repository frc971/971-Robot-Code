package(default_visibility = ['//visibility:public'])

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
