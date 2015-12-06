package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'log_replay',
  srcs = [
    'log_replay.cc',
  ],
  hdrs = [
    'log_replay.h',
  ],
  deps = [
    ':binary_log_file',
    '//aos/common:queues',
    '//aos/common/logging',
    '//aos/linux_code/ipc_lib:queue',
  ],
)

cc_binary(
  name = 'binary_log_writer',
  srcs = [
    'binary_log_writer.cc',
  ],
  deps = [
    '//aos/common/logging',
    '//aos/linux_code:init',
    '//aos/linux_code:configuration',
    '//aos/common:die',
    ':binary_log_file',
    '//aos/common:queue_types',
  ],
)

cc_binary(
  name = 'log_streamer',
  srcs = [
    'log_streamer.cc',
  ],
  deps = [
    '//aos/common/logging',
    '//aos/linux_code:init',
    '//aos/common:time',
    '//aos/linux_code/ipc_lib:queue',
  ],
)

cc_binary(
  name = 'log_displayer',
  srcs = [
    'log_displayer.cc',
  ],
  deps = [
    '//aos/common/logging',
    '//aos/linux_code:init',
    ':binary_log_file',
    '//aos/common:queue_types',
    '//aos/linux_code:configuration',
    '//aos/common/util:string_to_num',
  ],
)

cc_library(
  name = 'binary_log_file',
  srcs = [
    'binary_log_file.cc',
  ],
  hdrs = [
    'binary_log_file.h',
  ],
  deps = [
    '//aos/common/logging',
  ],
)

cc_library(
  name = 'linux_interface',
  visibility = ['//aos/common/logging:__pkg__'],
  srcs = [
    'linux_interface.cc',
  ],
  deps = [
    '//aos/linux_code:complex_thread_local',
    '//aos/common:die',
    '//aos/common/logging:context',
  ],
)

cc_library(
  name = 'linux_logging',
  visibility = [
    '//aos/linux_code:__subpackages__'
  ],
  hdrs = [
    'linux_logging.h',
  ],
  srcs = [
    'linux_logging.cc',
  ],
  deps = [
    '//aos/linux_code/ipc_lib:queue',
    '//aos/common:time',
    '//aos/common/logging:logging',
  ],
)

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
    '//aos/common/logging:linux_interface',
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
