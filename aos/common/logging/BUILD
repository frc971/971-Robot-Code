# The primary client logging interface.
cc_library(
  name = 'logging',
  visibility = ['//visibility:public'],
  hdrs = [
    'logging.h',
    'interface.h',
    'context.h',
  ],
  srcs = [
    'interface.cc',
    'context.cc',
  ],
  deps = [
    '//aos/common:die',
    '//aos/common/libc:aos_strerror',
    '//aos/common:macros',
    '//aos/linux_code:complex_thread_local',
    ':sizes',
  ],
)

cc_library(
  name = 'replay',
  visibility = ['//visibility:public'],
  srcs = [
    'replay.cc',
  ],
  hdrs = [
    'replay.h',
  ],
  deps = [
    ':binary_log_file',
    '//aos/common:queues',
    ':logging',
    '//aos/linux_code/ipc_lib:queue',
  ],
)

cc_binary(
  name = 'binary_log_writer',
  visibility = ['//visibility:public'],
  srcs = [
    'binary_log_writer.cc',
  ],
  deps = [
    ':logging',
    ':implementations',
    '//aos/linux_code:init',
    '//aos/linux_code:configuration',
    '//aos/linux_code/ipc_lib:queue',
    '//aos/common:die',
    ':binary_log_file',
    '//aos/common:queue_types',
    '//aos/common:time',
  ],
)

cc_binary(
  name = 'log_streamer',
  visibility = ['//visibility:public'],
  srcs = [
    'log_streamer.cc',
  ],
  deps = [
    ':logging',
    ':implementations',
    '//aos/linux_code:init',
    '//aos/common:time',
    '//aos/linux_code/ipc_lib:queue',
  ],
)

cc_binary(
  name = 'log_displayer',
  visibility = ['//visibility:public'],
  srcs = [
    'log_displayer.cc',
  ],
  deps = [
    ':logging',
    ':implementations',
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
    ':implementations',
  ],
)

cc_library(
  name = 'sizes',
  hdrs = [
    'sizes.h',
  ],
)

cc_test(
  name = 'implementations_test',
  srcs = [
    'implementations_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    ':logging',
    ':implementations',
  ],
)

cc_library(
  name = 'queue_logging',
  visibility = ['//visibility:public'],
  srcs = [
    'queue_logging.cc',
  ],
  hdrs = [
    'queue_logging.h',
  ],
  deps = [
    ':logging',
    ':sizes',
    '//aos/common:die',
    '//aos/common:queue_types',
  ],
)

cc_library(
  name = 'matrix_logging',
  visibility = ['//visibility:public'],
  srcs = [
    'matrix_logging.cc',
  ],
  hdrs = [
    'matrix_logging.h',
  ],
  deps = [
    '//aos/common:generated_queue_headers',
    ':logging',
    ':sizes',
    '//aos/common:die',
    '//aos/common:queue_types',
    '//third_party/eigen',
  ],
)

cc_library(
  name = 'printf_formats',
  visibility = ['//visibility:public'],
  hdrs = [
    'printf_formats.h',
  ],
  deps = [
    '//aos/common:macros',
  ],
)

cc_library(
  name = 'implementations',
  visibility = ['//visibility:public'],
  srcs = [
    'implementations.cc',
  ],
  hdrs = [
    'implementations.h',
  ],
  linkopts = [
    '-pthread',
  ],
  deps = [
    '//aos/common:die',
    '//aos/common:time',
    '//aos/common:once',
    '//aos/common:queue_types',
    ':logging',
    '//aos/common:type_traits',
    '//aos/common:mutex',
    '//aos/common:macros',
    ':sizes',
    '//aos/linux_code/ipc_lib:queue',
  ],
)
