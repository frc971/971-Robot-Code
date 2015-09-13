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
  ],
)

cc_library(
  name = 'linux_logging',
  visibility = ['//aos/common/logging:__pkg__'],
  srcs = [
    'linux_logging.cc',
  ],
  deps = [
    '//aos/linux_code/ipc_lib:queue',
    '//aos/common:time',
  ],
)
