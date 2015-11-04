package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'rollers_queue',
  srcs = [
    'rollers.q',
  ],
  deps = [
    '//aos/common/controls:control_loop_queues',
    '//frc971/control_loops:queues',
  ],
)

cc_library(
  name = 'rollers_lib',
  hdrs = [
    'rollers.h',
  ],
  srcs = [
    'rollers.cc',
  ],
  deps = [
    ':rollers_queue',
    '//aos/common/logging',
    '//aos/common/controls:control_loop',
  ],
)

cc_binary(
  name = 'rollers',
  srcs = [
    'rollers_main.cc',
  ],
  deps = [
    ':rollers_lib',
    '//aos/linux_code:init',
  ],
)
