package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

cc_library(
  name = 'replay_control_loop',
  hdrs = [
    'replay_control_loop.h',
  ],
  deps = [
    '//aos/common:queues',
    ':control_loop',
    '//aos/common/logging:replay',
    '//aos/common/logging:queue_logging',
    '//aos/common:time',
  ],
)

cc_library(
  name = 'control_loop_test',
  srcs = [
    'control_loop_test.cc',
  ],
  hdrs = [
    'control_loop_test.h',
  ],
  deps = [
    '//aos/common:time',
    '//aos/common/messages:robot_state',
    '//aos/testing:googletest',
    '//aos/testing:test_shm',
  ],
)

cc_library(
  name = 'polytope',
  hdrs = [
    'polytope.h',
  ],
  deps = [
    '//third_party/eigen',
    '//third_party/cddlib',
  ],
)

queue_library(
  name = 'control_loop_queues',
  srcs = [
    'control_loops.q',
  ],
)

cc_library(
  name = 'control_loop',
  srcs = [
    'control_loop.cc',
    'control_loop-tmpl.h',
  ],
  hdrs = [
    'control_loop.h',
  ],
  deps = [
    '//aos/common/messages:robot_state',
    '//aos/common/logging',
    '//aos/common/util:phased_loop',
    '//aos/common:time',
    ':control_loop_queues',
    '//aos/common/logging:queue_logging',
    '//aos/common/util:log_interval',
    '//aos/common:queues',
  ],
)
