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
    '//aos/common/logging:queue_logging',
    '//aos/common/messages:robot_state',
    '//aos/common:time',
    '//aos/testing:googletest',
    '//aos/testing:test_shm',
  ],
  testonly = True,
)

cc_library(
  name = 'polytope',
  hdrs = [
    'polytope.h',
  ],
  deps = [
    '//third_party/eigen',
    '//third_party/cddlib',
    '//aos/common/logging',
    '//aos/common/logging:matrix_logging',
  ],
)

cc_test(
  name = 'polytope_test',
  srcs = [
    'polytope_test.cc',
  ],
  deps = [
    ':polytope',
    '//aos/testing:googletest',
    '//third_party/eigen',
    '//third_party/googletest:googlemock',
    '//aos/testing:test_logging',
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
    '//aos/common:time',
    ':control_loop_queues',
    '//aos/common/logging:queue_logging',
    '//aos/common/util:log_interval',
    '//aos/common:queues',
  ],
)
