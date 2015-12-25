package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

cc_library(
  name = 'action_lib',
  srcs = [
    'actions.cc',
    'actor.cc',
  ],
  hdrs = [
    'actions.h',
    'actor.h',
  ],
  deps = [
    '//aos/common/logging',
    '//aos/common:queues',
    '//aos/common/logging:queue_logging',
    '//aos/common:time',
    '//aos/common/controls:control_loop',
    '//aos/common/util:phased_loop',
  ],
)

queue_library(
  name = 'action_queue',
  srcs = [
    'actions.q',
  ],
)

queue_library(
  name = 'test_action_queue',
  srcs = [
    'test_action.q',
  ],
  deps = [
    ':action_queue',
  ],
)

cc_test(
  name = 'action_test',
  srcs = [
    'action_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    ':action_lib',
    ':test_action_queue',
    '//aos/testing:test_shm',
    '//aos/common/logging',
    '//aos/common/logging:queue_logging',
    '//aos/common:queues',
    '//aos/common:time',
    ':action_queue',
    '//aos/common:event',
  ],
)
