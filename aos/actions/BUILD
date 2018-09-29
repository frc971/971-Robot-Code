package(default_visibility = ['//visibility:public'])

load('//aos/build:queues.bzl', 'queue_library')

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
    '//aos/logging',
    '//aos:queues',
    '//aos/logging:queue_logging',
    '//aos/time:time',
    '//aos/controls:control_loop',
    '//aos/util:phased_loop',
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
    '//aos/logging',
    '//aos/logging:queue_logging',
    '//aos:queues',
    '//aos/time:time',
    ':action_queue',
    '//aos:event',
  ],
)
