package(default_visibility = ['//visibility:public'])

load('//aos/build:queues.bzl', 'queue_library')

cc_binary(
  name = 'basic_test',
  srcs = ['basic_test.cc'],
  deps = [
    ':basic_queue',
    ':basic',
    '//aos/testing:googletest',
    '//aos:queues',
    '//aos/controls:control_loop_test',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:team_number_test_environment',
  ],
  testonly = 1,
)

cc_library(
  name = 'basic',
  srcs = ['basic.cc'],
  hdrs = ['basic.h'],
  deps = [
    ':basic_queue',
    '//aos/controls:control_loop',
  ],
)

queue_library(
  name = 'basic_queue',
  srcs = [
    'basic.q',
  ],
  deps = [
    '//aos/controls:control_loop_queues',
    '//frc971/control_loops:queues',
  ],
)
