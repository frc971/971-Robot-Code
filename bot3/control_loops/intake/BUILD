package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'intake_queue',
  srcs = [
    'intake.q',
  ],
  deps = [
    '//aos/common/controls:control_loop_queues',
  ],
)

cc_library(
  name = 'intake_lib',
  srcs = [
    'intake.cc',
  ],
  hdrs = [
    'intake.h',
  ],
  deps = [
    ':intake_queue',
    '//aos/common/controls:control_loop',
  ],
)

cc_test(
  name = 'intake_lib_test',
  srcs = [
    'intake_lib_test.cc',
  ],
  deps = [
    '//third_party/googletest',
    ':intake_lib',
    '//frc971/control_loops:state_feedback_loop',
    '//aos/common/controls:control_loop_test',
    '//aos/common:time',
    '//frc971/control_loops:team_number_test_environment',
  ],
)

cc_binary(
  name = 'intake',
  srcs = [
    'intake_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':intake_lib',
  ],
)
