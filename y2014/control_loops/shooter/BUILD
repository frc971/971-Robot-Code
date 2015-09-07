package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

cc_binary(
  name = 'replay_shooter',
  srcs = [
    'replay_shooter.cc',
  ],
  deps = [
    ':shooter_queue',
    '//aos/common/controls:replay_control_loop',
    '//aos/linux_code:init',
  ],
)

queue_library(
  name = 'shooter_queue',
  srcs = [
    'shooter.q',
  ],
)

cc_library(
  name = 'shooter_lib',
  srcs = [
    'shooter.cc',
    'shooter_motor_plant.cc',
    'unaugmented_shooter_motor_plant.cc',
  ],
  deps = [
    ':shooter_queue',
    '//aos/common/controls:control_loop',
    '//y2014:constants',
    '//frc971/control_loops:state_feedback_loop',
    '//aos/common/logging:queue_logging',
  ],
)

cc_binary(
  name = 'shooter_lib_test',
  srcs = [
    'shooter_lib_test.cc',
  ],
  deps = [
    '//third_party/gtest',
    ':shooter_queue',
    ':shooter_lib',
    '//aos/common/controls:control_loop_test',
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_binary(
  name = 'shooter',
  srcs = [
    'shooter_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':shooter_lib',
  ],
)
