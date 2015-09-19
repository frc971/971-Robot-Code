package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

cc_binary(
  name = 'replay_claw',
  srcs = [
    'replay_claw.cc',
  ],
  deps = [
    ':claw_queue',
    '//aos/common/controls:replay_control_loop',
    '//aos/linux_code:init',
  ],
)

queue_library(
  name = 'claw_queue',
  srcs = [
    'claw.q',
  ],
  deps = [
    '//aos/common/controls:control_loop_queues',
    '//frc971/control_loops:queues',
  ],
)

cc_library(
  name = 'claw_lib',
  srcs = [
    'claw.cc',
    'claw_motor_plant.cc',
  ],
  hdrs = [
    'claw.h',
    'claw_motor_plant.h',
  ],
  deps = [
    ':claw_queue',
    '//aos/common/controls:control_loop',
    '//aos/common:time',
    '//aos/common/util:trapezoid_profile',
    '//y2015:constants',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/zeroing:zeroing',
  ],
)

cc_test(
  name = 'claw_lib_test',
  srcs = [
    'claw_lib_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    ':claw_lib',
    '//frc971/control_loops:state_feedback_loop',
    '//aos/common/controls:control_loop_test',
    '//aos/common:time',
    '//frc971/control_loops:position_sensor_sim',
    '//frc971/control_loops:team_number_test_environment',
  ],
)

cc_binary(
  name = 'claw',
  srcs = [
    'claw_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':claw_lib',
  ],
)
