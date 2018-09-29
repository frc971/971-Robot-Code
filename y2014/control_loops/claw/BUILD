package(default_visibility = ['//visibility:public'])

load('//aos/build:queues.bzl', 'queue_library')

cc_binary(
  name = 'replay_claw',
  srcs = [
    'replay_claw.cc',
  ],
  deps = [
    ':claw_queue',
    '//aos/controls:replay_control_loop',
    '//aos/linux_code:init',
  ],
)

queue_library(
  name = 'claw_queue',
  srcs = [
    'claw.q',
  ],
  deps = [
    '//aos/controls:control_loop_queues',
    '//frc971/control_loops:queues',
  ],
)

genrule(
  name = 'genrule_claw',
  visibility = ['//visibility:private'],
  cmd = '$(location //y2014/control_loops/python:claw) $(OUTS)',
  tools = [
    '//y2014/control_loops/python:claw',
  ],
  outs = [
    'claw_motor_plant.h',
    'claw_motor_plant.cc',
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
    '//aos/controls:control_loop',
    '//aos/controls:polytope',
    '//aos/logging:queue_logging',
    '//aos/logging:matrix_logging',
    '//aos:math',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:coerce_goal',
    '//frc971/control_loops:hall_effect_tracker',
    '//y2014:constants',
  ],
  linkopts = [
    '-lm',
  ],
)

cc_test(
  name = 'claw_lib_test',
  srcs = [
    'claw_lib_test.cc',
  ],
  deps = [
    ':claw_lib',
    ':claw_queue',
    '//aos/controls:control_loop_test',
    '//aos/testing:googletest',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:team_number_test_environment',
  ],
)

cc_binary(
  name = 'claw_calibration',
  srcs = [
    'claw_calibration.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':claw_queue',
    '//aos/controls:control_loop',
    '//y2014:constants',
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
