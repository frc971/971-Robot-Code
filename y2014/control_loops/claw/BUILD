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
    '//aos/common/controls:control_loop',
    '//aos/common/controls:polytope',
    '//aos/common/logging:queue_logging',
    '//aos/common/logging:matrix_logging',
    '//aos/common:math',
    '//debian:libm',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971/control_loops:coerce_goal',
    '//frc971/control_loops:hall_effect_tracker',
    '//y2014:constants',
  ],
)

cc_test(
  name = 'claw_lib_test',
  srcs = [
    'claw_lib_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    ':claw_queue',
    ':claw_lib',
    '//frc971/control_loops:state_feedback_loop',
    '//aos/common/controls:control_loop_test',
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
    '//aos/common/controls:control_loop',
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
