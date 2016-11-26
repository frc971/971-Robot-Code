package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

cc_library(
  name = 'auto_lib',
  srcs = [
    'auto.cc',
  ],
  hdrs = [
    'auto.h',
  ],
  deps = [
    '//frc971/autonomous:auto_queue',
    '//aos/common/controls:control_loop',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//y2015:constants',
    '//aos/common:time',
    '//aos/common/util:phased_loop',
    '//aos/common/util:trapezoid_profile',
    '//aos/common/logging',
    '//y2015/actors:drivetrain_action_lib',
    '//aos/common/logging:queue_logging',
    '//y2015/control_loops/claw:claw_queue',
    '//y2015/control_loops/fridge:fridge_queue',
    '//y2015/actors:stack_action_lib',
    '//y2015/actors:held_to_lift_action_lib',
    '//y2015/actors:pickup_action_lib',
    ':auto_queue',
  ],
)

cc_binary(
  name = 'auto',
  srcs = [
    'auto_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//frc971/autonomous:auto_queue',
    ':auto_lib',
  ],
)

queue_library(
  name = 'auto_queue',
  srcs = [
    'auto.q',
  ],
)
