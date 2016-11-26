package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

filegroup(
  name = 'binaries',
  srcs = [
    ':drivetrain_action',
  ],
)

queue_library(
  name = 'drivetrain_action_queue',
  srcs = [
    'drivetrain_action.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
  ],
)

cc_library(
  name = 'drivetrain_action_lib',
  srcs = [
    'drivetrain_actor.cc',
  ],
  hdrs = [
    'drivetrain_actor.h',
  ],
  deps = [
    ':drivetrain_action_queue',
    '//aos/common:time',
    '//aos/common:math',
    '//aos/common/util:phased_loop',
    '//aos/common/logging',
    '//aos/common/actions:action_lib',
    '//aos/common/logging:queue_logging',
    '//third_party/eigen',
    '//aos/common/util:trapezoid_profile',
    '//frc971/control_loops/drivetrain:drivetrain_lib',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//y2015_bot3/control_loops/drivetrain:drivetrain_base',
    '//y2015_bot3/control_loops/drivetrain:polydrivetrain_plants',
  ],
)

cc_binary(
  name = 'drivetrain_action',
  srcs = [
    'drivetrain_actor_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common/actions:action_lib',
    ':drivetrain_action_queue',
    ':drivetrain_action_lib',
  ],
)
