package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

filegroup(
  name = 'binaries',
  srcs = [
    ':drivetrain_action',
    ':shoot_action',
  ],
)

queue_library(
  name = 'shoot_action_queue',
  srcs = [
    'shoot_action.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
  ],
)

cc_library(
  name = 'shoot_action_lib',
  srcs = [
    'shoot_actor.cc',
  ],
  hdrs = [
    'shoot_actor.h',
  ],
  deps = [
    ':shoot_action_queue',
    '//aos/common/actions:action_lib',
    '//y2014/queues:profile_params',
    '//aos/common/logging',
    '//y2014/control_loops/shooter:shooter_queue',
    '//y2014/control_loops/claw:claw_queue',
    '//y2014/control_loops/drivetrain:drivetrain_queue',
    '//y2014:constants',
  ],
)

cc_binary(
  name = 'shoot_action',
  srcs = [
    'shoot_actor_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':shoot_action_lib',
    ':shoot_action_queue',
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
    '//y2014:constants',
    '//aos/common:time',
    '//aos/common:math',
    '//aos/common/util:phased_loop',
    '//aos/common/logging',
    '//aos/common/actions:action_lib',
    '//aos/common/logging:queue_logging',
    '//third_party/eigen',
    '//aos/common/util:trapezoid_profile',
    '//y2014/control_loops/drivetrain:drivetrain_queue',
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_binary(
  name = 'drivetrain_action',
  srcs = [
    'drivetrain_actor_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':drivetrain_action_lib',
    ':drivetrain_action_queue',
  ],
)
