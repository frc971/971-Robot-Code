package(default_visibility = ['//visibility:public'])

load('//aos/build:queues.bzl', 'queue_library')

filegroup(
  name = 'binaries',
  srcs = [
    ':drivetrain_action',
    ':shoot_action',
  ],
)

filegroup(
  name = 'binaries.stripped',
  srcs = [
    ':drivetrain_action.stripped',
    ':shoot_action.stripped',
  ],
)

queue_library(
  name = 'shoot_action_queue',
  srcs = [
    'shoot_action.q',
  ],
  deps = [
    '//aos/actions:action_queue',
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
    '//aos/actions:action_lib',
    '//y2014/queues:profile_params',
    '//aos/logging',
    '//y2014/control_loops/shooter:shooter_queue',
    '//y2014/control_loops/claw:claw_queue',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//y2014:constants',
  ],
)

cc_binary(
  name = 'shoot_action',
  srcs = [
    'shoot_actor_main.cc',
  ],
  deps = [
    '//aos:init',
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
    '//aos/actions:action_queue',
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
    '//aos/time:time',
    '//aos:math',
    '//aos/util:phased_loop',
    '//aos/logging',
    '//aos/actions:action_lib',
    '//aos/logging:queue_logging',
    '//third_party/eigen',
    '//aos/util:trapezoid_profile',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/control_loops:state_feedback_loop',
  ],
)

cc_binary(
  name = 'drivetrain_action',
  srcs = [
    'drivetrain_actor_main.cc',
  ],
  deps = [
    '//aos:init',
    ':drivetrain_action_lib',
    ':drivetrain_action_queue',
  ],
)
