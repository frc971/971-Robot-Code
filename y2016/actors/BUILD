package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

filegroup(
  name = 'binaries',
  srcs = [
    ':drivetrain_action',
    ':superstructure_action',
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
    '//aos/common/util:trapezoid_profile',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/control_loops:state_feedback_loop',
    '//third_party/eigen',
    '//y2016:constants',
  ],
)

cc_binary(
  name = 'drivetrain_action',
  srcs = [
    'drivetrain_actor_main.cc',
  ],
  deps = [
    ':drivetrain_action_lib',
    ':drivetrain_action_queue',
    '//aos/linux_code:init',
  ],
)

queue_library(
  name = 'superstructure_action_queue',
  srcs = [
    'superstructure_action.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
  ],
)

cc_library(
  name = 'superstructure_action_lib',
  srcs = [
    'superstructure_actor.cc',
  ],
  hdrs = [
    'superstructure_actor.h',
  ],
  deps = [
    ':superstructure_action_queue',
    '//aos/common/util:phased_loop',
    '//aos/common/logging',
    '//aos/common/actions:action_lib',
    '//y2016/control_loops/superstructure:superstructure_queue',
  ],
)

cc_binary(
  name = 'superstructure_action',
  srcs = [
    'superstructure_actor_main.cc',
  ],
  deps = [
    ':superstructure_action_lib',
    ':superstructure_action_queue',
    '//aos/linux_code:init',
  ],
)
