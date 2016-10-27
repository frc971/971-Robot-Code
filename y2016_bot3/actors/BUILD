package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

filegroup(
  name = 'binaries',
  srcs = [
    ':autonomous_action',
  ],
)

queue_library(
  name = 'autonomous_action_queue',
  srcs = [
    'autonomous_action.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
  ],
)

cc_library(
  name = 'autonomous_action_lib',
  srcs = [
    'autonomous_actor.cc',
  ],
  hdrs = [
    'autonomous_actor.h',
  ],
  deps = [
    ':autonomous_action_queue',
    '//aos/common/util:phased_loop',
    '//aos/common/logging',
    '//aos/common/actions:action_lib',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//y2016_bot3/queues:ball_detector',
    '//y2016_bot3/control_loops/intake:intake_queue',
    '//y2016_bot3/control_loops/drivetrain:drivetrain_base',
    '//y2016_bot3/queues:profile_params',
  ],
)

cc_binary(
  name = 'autonomous_action',
  srcs = [
    'autonomous_actor_main.cc',
  ],
  deps = [
    ':autonomous_action_lib',
    ':autonomous_action_queue',
    '//aos/linux_code:init',
  ],
)
