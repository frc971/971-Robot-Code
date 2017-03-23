package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

filegroup(
  name = 'binaries',
  srcs = [
    ':autonomous_action',
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
    '//aos/common/util:phased_loop',
    '//aos/common/logging',
    '//aos/common/actions:action_lib',
    '//frc971/autonomous:base_autonomous_actor',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/control_loops/drivetrain:drivetrain_config',
    '//y2017/control_loops/drivetrain:drivetrain_base',
    '//y2017/control_loops/superstructure:superstructure_queue',
  ],
)

cc_binary(
  name = 'autonomous_action',
  srcs = [
    'autonomous_actor_main.cc',
  ],
  deps = [
    ':autonomous_action_lib',
    '//aos/linux_code:init',
    '//frc971/autonomous:auto_queue',
  ],
)
