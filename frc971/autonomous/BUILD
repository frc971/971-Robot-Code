package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'auto_queue',
  srcs = [
    'auto.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
  ],
)

cc_library(
  name = 'base_autonomous_actor',
  hdrs = [
    'base_autonomous_actor.h',
  ],
  srcs = [
    'base_autonomous_actor.cc',
  ],
  deps = [
    ':auto_queue',
    '//aos/common/actions:action_lib',
    '//aos/common/util:phased_loop',
    '//aos/common/logging',
    '//frc971/control_loops/drivetrain:drivetrain_config',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
  ],
)
