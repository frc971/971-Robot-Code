package(default_visibility = ['//visibility:public'])

load('//aos/build:queues.bzl', 'queue_library')

queue_library(
  name = 'auto_queue',
  srcs = [
    'auto.q',
  ],
  deps = [
    '//aos/actions:action_queue',
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
    '//aos/actions:action_lib',
    '//aos/util:phased_loop',
    '//aos/logging',
    '//frc971/control_loops/drivetrain:drivetrain_config',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
  ],
)
