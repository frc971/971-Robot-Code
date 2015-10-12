package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

queue_library(
  name = 'auto_queue',
  srcs = [
    'auto.q',
  ],
)

cc_library(
  name = 'auto_lib',
  srcs = [
    'auto.cc',
  ],
  hdrs = [
    'auto.h',
  ],
  deps = [
    ':auto_queue',
    '//aos/common/controls:control_loop',
    '//bot3/control_loops/drivetrain:drivetrain_queue',
    '//bot3/control_loops/drivetrain:drivetrain_lib',
    '//bot3/actors:drivetrain_action_lib',
    '//bot3/control_loops/elevator:elevator_queue',
    '//bot3/control_loops/intake:intake_queue',
    '//aos/common:time',
    '//aos/common/util:phased_loop',
    '//aos/common/util:trapezoid_profile',
    '//aos/common/logging',
    '//aos/common/logging:queue_logging',
  ],
)

cc_binary(
  name = 'auto_bot3',
  srcs = [
    'auto_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    ':auto_queue',
    ':auto_lib',
  ],
)
