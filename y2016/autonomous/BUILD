package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'auto_lib',
  srcs = [
    'auto.cc',
  ],
  hdrs = [
    'auto.h',
  ],
  deps = [
    '//aos/common/actions:action_lib',
    '//aos/common/controls:control_loop',
    '//aos/common/logging',
    '//aos/common/logging:queue_logging',
    '//aos/common:time',
    '//aos/common/util:phased_loop',
    '//aos/common/util:trapezoid_profile',
    '//frc971/autonomous:auto_queue',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//y2016:constants',
    '//y2016/queues:profile_params',
    '//y2016/actors:drivetrain_action_lib',
  ],
)

cc_binary(
  name = 'auto',
  srcs = [
    'auto_main.cc',
  ],
  deps = [
    ':auto_lib',
    '//aos/linux_code:init',
    '//frc971/autonomous:auto_queue',
  ],
)
