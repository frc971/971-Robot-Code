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
    '//frc971/autonomous:auto_queue',
    '//aos/controls:control_loop',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//y2014/control_loops/shooter:shooter_queue',
    '//y2014/control_loops/claw:claw_queue',
    '//y2014:constants',
    '//aos/time:time',
    '//aos/util:phased_loop',
    '//aos/util:trapezoid_profile',
    '//aos/logging',
    '//aos/actions:action_lib',
    '//y2014/actors:shoot_action_lib',
    '//y2014/actors:drivetrain_action_lib',
    '//y2014/queues:hot_goal',
    '//aos/logging:queue_logging',
    '//y2014/queues:profile_params',
    '//y2014/queues:auto_mode',
  ],
)

cc_binary(
  name = 'auto',
  srcs = [
    'auto_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//frc971/autonomous:auto_queue',
    ':auto_lib',
  ],
)
