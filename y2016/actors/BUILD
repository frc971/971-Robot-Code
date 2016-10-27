package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

filegroup(
  name = 'binaries',
  srcs = [
    ':superstructure_action',
    ':autonomous_action',
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
    ':vision_align_action_lib',
    '//aos/common/util:phased_loop',
    '//aos/common/logging',
    '//aos/common/actions:action_lib',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//y2016/queues:ball_detector',
    '//y2016/control_loops/superstructure:superstructure_queue',
    '//y2016/control_loops/shooter:shooter_queue',
    '//y2016/control_loops/drivetrain:drivetrain_base',
    '//y2016/queues:profile_params',
    '//y2016/vision:vision_queue',
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

queue_library(
  name = 'vision_align_action_queue',
  srcs = [
    'vision_align_action.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
  ],
)

cc_library(
  name = 'vision_align_action_lib',
  srcs = [
    'vision_align_actor.cc',
  ],
  hdrs = [
    'vision_align_actor.h',
  ],
  deps = [
    ':vision_align_action_queue',
    '//aos/common:time',
    '//aos/common:math',
    '//aos/common/util:phased_loop',
    '//aos/common/logging',
    '//aos/common/actions:action_lib',
    '//aos/common/logging:queue_logging',
    '//aos/common/util:trapezoid_profile',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//third_party/eigen',
    '//y2016:constants',
    '//y2016/control_loops/drivetrain:drivetrain_base',
    '//y2016/vision:vision_queue',
  ],
)

cc_binary(
  name = 'vision_align_action',
  srcs = [
    'vision_align_actor_main.cc',
  ],
  deps = [
    ':vision_align_action_lib',
    ':vision_align_action_queue',
    '//aos/linux_code:init',
  ],
)
