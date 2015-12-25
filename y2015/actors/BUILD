package(default_visibility = ['//visibility:public'])

load('/aos/build/queues', 'queue_library')

filegroup(
  name = 'binaries',
  srcs = [
    ':drivetrain_action',
    ':score_action',
    ':pickup_action',
    ':stack_action',
    ':stack_and_lift_action',
    ':stack_and_hold_action',
    ':held_to_lift_action',
    ':can_pickup_action',
    ':horizontal_can_pickup_action',
    ':lift_action',
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
    '//y2015:constants',
    '//aos/common:time',
    '//aos/common:math',
    '//aos/common/util:phased_loop',
    '//aos/common/logging',
    '//aos/common/actions:action_lib',
    '//aos/common/logging:queue_logging',
    '//third_party/eigen',
    '//aos/common/util:trapezoid_profile',
    '//y2015/control_loops/drivetrain:drivetrain_queue',
  ],
)

cc_binary(
  name = 'drivetrain_action',
  srcs = [
    'drivetrain_actor_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common/actions:action_lib',
    ':drivetrain_action_queue',
    ':drivetrain_action_lib',
  ],
)

cc_library(
  name = 'fridge_profile_lib',
  srcs = [
    'fridge_profile_lib.cc',
  ],
  hdrs = [
    'fridge_profile_lib.h',
  ],
  deps = [
    '//aos/common/logging',
    '//aos/common/actions:action_lib',
    '//y2015/control_loops/fridge:fridge_queue',
  ],
)

queue_library(
  name = 'score_action_queue',
  srcs = [
    'score_action.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
  ],
)

cc_library(
  name = 'score_action_lib',
  srcs = [
    'score_actor.cc',
  ],
  hdrs = [
    'score_actor.h',
  ],
  deps = [
    ':score_action_queue',
    '//aos/common/logging',
    '//aos/common/actions:action_lib',
    '//aos/common/controls:control_loop',
    '//y2015:constants',
    '//y2015/control_loops/fridge:fridge_queue',
    '//third_party/eigen',
    '//debian:libm',
    '//y2015/util:kinematics',
  ],
)

cc_binary(
  name = 'score_action',
  srcs = [
    'score_actor_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common/actions:action_lib',
    ':score_action_queue',
    ':score_action_lib',
  ],
)

cc_test(
  name = 'score_action_test',
  srcs = [
    'score_actor_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    '//aos/testing:test_shm',
    '//aos/common/logging:queue_logging',
    '//aos/common:queues',
    '//aos/common:time',
    '//aos/linux_code:init',
    '//aos/common/actions:action_lib',
    '//y2015/control_loops/fridge:fridge_queue',
    '//frc971/control_loops:team_number_test_environment',
    ':score_action_queue',
    ':score_action_lib',
  ],
)

queue_library(
  name = 'pickup_action_queue',
  srcs = [
    'pickup_action.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
  ],
)

cc_library(
  name = 'pickup_action_lib',
  srcs = [
    'pickup_actor.cc',
  ],
  hdrs = [
    'pickup_actor.h',
  ],
  deps = [
    ':pickup_action_queue',
    '//aos/common/logging',
    '//aos/common/actions:action_lib',
    '//aos/common/controls:control_loop',
    '//y2015/control_loops/claw:claw_queue',
  ],
)

cc_binary(
  name = 'pickup_action',
  srcs = [
    'pickup_actor_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common/actions:action_lib',
    ':pickup_action_queue',
    ':pickup_action_lib',
  ],
)

queue_library(
  name = 'can_pickup_action_queue',
  srcs = [
    'can_pickup_action.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
  ],
)

cc_library(
  name = 'can_pickup_action_lib',
  srcs = [
    'can_pickup_actor.cc',
  ],
  hdrs = [
    'can_pickup_actor.h',
  ],
  deps = [
    ':fridge_profile_lib',
    ':can_pickup_action_queue',
    '//aos/common/logging',
    '//aos/common/util:phased_loop',
    '//aos/common/actions:action_lib',
    '//y2015:constants',
    '//y2015/control_loops/claw:claw_queue',
    '//aos/common/controls:control_loop',
  ],
)

cc_binary(
  name = 'can_pickup_action',
  srcs = [
    'can_pickup_actor_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common/actions:action_lib',
    ':can_pickup_action_queue',
    ':can_pickup_action_lib',
  ],
)

queue_library(
  name = 'horizontal_can_pickup_action_queue',
  srcs = [
    'horizontal_can_pickup_action.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
  ],
)

cc_library(
  name = 'horizontal_can_pickup_action_lib',
  srcs = [
    'horizontal_can_pickup_actor.cc',
  ],
  hdrs = [
    'horizontal_can_pickup_actor.h',
  ],
  deps = [
    ':fridge_profile_lib',
    ':horizontal_can_pickup_action_queue',
    '//aos/common/logging',
    '//aos/common/util:phased_loop',
    '//aos/common/actions:action_lib',
    '//y2015:constants',
    '//y2015/control_loops/claw:claw_queue',
    '//aos/common/controls:control_loop',
  ],
)

cc_binary(
  name = 'horizontal_can_pickup_action',
  srcs = [
    'horizontal_can_pickup_actor_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common/actions:action_lib',
    ':horizontal_can_pickup_action_queue',
    ':horizontal_can_pickup_action_lib',
  ],
)

queue_library(
  name = 'held_to_lift_action_queue',
  srcs = [
    'held_to_lift_action.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
    ':lift_action_params',
  ],
)

cc_library(
  name = 'held_to_lift_action_lib',
  srcs = [
    'held_to_lift_actor.cc',
  ],
  hdrs = [
    'held_to_lift_actor.h',
  ],
  deps = [
    ':fridge_profile_lib',
    ':held_to_lift_action_queue',
    ':lift_action_lib',
    '//aos/common/logging',
    '//aos/common/util:phased_loop',
    '//aos/common/actions:action_lib',
    '//y2015:constants',
    '//y2015/control_loops/claw:claw_queue',
    '//aos/common/controls:control_loop',
  ],
)

cc_binary(
  name = 'held_to_lift_action',
  srcs = [
    'held_to_lift_actor_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common/actions:action_lib',
    ':held_to_lift_action_queue',
    ':held_to_lift_action_lib',
  ],
)

queue_library(
  name = 'stack_and_hold_action_queue',
  srcs = [
    'stack_and_hold_action.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
    ':stack_action_params',
  ],
)

cc_library(
  name = 'stack_and_hold_action_lib',
  srcs = [
    'stack_and_hold_actor.cc',
  ],
  hdrs = [
    'stack_and_hold_actor.h',
  ],
  deps = [
    ':fridge_profile_lib',
    ':stack_and_hold_action_queue',
    ':stack_action_lib',
    '//aos/common/logging',
    '//aos/common/util:phased_loop',
    '//aos/common/actions:action_lib',
    '//y2015:constants',
    '//y2015/control_loops/claw:claw_queue',
    '//aos/common/controls:control_loop',
  ],
)

cc_binary(
  name = 'stack_and_hold_action',
  srcs = [
    'stack_and_hold_actor_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common/actions:action_lib',
    ':stack_and_hold_action_queue',
    ':stack_and_hold_action_lib',
  ],
)

queue_library(
  name = 'stack_and_lift_action_queue',
  srcs = [
    'stack_and_lift_action.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
    ':stack_action_params',
    ':lift_action_params',
  ],
)

cc_library(
  name = 'stack_and_lift_action_lib',
  srcs = [
    'stack_and_lift_actor.cc',
  ],
  hdrs = [
    'stack_and_lift_actor.h',
  ],
  deps = [
    ':fridge_profile_lib',
    ':stack_and_lift_action_queue',
    ':stack_action_lib',
    ':lift_action_lib',
    '//aos/common/logging',
    '//aos/common/util:phased_loop',
    '//aos/common/actions:action_lib',
    '//y2015:constants',
    '//y2015/control_loops/claw:claw_queue',
    '//aos/common/controls:control_loop',
  ],
)

cc_binary(
  name = 'stack_and_lift_action',
  srcs = [
    'stack_and_lift_actor_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common/actions:action_lib',
    ':stack_and_lift_action_queue',
    ':stack_and_lift_action_lib',
  ],
)

queue_library(
  name = 'stack_action_queue',
  srcs = [
    'stack_action.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
    ':stack_action_params',
  ],
)

queue_library(
  name = 'stack_action_params',
  srcs = [
    'stack_action_params.q',
  ],
)

cc_test(
  name = 'stack_action_test',
  srcs = [
    'stack_actor_test.cc',
  ],
  deps = [
    '//aos/testing:googletest',
    '//aos/testing:test_shm',
    '//aos/common/logging:queue_logging',
    '//aos/common:queues',
    '//aos/common:time',
    '//aos/linux_code:init',
    '//aos/common/actions:action_lib',
    '//aos/common/controls:control_loop_test',
    '//y2015/control_loops/fridge:fridge_queue',
    '//frc971/control_loops:team_number_test_environment',
    ':stack_action_queue',
    ':stack_action_lib',
  ],
)

cc_library(
  name = 'stack_action_lib',
  srcs = [
    'stack_actor.cc',
  ],
  hdrs = [
    'stack_actor.h',
  ],
  deps = [
    ':fridge_profile_lib',
    ':stack_action_queue',
    '//aos/common/logging',
    '//aos/common/util:phased_loop',
    '//aos/common/actions:action_lib',
    '//y2015:constants',
    '//y2015/control_loops/claw:claw_queue',
  ],
)

cc_binary(
  name = 'stack_action',
  srcs = [
    'stack_actor_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common/actions:action_lib',
    ':stack_action_queue',
    ':stack_action_lib',
  ],
)

queue_library(
  name = 'lift_action_queue',
  srcs = [
    'lift_action.q',
  ],
  deps = [
    '//aos/common/actions:action_queue',
    ':lift_action_params',
  ],
)

queue_library(
  name = 'lift_action_params',
  srcs = [
    'lift_action_params.q',
  ],
)

cc_library(
  name = 'lift_action_lib',
  srcs = [
    'lift_actor.cc',
  ],
  hdrs = [
    'lift_actor.h',
  ],
  deps = [
    ':fridge_profile_lib',
    ':lift_action_queue',
    '//aos/common/logging',
    '//aos/common/actions:action_lib',
    '//y2015:constants',
    '//y2015/control_loops/claw:claw_queue',
  ],
)

cc_binary(
  name = 'lift_action',
  srcs = [
    'lift_actor_main.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common/actions:action_lib',
    ':lift_action_queue',
    ':lift_action_lib',
  ],
)
