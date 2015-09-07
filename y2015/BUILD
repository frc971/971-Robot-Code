package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'constants',
  srcs = [
    'constants.cc',
  ],
  hdrs = [
    'constants.h',
  ],
  deps = [
    '//aos/common/logging',
    '//aos/common:once',
    '//aos/common/network:team_number',
    '//frc971/control_loops:state_feedback_loop',
    '//y2015/control_loops/drivetrain:polydrivetrain_plants',
  ],
)

cc_binary(
  name = 'joystick_reader',
  srcs = [
    'joystick_reader.cc',
  ],
  deps = [
    '//aos/prime/input:joystick_input',
    '//aos/linux_code:init',
    '//aos/common/logging',
    '//aos/common:time',
    '//aos/common/util:log_interval',
    '//aos/common/actions:action_lib',
    '//frc971/queues:gyro',
    '//y2015/control_loops/claw:claw_queue',
    '//y2015/control_loops/drivetrain:drivetrain_queue',
    '//y2015/control_loops/fridge:fridge_queue',
    '//y2015:constants',
    '//frc971/autonomous:auto_queue',
    '//y2015/actors:stack_action_lib',
    '//y2015/actors:stack_and_lift_action_lib',
    '//y2015/actors:stack_and_hold_action_lib',
    '//y2015/actors:pickup_action_lib',
    '//y2015/actors:lift_action_lib',
    '//y2015/actors:held_to_lift_action_lib',
    '//y2015/actors:can_pickup_action_lib',
    '//y2015/actors:score_action_lib',
    '//y2015/actors:horizontal_can_pickup_action_lib',
    '//y2015/actors:fridge_profile_lib',
  ],
)
