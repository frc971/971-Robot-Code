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
    '//y2014/control_loops/drivetrain:polydrivetrain_plants',
    '//frc971:shifter_hall_effect',
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
    '//y2014/control_loops/drivetrain:drivetrain_queue',
    '//y2014:constants',
    '//frc971/queues:gyro',
    '//frc971/autonomous:auto_queue',
    '//y2014/control_loops/claw:claw_queue',
    '//y2014/control_loops/shooter:shooter_queue',
    '//y2014/actors:shoot_action_lib',
  ],
)
