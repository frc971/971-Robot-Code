load('/aos/downloader/downloader', 'aos_downloader')

cc_library(
  name = 'constants',
  visibility = ['//visibility:public'],
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
    '//frc971:shifter_hall_effect',
    '//frc971:constants',
    '//y2015/control_loops/drivetrain:polydrivetrain_plants',
    '//y2015/autonomous:auto_queue',
  ],
)

cc_binary(
  name = 'joystick_reader',
  srcs = [
    'joystick_reader.cc',
  ],
  deps = [
    '//aos/input:joystick_input',
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

aos_downloader(
  name = 'download',
  start_srcs = [
    '//aos:prime_start_binaries',
    '//y2015/control_loops/drivetrain:drivetrain',
    '//y2015/control_loops/fridge:fridge',
    '//y2015/control_loops/claw:claw',
    '//y2015/autonomous:auto',
    ':joystick_reader',
    '//y2015/http_status:http_status',
    '//y2015/actors:binaries',
    '//y2015/wpilib:wpilib_interface',
  ],
  srcs = [
    '//aos:prime_binaries',
  ],
)
