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
    '//aos/common:mutex',
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
    ':constants',
    '//aos/input:joystick_input',
    '//aos/linux_code:init',
    '//aos/common/logging',
    '//aos/common:time',
    '//aos/common/util:log_interval',
    '//aos/common/actions:action_lib',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/queues:gyro',
    '//frc971/autonomous:auto_queue',
    '//y2014/control_loops/claw:claw_queue',
    '//y2014/control_loops/shooter:shooter_queue',
    '//y2014/actors:shoot_action_lib',
  ],
)

aos_downloader(
  name = 'download',
  start_srcs = [
    ':hot_goal_reader',
    ':joystick_reader',
    ':wpilib_interface',
    '//y2014/control_loops/drivetrain:drivetrain',
    '//y2014/control_loops/claw:claw',
    '//y2014/control_loops/shooter:shooter',
    '//y2014/autonomous:auto',
    '//y2014/actors:binaries',
    '//aos:prime_start_binaries',
  ],
  srcs = [
    '//aos:prime_binaries',
  ],
)

cc_binary(
  name = 'hot_goal_reader',
  srcs = [
    'hot_goal_reader.cc',
  ],
  deps = [
    '//aos/common:time',
    '//aos/common/logging',
    '//aos/common/logging:queue_logging',
    '//aos/linux_code:init',
    '//y2014/queues:hot_goal',
  ],
)

cc_binary(
  name = 'wpilib_interface',
  srcs = [
    'wpilib_interface.cc',
  ],
  deps = [
    ':constants',
    '//aos/linux_code:init',
    '//aos/common:stl_mutex',
    '//aos/common/logging',
    '//third_party:wpilib',
    '//y2014/queues:auto_mode',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//y2014/control_loops/shooter:shooter_queue',
    '//y2014/control_loops/claw:claw_queue',
    '//aos/common/controls:control_loop',
    '//aos/common/util:log_interval',
    '//aos/common:time',
    '//aos/common/logging:queue_logging',
    '//aos/common/messages:robot_state',
    '//aos/common/util:phased_loop',
    '//aos/common/util:wrapping_counter',
    '//frc971/wpilib:joystick_sender',
    '//frc971/wpilib:loop_output_handler',
    '//frc971/wpilib:buffered_pcm',
    '//frc971/wpilib:gyro_sender',
    '//frc971/wpilib:dma_edge_counting',
    '//frc971/wpilib:interrupt_edge_counting',
    '//frc971/wpilib:wpilib_robot_base',
    '//frc971/wpilib:encoder_and_potentiometer',
    '//frc971/control_loops:queues',
    '//frc971/wpilib:logging_queue',
    '//frc971/wpilib:wpilib_interface',
    '//frc971/wpilib:pdp_fetcher',
    '//frc971/wpilib:dma',
  ],
)
