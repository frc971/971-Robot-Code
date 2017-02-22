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
    '//frc971:shifter_hall_effect',
    '//frc971:constants',
    '//y2016/control_loops/drivetrain:polydrivetrain_plants',
  ],
)

cc_binary(
  name = 'joystick_reader',
  srcs = [
    'joystick_reader.cc',
  ],
  deps = [
    ':constants',
    '//aos/common/actions:action_lib',
    '//aos/common/logging',
    '//aos/common/util:log_interval',
    '//aos/common:time',
    '//aos/input:joystick_input',
    '//aos/linux_code:init',
    '//frc971/autonomous:auto_queue',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/queues:gyro',
    '//y2016/actors:autonomous_action_lib',
    '//y2016/actors:superstructure_action_lib',
    '//y2016/actors:vision_align_action_lib',
    '//y2016/control_loops/shooter:shooter_queue',
    '//y2016/control_loops/superstructure:superstructure_lib',
    '//y2016/control_loops/superstructure:superstructure_queue',
    '//y2016/queues:ball_detector',
  ],
)

aos_downloader(
  name = 'download',
  start_srcs = [
    ':joystick_reader',
    ':wpilib_interface',
    '//aos:prime_start_binaries',
    '//y2016/control_loops/drivetrain:drivetrain',
    '//y2016/control_loops/superstructure:superstructure',
    '//y2016/control_loops/shooter:shooter',
    '//y2016/dashboard:dashboard',
    '//y2016/actors:autonomous_action',
    '//y2016/actors:superstructure_action',
    '//y2016/actors:vision_align_action',
    '//y2016/vision:target_receiver',
  ],
  srcs = [
    '//aos:prime_binaries',
  ],
  dirs = [
    '//y2016/dashboard:www_files',
  ],
)

aos_downloader(
  name = 'download_stripped',
  start_srcs = [
    ':joystick_reader.stripped',
    ':wpilib_interface.stripped',
    '//aos:prime_start_binaries_stripped',
    '//y2016/control_loops/drivetrain:drivetrain.stripped',
    '//y2016/control_loops/superstructure:superstructure.stripped',
    '//y2016/control_loops/shooter:shooter.stripped',
    '//y2016/dashboard:dashboard.stripped',
    '//y2016/actors:autonomous_action.stripped',
    '//y2016/actors:superstructure_action.stripped',
    '//y2016/actors:vision_align_action.stripped',
    '//y2016/vision:target_receiver.stripped',
  ],
  srcs = [
    '//aos:prime_binaries_stripped',
  ],
  dirs = [
    '//y2016/dashboard:www_files',
  ],
)

cc_binary(
  name = 'wpilib_interface',
  srcs = [
    'wpilib_interface.cc',
  ],
  deps = [
    ':constants',
    '//aos/common:stl_mutex',
    '//aos/common/logging',
    '//aos/common:math',
    '//aos/common/controls:control_loop',
    '//aos/common/util:log_interval',
    '//aos/common:time',
    '//aos/common/logging:queue_logging',
    '//aos/common/messages:robot_state',
    '//aos/common/util:phased_loop',
    '//aos/common/util:wrapping_counter',
    '//aos/linux_code:init',
    '//third_party:wpilib',
    '//frc971/autonomous:auto_queue',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/control_loops:queues',
    '//frc971/wpilib:joystick_sender',
    '//frc971/wpilib:loop_output_handler',
    '//frc971/wpilib:buffered_pcm',
    '//frc971/wpilib:gyro_sender',
    '//frc971/wpilib:dma_edge_counting',
    '//frc971/wpilib:interrupt_edge_counting',
    '//frc971/wpilib:wpilib_robot_base',
    '//frc971/wpilib:encoder_and_potentiometer',
    '//frc971/wpilib:logging_queue',
    '//frc971/wpilib:wpilib_interface',
    '//frc971/wpilib:pdp_fetcher',
    '//frc971/wpilib:ADIS16448',
    '//frc971/wpilib:dma',
    '//y2016/control_loops/drivetrain:polydrivetrain_plants',
    '//y2016/control_loops/shooter:shooter_queue',
    '//y2016/control_loops/superstructure:superstructure_queue',
    '//y2016/queues:ball_detector',
  ],
)
