load('//aos/downloader:downloader.bzl', 'aos_downloader')

cc_binary(
  name = 'joystick_reader',
  srcs = [
    'joystick_reader.cc',
  ],
  deps = [
    '//aos/common/actions:action_lib',
    '//aos/common/logging',
    '//aos/common/util:log_interval',
    '//aos/common:time',
    '//aos/input:drivetrain_input',
    '//aos/input:joystick_input',
    '//aos/linux_code:init',
    '//frc971/autonomous:auto_queue',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/autonomous:base_autonomous_actor',
    '//y2017_bot3/control_loops/drivetrain:drivetrain_base',
    '//y2017_bot3/control_loops/superstructure:superstructure_queue',
    '//y2017_bot3/control_loops/superstructure:superstructure_lib',
  ],
)

cc_binary(
  name = 'wpilib_interface',
  srcs = [
    'wpilib_interface.cc',
  ],
  deps = [
    '//aos/common/controls:control_loop',
    '//aos/common/logging',
    '//aos/common/logging:queue_logging',
    '//aos/common/messages:robot_state',
    '//aos/common/util:log_interval',
    '//aos/common/util:phased_loop',
    '//aos/common:math',
    '//aos/common:stl_mutex',
    '//aos/common:time',
    '//aos/linux_code:init',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/control_loops:queues',
    '//frc971/wpilib:ADIS16448',
    '//frc971/wpilib:buffered_pcm',
    '//frc971/wpilib:dma',
    '//frc971/wpilib:dma_edge_counting',
    '//frc971/wpilib:encoder_and_potentiometer',
    '//frc971/wpilib:gyro_sender',
    '//frc971/wpilib:interrupt_edge_counting',
    '//frc971/wpilib:joystick_sender',
    '//frc971/wpilib:logging_queue',
    '//frc971/wpilib:loop_output_handler',
    '//frc971/wpilib:pdp_fetcher',
    '//frc971/wpilib:wpilib_interface',
    '//frc971/wpilib:wpilib_robot_base',
    '//third_party:wpilib',
    '//y2017_bot3/control_loops/drivetrain:polydrivetrain_plants',
    '//y2017_bot3/control_loops/superstructure:superstructure_queue',
  ],
  restricted_to = ['//tools:roborio'],
)

aos_downloader(
  name = 'download',
  start_srcs = [
    ':wpilib_interface',
    ':joystick_reader',
    '//aos:prime_start_binaries',
    '//y2017_bot3/control_loops/drivetrain:drivetrain',
    '//y2017_bot3/control_loops/superstructure:superstructure',
  ],
  srcs = [
    '//aos:prime_binaries',
  ],
  restricted_to = ['//tools:roborio'],
)

aos_downloader(
  name = 'download_stripped',
  start_srcs = [
    ':wpilib_interface.stripped',
    ':joystick_reader.stripped',
    '//aos:prime_start_binaries_stripped',
    '//y2017_bot3/control_loops/drivetrain:drivetrain.stripped',
    '//y2017_bot3/control_loops/superstructure:superstructure.stripped',
  ],
  srcs = [
    '//aos:prime_binaries_stripped',
  ],
  restricted_to = ['//tools:roborio'],
)
