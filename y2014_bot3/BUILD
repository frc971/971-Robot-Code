load('/aos/downloader/downloader', 'aos_downloader')

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
    '//y2014_bot3/autonomous:auto_queue',
    '//y2014_bot3/control_loops/rollers:rollers_queue',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
  ],
)

aos_downloader(
  name = 'download_stripped',
  start_srcs = [
    ':joystick_reader.stripped',
    ':wpilib_interface.stripped',
    '//aos:prime_start_binaries_stripped',
    '//y2014_bot3/autonomous:auto.stripped',
    '//y2014_bot3/control_loops/drivetrain:drivetrain.stripped',
    '//y2014_bot3/control_loops/rollers:rollers.stripped',

  ],
  srcs = [
    '//aos:prime_binaries_stripped',
  ],
)

cc_binary(
  name = 'wpilib_interface',
  srcs = [
    'wpilib_interface.cc',
  ],
  deps = [
    '//aos/linux_code:init',
    '//aos/common:stl_mutex',
    '//aos/common/logging',
    '//third_party:wpilib',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
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
    '//frc971/control_loops:queues',
    '//frc971/wpilib:logging_queue',
    '//frc971/wpilib:wpilib_robot_base',
    '//frc971/wpilib:wpilib_interface',
    '//frc971/wpilib:pdp_fetcher',
    '//frc971/wpilib:dma',
    '//y2014_bot3/autonomous:auto_queue',
    '//y2014_bot3/control_loops/rollers:rollers_lib',
    '//y2014_bot3/control_loops/drivetrain:drivetrain_base',
  ],
)
