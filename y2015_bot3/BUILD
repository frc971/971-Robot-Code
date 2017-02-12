load('/aos/downloader/downloader', 'aos_downloader')

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
    '//aos/input:joystick_input',
    '//aos/linux_code:init',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/queues:gyro',
    '//y2015_bot3/autonomous:auto_queue',
    '//y2015_bot3/control_loops/elevator:elevator_lib',
    '//y2015_bot3/control_loops/elevator:elevator_queue',
    '//y2015_bot3/control_loops/intake:intake_queue',
  ],
)

aos_downloader(
  name = 'download',
  start_srcs = [
    '//aos:prime_start_binaries',
    '//y2015_bot3/control_loops/intake:intake',
    ':joystick_reader',
    ':wpilib_interface',
    '//y2015_bot3/control_loops/elevator:elevator',
    '//y2015_bot3/autonomous:auto',
    '//y2015_bot3/actors:binaries',
  ],
  srcs = [
    '//aos:prime_binaries',
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
    '//aos/common/util:wrapping_counter',
    '//aos/common:stl_mutex',
    '//aos/common:time',
    '//aos/linux_code:init',
    '//frc971/control_loops/drivetrain:drivetrain_lib',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/control_loops:queues',
    '//frc971/wpilib:buffered_pcm',
    '//frc971/wpilib:dma',
    '//frc971/wpilib:gyro_sender',
    '//frc971/wpilib:joystick_sender',
    '//frc971/wpilib:logging_queue',
    '//frc971/wpilib:loop_output_handler',
    '//frc971/wpilib:pdp_fetcher',
    '//frc971/wpilib:wpilib_interface',
    '//frc971/wpilib:wpilib_robot_base',
    '//third_party:wpilib',
    '//y2015_bot3/autonomous:auto_queue',
    '//y2015_bot3/control_loops/drivetrain:drivetrain_base',
    '//y2015_bot3/control_loops/elevator:elevator_lib',
    '//y2015_bot3/control_loops/intake:intake_lib',
  ],
)
