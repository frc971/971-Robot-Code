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
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//y2012/control_loops/accessories:accessories_queue',
    '//frc971/queues:gyro',
    '//frc971/autonomous:auto_queue',
  ],
)

aos_downloader(
  name = 'download',
  start_srcs = [
    ':joystick_reader',
    ':wpilib_interface',
    '//y2012/control_loops/drivetrain',
    '//y2012/control_loops/accessories',
    '//aos:prime_start_binaries',
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
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/control_loops:queues',
    '//frc971/wpilib:buffered_pcm',
    '//frc971/wpilib:dma',
    '//frc971/wpilib:dma_edge_counting',
    '//frc971/wpilib:encoder_and_potentiometer',
    '//frc971/wpilib:gyro_sender',
    '//frc971/wpilib:interrupt_edge_counting',
    '//frc971/wpilib:joystick_sender',
    '//frc971/wpilib:logging_queue',
    '//frc971/wpilib:loop_output_handler',
    '//frc971/wpilib:wpilib_interface',
    '//frc971/wpilib:wpilib_robot_base',
    '//third_party:wpilib',
    '//y2012/control_loops/accessories:accessories_queue',
  ],
)
