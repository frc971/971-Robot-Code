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
    '//frc971/autonomous:auto_queue',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/queues:gyro',
    '//y2016_bot4/control_loops/drivetrain:drivetrain_base',
  ],
)

aos_downloader(
  name = 'download',
  start_srcs = [
    ':joystick_reader',
    '//aos:prime_start_binaries',
    '//y2016_bot4/control_loops/drivetrain:drivetrain',
    '//y2016_bot4/wpilib:wpilib_interface',
  ],
  srcs = [
    '//aos:prime_binaries',
  ],
)

aos_downloader(
  name = 'download_stripped',
  start_srcs = [
    ':joystick_reader.stripped',
    '//aos:prime_start_binaries_stripped',
    '//y2016_bot4/control_loops/drivetrain:drivetrain.stripped',
    '//y2016_bot4/wpilib:wpilib_interface.stripped',
  ],
  srcs = [
    '//aos:prime_binaries_stripped',
  ],
  default_target = 'roboRIO-6971-frc.local',
)
