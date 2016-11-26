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
    '//y2012/control_loops/drivetrain',
    '//y2012/control_loops/accessories',
    '//y2012/wpilib:wpilib_interface',
    '//aos:prime_start_binaries',
  ],
  srcs = [
    '//aos:prime_binaries',
  ],
)
