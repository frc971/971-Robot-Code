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
    '//aos:prime_start_binaries_stripped',
    '//y2014_bot3/autonomous:auto.stripped',
    '//y2014_bot3/control_loops/drivetrain:drivetrain.stripped',
    '//y2014_bot3/control_loops/rollers:rollers.stripped',
    '//y2014_bot3/wpilib:wpilib_interface.stripped',

  ],
  srcs = [
    '//aos:prime_binaries_stripped',
  ],
)
