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
    '//y2015_bot3/control_loops/elevator:elevator',
    '//y2015_bot3/autonomous:auto',
    '//y2015_bot3/actors:binaries',
    '//y2015_bot3/wpilib:wpilib_interface',
  ],
  srcs = [
    '//aos:prime_binaries',
  ],
)
