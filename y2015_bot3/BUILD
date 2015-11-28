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
    '//y2015_bot3/control_loops/elevator:elevator_lib',
    '//y2015_bot3/control_loops/drivetrain:drivetrain_queue',
    '//y2015_bot3/control_loops/elevator:elevator_queue',
    '//y2015_bot3/control_loops/intake:intake_queue',
    '//y2015_bot3/autonomous:auto_queue',
  ],
)

aos_downloader(
  name = 'download',
  start_srcs = [
    '//aos:prime_start_binaries',
    '//y2015_bot3/control_loops/drivetrain:drivetrain',
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
