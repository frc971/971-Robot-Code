load('/aos/downloader/downloader', 'aos_downloader')


aos_downloader(
  name = 'download',
  start_srcs = [
    ':joystick_reader',
    '//aos:prime_start_binaries',
    '//y2018/control_loops/drivetrain:drivetrain',
  ],
  srcs = [
    '//aos:prime_binaries',
  ],
  restricted_to = ['//tools:roborio'],
)

aos_downloader(
  name = 'download_stripped',
  start_srcs = [
   ':joystick_reader.stripped',
    '//aos:prime_start_binaries_stripped',
    '//y2018/control_loops/drivetrain:drivetrain.stripped',
  ],
  srcs = [
    '//aos:prime_binaries_stripped',
  ],
  restricted_to = ['//tools:roborio'],
)

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
    '//aos/input:drivetrain_input',
    '//aos/linux_code:init',
    '//frc971/autonomous:auto_queue',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//y2018/control_loops/drivetrain:drivetrain_base',
  ],
)
