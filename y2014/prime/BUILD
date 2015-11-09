package(default_visibility = ['//visibility:public'])

load('/aos/prime/downloader/downloader', 'aos_downloader')

aos_downloader(
  name = 'download',
  start_srcs = [
    '//y2014/control_loops/drivetrain:drivetrain',
    '//y2014/control_loops/claw:claw',
    '//y2014/control_loops/shooter:shooter',
    '//y2014/autonomous:auto',
    '//y2014/wpilib:wpilib_interface',
    '//y2014:joystick_reader',
    '//y2014/actors:binaries',
    ':hot_goal_reader',
    '//aos:prime_start_binaries',
  ],
  srcs = [
    '//aos:prime_binaries',
  ],
)

cc_binary(
  name = 'hot_goal_reader',
  srcs = [
    'hot_goal_reader.cc',
  ],
  deps = [
    '//aos/common:time',
    '//aos/common/logging',
    '//aos/common/logging:queue_logging',
    '//aos/linux_code:init',
    '//y2014/queues:hot_goal',
  ],
)
