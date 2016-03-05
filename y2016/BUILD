load('/aos/downloader/downloader', 'aos_downloader')

cc_library(
  name = 'constants',
  visibility = ['//visibility:public'],
  srcs = [
    'constants.cc',
  ],
  hdrs = [
    'constants.h',
  ],
  deps = [
    '//aos/common/logging',
    '//aos/common:once',
    '//aos/common/network:team_number',
    '//aos/common:mutex',
    '//frc971/control_loops:state_feedback_loop',
    '//frc971:shifter_hall_effect',
    '//frc971:constants',
    '//y2016/control_loops/drivetrain:polydrivetrain_plants',
  ],
)

cc_binary(
  name = 'joystick_reader',
  srcs = [
    'joystick_reader.cc',
  ],
  deps = [
    ':constants',
    '//aos/input:joystick_input',
    '//aos/linux_code:init',
    '//aos/common/logging',
    '//aos/common:time',
    '//aos/common/util:log_interval',
    '//aos/common/actions:action_lib',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/queues:gyro',
    '//frc971/autonomous:auto_queue',
    '//y2016/control_loops/shooter:shooter_queue',
    '//y2016/control_loops/superstructure:superstructure_queue',
  ],
)

aos_downloader(
  name = 'download',
  start_srcs = [
    ':joystick_reader',
    '//aos:prime_start_binaries',
    '//y2016/control_loops/drivetrain:drivetrain',
    '//y2016/control_loops/superstructure:superstructure',
    '//y2016/control_loops/shooter:shooter',
    '//y2016/autonomous:auto',
    '//y2016/wpilib:wpilib_interface',
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
    '//y2016/control_loops/drivetrain:drivetrain.stripped',
    '//y2016/control_loops/superstructure:superstructure.stripped',
    '//y2016/control_loops/shooter:shooter.stripped',
    '//y2016/autonomous:auto.stripped',
    '//y2016/wpilib:wpilib_interface.stripped',
  ],
  srcs = [
    '//aos:prime_binaries_stripped',
  ],
)
