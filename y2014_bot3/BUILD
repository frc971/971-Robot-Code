package(default_visibility = ['//visibility:public'])

cc_binary(
  name = 'joystick_reader_y2014_bot3',
  srcs = [
    'joystick_reader.cc',
  ],
  deps = [
    '//aos/prime/input:joystick_input',
    '//aos/linux_code:init',
    '//aos/common/logging',
    '//aos/common:time',
    '//aos/common/util:log_interval',
    '//aos/common/actions:action_lib',

    '//frc971/queues:gyro',
    '//y2014_bot3/autonomous:auto_queue',
    '//y2014_bot3/control_loops/rollers:rollers_queue',
    '//y2014_bot3/control_loops/drivetrain:drivetrain_queue',
  ],
)
