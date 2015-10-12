package(default_visibility = ['//visibility:public'])

cc_binary(
  name = 'joystick_reader_bot3',
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
    '//bot3/control_loops/elevator:elevator_lib',
    '//bot3/control_loops/drivetrain:drivetrain_queue',
    '//bot3/control_loops/elevator:elevator_queue',
    '//bot3/control_loops/intake:intake_queue',
    '//bot3/autonomous:auto_queue',
  ],
)
