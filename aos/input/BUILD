package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'joystick_input',
  srcs = [
    'joystick_input.cc',
  ],
  hdrs = [
    'joystick_input.h',
  ],
  deps = [
    '//aos/common/input:driver_station_data',
    '//aos/common/messages:robot_state',
    '//aos/common/network:socket',
    '//aos/common/logging',
    '//aos/common/logging:queue_logging',
  ],
)

cc_library(
  name = 'drivetrain_input',
  srcs = [
    'drivetrain_input.cc'
  ],
  hdrs = [
    'drivetrain_input.h',
  ],
  deps = [
    '//aos/common/input:driver_station_data',
    '//aos/common/logging',
    '//aos/common/logging:queue_logging',
    '//aos/common/messages:robot_state',
    '//aos/common/network:socket',
    '//aos/common:math',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/control_loops/drivetrain:drivetrain_config',
  ],
)
