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
    '//aos/input:driver_station_data',
    '//aos/robot_state:robot_state',
    '//aos/logging',
    '//aos/logging:queue_logging',
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
    '//aos/input:driver_station_data',
    '//aos/logging',
    '//aos/logging:queue_logging',
    '//aos/robot_state:robot_state',
    '//aos:math',
    '//frc971/control_loops/drivetrain:drivetrain_queue',
    '//frc971/control_loops/drivetrain:drivetrain_config',
  ],
)

cc_library(
  name = 'driver_station_data',
  srcs = [
    'driver_station_data.cc',
  ],
  hdrs = [
    'driver_station_data.h',
  ],
  deps = [
    '//aos/robot_state:robot_state',
  ],
)
