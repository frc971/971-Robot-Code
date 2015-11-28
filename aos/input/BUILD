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
