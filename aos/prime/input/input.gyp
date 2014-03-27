{
  'targets': [
    {
      'target_name': 'joystick_input',
      'type': 'static_library',
      'sources': [
        'joystick_input.cc',
      ],
      'dependencies': [
        '<(AOS)/common/input/input.gyp:driver_station_data',
        '<(AOS)/common/messages/messages.gyp:robot_state',
        '<(AOS)/common/network/network.gyp:socket',
        '<(EXTERNALS):WPILib-NetworkRobotValues',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/input/input.gyp:driver_station_data',
      ],
    },
  ],
}
