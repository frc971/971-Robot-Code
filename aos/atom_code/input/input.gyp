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
        '<(AOS)/common/messages/messages.gyp:aos_queues',
        '<(AOS)/common/network/network.gyp:socket',
        '<(AOS)/common/common.gyp:common',
        '<(EXTERNALS):WPILib-NetworkRobotValues',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/input/input.gyp:driver_station_data',
      ],
    },
  ],
}
