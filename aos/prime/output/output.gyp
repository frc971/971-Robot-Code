{
  'targets': [
    {
      'target_name': 'motor_output',
      'type': 'static_library',
      'sources': [
        'motor_output.cc',
      ],
      'dependencies': [
        '<(AOS)/common/network/network.gyp:socket',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(EXTERNALS):WPILib-NetworkRobotValues',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/messages/messages.gyp:robot_state',
        '<(AOS)/common/util/util.gyp:log_interval',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/network/network.gyp:socket',
        '<(EXTERNALS):WPILib-NetworkRobotValues',
        '<(AOS)/common/util/util.gyp:log_interval',
      ],
    },
    {
      'target_name': 'motor_output_test',
      'type': 'executable',
      'sources': [
        'motor_output_test.cc',
      ],
      'dependencies': [
        'motor_output',
        '<(EXTERNALS):gtest',
      ],
    },
  ],
}
