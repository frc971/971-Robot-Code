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
        '<(AOS)/common/common.gyp:timing',
        '<(EXTERNALS):WPILib-NetworkRobotValues',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/network/network.gyp:socket',
        '<(EXTERNALS):WPILib-NetworkRobotValues',
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
