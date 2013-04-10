{
  'targets': [
    {
      'target_name': 'http_server',
      'type': 'static_library',
      'sources': [
        'HTTPServer.cpp',
        'evhttp_ctemplate_emitter.cc',
        'ctemplate_cache.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        '<(EXTERNALS):libevent',
        '<(EXTERNALS):ctemplate',
        '<(AOS)/common/common.gyp:once',
      ],
      'export_dependent_settings': [
# Our headers #include headers from both of these.
        '<(EXTERNALS):libevent',
        '<(EXTERNALS):ctemplate',
      ],
    },
    {
      'target_name': 'motor_output',
      'type': 'static_library',
      'sources': [
        'motor_output.cc',
      ],
      'dependencies': [
        '<(AOS)/common/network/network.gyp:socket',
        '<(AOS)/common/common.gyp:common',
        '<(AOS)/common/common.gyp:timing',
        '<(EXTERNALS):WPILib-NetworkRobotValues',
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
