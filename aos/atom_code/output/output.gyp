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
        'MotorOutput.cpp',
      ],
      'dependencies': [
        '<(AOS)/common/network/network.gyp:socket',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/network/network.gyp:socket',
      ],
    },
  ],
}
