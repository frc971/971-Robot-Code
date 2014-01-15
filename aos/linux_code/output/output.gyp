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
        '<(EXTERNALS):libevent',
        '<(EXTERNALS):ctemplate',
        '<(AOS)/common/common.gyp:once',
        '<(AOS)/common/common.gyp:scoped_fd',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):libevent',
        '<(EXTERNALS):ctemplate',
      ],
    },
  ],
}
