{
  'targets': [
    {
      'target_name': 'dump_rtprio',
      'type': 'executable',
      'sources': [
        'dump_rtprio.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
      ],
    },
    {
      'target_name': 'complex_thread_local',
      'type': 'static_library',
      'sources': [
        'complex_thread_local.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:once',
        '<(AOS)/common/common.gyp:die',
      ],
    },
    {
      'target_name': 'complex_thread_local_test',
      'type': 'executable',
      'sources': [
        'complex_thread_local_test.cc',
      ],
      'dependencies': [
        'complex_thread_local',
        '<(EXTERNALS):gtest',
        '<(AOS)/common/util/util.gyp:thread',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'init',
      'type': 'static_library',
      'sources': [
        'init.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:shared_mem',
        '<(AOS)/common/common.gyp:die',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'configuration',
      'type': 'static_library',
      'sources': [
        'configuration.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:once',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'core',
      'type': 'executable',
      'sources': [
        'core.cc',
      ],
      'dependencies': [
        'init',
        '<(AOS)/common/util/util.gyp:run_command',
      ],
    },
  ],
}
