{
  'targets': [
    {
      'target_name': 'aos_sync',
      'type': 'static_library',
      'sources': [
        'aos_sync.c',
      ],
    },
    {
      'target_name': 'core_lib',
      'type': 'static_library',
      'sources': [
        'core_lib.c',
      ],
      'dependencies': [
        'aos_sync',
        'shared_mem',
      ],
      'export_dependent_settings': [
        'aos_sync',
      ],
    },
    {
      'target_name': 'shared_mem',
      'type': 'static_library',
      'sources': [
        'shared_mem.c',
      ],
      'dependencies': [
        'aos_sync',
      ],
      'export_dependent_settings': [
        'aos_sync',
      ],
    },
    {
      'target_name': 'queue',
      'type': 'static_library',
      'sources': [
        'queue.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:condition',
        '<(AOS)/common/common.gyp:mutex',
        'core_lib',
        # TODO(brians): fix this once there's a nice logging interface to use
        # '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'raw_queue_test',
      'type': 'executable',
      'sources': [
        'queue_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'queue',
        '<(AOS)/build/aos.gyp:logging',
        'core_lib',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(AOS)/common/common.gyp:time',
      ],
    },
    {
      'target_name': 'ipc_stress_test',
      'type': 'executable',
      'sources': [
        'ipc_stress_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(AOS)/common/common.gyp:mutex',
        'core_lib',
        '<(AOS)/common/common.gyp:die',
      ],
    },
  ],
}
