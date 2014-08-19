{
  'targets': [
    {
      'target_name': 'aos_sync',
      'type': 'static_library',
      'sources': [
        'aos_sync.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging_interface',
        '<(AOS)/common/common.gyp:once',
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
        '<(AOS)/build/aos.gyp:logging_interface',
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
        '<(AOS)/build/aos.gyp:logging_interface',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:logging_interface',
      ],
    },
    {
      'target_name': 'raw_queue_test',
      'type': 'executable',
      'sources': [
        'raw_queue_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'queue',
        '<(AOS)/build/aos.gyp:logging',
        'core_lib',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:die',
        '<(AOS)/common/util/util.gyp:thread',
        '<(AOS)/common/util/util.gyp:death_test_log_implementation',
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
        '<(AOS)/common/libc/libc.gyp:dirname',
        '<(AOS)/common/libc/libc.gyp:aos_strsignal',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'variables': {
        'is_special_test': 1,
      },
    },
    {
      'target_name': 'scoped_message_ptr',
      'type': 'static_library',
      'sources': [
        #'scoped_message_ptr.h',
      ],
      'dependencies': [
        'queue',
      ],
      'export_dependent_settings': [
        'queue',
      ],
    },
  ],
}
