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
        '<(AOS)/build/aos.gyp:logging_interface',
      ],
      'defines': [
        'QUEUE_DEBUG=0',
      ],
    },
    # A version of the queue code compiled with assertions enabled etc.
    {
      'target_name': 'queue_debug',
      'type': 'static_library',
      'sources': [
        'queue.cc',
      ],
      'dependencies': [
        'queue',
      ],
      'export_dependent_settings': [
        'queue',
      ],
      'defines': [
        'QUEUE_DEBUG=1',
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
        'queue_debug',
        '<(AOS)/build/aos.gyp:logging',
        'core_lib',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:die',
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
