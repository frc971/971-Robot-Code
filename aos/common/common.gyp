{
  'targets': [
    {
      'target_name': 'test_queue',
      'type': 'static_library',
      'sources': [
        '<(AOS)/common/test_queue.q',
      ],
      'variables': {
        'header_path': 'aos/common',
      },
      'includes': ['../build/queues.gypi'],
    },
    {
      'target_name': 'queue_testutils',
      'type': 'static_library',
      'sources': [
        'queue_testutils.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        'once',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:shared_mem',
        'mutex',
      ],
      'export_dependent_settings': [
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:shared_mem',
       ],
    },
    {
      'target_name': 'time',
      'type': 'static_library',
      'sources': [
        'time.cc'
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging_interface',
        'mutex',
      ],
    },
    {
      'target_name': 'queue_types',
      'type': 'static_library',
      'variables': {
        'print_field_cc': '<(SHARED_INTERMEDIATE_DIR)/print_field.cc',
        'queue_primitives_h': '<(SHARED_INTERMEDIATE_DIR)/aos_queue_primitives/aos/queue_primitives.h',
      },
      'sources': [
        'queue_types.cc',
        '<(print_field_cc)',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging_interface',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:shared_mem',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:core_lib',
        'mutex',
        'time',
      ],
      'export_dependent_settings': [
        'time',
      ],
      'actions': [
        {
          'variables': {
            'script': '<(AOS)/build/queues/print_field.rb',
          },
          'action_name': 'gen_print_field',
          'inputs': [
            '<(script)',
            '<!@(find <(AOS)/build/queues/ -name *.rb)',
          ],
          'outputs': [
            '<(print_field_cc)',
          ],
          'action': ['ruby', '<(script)', '<(print_field_cc)'],
          'message': 'Generating print_field.cc',
        },
        {
          'variables': {
            'script': '<(AOS)/build/queues/queue_primitives.rb',
          },
          'action_name': 'gen_queue_primitives',
          'inputs': [
            '<(script)',
            '<!@(find <(AOS)/build/queues/ -name *.rb)',
          ],
          'outputs': [
            '<(queue_primitives_h)',
          ],
          'action': ['ruby', '<(script)', '<(queue_primitives_h)'],
          'message': 'Generating queue_primitives.h',
        },
      ],
      'direct_dependent_settings': {
        'include_dirs': [
          '<(SHARED_INTERMEDIATE_DIR)/aos_queue_primitives',
        ],
      },
      'hard_dependency': 1,
    },
    {
      'target_name': 'queue_types_test',
      'type': 'executable',
      'sources': [
        'queue_types_test.cc',
      ],
      'dependencies': [
        'queue_types',
        '<(EXTERNALS):gtest',
        'test_queue',
        '<(AOS)/build/aos.gyp:logging',
        'queue_testutils',
      ],
    },
    {
      'target_name': 'queues',
      'type': 'static_library',
      'sources': [
        'queue.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:queue',
        'time',
      ],
      'export_dependent_settings': [
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:queue',
        'time',
      ],
    },
    {
      'target_name': 'scoped_fd',
      'type': 'static_library',
      'sources': [
        # 'scoped_fd.h'
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'queue_test',
      'type': 'executable',
      'sources': [
        'queue_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'queue_testutils',
        'test_queue',
        '<(AOS)/common/util/util.gyp:thread',
        'die',
      ],
    },
    {
      'target_name': 'type_traits_test',
      'type': 'executable',
      'sources': [
        'type_traits_test.cpp',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
      ],
    },
    {
      'target_name': 'gtest_prod',
      'type': 'static_library',
      'dependencies': [
        '<(EXTERNALS):gtest_prod',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):gtest_prod',
      ],
    },
    {
      'target_name': 'once',
      'type': 'static_library',
      'dependencies': [
        '<(EXTERNALS):gtest_prod',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):gtest_prod',
      ],
    },
    {
      'target_name': 'once_test',
      'type': 'executable',
      'sources': [
        'once_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'once',
      ],
    },
    {
      'target_name': 'time_test',
      'type': 'executable',
      'sources': [
        'time_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'time',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:death_test_log_implementation',
      ],
    },
    {
      'target_name': 'die',
      'type': 'static_library',
      'sources': [
        'die.cc',
      ],
      'dependencies': [
        '<(AOS)/common/libc/libc.gyp:aos_strerror',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/libc/libc.gyp:aos_strerror',
      ],
    },
    {
      'target_name': 'condition',
      'type': 'static_library',
      'sources': [
        '<(AOS)/linux_code/ipc_lib/condition.cc',
      ],
      'dependencies': [
        'mutex',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:aos_sync',
        '<(AOS)/build/aos.gyp:logging_interface',
      ],
      'export_dependent_settings': [
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:aos_sync',
      ],
    },
    {
      'target_name': 'mutex',
      'type': 'static_library',
      'sources': [
        '<(AOS)/linux_code/ipc_lib/mutex.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:aos_sync',
        '<(AOS)/build/aos.gyp:logging_interface',
      ],
      'export_dependent_settings': [
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:aos_sync',
      ],
    },
    {
      'target_name': 'event',
      'type': 'static_library',
      'sources': [
        '<(AOS)/linux_code/ipc_lib/event.cc',
      ],
      'dependencies': [
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:aos_sync',
        '<(AOS)/build/aos.gyp:logging_interface',
      ],
      'export_dependent_settings': [
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:aos_sync',
      ],
    },
    {
      'target_name': 'queue_testutils_test',
      'type': 'executable',
      'sources': [
        'queue_testutils_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'queue_testutils',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'mutex_test',
      'type': 'executable',
      'sources': [
        'mutex_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'mutex',
        'die',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:death_test_log_implementation',
        '<(AOS)/common/util/util.gyp:thread',
        '<(AOS)/common/common.gyp:time',
        'queue_testutils',
      ],
    },
    {
      'target_name': 'event_test',
      'type': 'executable',
      'sources': [
        'event_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'event',
        'queue_testutils',
        'time',
      ],
    },
    {
      'target_name': 'condition_test',
      'type': 'executable',
      'sources': [
        'condition_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'condition',
        '<(AOS)/common/util/util.gyp:thread',
        'time',
        'mutex',
        '<(AOS)/build/aos.gyp:logging',
        'queue_testutils',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:core_lib',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:aos_sync',
        'die',
        '<(AOS)/common/util/util.gyp:thread',
       ],
    },
    {
      'target_name': 'die_test',
      'type': 'executable',
      'sources': [
        'die_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'die',
      ],
    },
    {
      'target_name': 'stl_mutex',
      'type': 'static_library',
      'sources': [
        #'stl_mutex.h'
      ],
      'dependencies': [
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:aos_sync',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:aos_sync',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'stl_mutex_test',
      'type': 'executable',
      'sources': [
        'stl_mutex_test.cc',
      ],
      'dependencies': [
        'stl_mutex',
        '<(EXTERNALS):gtest',
        'queue_testutils',
        '<(AOS)/common/util/util.gyp:thread',
        'die',
      ],
    },
  ],
}
