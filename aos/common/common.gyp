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
      'dependencies': [
        '<(AOS)/common/common.gyp:queues',
      ],
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
        '<(EXTERNALS):gtest',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:shared_mem',
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
      ],
    },
    {
      'target_name': 'queues',
      'type': 'static_library',
      'sources': [
        'queue.cc',
      ],
      'conditions': [
        ['OS=="crio"', {
          'dependencies': [
            '<(EXTERNALS):WPILib',
          ],
        },
        {
          'dependencies': [
            '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:queue',
          ],
          'export_dependent_settings': [
            '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:queue',
          ],
        }]
      ],
      'dependencies': [
        'time',
      ],
      'export_dependent_settings': [
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
      'target_name': 'control_loop_queues',
      'type': 'static_library',
      'sources': [ '<(AOS)/common/controls/control_loops.q' ],
      'variables': {
        'header_path': 'aos/common/controls',
      },
      'dependencies': [
        '<(AOS)/common/common.gyp:queues',
      ],
      'includes': ['../build/queues.gypi'],
    },
    {
      'target_name': 'controls',
      'type': 'static_library',
      'sources': [
        'controls/control_loop.cc',
      ],
      'dependencies': [
        '<(AOS)/common/messages/messages.gyp:robot_state',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:phased_loop',
        'time',
        'control_loop_queues',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/util/util.gyp:log_interval',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:sensor_generation',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/messages/messages.gyp:robot_state',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:phased_loop',
        'time',
        'control_loop_queues',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/util/util.gyp:log_interval',
        '<(DEPTH)/bbb_cape/src/bbb/bbb.gyp:sensor_generation',
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
        # We want to run it with the assertions etc to try and catch bugs there.
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:queue_debug',
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
      ],
    },
    {
      'target_name': 'die',
      'type': 'static_library',
      'sources': [
        'die.cc',
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
        'mutex',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:aos_sync',
      ],
    },
    {
      'target_name': 'mutex',
      'type': 'static_library',
      'conditions': [
        ['OS=="crio"', {
          'sources': [
            '<(AOS)/crio/shared_libs/mutex.cpp',
          ],
        }, {
          'sources': [
            '<(AOS)/linux_code/ipc_lib/mutex.cpp',
          ],
          'dependencies': [
            '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:aos_sync',
          ],
          'export_dependent_settings': [
            '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:aos_sync',
          ],
        }],
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging_interface',
      ],
    },
    {
      'target_name': 'mutex_test',
      'type': 'executable',
      'sources': [
        'mutex_test.cpp',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'mutex',
        'die',
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
        'die',
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
  ],
}
