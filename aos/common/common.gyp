{
  'targets': [
    {
      'target_name': 'queue_test_queue',
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
      'sources': [
        'queue_types.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging_interface',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:shared_mem',
        '<(AOS)/linux_code/ipc_lib/ipc_lib.gyp:core_lib',
      ],
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
      'sources': [ '<(AOS)/common/control_loop/control_loops.q' ],
      'variables': {
        'header_path': 'aos/common/control_loop',
      },
      'dependencies': [
        '<(AOS)/common/common.gyp:queues',
      ],
      'includes': ['../build/queues.gypi'],
    },
    {
      'target_name': 'timing_so',
      'type': 'shared_library',
      'sources': [
        'control_loop/Timing.cpp'
      ],
      'variables': {'no_rsync': 1},
      'dependencies': [
      ],
      'direct_dependent_settings': {
        'variables': {
          'jni_libs': [
            'timing_so',
          ],
        },
      },
      'export_dependent_settings': [
      ],
    },
    {
      'target_name': 'timing',
      'type': 'static_library',
      'sources': [
        'control_loop/Timing.cpp'
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        'time',
      ],
    },
    {
      'target_name': 'controls',
      'type': 'static_library',
      'sources': [
        'control_loop/ControlLoop.cc',
      ],
      'dependencies': [
        '<(AOS)/common/messages/messages.gyp:aos_queues',
        '<(AOS)/build/aos.gyp:logging',
        'timing',
        'time',
        'control_loop_queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/messages/messages.gyp:aos_queues',
        '<(AOS)/build/aos.gyp:logging',
        'timing',
        'time',
        'control_loop_queues',
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
        'queue_test_queue',
        '<(AOS)/common/util/util.gyp:thread',
        'die',
      ],
    },
    {
      'target_name': 'type_traits_test',
      'type': '<(aos_target)',
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
      'type': '<(aos_target)',
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
      'type': '<(aos_target)',
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
      'type': '<(aos_target)',
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
    {
      'target_name': 'util',
      'type': 'static_library',
      'sources': [
        'util.cc',
      ],
    },
  ],
}
