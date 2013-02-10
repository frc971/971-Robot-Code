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
        '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib'
      ],
    },
    {
      'target_name': 'time',
      'type': 'static_library',
      'sources': [
        'time.cc'
      ],
      'dependencies': [
         # TODO(aschuh): Fix this dependency loop by
         # providing a logging interface.
         # '<(AOS)/build/aos.gyp:logging',
         '<(AOS)/build/aos.gyp:aos/ResourceList.h',
      ],
    },
    {
      'target_name': 'common',
      'type': 'static_library',
      'sources': [
        'Configuration.cpp',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
      ],
      'conditions': [
        ['OS=="crio"', {
          'dependencies': [
            '<(EXTERNALS):WPILib',
        ]}],
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
            '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib',
          ],
          'export_dependent_settings': [
            '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib',
          ],
        }]
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:common',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:common',
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
        '<(AOS)/build/aos.gyp:aos_shared_lib',
      ],
      'direct_dependent_settings': {
        'variables': {
          'jni_libs': [
            'timing_so',
          ],
        },
      },
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:aos_shared_lib',
      ],
    },
    {
      'target_name': 'timing',
      'type': 'static_library',
      'sources': [
        'control_loop/Timing.cpp'
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:libaos',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'controls',
      'type': 'static_library',
      'sources': [],
      'dependencies': [
        '<(AOS)/common/messages/messages.gyp:aos_queues',
        '<(AOS)/build/aos.gyp:logging',
        'timing',
      ],
    },
    {
      'target_name': 'queue_test',
      'type': 'executable',
      'sources': [
        '<(AOS)/common/queue_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(AOS)/build/aos.gyp:libaos',
        'queue_testutils',
        'common',
        'queue_test_queue',
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
        '<(AOS)/build/aos.gyp:libaos',
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
        '<(AOS)/build/aos.gyp:libaos',
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
        '<(AOS)/build/aos.gyp:libaos',
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
        '<(AOS)/build/aos.gyp:libaos',
      ],
    },
  ],
}
