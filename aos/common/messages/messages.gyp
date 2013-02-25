{
  'targets': [
    {
      'target_name': 'QueueHolder',
      'type': 'static_library',
      'sources': [
        # 'QueueHolder.h'
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:timing',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:timing',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'conditions': [
        ['OS!="crio"', {
          'dependencies': [
            '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib',
          ],
          'export_dependent_settings': [
            '<(AOS)/atom_code/ipc_lib/ipc_lib.gyp:ipc_lib',
          ],
        }],
      ],
    },
    {
      'target_name': 'aos_queues',
      'type': 'static_library',
      'sources': [
        'RobotState.q',
      ],
      'variables': {
        'header_path': 'aos/common/messages',
      },
      'dependencies': [
        '<(AOS)/build/aos.gyp:aos_internal_nolib',
        '<(AOS)/common/common.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:aos_internal_nolib',
        '<(AOS)/common/common.gyp:queues',
      ],
      'includes': ['../../build/queues.gypi'],
    },
    {
      'target_name': 'queues_so',
      'type': 'shared_library',
      'sources': [
        'RobotState.q',
      ],
      'variables': {
        'header_path': 'aos/common/messages',
      },
      'dependencies': [
        '<(AOS)/build/aos.gyp:aos_internal_nolib',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:aos_internal_nolib',
      ],
      'direct_dependent_settings': {
        'variables': {
          'jni_libs': ['queues_so'],
        },
      },
      'includes': ['../../build/queues.gypi'],
    },
  ],
}
