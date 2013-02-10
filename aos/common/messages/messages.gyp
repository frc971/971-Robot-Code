{
  'targets': [
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
