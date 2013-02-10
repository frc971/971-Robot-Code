{
  'variables': {
    'queue_files': [
      'GyroAngle.q',
      'Piston.q',
    ]
  },
  'targets': [
    {
      'target_name': 'queues',
      'type': 'static_library',
      'sources': ['<@(queue_files)'],
      'variables': {
        'header_path': 'frc971/queues',
      },
      'dependencies': [
        '<(AOS)/common/common.gyp:queues',
        '<(AOS)/build/aos.gyp:libaos',
      ],
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'frc971_queues_so',
      'type': 'loadable_module',
      'sources': ['<@(queue_files)'],
      'variables': {
        'header_path': 'frc971/queues',
      },
      'dependencies': [
        '<(AOS)/build/aos.gyp:aos_shared_lib',
      ],
      'direct_dependent_settings': {
        'variables': {
          'jni_libs': ['frc971_queues_so'],
        },
      },
      'includes': ['../../aos/build/queues.gypi'],
    },
  ],
}
