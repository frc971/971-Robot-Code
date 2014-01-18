{
  'variables': {
    'queue_files': [
      'gyro_angle.q',
      'photo_sensor.q',
      'piston.q',
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
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:queues',
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
