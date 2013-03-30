{
  'variables': {
    'queue_files': [
      'CameraEnableQueue.q',
      'GyroAngle.q',
      'CameraTarget.q',
      'PhotoSensor.q',
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
