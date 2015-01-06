{
  'targets': [
    {
      'target_name': 'gyro',
      'type': 'static_library',
      'sources': [
        'gyro.q',
      ],
      'variables': {
        'header_path': 'frc971/queues',
      },
      'includes': ['../../aos/build/queues.gypi'],
    },
  ],
}
