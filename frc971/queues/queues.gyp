{
  'variables': {
    'queue_files': [
      'other_sensors.q',
      'to_log.q',
    ]
  },
  'targets': [
    {
      'target_name': 'hot_goal',
      'type': 'static_library',
      'sources': [
        'hot_goal.q',
      ],
      'variables': {
        'header_path': 'frc971/queues',
      },
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'queues',
      'type': 'static_library',
      'sources': ['<@(queue_files)'],
      'variables': {
        'header_path': 'frc971/queues',
      },
      'includes': ['../../aos/build/queues.gypi'],
    },
  ],
}
