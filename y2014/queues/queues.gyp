{
  'targets': [
    {
      'target_name': 'profile_params',
      'type': 'static_library',
      'sources': [
        'profile_params.q',
      ],
      'variables': {
        'header_path': 'y2014/queues',
      },
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'hot_goal',
      'type': 'static_library',
      'sources': [
        'hot_goal.q',
      ],
      'variables': {
        'header_path': 'y2014/queues',
      },
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'auto_mode',
      'type': 'static_library',
      'sources': [
        'auto_mode.q',
      ],
      'variables': {
        'header_path': 'y2014/queues',
      },
      'includes': ['../../aos/build/queues.gypi'],
    },
  ],
}
