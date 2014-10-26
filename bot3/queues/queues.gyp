{
  'variables': {
    'queue_files': [
      'rollers.q',
      'to_log.q',
    ]
  },
  'targets': [
    {
      'target_name': 'queues',
      'type': 'static_library',
      'sources': ['<@(queue_files)'],
      'variables': {
        'header_path': 'bot3/queues',
      },
      'includes': ['../../aos/build/queues.gypi'],
    },
  ],
}
