{
  'targets': [
    {
      'target_name': 'robot_state',
      'type': 'static_library',
      'sources': [
        'robot_state.q',
      ],
      'variables': {
        'header_path': 'aos/common/messages',
      },
      'includes': ['../../build/queues.gypi'],
    },
  ],
}
