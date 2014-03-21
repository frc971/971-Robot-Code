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
      'dependencies': [
        '<(AOS)/common/common.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:queues',
      ],
      'includes': ['../../build/queues.gypi'],
    },
  ],
}
