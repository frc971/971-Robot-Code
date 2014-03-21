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
        '<(AOS)/common/common.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:queues',
      ],
      'includes': ['../../build/queues.gypi'],
    },
  ],
}
