{
  'targets': [
    {
      'target_name': 'action_lib',
      'type': 'static_library',
      'sources': [
        'actions.cc',
        'actor.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:queues',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(AOS)/common/util/util.gyp:phased_loop',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:queues',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/controls/controls.gyp:control_loop',
        '<(AOS)/common/util/util.gyp:phased_loop',
      ],
    },
    {
      'target_name': 'action_queue',
      'type': 'static_library',
      'sources': ['actions.q'],
      'variables': {
        'header_path': 'aos/common/actions',
      },
      'includes': ['../../build/queues.gypi'],
    },
    {
      'target_name': 'test_action_queue',
      'type': 'static_library',
      'sources': ['test_action.q'],
      'variables': {
        'header_path': 'aos/common/actions',
      },
      'dependencies': [
        'action_queue',
      ],
      'export_dependent_settings': [
        'action_queue',
      ],
      'includes': ['../../build/queues.gypi'],
    },
    {
      'target_name': 'action_test',
      'type': 'executable',
      'sources': [
        'action_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        'action_lib',
        'test_action_queue',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/common.gyp:queues',
        '<(AOS)/common/common.gyp:time',
        'action_queue'
      ],
    },
  ],
}
