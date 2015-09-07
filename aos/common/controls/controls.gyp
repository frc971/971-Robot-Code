{
  'targets': [
    {
      'target_name': 'replay_control_loop',
      'type': 'static_library',
      'sources': [
        #'replay_control_loop.h',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:queues',
        'control_loop',
        '<(AOS)/linux_code/logging/logging.gyp:log_replay',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/common.gyp:time',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:queues',
        'control_loop',
        '<(AOS)/linux_code/logging/logging.gyp:log_replay',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/common.gyp:time',
      ],
    },
    {
      'target_name': 'control_loop_test',
      'type': 'static_library',
      'sources': [
        'control_loop_test.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/messages/messages.gyp:robot_state',
        '<(EXTERNALS):gtest',
        '<(AOS)/common/common.gyp:queue_testutils',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):gtest',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/common/common.gyp:queue_testutils',
      ],
    },
    {
      'target_name': 'polytope',
      'type': 'static_library',
      'sources': [
        #'polytope.h',
      ],
      'dependencies': [
        '<(EXTERNALS):eigen',
        '<(EXTERNALS):libcdd',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):eigen',
        '<(EXTERNALS):libcdd',
      ],
    },
    {
      'target_name': 'control_loop_queues',
      'type': 'static_library',
      'sources': [ 'control_loops.q' ],
      'variables': {
        'header_path': 'aos/common/controls',
      },
      'includes': ['../../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'control_loop',
      'type': 'static_library',
      'sources': [
        'control_loop.cc',
      ],
      'dependencies': [
        '<(AOS)/common/messages/messages.gyp:robot_state',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/common.gyp:time',
        'control_loop_queues',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/util/util.gyp:log_interval',
        '<(AOS)/common/common.gyp:queues',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/messages/messages.gyp:robot_state',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/common.gyp:time',
        'control_loop_queues',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/util/util.gyp:log_interval',
        '<(AOS)/common/common.gyp:queues',
      ],
    },
  ],
}
