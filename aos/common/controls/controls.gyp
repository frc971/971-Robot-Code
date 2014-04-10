{
  'targets': [
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
      'target_name': 'sensor_generation',
      'type': 'static_library',
      'sources': [
        'sensor_generation.q',
      ],
      'variables': {
        'header_path': 'aos/common/controls',
      },
      'includes': ['../../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'output_check',
      'type': 'static_library',
      'sources': [
        'output_check.q',
      ],
      'variables': {
        'header_path': 'aos/common/controls',
      },
      'includes': ['../../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'control_loop_queues',
      'type': 'static_library',
      'sources': [ '<(AOS)/common/controls/control_loops.q' ],
      'variables': {
        'header_path': 'aos/common/controls',
      },
      'dependencies': [
        '<(AOS)/common/common.gyp:queues',
      ],
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
        'sensor_generation',
        'output_check',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/messages/messages.gyp:robot_state',
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/util/util.gyp:phased_loop',
        '<(AOS)/common/common.gyp:time',
        'control_loop_queues',
        '<(AOS)/common/logging/logging.gyp:queue_logging',
        '<(AOS)/common/util/util.gyp:log_interval',
        'sensor_generation',
        'output_check',
      ],
    },
  ],
}
