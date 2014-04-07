{
  'targets': [
    {
      'target_name': 'hall_effect_tracker',
      'type': 'static_library',
      'sources': [
        #'hall_effect_tracker.h',
      ],
      'dependencies': [
        'queues',
      ],
      'export_dependent_settings': [
        'queues',
      ],
    },
    {
      'target_name': 'queues',
      'type': 'static_library',
      'sources': [
        'control_loops.q',
      ],
      'variables': {
        'header_path': 'frc971/control_loops',
      },
      'includes': ['../../aos/build/queues.gypi'],
    },
    {
      'target_name': 'coerce_goal',
      'type': 'static_library',
      'sources': [
        'coerce_goal.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):eigen',
        '<(AOS)/common/controls/controls.gyp:polytope',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):eigen',
        '<(AOS)/common/controls/controls.gyp:polytope',
      ],
    },
    {
      'target_name': 'state_feedback_loop',
      'type': 'static_library',
      'sources': [
        #'state_feedback_loop.h'
      ],
      'dependencies': [
        '<(EXTERNALS):eigen',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):eigen',
      ],
    },
  ],
}
