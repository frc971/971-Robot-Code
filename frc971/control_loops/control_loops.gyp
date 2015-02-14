{
  'targets': [
    {
      'target_name': 'team_number_test_environment',
      'type': 'static_library',
      'sources': [
        'team_number_test_environment.cc'
      ],
      'dependencies': [
        '<(AOS)/common/network/network.gyp:team_number',
        '<(EXTERNALS):gtest',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):gtest',
      ],
    },
    {
      'target_name': 'state_feedback_loop_test',
      'type': 'executable',
      'sources': [
        'state_feedback_loop_test.cc',
      ],
      'dependencies': [
        'state_feedback_loop',
        '<(EXTERNALS):gtest',
      ],
    },
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
      'target_name': 'position_sensor_sim_test',
      'type': 'executable',
      'sources': [
        'position_sensor_sim_test.cc',
      ],
      'dependencies': [
        'queues',
        'position_sensor_sim',
        '<(EXTERNALS):gtest',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'position_sensor_sim',
      'type': 'static_library',
      'sources': [
        'position_sensor_sim.cc',
      ],
      'dependencies': [
        'queues',
        'gaussian_noise',
      ],
    },
    {
      'target_name': 'gaussian_noise',
      'type': 'static_library',
      'sources': [
        'gaussian_noise.cc',
      ],
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
