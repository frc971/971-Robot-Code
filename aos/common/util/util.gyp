{
  'targets': [
    {
      'target_name': 'run_command',
      'type': 'static_library',
      'sources': [
        'run_command.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging_interface',
      ],
    },
    {
      'target_name': 'run_command_test',
      'type': 'executable',
      'sources': [
        'run_command_test.cc',
      ],
      'dependencies': [
        'run_command',
        '<(EXTERNALS):gtest',
        '<(AOS)/build/aos.gyp:logging',
        'thread',
      ],
    },
    {
      'target_name': 'death_test_log_implementation',
      'type': 'static_library',
      'sources': [
        #'death_test_log_implementation',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'inet_addr',
      'type': 'static_library',
      'sources': [
        'inet_addr.cc',
      ],
    },
    {
      'target_name': 'phased_loop',
      'type': 'static_library',
      'sources': [
        'phased_loop.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:time',
      ],
    },
    {
      'target_name': 'log_interval',
      'type': 'static_library',
      'sources': [
        #'log_interval.h',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'string_to_num',
      'type': 'static_library',
      'sources': [
        #'string_to_num.h',
      ],
    },
    {
      'target_name': 'string_to_num_test',
      'type': 'executable',
      'sources': [
        'string_to_num_test.cc',
      ],
      'dependencies': [
        ':string_to_num',
        '<(EXTERNALS):gtest',
      ],
    },
    {
      'target_name': 'thread',
      'type': 'static_library',
      'sources': [
        'thread.cc',
      ],
    },
    {
      'target_name': 'trapezoid_profile',
      'type': 'static_library',
      'sources': [
        'trapezoid_profile.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):eigen',
        '<(AOS)/common/common.gyp:time',
        '<(AOS)/build/aos.gyp:logging',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):eigen',
        '<(AOS)/common/common.gyp:time',
      ],
    },
    {
      'target_name': 'trapezoid_profile_test',
      'type': 'executable',
      'sources': [
        'trapezoid_profile_test.cc',
      ],
      'dependencies': [
        ':trapezoid_profile',
        '<(EXTERNALS):gtest',
      ],
    },
    {
      'target_name': 'wrapping_counter',
      'type': 'static_library',
      'sources': [
        'wrapping_counter.cc',
      ],
    },
    {
      'target_name': 'wrapping_counter_test',
      'type': 'executable',
      'sources': [
        'wrapping_counter_test.cc',
      ],
      'dependencies': [
        'wrapping_counter',
        '<(EXTERNALS):gtest',
      ],
    },
    {
      'target_name': 'options_test',
      'type': 'executable',
      'sources': [
        'options_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
      ],
    },
    {
      'target_name': 'kinematics',
      'type': 'static_library',
      'sources': [
        #'kinematics.h',
      ],
      'dependencies': [
        '<(EXTERNALS):eigen',
        '<(DEPTH)/frc971/frc971.gyp:constants',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):eigen',
        '<(DEPTH)/frc971/frc971.gyp:constants',
      ],
    },
    {
      'target_name': 'kinematics_test',
      'type': 'executable',
      'sources': [
        'kinematics_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(AOS)/common/common.gyp:queue_testutils',
        '<(AOS)/build/aos.gyp:logging',
        '<(DEPTH)/frc971/control_loops/control_loops.gyp:team_number_test_environment',
        'kinematics'
      ],
    },
  ],
}
