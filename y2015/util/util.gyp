{
  'targets': [
    {
      'target_name': 'kinematics',
      'type': 'static_library',
      'sources': [
        #'kinematics.h',
      ],
      'dependencies': [
        '<(EXTERNALS):eigen',
        '<(DEPTH)/y2015/y2015.gyp:constants',
      ],
      'export_dependent_settings': [
        '<(EXTERNALS):eigen',
        '<(DEPTH)/y2015/y2015.gyp:constants',
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
