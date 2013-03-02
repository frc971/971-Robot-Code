{
  'targets': [
    {
      'target_name': 'trapezoid_profile',
      'type': 'static_library',
      'sources': [
        'trapezoid_profile.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):eigen',
        '<(AOS)/common/common.gyp:time',
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
        # TODO(brians): remove this when time no longer requires it
        '<(AOS)/build/aos.gyp:logging',
        # TODO(brians): remove this when logging no longer requires it
        #'<(AOS)/common/common.gyp:die',
        '<(AOS)/build/aos.gyp:libaos',
      ],
    },
  ],
}
