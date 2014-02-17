{
  'targets': [
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
      'target_name': 'thread',
      'type': 'static_library',
      'sources': [
        'thread.cc',
      ],
      'dependencies': [
        '<(AOS)/common/common.gyp:mutex',
      ],
      'export_dependent_settings': [
        '<(AOS)/common/common.gyp:mutex',
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
  ],
}
