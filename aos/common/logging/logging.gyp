{
  'targets': [
    {
      'target_name': 'logging_impl_test',
      'type': '<(aos_target)',
      'sources': [
        'logging_impl_test.cc',
      ],
      'dependencies': [
        '<(EXTERNALS):gtest',
        '<(AOS)/build/aos.gyp:logging',
      ],
    },
    {
      'target_name': 'queue_logging',
      'type': 'static_library',
      'sources': [
        'queue_logging.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:die',
        '<(AOS)/common/common.gyp:queue_types',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:die',
      ],
    },
    {
      'target_name': 'matrix_logging',
      'type': 'static_library',
      'sources': [
        'matrix_logging.cc',
      ],
      'dependencies': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:die',
        '<(AOS)/common/common.gyp:queue_types',
        '<(EXTERNALS):eigen',
      ],
      'export_dependent_settings': [
        '<(AOS)/build/aos.gyp:logging',
        '<(AOS)/common/common.gyp:die',
        '<(AOS)/common/common.gyp:queue_types',
        '<(EXTERNALS):eigen',
      ],
    },
  ],
}
